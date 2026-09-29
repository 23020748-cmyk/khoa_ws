"""MoveIt 2-backed pick, place and home operations for the configured scene."""

import copy
import threading

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint


class RobotSkills:
    SUCCESS = "SUCCESS"
    PLANNING_FAILED = "PLANNING_FAILED"
    EXECUTION_FAILED = "EXECUTION_FAILED"
    FAILURE = "FAILURE"
    INVALID_OBJECT = "INVALID_OBJECT"
    INVALID_REGION = "INVALID_REGION"

    def __init__(self, node, scene):
        self.node = node
        self.scene = scene
        self.frame = scene["world_frame"]
        self.group = scene["planning_group"]
        self.eef = scene["eef_link"]
        self.orientation = scene.get("tool_orientation_xyzw", [1.0, 0.0, 0.0, 0.0])
        self.objects = copy.deepcopy(scene["objects"])
        self.zones = scene["zones"]
        self.table = scene["table"]
        self._held = None
        self._lock = threading.Lock()
        self.move_client = ActionClient(node, MoveGroup, scene.get("move_action", "/move_action"))
        self.execute_client = ActionClient(
            node, ExecuteTrajectory, scene.get("execute_action", "/execute_trajectory")
        )
        self.scene_client = node.create_client(
            ApplyPlanningScene, scene.get("apply_scene_service", "/apply_planning_scene")
        )
        self.gripper = scene.get("gripper", {})
        self.gripper_client = None
        if self.gripper.get("enabled", False):
            self.gripper_client = ActionClient(
                node,
                FollowJointTrajectory,
                self.gripper.get("action", "/gripper_controller/follow_joint_trajectory"),
            )

    def initialize(self, timeout_sec=2.0):
        for client, label in (
            (self.move_client, "MoveGroup action"),
            (self.execute_client, "ExecuteTrajectory action"),
        ):
            if not client.wait_for_server(timeout_sec=timeout_sec):
                return False, f"{label} is unavailable."
        if not self.scene_client.wait_for_service(timeout_sec=timeout_sec):
            return False, "ApplyPlanningScene service is unavailable."
        if self.gripper_client is not None and not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            return False, "Configured gripper action server is unavailable."
        if not self._apply_world_scene():
            return False, "MoveIt rejected the configured planning scene."
        return True, "Ready."

    def pick(self, object_id):
        with self._lock:
            if object_id not in self.objects:
                return self.INVALID_OBJECT, f"Object {object_id!r} is not in the scene catalog."
            if self._held is not None:
                return self.FAILURE, f"Already holding {self._held}."
            if self.gripper_client is None and not self.gripper.get("simulation_scene_only", False):
                return self.FAILURE, "Gripper is not configured; refusing a non-physical pick."
            obj = self.objects[object_id]
            if not self._gripper(opening=True):
                return self.FAILURE, "Could not open gripper."
            tcp_pose = self._object_tcp_pose(obj)
            approach = self._offset_pose(tcp_pose, float(self.scene.get("approach_height_m", 0.12)))
            status, reason = self._move_pose(approach)
            if status != self.SUCCESS:
                return status, "Could not reach the approach pose: " + reason
            # The target object is removed only for the final contact motion; other scene
            # obstacles, the table and MoveIt's robot self-collision checks stay active.
            if not self._remove_world_object(object_id):
                return self.FAILURE, "Could not update the MoveIt scene for grasp contact."
            status, reason = self._move_pose(tcp_pose)
            if status != self.SUCCESS:
                self._add_world_object(object_id, obj)
                return status, "Could not reach the grasp pose: " + reason
            if not self._gripper(opening=False):
                self._add_world_object(object_id, obj)
                return self.FAILURE, "Gripper failed to close."
            if not self._attach_object(object_id, obj):
                self._gripper(opening=True)
                self._add_world_object(object_id, obj)
                return self.FAILURE, "Could not attach object in the MoveIt scene."
            self._held = object_id
            status, reason = self._move_pose(approach)
            if status != self.SUCCESS:
                return status, "Object grasped, but retreat failed: " + reason
            return self.SUCCESS, "Object grasped and attached in the MoveIt scene."

    def place(self, object_id, zone_id):
        with self._lock:
            if object_id not in self.objects:
                return self.INVALID_OBJECT, f"Object {object_id!r} is not in the scene catalog."
            if zone_id not in self.zones:
                return self.INVALID_REGION, f"Zone {zone_id!r} is not in the scene catalog."
            if self._held != object_id:
                return self.FAILURE, f"Robot is not holding {object_id}."
            obj = self.objects[object_id]
            dimensions = obj["size_xyz"]
            surface = self.zones[zone_id]["surface_xyz"]
            target = Pose()
            target.position.x = float(surface[0])
            target.position.y = float(surface[1])
            target.position.z = float(surface[2]) + float(dimensions[2]) + float(
                self.scene.get("place_clearance_m", 0.002)
            )
            target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = self.orientation
            approach = self._offset_pose(target, float(self.scene.get("approach_height_m", 0.12)))
            status, reason = self._move_pose(approach)
            if status != self.SUCCESS:
                return status, "Could not reach the zone approach pose: " + reason
            status, reason = self._move_pose(target)
            if status != self.SUCCESS:
                return status, "Could not reach the placement pose: " + reason
            if not self._gripper(opening=True):
                return self.FAILURE, "Gripper failed to open at the destination."
            placed_obj = copy.deepcopy(obj)
            placed_obj["position_xyz"] = [
                target.position.x,
                target.position.y,
                target.position.z - float(dimensions[2]) / 2.0,
            ]
            if not self._detach_and_add_world(object_id, placed_obj):
                return self.FAILURE, "Could not update the MoveIt scene after release."
            self.objects[object_id] = placed_obj
            self._held = None
            status, reason = self._move_pose(approach)
            if status != self.SUCCESS:
                return status, "Object placed, but retreat failed: " + reason
            return self.SUCCESS, f"Object placed in {zone_id}."

    def home(self):
        with self._lock:
            if self._held is not None:
                return self.FAILURE, "Refusing home() while an object is held."
            positions = self.scene.get("home_joint_positions", {})
            if not positions:
                return self.FAILURE, "home_joint_positions is missing from scene configuration."
            constraints = Constraints()
            for name, value in positions.items():
                joint = JointConstraint()
                joint.joint_name = name
                joint.position = float(value)
                joint.tolerance_above = float(self.scene.get("home_joint_tolerance_rad", 0.04))
                joint.tolerance_below = joint.tolerance_above
                joint.weight = 1.0
                constraints.joint_constraints.append(joint)
            return self._move_constraints(constraints)

    def _move_pose(self, pose):
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.frame
        position.link_name = self.eef
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(copy.deepcopy(pose))
        position.weight = 1.0
        orient = OrientationConstraint()
        orient.header.frame_id = self.frame
        orient.link_name = self.eef
        orient.orientation = copy.deepcopy(pose.orientation)
        orient.absolute_x_axis_tolerance = float(self.scene.get("orientation_tolerance_rad", 0.12))
        orient.absolute_y_axis_tolerance = orient.absolute_x_axis_tolerance
        orient.absolute_z_axis_tolerance = orient.absolute_x_axis_tolerance
        orient.weight = 1.0
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orient)
        return self._move_constraints(constraints)

    def _move_constraints(self, constraints):
        request = MotionPlanRequest()
        request.group_name = self.group
        request.pipeline_id = self.scene.get("planning_pipeline", "ompl")
        request.planner_id = self.scene.get("planner_id", "")
        request.num_planning_attempts = int(self.scene.get("planning_attempts", 8))
        request.allowed_planning_time = float(self.scene.get("planning_time_sec", 5.0))
        request.max_velocity_scaling_factor = float(self.scene.get("velocity_scaling", 0.15))
        request.max_acceleration_scaling_factor = float(self.scene.get("acceleration_scaling", 0.15))
        request.start_state = RobotState()
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        planned = self._send_action(self.move_client, goal)
        if planned is None or planned.status != GoalStatus.STATUS_SUCCEEDED:
            return self.PLANNING_FAILED, "MoveGroup action failed while planning."
        result = planned.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return self.PLANNING_FAILED, f"MoveIt planning error {result.error_code.val}."
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = result.planned_trajectory
        executed = self._send_action(self.execute_client, execute_goal, timeout=180.0)
        if executed is None or executed.status != GoalStatus.STATUS_SUCCEEDED:
            return self.EXECUTION_FAILED, "ExecuteTrajectory action failed."
        if executed.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return self.EXECUTION_FAILED, f"MoveIt execution error {executed.result.error_code.val}."
        return self.SUCCESS, "MoveIt planned and executed the motion."

    def _send_action(self, client, goal, timeout=45.0):
        try:
            handle = self._await_future(client.send_goal_async(goal), timeout)
            if handle is None or not handle.accepted:
                return None
            return self._await_future(handle.get_result_async(), timeout)
        except Exception as error:
            self.node.get_logger().error(f"Action failed or timed out: {error}")
            return None

    def _gripper(self, opening):
        if self.gripper_client is None:
            return bool(self.gripper.get("simulation_scene_only", False))
        joints = self.gripper.get("joint_names", [])
        values = self.gripper.get("open_positions" if opening else "closed_positions", [])
        if not joints or len(joints) != len(values):
            self.node.get_logger().error("Configure matching gripper joint_names and position arrays.")
            return False
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in values]
        point.time_from_start.sec = int(self.gripper.get("motion_time_sec", 1))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        goal.trajectory.points = [point]
        try:
            handle = self._await_future(self.gripper_client.send_goal_async(goal), 10.0)
            if handle is None or not handle.accepted:
                return False
            result = self._await_future(handle.get_result_async(), 15.0)
            return (
                result is not None
                and result.status == GoalStatus.STATUS_SUCCEEDED
                and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
        except Exception as error:
            self.node.get_logger().error(f"Gripper action failed: {error}")
            return False

    def _object_tcp_pose(self, obj):
        pose = Pose()
        position = obj["position_xyz"]
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2]) + float(obj["size_xyz"][2]) / 2.0
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = self.orientation
        return pose

    @staticmethod
    def _offset_pose(pose, dz):
        result = copy.deepcopy(pose)
        result.position.z += dz
        return result

    def _collision_object(self, object_id, obj):
        collision = CollisionObject()
        collision.header.frame_id = self.frame
        collision.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in obj["size_xyz"]]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [
            float(value) for value in obj["position_xyz"]
        ]
        pose.orientation.w = 1.0
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        return collision

    def _apply_world_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self._collision_object(
            "work_table",
            {"size_xyz": self.table["size_xyz"], "position_xyz": self.table["center_xyz"]},
        ))
        for object_id, obj in self.objects.items():
            scene.world.collision_objects.append(self._collision_object(object_id, obj))
        return self._apply_scene(scene)

    def _remove_world_object(self, object_id):
        scene = PlanningScene()
        scene.is_diff = True
        collision = CollisionObject()
        collision.header.frame_id = self.frame
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(collision)
        return self._apply_scene(scene)

    def _add_world_object(self, object_id, obj):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self._collision_object(object_id, obj))
        return self._apply_scene(scene)

    def _attach_object(self, object_id, obj):
        attached = AttachedCollisionObject()
        attached.link_name = self.eef
        attached.touch_links = list(self.gripper.get("touch_links", [self.eef]))
        attached.object.id = object_id
        attached.object.header.frame_id = self.eef
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in obj["size_xyz"]]
        pose = Pose()
        # tool_orientation_xyzw is a 180-degree X rotation by default, so local +Z points down.
        lift = float(self.gripper.get("attached_lift_m",
                                      0.01 if self.gripper.get("simulation_scene_only", False) else 0.0))
        pose.position.z = float(obj["size_xyz"][2]) / 2.0 - lift
        pose.orientation.w = 1.0
        attached.object.primitives = [primitive]
        attached.object.primitive_poses = [pose]
        attached.object.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        return self._apply_scene(scene)

    def _detach_and_add_world(self, object_id, obj):
        scene = PlanningScene()
        scene.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = self.eef
        attached.object.id = object_id
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        scene.world.collision_objects.append(self._collision_object(object_id, obj))
        return self._apply_scene(scene)

    @staticmethod
    def _await_future(future, timeout):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout):
            return None
        return future.result()

    def _apply_scene(self, scene):
        try:
            request = ApplyPlanningScene.Request()
            request.scene = scene
            response = self._await_future(self.scene_client.call_async(request), 10.0)
            return response is not None and response.success
        except Exception as error:
            self.node.get_logger().error(f"ApplyPlanningScene failed: {error}")
            return False
