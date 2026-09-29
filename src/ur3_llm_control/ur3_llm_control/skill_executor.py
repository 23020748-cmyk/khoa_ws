"""Revalidates plans at the robot boundary and runs skills sequentially."""

import json
import queue
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .robot_skills import RobotSkills
from .task_validator import PlanValidationError, TaskValidator


class SkillExecutor(Node):
    def __init__(self):
        super().__init__("skill_executor")
        share = get_package_share_directory("ur3_llm_control")
        self.declare_parameter("scene_config", share + "/config/scene.yaml")
        self.declare_parameter("student_config", share + "/config/student_config.yaml")
        self.declare_parameter("simulation_scene_only_gripper", False)
        with open(self.get_parameter("scene_config").value, "r", encoding="utf-8") as stream:
            self.scene = yaml.safe_load(stream)
        self.validator = TaskValidator(self.scene["objects"], self.scene["zones"])
        self.scene["gripper"]["simulation_scene_only"] = bool(
            self.get_parameter("simulation_scene_only_gripper").value
        )
        if self.scene["gripper"]["simulation_scene_only"]:
            self.get_logger().warning("Gazebo demo has no gripper: pick/place will update MoveIt only, with no physical grasp.")
        self.robot = RobotSkills(self, self.scene)
        self._initialized = threading.Event()
        self._initializing = False
        self._plans = queue.Queue()
        self.create_subscription(String, "/validated_plan", self._on_plan, 10)
        self.ready_pub = self.create_publisher(Bool, "/skill_executor/ready", 10)
        self.result_pub = self.create_publisher(String, "/task_result", 10)
        self.create_timer(0.5, self._initialize)
        threading.Thread(target=self._execute_queue, daemon=True).start()

    def _initialize(self):
        if self._initialized.is_set():
            self.ready_pub.publish(Bool(data=True))
            return
        if self._initializing:
            return
        self._initializing = True
        threading.Thread(target=self._initialize_worker, daemon=True).start()

    def _initialize_worker(self):
        ready, reason = self.robot.initialize()
        self._initializing = False
        if ready:
            self._initialized.set()
            self.get_logger().info("MoveIt scene loaded; skill executor ready.")
        else:
            self.get_logger().warning(f"Waiting for MoveIt: {reason}")

    def _on_plan(self, message):
        self._plans.put(message.data)

    def _execute_queue(self):
        while rclpy.ok():
            try:
                encoded = self._plans.get(timeout=0.2)
            except queue.Empty:
                continue
            self._execute_plan(encoded)

    def _execute_plan(self, encoded):
        if not self._initialized.wait(timeout=120.0):
            self.get_logger().error("MoveIt is not ready; rejected plan.")
            return
        task_id = None
        try:
            envelope = json.loads(encoded)
            task_id = envelope["task_id"]
            plan = self.validator.validate({"plan": envelope["plan"]})
        except (ValueError, KeyError, PlanValidationError, TypeError) as error:
            self.get_logger().error(f"Plan rejected at executor boundary: {error}")
            if task_id:
                self._publish_result(task_id, False, str(error))
            return

        print("\nTHỰC HIỆN:", flush=True)
        success = True
        reason = ""
        for step in plan:
            status, reason = self._execute_step(step)
            line = f"{self._format_step(step)} ........ {self._vietnamese_status(status)}"
            if status != RobotSkills.SUCCESS:
                line += f" — {reason}"
            print(line, flush=True)
            if status != RobotSkills.SUCCESS:
                success = False
                break
        self._publish_result(task_id, success, reason)

    def _execute_step(self, step):
        if step["skill"] == "pick":
            return self.robot.pick(step["object"])
        if step["skill"] == "place":
            return self.robot.place(step["object"], step["zone"])
        return self.robot.home()

    def _format_step(self, step):
        if step["skill"] == "pick":
            return f"pick({self.scene['objects'][step['object']].get('label_vi', step['object'])})"
        if step["skill"] == "place":
            label = self.scene["objects"][step["object"]].get("label_vi", step["object"])
            zone = self.scene["zones"][step["zone"]].get("label_vi", step["zone"])
            return f"place({label}, {zone})"
        return "home()"

    @staticmethod
    def _vietnamese_status(status):
        return {
            RobotSkills.SUCCESS: "THÀNH CÔNG",
            RobotSkills.INVALID_OBJECT: "ĐỐI TƯỢNG KHÔNG HỢP LỆ",
            RobotSkills.INVALID_REGION: "VÙNG KHÔNG HỢP LỆ",
            RobotSkills.PLANNING_FAILED: "LẬP KẾ HOẠCH THẤT BẠI",
            RobotSkills.EXECUTION_FAILED: "THỰC THI THẤT BẠI",
        }.get(status, "THẤT BẠI")

    def _publish_result(self, task_id, success, reason):
        message = String()
        message.data = json.dumps(
            {"task_id": task_id, "success": success, "reason": reason}, ensure_ascii=False
        )
        self.result_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SkillExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
