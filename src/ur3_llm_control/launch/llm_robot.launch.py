"""Start the UR simulation, MoveIt 2, LLM planner and validated skill executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    ollama_url = LaunchConfiguration("ollama_url")
    model = LaunchConfiguration("model")
    scene = PathJoinSubstitution([
        FindPackageShare("ur3_llm_control"), "config", "scene.yaml"
    ])
    student = PathJoinSubstitution([
        FindPackageShare("ur3_llm_control"), "config", "student_config.yaml"
    ])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"
        ])),
        launch_arguments={
            "ur_type": ur_type,
            "runtime_config_package": "ur3_llm_control",
            "controllers_file": "ur_controllers.yaml",
            "launch_rviz": "false",
            "gazebo_gui": gazebo_gui,
            "start_joint_controller": "true",
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"
        ])),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": launch_rviz,
            "launch_servo": "false",
        }.items(),
    )
    planner = Node(
        package="ur3_llm_control",
        executable="llm_planner",
        name="llm_task_planner",
        parameters=[{
            "scene_config": scene,
            "student_config": student,
            "ollama_url": ollama_url,
            "model": model,
        }],
        output="screen",
    )
    executor = Node(
        package="ur3_llm_control",
        executable="skill_executor",
        name="skill_executor",
        parameters=[
            {
                "scene_config": scene,
                "student_config": student,
                "use_sim_time": True,
                "simulation_scene_only_gripper": LaunchConfiguration("simulation_scene_only_gripper"),
            }
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type", default_value="ur3e", choices=["ur3", "ur3e"],
            description="UR model to simulate",
        ),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument(
            "launch_rviz", default_value="false",
            description="Launch MoveIt RViz for a visible demo",
        ),
        DeclareLaunchArgument(
            "ollama_url", default_value="http://localhost:11434/api/chat",
            description="Ollama chat API URL reachable from the ROS environment",
        ),
        DeclareLaunchArgument("model", default_value="qwen2.5:0.5b"),
        DeclareLaunchArgument(
            "simulation_scene_only_gripper", default_value="true",
            description="Use MoveIt-only attachment in Gazebo; no physical gripper is simulated",
        ),
        simulation,
        moveit,
        planner,
        executor,
    ])
