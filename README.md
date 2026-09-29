Terminal 1 — ROS 2 và mô phỏng
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py ur_type:=ur3e gazebo_gui:=false
Terminal 2 — cửa sổ RViz
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d /opt/ros/humble/share/
