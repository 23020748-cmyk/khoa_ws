# khoa_ws# robot_khoa

Package ROS 2 Humble mo phong robot 4 banh mecanum co tay may 2 khop, camera RGB, lidar 2D va GPS. Package duoc dong goi de nop bai giua ky duoi dang mot package doc lap, co the clone ve va build rieng.

## 1. Cau truc package

- `urdf/robot_khoa.urdf`: mo ta co khi, link/joint, frame `base_footprint/base_link` va plugin Gazebo.
- `meshes/`: cac file `.STL` xuat tu SolidWorks.
- `launch/sim.launch.py`: launch chinh, mo Gazebo, spawn robot, bat `robot_state_publisher`, `arm_commander.py` va RViz.
- `launch/display.launch.py`: xem mo hinh trong RViz ma khong can Gazebo.
- `scripts/arm_commander.py`: node nhan lenh `/arm_cmd`, goi `/apply_joint_effort` cho 2 khop tay may va phat `joint_states`.
- `scripts/mecanum_wheel_controller.py`: node quy doi dong hoc mecanum tu `Twist` sang toc do banh de mo rong them neu can.
- `rviz/robot_khoa.rviz`: cau hinh RViz co san `RobotModel`, `TF`, `LaserScan`, `Image`.

## 2. Moi truong su dung

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

## 3. Phu thuoc can co

Can co cac package ROS 2 sau:

- `gazebo_ros`
- `gazebo_plugins`
- `gazebo_planar_move_plugin`
- `gazebo_msgs`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `teleop_twist_keyboard`

Neu chua co thi cai bang `apt` tren ROS 2 Humble.

## 4. Build package

```bash
cd ~/khoa_ws
colcon build --packages-select robot_khoa
source install/setup.bash
```

## 5. Chay mo phong chinh

```bash
ros2 launch robot_khoa sim.launch.py
```

Khi chay thanh cong:

- Gazebo mo len va robot duoc spawn vao world.
- RViz2 mo len va robot xuat hien trong `RobotModel`.
- Robot nhan `geometry_msgs/msg/Twist` tren topic `/cmd_vel`.
- Plugin Gazebo phat `odom` va TF `odom -> base_link`.
- Topic cam bien xuat hien:
  - `/scan`
  - `/camera/image_raw`
  - `/camera/camera_info`
  - `/gps/data`
  - `/joint_states`

Neu muon chay khong mo RViz:

```bash
ros2 launch robot_khoa sim.launch.py rviz:=false
```

## 6. Dieu khien robot bang ban phim

Mo terminal thu 2:

```bash
source ~/khoa_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

Robot nhan `Twist` va di chuyen holonomic trong Gazebo:

- `i`, `,`: tien/lui
- `j`, `l`: quay trai/phai
- `u`, `o`, `m`, `.`: di cheo
- `J`, `L`: truot ngang

## 7. Dieu khien tay may

Node `arm_commander.py` duoc bat san trong launch. Gui lenh tren `/arm_cmd`:

```bash
source ~/khoa_ws/install/setup.bash
ros2 topic pub --once /arm_cmd std_msgs/msg/String "{data: joint1+}"
ros2 topic pub --once /arm_cmd std_msgs/msg/String "{data: joint1-}"
ros2 topic pub --once /arm_cmd std_msgs/msg/String "{data: joint2+}"
ros2 topic pub --once /arm_cmd std_msgs/msg/String "{data: joint2-}"
ros2 topic pub --once /arm_cmd std_msgs/msg/String "{data: home}"
```

## 8. Noi dung da dap ung cho bai nop

- Co package robot doc lap, khong phu thuoc vao viec nop ca `src/`.
- Co `urdf/`, `meshes/`, `launch/`, `rviz/`, `scripts/`, `config/`, `docs/`.
- Co file bao cao trong `docs/`.
- Co huong dan build, launch va cac package can cai them.
- Launch chinh bat Gazebo va RViz, robot xuat hien trong ca hai moi truong.
- Robot nhan `Twist` tu ban phim va co node dieu khien tay may.
- Cam bien duoc add san vao file RViz.
