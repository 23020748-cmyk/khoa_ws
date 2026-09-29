# ur3_llm_control

Gói ROS 2 cho UR3/UR3e: nhận yêu cầu tiếng Việt, gọi LLM cục bộ để lập kế hoạch skill, xác thực danh sách cho phép, rồi dùng MoveIt 2 để lập kế hoạch và thực thi chuyển động.

## MSSV và ánh xạ nhiệm vụ

Sinh viên: **Lục Văn Khoa**, MSSV **23020748**.

Hai chữ số cuối là **48**, do đó **P = 48 mod 6 = 0**. Nhiệm vụ sắp xếp tất cả vật dùng ánh xạ:

- Vùng A → khối đỏ
- Vùng B → khối vàng
- Vùng C → khối xanh lam

Ánh xạ được lưu trong student_config.yaml; planner tính P từ hai chữ số cuối MSSV khi tạo prompt.

## Kiến trúc

/task_command → Ollama LLM → JSON plan → TaskValidator → /validated_plan → xác thực lại tại skill_executor → MoveGroup action → ExecuteTrajectory action → UR3/UR3e.

Whitelist gồm pick(object), place(object, zone), home(). Mỗi lần gắp phải được đặt trước lần gắp kế tiếp; home() là bước cuối. Object và zone phải trùng ID trong scene.yaml. Kế hoạch sai schema, kỹ năng, object, zone hoặc thứ tự bị từ chối trước khi robot chạy.

robot_skills.py đưa table và các khối vào MoveIt PlanningScene. Mỗi chuyển động được lập kế hoạch rồi thực thi qua MoveIt; các giới hạn khớp do cấu hình UR/MoveIt áp dụng, còn velocity/acceleration được scale xuống 5%. Nếu MoveIt không tìm được kế hoạch, thực thi lỗi, hoặc gripper không sẵn sàng thì skill trả trạng thái thất bại và chuỗi dừng tại đó.

## Cài đặt và khởi chạy mô phỏng

ROS 2 Humble và các gói UR/MoveIt 2 cần thiết phải được cài đặt. Ollama phải chạy trong cùng môi trường mạng với ROS:

~~~bash
ollama pull qwen2.5:0.5b
ollama serve
~~~

Trong terminal ROS:

~~~bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ur3_llm_control
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py ur_type:=ur3e
~~~

Gửi lệnh ở terminal khác:

~~~bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /task_command std_msgs/msg/String \
  "{data: 'Hãy lấy khối màu đỏ và đặt vào ô B'}"
~~~

Các cách diễn đạt tự nhiên khác:

~~~bash
ros2 topic pub --once /task_command std_msgs/msg/String \
  "{data: 'Di chuyển vật màu đỏ sang vùng B'}"
ros2 topic pub --once /task_command std_msgs/msg/String \
  "{data: 'Sắp xếp tất cả đồ vật theo mã số sinh viên của tôi'}"
~~~

LLM phải lập kế hoạch theo yêu cầu nội dung. Ví dụ một lệnh đặt khối đỏ vào B tạo kế hoạch dạng:

~~~json
{
  "plan": [
    {"skill": "pick", "object": "red_cube"},
    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
    {"skill": "home"}
  ]
}
~~~

Terminal hiển thị lệnh, kế hoạch LLM, trạng thái từng skill (THÀNH CÔNG, LẬP KẾ HOẠCH THẤT BẠI, ĐỐI TƯỢNG KHÔNG HỢP LỆ, ...) và kết quả nhiệm vụ.

## Cấu hình scene và gripper

scene.yaml khai báo frame, planning group, end-effector, vị trí/kích thước table, vật, vùng, tư thế home và giới hạn tốc độ. Các vị trí mẫu phải được đo/căn chỉnh lại theo bàn và gốc base_link của robot thực tế. Đây là scene cấu hình tĩnh; gói chưa có camera/perception để tự cập nhật vị trí đồ vật. Các khối trong scene.yaml chỉ là collision objects của MoveIt, không tự được spawn thành mô hình vật lý trong Gazebo.

Mô phỏng UR trong workspace chỉ có controller cánh tay, chưa có gripper hay mô hình khối trong Gazebo. Launch mô phỏng bật `simulation_scene_only_gripper` để chạy lập kế hoạch và di chuyển cánh tay, đồng thời cập nhật attach/detach trong MoveIt PlanningScene. Lệnh kẹp là no-op; không có grasp vật lý và khối không hiển thị/di chuyển trong Gazebo.

Khi chạy robot thật, executor mặc định tắt chế độ scene-only. Cần gripper có action server `control_msgs/FollowJointTrajectory`; cấu hình đúng action, joint names, vị trí mở/đóng và touch links rồi đặt `gripper.enabled: true`.

MoveIt kiểm tra self-collision và va chạm với các đối tượng hiện diện trong PlanningScene. Cần khai báo đúng bàn/vật và tích hợp cảm biến nếu môi trường có chướng ngại thay đổi; scene tĩnh không thể phát hiện vật cản chưa được khai báo.

## Thay model / giao tiếp

Planner gọi Ollama /api/chat, mặc định qwen2.5:0.5b; có thể đổi URL/model khi launch:

~~~bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  ollama_url:=http://localhost:11434/api/chat model:=qwen2.5:0.5b
~~~

LLM chỉ sinh skill-plan JSON, không sinh góc khớp hoặc lệnh điều khiển mức thấp. Muốn thêm skill phải cập nhật đồng thời whitelist validator và executor/backend, rồi mới đưa skill đó vào prompt.
