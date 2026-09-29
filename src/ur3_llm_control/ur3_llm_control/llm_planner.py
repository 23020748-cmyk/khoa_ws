"""Ollama-backed natural-language planner. The LLM never emits joint commands."""

import json
import queue
import threading
import urllib.error
import urllib.request
import uuid

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .task_validator import PlanValidationError, TaskValidator


def read_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class LlmPlanner(Node):
    def __init__(self):
        super().__init__("llm_task_planner")
        share = get_package_share_directory("ur3_llm_control")
        self.declare_parameter("scene_config", share + "/config/scene.yaml")
        self.declare_parameter("student_config", share + "/config/student_config.yaml")
        self.declare_parameter("ollama_url", "http://localhost:11434/api/chat")
        self.declare_parameter("model", "qwen2.5:0.5b")
        self.declare_parameter("request_timeout_sec", 300.0)
        self.scene = read_yaml(self.get_parameter("scene_config").value)
        self.student = read_yaml(self.get_parameter("student_config").value)
        self.objects = self.scene["objects"]
        self.zones = self.scene["zones"]
        self.validator = TaskValidator(self.objects, self.zones)
        self._commands = queue.Queue()
        self._ready = threading.Event()
        self._condition = threading.Condition()
        self._results = {}
        self.create_subscription(String, "/task_command", self._on_command, 10)
        self.create_subscription(Bool, "/skill_executor/ready", self._on_ready, 10)
        self.create_subscription(String, "/task_result", self._on_result, 10)
        self.plan_pub = self.create_publisher(String, "/validated_plan", 10)
        threading.Thread(target=self._process_commands, daemon=True).start()
        self.get_logger().info("Listening on /task_command; planning with local Ollama.")

    def _on_command(self, message):
        command = message.data.strip()
        if command:
            self._commands.put(command)
        else:
            self.get_logger().warning("Ignoring an empty task command.")

    def _on_ready(self, message):
        if message.data:
            self._ready.set()

    def _on_result(self, message):
        try:
            result = json.loads(message.data)
            with self._condition:
                self._results[result["task_id"]] = result
                self._condition.notify_all()
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning("Ignoring malformed /task_result message.")

    def _process_commands(self):
        while rclpy.ok():
            try:
                command = self._commands.get(timeout=0.2)
            except queue.Empty:
                continue
            self._run_command(command)

    def _run_command(self, command):
        print("\nLỆNH NGƯỜI DÙNG:\n" + command, flush=True)
        try:
            plan = self.validator.validate(self._ask_model(command))
        except (PlanValidationError, ValueError, KeyError, OSError, TimeoutError) as error:
            print(f"\nKẾ HOẠCH BỊ TỪ CHỐI: {error}\nNHIỆM VỤ THẤT BẠI", flush=True)
            self.get_logger().error(f"Planner rejected task: {error}")
            return

        task_id = str(uuid.uuid4())
        print("\nKẾ HOẠCH LLM:", flush=True)
        for number, step in enumerate(plan, 1):
            print(f"{number}. {self._format_step(step)}", flush=True)
        self._ready.wait()
        payload = String()
        payload.data = json.dumps(
            {"task_id": task_id, "command": command, "plan": plan}, ensure_ascii=False
        )
        self.plan_pub.publish(payload)
        with self._condition:
            self._condition.wait_for(
                lambda: task_id in self._results or not rclpy.ok(), timeout=600.0
            )
            result = self._results.pop(task_id, None)
        if result is None:
            print("\nNHIỆM VỤ THẤT BẠI: hết thời gian chờ skill executor.", flush=True)
        else:
            print("\n" + ("NHIỆM VỤ THÀNH CÔNG" if result.get("success") else "NHIỆM VỤ THẤT BẠI"), flush=True)

    def _format_step(self, step):
        if step["skill"] == "pick":
            return f"pick({self.objects[step['object']].get('label_vi', step['object'])})"
        if step["skill"] == "place":
            label = self.objects[step["object"]].get("label_vi", step["object"])
            zone = self.zones[step["zone"]].get("label_vi", step["zone"])
            return f"place({label}, {zone})"
        return "home()"

    def _ask_model(self, command):
        student_id = str(self.student["student_id"])
        last_two = int(student_id[-2:])
        p_value = last_two % 6
        mapping = self.student["assignments"][str(p_value)]
        object_catalog = {
            key: item.get("label_vi", key) for key, item in self.objects.items()
        }
        zone_catalog = {
            key: item.get("label_vi", key) for key, item in self.zones.items()
        }
        system = (
            "Bạn lập kế hoạch ngôn ngữ tự nhiên cho robot UR3/UR3e. Chuyển yêu cầu thành JSON; "
            "không tạo lệnh khớp hay mã điều khiển robot. Chỉ dùng skill pick(object), "
            "place(object, zone), home(). Mỗi vật được gắp phải được đặt đúng một lần; home() "
            "là bước cuối. Dùng ID chính xác trong danh mục. Với yêu cầu sắp xếp tất cả vật theo "
            "MSSV, đặt từng vật vào vùng ánh xạ cá nhân. Mỗi yêu cầu đặt vật luôn phải sinh pick ngay trước place, dù câu lệnh chỉ nói đặt. Ví dụ yêu cầu Đặt khối lập phương màu đỏ vào khu vực B phải tạo pick red_cube, place red_cube zone_b, home. Nếu vật/vùng không có trong danh mục hoặc "
            "yêu cầu không rõ, trả plan rỗng để validator từ chối. Chỉ trả object JSON có dạng "
            '{"plan":[{"skill":"pick","object":"red_cube"},'
            '{"skill":"place","object":"red_cube","zone":"zone_a"},{"skill":"home"}]}.'
            f"\nDanh mục vật: {json.dumps(object_catalog, ensure_ascii=False)}"
            f"\nDanh mục vùng: {json.dumps(zone_catalog, ensure_ascii=False)}"
            f"\nSinh viên: {self.student['student_name']}, MSSV {student_id}; P={last_two} mod 6={p_value}."
            f"\nÁnh xạ vùng sang vật khi sắp xếp tất cả: {json.dumps(mapping, ensure_ascii=False)}"
        )
        body = json.dumps({
            "model": self.get_parameter("model").value,
            "stream": False,
            "format": "json",
            "options": {"num_predict": 256, "num_ctx": 2048, "temperature": 0.1},
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": command},
            ],
        }).encode("utf-8")
        request = urllib.request.Request(
            self.get_parameter("ollama_url").value,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request, timeout=float(self.get_parameter("request_timeout_sec").value)
            ) as response:
                reply = json.loads(response.read().decode("utf-8"))
        except urllib.error.URLError as error:
            raise OSError(f"Không kết nối được Ollama: {error}") from error
        content = reply.get("message", {}).get("content", "")
        try:
            return json.loads(content)
        except (TypeError, ValueError) as error:
            raise ValueError("LLM không trả về JSON hợp lệ.") from error


def main(args=None):
    rclpy.init(args=args)
    node = LlmPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
