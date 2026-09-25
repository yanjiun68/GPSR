from __future__ import annotations
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Tuple
import uuid

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .core import FixedInformationStore

class FixedInformationDialogueNode(Node):
    def __init__(self) -> None:
        super().__init__("fixed_information_dialogue")
        default_config = str(
            Path(get_package_share_directory("fixed_information_dialogue"))
            / "config" / "fixed_information.yaml"
        )
        self.declare_parameter("config_file", default_config)
        self.declare_parameter("query_topic", "/fixed_information/query")
        self.declare_parameter("answer_topic", "/fixed_information/answer")
        self.declare_parameter("answer_text_topic", "/fixed_information/answer_text")
        self.declare_parameter("ready_topic", "/fixed_information/ready")

        self.store = FixedInformationStore(self.get_parameter("config_file").value)
        self.answer_pub = self.create_publisher(
            String, self.get_parameter("answer_topic").value, 10
        )
        self.answer_text_pub = self.create_publisher(
            String, self.get_parameter("answer_text_topic").value, 10
        )
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_pub = self.create_publisher(
            Bool, self.get_parameter("ready_topic").value, qos
        )
        self.query_sub = self.create_subscription(
            String, self.get_parameter("query_topic").value, self._on_query, 10
        )
        self.reload_srv = self.create_service(
            Trigger, "/fixed_information/reload", self._on_reload
        )
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info("Fixed-information dialogue ready")

    @staticmethod
    def _parse_request(data: str) -> Tuple[str, str, Dict[str, Any]]:
        data = data.strip()
        request_id = str(uuid.uuid4())
        # Section: Decode JSON containers explicitly so arrays cannot masquerade as questions.
        if data.startswith(("{", "[")):
            try:
                payload = json.loads(data)
                # Section: Only object envelopes carry request IDs, text, and context fields.
                if not isinstance(payload, dict):
                    return request_id, "", {}
                context = payload.get("context") or {}
                return (
                    str(payload.get("request_id") or request_id),
                    str(payload.get("text", "")).strip(),
                    context if isinstance(context, dict) else {},
                )
            except json.JSONDecodeError:
                pass
        return request_id, data, {}

    def _on_query(self, msg: String) -> None:
        request_id, text, context = self._parse_request(msg.data)
        result = self.store.query(text)
        payload = {
            "request_id": request_id,
            "question_text": text,
            "status": result.status,
            "matched_key": result.key,
            "answer_text": result.answer_text,
            "source": result.source,
            "confidence": round(result.confidence, 4),
            "candidates": list(result.candidates),
            "metadata": result.metadata or {},
            "context": context,
            "answered_at": datetime.now().astimezone().isoformat(),
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.answer_pub.publish(out)

        if result.status == "FOUND" and result.answer_text:
            speech = String()
            speech.data = result.answer_text
            self.answer_text_pub.publish(speech)

    def _on_reload(self, request, response):
        del request
        try:
            self.store.reload()
            response.success = True
            response.message = "Fixed-information database reloaded."
            self.ready_pub.publish(Bool(data=True))
        except Exception as exc:
            response.success = False
            response.message = f"Reload failed: {exc}"
            self.ready_pub.publish(Bool(data=False))
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FixedInformationDialogueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
