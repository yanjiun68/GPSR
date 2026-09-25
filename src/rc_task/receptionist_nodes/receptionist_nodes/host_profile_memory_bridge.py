#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_ai_interfaces.msg import PersonProfile
from robot_ai_interfaces.srv import SetHostProfile


class HostProfileMemoryBridge(Node):
    def __init__(self):
        super().__init__("host_profile_memory_bridge")

        self.declare_parameter("profile_topic", "/dialogue/current_host_profile")
        self.declare_parameter("set_host_service", "/guest_memory/set_host_profile")

        profile_topic = str(self.get_parameter("profile_topic").value)
        service_name = str(self.get_parameter("set_host_service").value)

        self.memory_client = self.create_client(SetHostProfile, service_name)
        self.create_subscription(PersonProfile, profile_topic, self._on_profile, 10)

        self.get_logger().info(
            f"Host profile bridge ready: {profile_topic} -> {service_name}"
        )

    def _on_profile(self, msg: PersonProfile):
        if not self.memory_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                "Host memory service is unavailable; dropping received host profile."
            )
            return

        request = SetHostProfile.Request()
        request.profile = msg

        future = self.memory_client.call_async(request)
        future.add_done_callback(
            lambda done_future, guest_name=msg.name: self._on_store_done(done_future, guest_name)
        )

    def _on_store_done(self, future, guest_name: str):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to store host profile: {exc}")
            return

        if response.success:
            self.get_logger().info(
                f"Stored host profile from topic for {guest_name or 'unknown host'}."
            )
        else:
            self.get_logger().warn(f"Memory node rejected host profile: {response.message}")


def main(args=None):
    rclpy.init(args=args)
    node = HostProfileMemoryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
