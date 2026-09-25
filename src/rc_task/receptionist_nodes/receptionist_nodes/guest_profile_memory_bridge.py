#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robot_ai_interfaces.msg import PersonProfile
from robot_ai_interfaces.srv import SetCurrentGuestProfile


class GuestProfileMemoryBridge(Node):
    def __init__(self):
        super().__init__("guest_profile_memory_bridge")

        self.declare_parameter("profile_topic", "/dialogue/current_guest_profile")
        self.declare_parameter(
            "set_current_guest_service", "/guest_memory/set_current_guest_profile"
        )

        profile_topic = str(self.get_parameter("profile_topic").value)
        service_name = str(self.get_parameter("set_current_guest_service").value)

        self.memory_client = self.create_client(SetCurrentGuestProfile, service_name)
        self.create_subscription(PersonProfile, profile_topic, self._on_profile, 10)

        self.get_logger().info(
            f"Guest profile bridge ready: {profile_topic} -> {service_name}"
        )

    def _on_profile(self, msg: PersonProfile):
        if not self.memory_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                "Current-guest memory service is unavailable; dropping received profile."
            )
            return

        request = SetCurrentGuestProfile.Request()
        request.profile = msg

        future = self.memory_client.call_async(request)
        future.add_done_callback(
            lambda done_future, profile_id=msg.profile_id, guest_name=msg.name: self._on_store_done(
                done_future,
                profile_id,
                guest_name,
            )
        )

    def _on_store_done(self, future, profile_id: str, guest_name: str):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to store current guest profile: {exc}")
            return

        if response.success:
            self.get_logger().info(
                "Stored current guest profile from topic "
                f"for {guest_name or 'unknown guest'} as {profile_id or 'current_guest'}."
            )
        else:
            self.get_logger().warn(f"Memory node rejected current guest profile: {response.message}")


def main(args=None):
    rclpy.init(args=args)
    node = GuestProfileMemoryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
