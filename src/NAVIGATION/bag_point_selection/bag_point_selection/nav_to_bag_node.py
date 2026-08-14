#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String


class NavToBagNode(Node):
    def __init__(self):
        super().__init__("nav_to_bag_node")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("navigation_enabled_default", True)
        self.publisher = self.create_publisher(String, "/bag_nav_status", 10)
        self.client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_to_pose_action").value),
        )
        self.current_goal_handle = None
        self.latest_goal: Optional[PoseStamped] = None
        self.navigation_enabled = bool(self.get_parameter("navigation_enabled_default").value)
        self.create_subscription(PoseStamped, "/bag_front_goal", self.goal_callback, 10)
        self.create_subscription(Bool, "/bag_selection/navigation_enabled", self.navigation_enabled_callback, 10)
        self.create_subscription(Bool, "/bag_selection/reset", self.reset_callback, 10)
        self.publish_status("idle")

    def publish_status(self, status: str) -> None:
        self.publisher.publish(String(data=status))

    def goal_callback(self, msg: PoseStamped) -> None:
        self.latest_goal = msg
        if not self.navigation_enabled:
            self.publish_status("idle")
            return
        if self.current_goal_handle is not None:
            self.get_logger().info("Ignoring new goal while navigation is already active.")
            return
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.publish_status("failed")
            self.get_logger().error("NavigateToPose action server is unavailable.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.publish_status("navigating")
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.publish_status("failed")
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future) -> None:
        self.current_goal_handle = None
        result = future.result()
        status = "succeeded" if result.status == 4 else "failed"
        self.publish_status(status)

    def navigation_enabled_callback(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if self.navigation_enabled == enabled:
            return
        self.navigation_enabled = enabled
        if not enabled:
            self.cancel_active_goal()
            self.publish_status("idle")

    def reset_callback(self, msg: Bool) -> None:
        if bool(msg.data):
            self.latest_goal = None
            self.cancel_active_goal()
            self.publish_status("idle")

    def cancel_active_goal(self) -> None:
        if self.current_goal_handle is None:
            return
        future = self.current_goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future) -> None:
        del future
        self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = NavToBagNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
