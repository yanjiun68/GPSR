#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FrontPosePlannerNode(Node):
    def __init__(self):
        super().__init__("front_pose_planner_node")
        self.declare_parameter("enable_lidar_association", True)
        self.declare_parameter("front_offset_distance_m", 0.60)
        self.declare_parameter("goal_validation_clearance_m", 0.20)
        self.base_point: Optional[PointStamped] = None
        self.cluster_point: Optional[PointStamped] = None
        self.publisher = self.create_publisher(PoseStamped, "/bag_front_goal", 10)
        self.create_subscription(PointStamped, "/selected_bag_point_base", self.base_callback, 10)
        self.create_subscription(PointStamped, "/selected_bag_cluster", self.cluster_callback, 10)

    def base_callback(self, msg: PointStamped) -> None:
        self.base_point = msg
        if not bool(self.get_parameter("enable_lidar_association").value):
            self.plan(msg)

    def cluster_callback(self, msg: PointStamped) -> None:
        self.cluster_point = msg
        if bool(self.get_parameter("enable_lidar_association").value):
            self.plan(msg)

    def plan(self, target: PointStamped) -> None:
        x = target.point.x
        y = target.point.y
        distance = math.hypot(x, y)
        offset = float(self.get_parameter("front_offset_distance_m").value)
        clearance = float(self.get_parameter("goal_validation_clearance_m").value)
        if distance <= max(offset, clearance):
            self.get_logger().warn("Selected bag is too close for a valid stand-off pose.", throttle_duration_sec=2.0)
            return

        theta = math.atan2(y, x)
        goal_x = x - offset * math.cos(theta)
        goal_y = y - offset * math.sin(theta)
        if goal_x <= 0.0:
            self.get_logger().warn("Computed stand-off point would be behind or at the robot.", throttle_duration_sec=2.0)
            return
        if math.hypot(goal_x - x, goal_y - y) < clearance:
            self.get_logger().warn("Computed stand-off point violates clearance gate.", throttle_duration_sec=2.0)
            return

        pose = PoseStamped()
        pose.header = target.header
        pose.pose.position.x = float(goal_x)
        pose.pose.position.y = float(goal_y)
        pose.pose.orientation = yaw_to_quaternion(theta)
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = FrontPosePlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
