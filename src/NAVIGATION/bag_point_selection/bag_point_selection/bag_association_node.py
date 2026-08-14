#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401

from putrabot_interfaces.msg import LidarClusters


class BagAssociationNode(Node):
    def __init__(self):
        super().__init__("bag_association_node")
        self.declare_parameter("enable_lidar_association", True)
        self.declare_parameter("cluster_match_max_dist_m", 0.25)
        self.declare_parameter("max_lidar_age_sec", 0.75)
        self.declare_parameter("max_cross_sensor_skew_sec", 0.35)
        self.declare_parameter("base_frame", "base_link")
        self.latest_point: Optional[PointStamped] = None
        self.latest_clusters: Optional[LidarClusters] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PointStamped, "/selected_bag_point_base", self.point_callback, 10)
        self.create_subscription(LidarClusters, "/lidar_clusters", self.cluster_callback, 10)
        self.publisher = self.create_publisher(PointStamped, "/selected_bag_cluster", 10)

    def point_callback(self, msg: PointStamped) -> None:
        self.latest_point = msg
        self.try_publish()

    def cluster_callback(self, msg: LidarClusters) -> None:
        self.latest_clusters = msg
        self.try_publish()

    def _stamp_age_sec(self, stamp) -> Optional[float]:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        msg_time = rclpy.time.Time.from_msg(stamp)
        return max(0.0, (self.get_clock().now() - msg_time).nanoseconds / 1e9)

    def _skew_sec(self, a_stamp, b_stamp) -> Optional[float]:
        if (a_stamp.sec == 0 and a_stamp.nanosec == 0) or (b_stamp.sec == 0 and b_stamp.nanosec == 0):
            return None
        a_time = rclpy.time.Time.from_msg(a_stamp)
        b_time = rclpy.time.Time.from_msg(b_stamp)
        return abs((a_time - b_time).nanoseconds) / 1e9

    def _transform_point(self, point: PointStamped, target_frame: str) -> Optional[PointStamped]:
        source_frame = point.header.frame_id
        if source_frame == target_frame:
            return point
        if not source_frame:
            self.get_logger().warn(
                "Skipping LiDAR association because a point has an empty frame_id.",
                throttle_duration_sec=2.0,
            )
            return None
        try:
            return self.tf_buffer.transform(point, target_frame, timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().warn(
                f"Skipping LiDAR association: cannot transform {source_frame} to {target_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

    def _cluster_as_point(self, cluster) -> PointStamped:
        point = PointStamped()
        point.header = self.latest_clusters.header
        point.point.x = float(cluster.point.x)
        point.point.y = float(cluster.point.y)
        point.point.z = float(cluster.point.z)
        return point

    def try_publish(self) -> None:
        if not bool(self.get_parameter("enable_lidar_association").value):
            return
        if self.latest_point is None or self.latest_clusters is None or not self.latest_clusters.clusters:
            return

        point_age = self._stamp_age_sec(self.latest_point.header.stamp)
        if point_age is not None and point_age > float(self.get_parameter("max_lidar_age_sec").value):
            self.get_logger().warn("Skipping LiDAR association due to stale bag point.", throttle_duration_sec=2.0)
            return
        clusters_age = self._stamp_age_sec(self.latest_clusters.header.stamp)
        if clusters_age is not None and clusters_age > float(self.get_parameter("max_lidar_age_sec").value):
            self.get_logger().warn("Skipping LiDAR association due to stale cluster message.", throttle_duration_sec=2.0)
            return
        skew = self._skew_sec(self.latest_point.header.stamp, self.latest_clusters.header.stamp)
        if skew is not None and skew > float(self.get_parameter("max_cross_sensor_skew_sec").value):
            self.get_logger().warn("Skipping LiDAR association due to bag-point/cluster timestamp skew.", throttle_duration_sec=2.0)
            return

        target_frame = str(self.get_parameter("base_frame").value)
        selected_point = self._transform_point(self.latest_point, target_frame)
        if selected_point is None:
            return

        best = None
        best_point = None
        best_dist = None
        px = selected_point.point.x
        py = selected_point.point.y
        for cluster in self.latest_clusters.clusters:
            cluster_point = self._transform_point(self._cluster_as_point(cluster), target_frame)
            if cluster_point is None:
                return
            dist = math.hypot(cluster_point.point.x - px, cluster_point.point.y - py)
            if best is None or dist < best_dist:
                best = cluster
                best_point = cluster_point
                best_dist = dist

        if best is None or best_point is None or best_dist is None:
            return
        if best_dist > float(self.get_parameter("cluster_match_max_dist_m").value):
            self.get_logger().warn("No LiDAR cluster matched selected bag strongly enough.", throttle_duration_sec=2.0)
            return

        out = PointStamped()
        out.header = best_point.header
        out.point.x = float(best_point.point.x)
        out.point.y = float(best_point.point.y)
        out.point.z = float(best_point.point.z)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BagAssociationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
