#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import PointStamped, Pose, PoseArray, Vector3
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401
from visualization_msgs.msg import Marker, MarkerArray


class LidarClusterNode(Node):
    def __init__(self):
        super().__init__("lidar_cluster_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_cluster_dist_thresh", 0.08)
        self.declare_parameter("front_sector_half_angle_deg", 60.0)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("lidar_cluster_markers_enabled", True)
        self.declare_parameter("lidar_cluster_markers_topic", "/lidar_clusters_markers")
        self.declare_parameter("lidar_cluster_marker_scale_m", 0.10)
        self.declare_parameter("lidar_cluster_text_scale_m", 0.10)
        self.declare_parameter("lidar_cluster_marker_alpha", 0.9)
        self.declare_parameter("lidar_cluster_marker_lifetime_sec", 0.0)

        self.publisher = self.create_publisher(PoseArray, "/lidar_cluster_centers", 10)
        self.markers_enabled = bool(self.get_parameter("lidar_cluster_markers_enabled").value)
        self.marker_pub = None
        self.prev_marker_count = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.markers_enabled:
            self.marker_pub = self.create_publisher(
                MarkerArray, str(self.get_parameter("lidar_cluster_markers_topic").value), 10
            )

        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value, self.scan_callback, 10)

    def _marker_lifetime(self) -> MsgDuration:
        lifetime_sec = max(0.0, float(self.get_parameter("lidar_cluster_marker_lifetime_sec").value))
        whole = int(lifetime_sec)
        frac = int((lifetime_sec - whole) * 1e9)
        return MsgDuration(sec=whole, nanosec=frac)

    def _transform_point(self, point: PointStamped, target_frame: str) -> Optional[PointStamped]:
        if point.header.frame_id == target_frame:
            return point
        try:
            return self.tf_buffer.transform(point, target_frame, timeout=Duration(seconds=0.15))
        except TransformException as exc:
            self.get_logger().warn(
                f"Skipping LiDAR cluster transform from {point.header.frame_id} to {target_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

    def _publish_markers(self, header, clusters) -> None:
        if not self.markers_enabled or self.marker_pub is None:
            return

        marker_scale = max(0.01, float(self.get_parameter("lidar_cluster_marker_scale_m").value))
        text_scale = max(0.01, float(self.get_parameter("lidar_cluster_text_scale_m").value))
        alpha = min(1.0, max(0.0, float(self.get_parameter("lidar_cluster_marker_alpha").value)))
        lifetime = self._marker_lifetime()
        marker_array = MarkerArray()

        for idx, cluster in enumerate(clusters):
            sphere = Marker()
            sphere.header = header
            sphere.ns = "lidar_clusters_spheres"
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(cluster["x"])
            sphere.pose.position.y = float(cluster["y"])
            sphere.pose.position.z = 0.0
            sphere.pose.orientation.w = 1.0
            sphere.scale = Vector3(x=marker_scale, y=marker_scale, z=marker_scale)
            sphere.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=alpha)
            sphere.lifetime = lifetime
            marker_array.markers.append(sphere)

            text = Marker()
            text.header = header
            text.ns = "lidar_clusters_text"
            text.id = 1000 + idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(cluster["x"])
            text.pose.position.y = float(cluster["y"])
            text.pose.position.z = marker_scale * 1.5
            text.pose.orientation.w = 1.0
            text.scale.z = text_scale
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=alpha)
            text.text = f"#{idx} ({int(cluster['size'])})"
            text.lifetime = lifetime
            marker_array.markers.append(text)

        for stale_idx in range(len(clusters), self.prev_marker_count):
            stale_sphere = Marker()
            stale_sphere.header = header
            stale_sphere.ns = "lidar_clusters_spheres"
            stale_sphere.id = stale_idx
            stale_sphere.action = Marker.DELETE
            marker_array.markers.append(stale_sphere)

            stale_text = Marker()
            stale_text.header = header
            stale_text.ns = "lidar_clusters_text"
            stale_text.id = 1000 + stale_idx
            stale_text.action = Marker.DELETE
            marker_array.markers.append(stale_text)

        self.prev_marker_count = len(clusters)
        self.marker_pub.publish(marker_array)

    def scan_callback(self, msg: LaserScan) -> None:
        half_angle = math.radians(float(self.get_parameter("front_sector_half_angle_deg").value))
        points = []
        angle = msg.angle_min
        for rng in msg.ranges:
            if math.isfinite(rng) and msg.range_min < rng < msg.range_max and abs(angle) <= half_angle:
                points.append((rng * math.cos(angle), rng * math.sin(angle)))
            angle += msg.angle_increment

        raw_clusters = []
        current = []
        threshold = float(self.get_parameter("lidar_cluster_dist_thresh").value)
        for point in points:
            if not current:
                current = [point]
                continue
            prev = current[-1]
            if math.hypot(point[0] - prev[0], point[1] - prev[1]) <= threshold:
                current.append(point)
            else:
                raw_clusters.append(current)
                current = [point]
        if current:
            raw_clusters.append(current)

        min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        target_frame = str(self.get_parameter("base_frame").value)
        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = target_frame
        marker_clusters = []

        for cluster in raw_clusters:
            if len(cluster) < min_cluster_points:
                continue
            mean_x = sum(point[0] for point in cluster) / len(cluster)
            mean_y = sum(point[1] for point in cluster) / len(cluster)
            point = PointStamped()
            point.header = msg.header
            point.point.x = float(mean_x)
            point.point.y = float(mean_y)
            point.point.z = 0.0
            transformed = self._transform_point(point, target_frame)
            if transformed is None:
                continue

            pose = Pose()
            pose.position.x = float(transformed.point.x)
            pose.position.y = float(transformed.point.y)
            pose.position.z = float(len(cluster))
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            marker_clusters.append(
                {
                    "x": float(transformed.point.x),
                    "y": float(transformed.point.y),
                    "size": len(cluster),
                }
            )

        self.publisher.publish(pose_array)
        self._publish_markers(pose_array.header, marker_clusters)


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
