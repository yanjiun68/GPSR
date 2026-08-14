#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, String


class TargetFusionNode(Node):
    def __init__(self):
        super().__init__("target_fusion_node")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_confidence_high", 0.70)
        self.declare_parameter("camera_confidence_low", 0.35)
        self.declare_parameter("lidar_match_max_dist_m", 0.30)
        self.declare_parameter("lidar_assist_timeout_sec", 1.25)
        self.declare_parameter("target_stale_timeout_sec", 0.6)
        self.declare_parameter("max_lidar_age_sec", 0.75)
        self.declare_parameter("camera_fusion_weight_high", 0.85)
        self.declare_parameter("camera_fusion_weight_low", 0.65)

        self.latest_camera_point: Optional[PointStamped] = None
        self.latest_camera_confidence = 0.0
        self.latest_lidar_clusters: Optional[PoseArray] = None
        self.last_camera_seen_at: Optional[Time] = None
        self.last_fused_point: Optional[PointStamped] = None

        self.create_subscription(PointStamped, "/selected_bag_centroid_base", self.camera_callback, 10)
        self.create_subscription(Float32, "/selected_bag_cluster_confidence", self.camera_conf_callback, 10)
        self.create_subscription(PoseArray, "/lidar_cluster_centers", self.lidar_callback, 10)

        self.fused_pub = self.create_publisher(PointStamped, "/selected_bag_fused_center", 10)
        self.support_pub = self.create_publisher(PointStamped, "/selected_bag_support_cluster", 10)
        self.mode_pub = self.create_publisher(String, "/target_tracking_mode", 10)
        self.conf_pub = self.create_publisher(Float32, "/selected_bag_fused_confidence", 10)

    def camera_callback(self, msg: PointStamped) -> None:
        self.latest_camera_point = msg
        self.last_camera_seen_at = self.get_clock().now()

    def camera_conf_callback(self, msg: Float32) -> None:
        self.latest_camera_confidence = max(0.0, min(1.0, float(msg.data)))

    def lidar_callback(self, msg: PoseArray) -> None:
        self.latest_lidar_clusters = msg

    def _stamp_age_sec(self, stamp) -> Optional[float]:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        msg_time = Time.from_msg(stamp)
        return max(0.0, (self.get_clock().now() - msg_time).nanoseconds / 1e9)

    def _point_distance_xy(self, a: PointStamped, b: PointStamped) -> float:
        return math.hypot(a.point.x - b.point.x, a.point.y - b.point.y)

    def _make_point(self, x: float, y: float, z: float, stamp) -> PointStamped:
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = str(self.get_parameter("base_frame").value)
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        return msg

    def _weighted_average(self, camera_point: PointStamped, lidar_point: PointStamped, camera_weight: float) -> PointStamped:
        lidar_weight = max(0.0, 1.0 - camera_weight)
        total = max(1e-6, camera_weight + lidar_weight)
        return self._make_point(
            (camera_weight * camera_point.point.x + lidar_weight * lidar_point.point.x) / total,
            (camera_weight * camera_point.point.y + lidar_weight * lidar_point.point.y) / total,
            (camera_weight * camera_point.point.z + lidar_weight * lidar_point.point.z) / total,
            camera_point.header.stamp,
        )

    def _closest_lidar_cluster(self, reference: PointStamped) -> tuple[Optional[PointStamped], Optional[float]]:
        if self.latest_lidar_clusters is None or not self.latest_lidar_clusters.poses:
            return None, None
        age = self._stamp_age_sec(self.latest_lidar_clusters.header.stamp)
        if age is not None and age > float(self.get_parameter("max_lidar_age_sec").value):
            return None, None

        best = None
        best_dist = None
        max_dist = float(self.get_parameter("lidar_match_max_dist_m").value)
        for pose in self.latest_lidar_clusters.poses:
            # LiDAR clusters are floor-plane support points, so association
            # should gate on XY proximity even when the camera centroid is raised.
            cluster_point = self._make_point(
                pose.position.x,
                pose.position.y,
                0.0,
                self.latest_lidar_clusters.header.stamp,
            )
            dist = self._point_distance_xy(reference, cluster_point)
            if dist > max_dist:
                continue
            if best is None or dist < best_dist:
                best = cluster_point
                best_dist = dist
        return best, best_dist

    def _publish(self, mode: str, confidence: float, fused_point: Optional[PointStamped], support_point: Optional[PointStamped]) -> None:
        self.mode_pub.publish(String(data=mode))
        self.conf_pub.publish(Float32(data=float(max(0.0, min(1.0, confidence)))))
        if support_point is not None:
            self.support_pub.publish(support_point)
        if fused_point is not None:
            self.fused_pub.publish(fused_point)
            self.last_fused_point = fused_point

    def step(self) -> None:
        now = self.get_clock().now()
        stale_timeout = float(self.get_parameter("target_stale_timeout_sec").value)
        assist_timeout = float(self.get_parameter("lidar_assist_timeout_sec").value)
        camera_high = float(self.get_parameter("camera_confidence_high").value)
        camera_low = float(self.get_parameter("camera_confidence_low").value)

        camera_fresh = False
        if self.latest_camera_point is not None:
            age = self._stamp_age_sec(self.latest_camera_point.header.stamp)
            camera_fresh = age is None or age <= stale_timeout

        camera_point = self.latest_camera_point if camera_fresh else None
        match_reference = camera_point if camera_point is not None else self.last_fused_point
        support_point = None
        match_score = 0.0
        if match_reference is not None:
            support_point, dist = self._closest_lidar_cluster(match_reference)
            if support_point is not None and dist is not None:
                max_dist = max(1e-6, float(self.get_parameter("lidar_match_max_dist_m").value))
                match_score = max(0.0, min(1.0, 1.0 - dist / max_dist))

        # Camera owns identity. LiDAR only stabilizes the already-selected target
        # or briefly covers a short blind spot after the last camera lock.
        if camera_point is not None:
            camera_conf = self.latest_camera_confidence
            if support_point is not None:
                if camera_conf >= camera_high:
                    fused = self._weighted_average(
                        camera_point,
                        support_point,
                        float(self.get_parameter("camera_fusion_weight_high").value),
                    )
                    fused_conf = 0.75 * camera_conf + 0.25 * match_score
                    self._publish("camera_lidar_fused", fused_conf, fused, support_point)
                    return
                if camera_conf >= camera_low:
                    fused = self._weighted_average(
                        camera_point,
                        support_point,
                        float(self.get_parameter("camera_fusion_weight_low").value),
                    )
                    fused_conf = 0.55 * camera_conf + 0.45 * match_score
                    self._publish("camera_lidar_fused", fused_conf, fused, support_point)
                    return
                fused_conf = max(camera_conf, 0.30 + 0.40 * match_score)
                self._publish("lidar_assist", fused_conf, support_point, support_point)
                return

            self._publish("camera_only", camera_conf, camera_point, None)
            return

        if self.last_camera_seen_at is not None:
            blind_time = (now - self.last_camera_seen_at).nanoseconds / 1e9
            if blind_time <= assist_timeout and support_point is not None:
                fused_conf = 0.35 + 0.35 * match_score
                self._publish("lidar_assist", fused_conf, support_point, support_point)
                return

        self._publish("target_lost", 0.0, None, None)


def main(args=None):
    rclpy.init(args=args)
    node = TargetFusionNode()
    timer = node.create_timer(0.1, node.step)
    del timer
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
