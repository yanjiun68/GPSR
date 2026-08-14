#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as DurationMsg  # ✅ correct message type

import tf2_ros


class LidarPeopleCandidates(Node):
    """
    LiDAR People Candidate Detector (ROS2 Humble)

    Input:
      - /scan (sensor_msgs/LaserScan)

    Output:
      - /people_candidates (geometry_msgs/PoseArray) in base_frame
      - /people_candidates_markers (visualization_msgs/MarkerArray) in base_frame

    Method:
      1) Convert scan to XY points
      2) Transform points -> base_frame using TF
      3) ROI filtering in base_frame
      4) Fast sequential clustering (based on neighbor distance in scan order)
      5) Cluster filtering (point count + physical size)
      6) Publish centroids
    """

    def __init__(self):
        super().__init__('lidar_people_candidates')

        def _param_bool(v) -> bool:
            if isinstance(v, bool):
                return v
            if isinstance(v, (int, float)):
                return bool(v)
            if isinstance(v, str):
                s = v.strip().lower()
                if s in ('1', 'true', 't', 'yes', 'y', 'on'):
                    return True
                if s in ('0', 'false', 'f', 'no', 'n', 'off'):
                    return False
            return bool(v)

        # ---------------- Parameters ----------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_footprint')   # or base_link

        # ROI in base_frame
        self.declare_parameter('min_range_m', 0.2)
        self.declare_parameter('max_range_m', 6.0)
        self.declare_parameter('min_x_m', 0.0)      # only in front of robot
        self.declare_parameter('max_x_m', 6.0)
        self.declare_parameter('min_y_m', -3.0)
        self.declare_parameter('max_y_m',  3.0)

        # Clustering
        self.declare_parameter('cluster_dist_m', 0.18)   # neighbor distance threshold
        self.declare_parameter('min_cluster_points', 4)
        self.declare_parameter('max_cluster_points', 80)

        # Cluster physical size (bounding box in base frame)
        self.declare_parameter('min_cluster_size_m', 0.05)
        self.declare_parameter('max_cluster_size_m', 0.60)

        # TF settings
        self.declare_parameter('tf_timeout_sec', 0.10)

        # RViz marker settings
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_scale_m', 0.20)
        self.declare_parameter('marker_lifetime_sec', 0.30)

        # ---------------- Read params ----------------
        self.scan_topic = self.get_parameter('scan_topic').value
        self.base_frame = self.get_parameter('base_frame').value

        self.min_range = float(self.get_parameter('min_range_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)

        self.min_x = float(self.get_parameter('min_x_m').value)
        self.max_x = float(self.get_parameter('max_x_m').value)
        self.min_y = float(self.get_parameter('min_y_m').value)
        self.max_y = float(self.get_parameter('max_y_m').value)

        self.cluster_dist = float(self.get_parameter('cluster_dist_m').value)
        self.min_pts = int(self.get_parameter('min_cluster_points').value)
        self.max_pts = int(self.get_parameter('max_cluster_points').value)

        self.min_size = float(self.get_parameter('min_cluster_size_m').value)
        self.max_size = float(self.get_parameter('max_cluster_size_m').value)

        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)

        self.publish_markers = _param_bool(self.get_parameter('publish_markers').value)
        self.marker_scale = float(self.get_parameter('marker_scale_m').value)
        self.marker_lifetime = float(self.get_parameter('marker_lifetime_sec').value)

        # ---------------- QoS ----------------
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        # ---------------- TF ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- Pub/Sub ----------------
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_sensor)

        self.pub_candidates = self.create_publisher(PoseArray, '/people_candidates', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/people_candidates_markers', 10)

        self.get_logger().info('[LidarPeopleCandidates] Started')
        self.get_logger().info(f'  scan_topic : {self.scan_topic}')
        self.get_logger().info(f'  base_frame : {self.base_frame}')

    # ==========================================================
    # Main callback
    # ==========================================================
    def scan_cb(self, scan: LaserScan):
        # 1) Build points in scan frame (preserve scan order)
        pts_scan = []
        angle = scan.angle_min

        for r in scan.ranges:
            if math.isfinite(r) and self.min_range <= r <= self.max_range:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                pts_scan.append((x, y))
            else:
                pts_scan.append(None)
            angle += scan.angle_increment

        # 2) TF: scan frame -> base_frame at scan time (or fallback latest)
        stamp = Time.from_msg(scan.header.stamp)
        tf = self.lookup_tf(self.base_frame, scan.header.frame_id, stamp)
        if tf is None:
            return

        R, t = self.tf_to_R_t(tf)

        # 3) Transform to base_frame + ROI filtering (keep scan order)
        pts_base = []
        for p in pts_scan:
            if p is None:
                pts_base.append(None)
                continue
            xb, yb = self.transform_2d(R, t, p[0], p[1])
            if (self.min_x <= xb <= self.max_x) and (self.min_y <= yb <= self.max_y):
                pts_base.append((xb, yb))
            else:
                pts_base.append(None)

        # 4) Cluster in scan order (fast)
        clusters = self.sequential_clusters(pts_base, self.cluster_dist)

        # 5) Filter clusters + compute centroids
        centroids = []
        for c in clusters:
            if not (self.min_pts <= len(c) <= self.max_pts):
                continue

            xs = np.array([p[0] for p in c], dtype=np.float64)
            ys = np.array([p[1] for p in c], dtype=np.float64)

            # bounding box size
            size_x = float(xs.max() - xs.min())
            size_y = float(ys.max() - ys.min())
            size = max(size_x, size_y)

            if size < self.min_size or size > self.max_size:
                continue

            cx = float(xs.mean())
            cy = float(ys.mean())
            centroids.append((cx, cy))

        # 6) Publish PoseArray
        pa = PoseArray()
        pa.header.frame_id = self.base_frame
        pa.header.stamp = scan.header.stamp

        for (cx, cy) in centroids:
            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pa.poses.append(pose)

        self.pub_candidates.publish(pa)

        # 7) Publish RViz markers
        if self.publish_markers:
            self.pub_markers.publish(self.make_markers(centroids, scan.header.stamp))

    # ==========================================================
    # Clustering (scan-order)
    # ==========================================================
    @staticmethod
    def sequential_clusters(pts_base, neighbor_dist):
        clusters = []
        current = []
        prev = None

        for p in pts_base:
            if p is None:
                if current:
                    clusters.append(current)
                    current = []
                prev = None
                continue

            if prev is None:
                current = [p]
            else:
                d = math.hypot(p[0] - prev[0], p[1] - prev[1])
                if d <= neighbor_dist:
                    current.append(p)
                else:
                    clusters.append(current)
                    current = [p]

            prev = p

        if current:
            clusters.append(current)

        return clusters

    # ==========================================================
    # TF helpers
    # ==========================================================
    def lookup_tf(self, target_frame, source_frame, stamp: Time):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout)
            )
        except Exception:
            # fallback to latest
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout)
                )
            except Exception as e2:
                self.get_logger().warn(f'[LidarPeopleCandidates] TF lookup failed: {e2}')
                return None

    @staticmethod
    def tf_to_R_t(tf):
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z

        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        # Full 3D rotation, we use XY only
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        R = np.array([
            [1.0 - 2.0 * (yy + zz),     2.0 * (xy - wz),         2.0 * (xz + wy)],
            [2.0 * (xy + wz),           1.0 - 2.0 * (xx + zz),   2.0 * (yz - wx)],
            [2.0 * (xz - wy),           2.0 * (yz + wx),         1.0 - 2.0 * (xx + yy)]
        ], dtype=np.float64)

        t = np.array([tx, ty, tz], dtype=np.float64)
        return R, t

    @staticmethod
    def transform_2d(R, t, x, y):
        p = np.array([x, y, 0.0], dtype=np.float64)
        pb = R @ p + t
        return float(pb[0]), float(pb[1])

    # ==========================================================
    # RViz markers
    # ==========================================================
    def make_markers(self, centroids, stamp_msg):
        ma = MarkerArray()

        # ✅ Correct type for Marker.lifetime
        lifetime_msg = DurationMsg()
        lifetime_msg.sec = int(self.marker_lifetime)
        lifetime_msg.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)

        for i, (x, y) in enumerate(centroids):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = stamp_msg
            m.ns = "people_candidates"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            s = self.marker_scale
            m.scale.x = s
            m.scale.y = s
            m.scale.z = s

            # Blue-ish
            m.color.r = 0.0
            m.color.g = 0.3
            m.color.b = 1.0
            m.color.a = 1.0

            m.lifetime = lifetime_msg
            ma.markers.append(m)

        return ma


def main(args=None):
    rclpy.init(args=args)
    node = LidarPeopleCandidates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
