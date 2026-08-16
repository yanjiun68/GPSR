#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__('scan_self_filter')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_filtered')
        self.declare_parameter('base_frame', 'base_footprint')
        # Legacy circular mode (used when footprint_polygon is empty).
        self.declare_parameter('robot_radius', 0.30)
        self.declare_parameter('padding', 0.02)
        # Polygon mode: flat list [x1, y1, x2, y2, ...] in base_frame.
        # Empty list falls back to the circular filter above.
        self.declare_parameter('footprint_polygon', [0.0])
        self.declare_parameter('footprint_padding', 0.03)
        self.declare_parameter('timestamp_offset_sec', 0.12)
        self.declare_parameter('warn_interval_sec', 5.0)

        self.input_scan_topic = self.get_parameter('input_scan_topic').value
        self.output_scan_topic = self.get_parameter('output_scan_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.padding = float(self.get_parameter('padding').value)
        self.footprint_padding = max(
            0.0, float(self.get_parameter('footprint_padding').value)
        )
        self.timestamp_offset_sec = max(
            0.0,
            float(self.get_parameter('timestamp_offset_sec').value),
        )
        self.warn_interval_sec = float(self.get_parameter('warn_interval_sec').value)

        flat_poly = list(self.get_parameter('footprint_polygon').value or [])
        if len(flat_poly) >= 6 and len(flat_poly) % 2 == 0:
            pts = np.array(flat_poly, dtype=np.float64).reshape(-1, 2)
            self.polygon = pts
            self.poly_next = np.roll(pts, -1, axis=0)
            self.poly_circumscribed = float(
                np.max(np.hypot(pts[:, 0], pts[:, 1])) + self.footprint_padding
            )
            edges_d = np.array(
                [
                    _point_to_segment_dist(
                        0.0, 0.0, a[0], a[1], b[0], b[1]
                    )
                    for a, b in zip(pts, self.poly_next)
                ]
            )
            self.poly_inscribed = float(np.min(edges_d))
            mode_desc = (
                f'polygon ({pts.shape[0]} verts, '
                f'inscribed={self.poly_inscribed:.3f} m, '
                f'circumscribed={self.poly_circumscribed - self.footprint_padding:.3f} m, '
                f'padding={self.footprint_padding:.3f} m)'
            )
        else:
            self.polygon = None
            self.poly_next = None
            self.poly_circumscribed = None
            self.poly_inscribed = None
            mode_desc = (
                f'circle (radius={self.robot_radius:.3f} m, '
                f'padding={self.padding:.3f} m)'
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_warn_time = None
        self.timestamp_offset = Duration(
            nanoseconds=int(self.timestamp_offset_sec * 1e9)
        )

        filtered_scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher = self.create_publisher(
            LaserScan,
            self.output_scan_topic,
            filtered_scan_qos,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'scan_self_filter: {self.input_scan_topic} -> {self.output_scan_topic} '
            f'base_frame={self.base_frame}, mode={mode_desc}, '
            f'timestamp_offset={self.timestamp_offset_sec:.3f} s'
        )

    def scan_callback(self, msg: LaserScan):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self._warn_throttled(
                f'Waiting for TF from {msg.header.frame_id} to {self.base_frame}: {exc}'
            )
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        yaw = self._yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.shape[0]
        if n == 0:
            self.publisher.publish(msg)
            return

        angles = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
        finite = np.isfinite(ranges)

        # Section: Transform finite returns only; multiplying infinity by zero produces NaNs.
        xs_base = np.full(n, np.nan, dtype=np.float64)
        ys_base = np.full(n, np.nan, dtype=np.float64)
        px_laser = ranges[finite] * np.cos(angles[finite])
        py_laser = ranges[finite] * np.sin(angles[finite])
        xs_base[finite] = tx + cos_yaw * px_laser - sin_yaw * py_laser
        ys_base[finite] = ty + sin_yaw * px_laser + cos_yaw * py_laser

        if self.polygon is not None:
            mask_inside = self._inside_polygon(xs_base, ys_base)
        else:
            r = self.robot_radius + self.padding
            mask_inside = (xs_base * xs_base + ys_base * ys_base) <= (r * r)

        # Only mark points that had a finite return; untouched bins stay as-is.
        drop = mask_inside & finite
        ranges_out = ranges.copy()
        ranges_out[drop] = math.inf

        filtered = LaserScan()
        filtered.header.frame_id = msg.header.frame_id
        filtered.header.stamp = self._resolve_output_stamp(msg.header.stamp)
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = ranges_out.astype(np.float32).tolist()
        filtered.intensities = list(msg.intensities)

        self.publisher.publish(filtered)

    def _inside_polygon(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        dist = np.hypot(xs, ys)
        # Fast reject: outside bounding circle + padding
        outside = dist > self.poly_circumscribed
        # Fast accept: inside inscribed circle (robot body, no padding needed)
        inside_fast = dist <= self.poly_inscribed
        result = inside_fast.copy()

        ambiguous = ~(outside | inside_fast)
        if not np.any(ambiguous):
            return result

        xa = xs[ambiguous]
        ya = ys[ambiguous]

        pip = _pip_vec(xa, ya, self.polygon, self.poly_next)
        if self.footprint_padding > 0.0:
            edge_d = _edge_dist_vec(xa, ya, self.polygon, self.poly_next)
            in_ring = pip | (edge_d <= self.footprint_padding)
        else:
            in_ring = pip

        amb_idx = np.nonzero(ambiguous)[0]
        result[amb_idx[in_ring]] = True
        return result

    def _resolve_output_stamp(self, incoming_stamp):
        stamp = Time.from_msg(incoming_stamp)
        if stamp.nanoseconds <= 0:
            stamp = self.get_clock().now()

        if stamp.nanoseconds > self.timestamp_offset.nanoseconds:
            stamp = stamp - self.timestamp_offset

        return stamp.to_msg()

    def _warn_throttled(self, message: str):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        warning_interval_elapsed = (
            self.last_warn_time is None
            or (now_sec - self.last_warn_time) >= self.warn_interval_sec
        )
        if warning_interval_elapsed:
            self.last_warn_time = now_sec
            self.get_logger().warn(message)

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)


def _point_to_segment_dist(px: float, py: float,
                           ax: float, ay: float,
                           bx: float, by: float) -> float:
    dx = bx - ax
    dy = by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    cx = ax + t * dx
    cy = ay + t * dy
    return math.hypot(px - cx, py - cy)


def _pip_vec(xs: np.ndarray, ys: np.ndarray,
             poly: np.ndarray, poly_next: np.ndarray) -> np.ndarray:
    """Ray-casting point-in-polygon, vectorised over xs/ys."""
    inside = np.zeros(xs.shape, dtype=bool)
    for (xi, yi), (xj, yj) in zip(poly, poly_next):
        dy = yj - yi
        if abs(dy) < 1e-12:
            continue
        crossing = (yi > ys) != (yj > ys)
        xint = (xj - xi) * (ys - yi) / dy + xi
        inside ^= crossing & (xs < xint)
    return inside


def _edge_dist_vec(xs: np.ndarray, ys: np.ndarray,
                   poly: np.ndarray, poly_next: np.ndarray) -> np.ndarray:
    """Shortest distance from each (xs, ys) to any polygon edge."""
    best = np.full(xs.shape, np.inf, dtype=np.float64)
    for (ax, ay), (bx, by) in zip(poly, poly_next):
        dx = bx - ax
        dy = by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-12:
            d = np.hypot(xs - ax, ys - ay)
        else:
            t = ((xs - ax) * dx + (ys - ay) * dy) / seg_len_sq
            np.clip(t, 0.0, 1.0, out=t)
            cx = ax + t * dx
            cy = ay + t * dy
            d = np.hypot(xs - cx, ys - cy)
        np.minimum(best, d, out=best)
    return best


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
