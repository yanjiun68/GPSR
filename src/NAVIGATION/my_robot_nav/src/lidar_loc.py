#!/usr/bin/env python3

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformBroadcaster,
    TransformListener,
)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


def pose_inverse(pose: Pose2D) -> Pose2D:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    x = -(c * pose.x + s * pose.y)
    y = -(-s * pose.x + c * pose.y)
    return Pose2D(x=x, y=y, yaw=wrap_angle(-pose.yaw))


def pose_compose(a: Pose2D, b: Pose2D) -> Pose2D:
    c = math.cos(a.yaw)
    s = math.sin(a.yaw)
    x = a.x + c * b.x - s * b.y
    y = a.y + s * b.x + c * b.y
    return Pose2D(x=x, y=y, yaw=wrap_angle(a.yaw + b.yaw))


def pose_delta(old_pose: Pose2D, new_pose: Pose2D) -> Pose2D:
    return pose_compose(pose_inverse(old_pose), new_pose)


class LidarLocalizer(Node):
    def __init__(self):
        super().__init__('lidar_loc')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('pose_topic', '/lidar_loc_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_refinement_time_sec', 0.0)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('score_radius', 0.18)
        self.declare_parameter('beam_stride', 5)
        self.declare_parameter('max_beams', 120)
        self.declare_parameter('coarse_xy_window', 0.30)
        self.declare_parameter('coarse_xy_step', 0.05)
        self.declare_parameter('coarse_yaw_window', 0.35)
        self.declare_parameter('coarse_yaw_step', 0.07)
        self.declare_parameter('fine_xy_window', 0.08)
        self.declare_parameter('fine_xy_step', 0.02)
        self.declare_parameter('fine_yaw_window', 0.10)
        self.declare_parameter('fine_yaw_step', 0.025)
        self.declare_parameter('min_valid_beams', 18)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('translation_penalty', 1.5)
        self.declare_parameter('rotation_penalty', 0.7)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.max_refinement_time_sec = float(
            self.get_parameter('max_refinement_time_sec').value
        )
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.beam_stride = max(1, int(self.get_parameter('beam_stride').value))
        self.max_beams = max(10, int(self.get_parameter('max_beams').value))
        self.coarse_xy_window = float(self.get_parameter('coarse_xy_window').value)
        self.coarse_xy_step = float(self.get_parameter('coarse_xy_step').value)
        self.coarse_yaw_window = float(self.get_parameter('coarse_yaw_window').value)
        self.coarse_yaw_step = float(self.get_parameter('coarse_yaw_step').value)
        self.fine_xy_window = float(self.get_parameter('fine_xy_window').value)
        self.fine_xy_step = float(self.get_parameter('fine_xy_step').value)
        self.fine_yaw_window = float(self.get_parameter('fine_yaw_window').value)
        self.fine_yaw_step = float(self.get_parameter('fine_yaw_step').value)
        self.min_valid_beams = int(self.get_parameter('min_valid_beams').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.translation_penalty = float(self.get_parameter('translation_penalty').value)
        self.rotation_penalty = float(self.get_parameter('rotation_penalty').value)

        self.weight_map = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None
        self.current_odom_pose = None
        self.last_odom_pose = None
        self.map_to_base_pose = None
        self.have_initial_pose = False
        self.initial_pose_time_sec = None
        self.last_scan_time_ns = None
        self._scan_frame_cache = {}

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        # This node is already spun by the main executor, so avoid an extra TF spin
        # thread that complicates shutdown and can race with node destruction.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('pose_topic').value,
            10,
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.map_callback,
            map_qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            50,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('initial_pose_topic').value,
            self.initial_pose_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            20,
        )
        if self.publish_tf:
            self.create_timer(0.05, self.republish_transform)

    def map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        occupied = data >= self.occupied_threshold
        weight_map = self.build_weight_map(occupied, msg.info.resolution)

        self.map_width = width
        self.map_height = height
        self.map_resolution = float(msg.info.resolution)
        self.map_origin_x = float(msg.info.origin.position.x)
        self.map_origin_y = float(msg.info.origin.position.y)
        self.weight_map = weight_map

        self.get_logger().info(
            f'Loaded map for lidar localization: {width}x{height} @ {self.map_resolution:.4f} m'
        )

    def build_weight_map(self, occupied: np.ndarray, resolution: float) -> np.ndarray:
        radius_m = float(self.get_parameter('score_radius').value)
        radius_cells = max(1, int(math.ceil(radius_m / resolution)))
        kernel = np.zeros((radius_cells * 2 + 1, radius_cells * 2 + 1), dtype=np.float32)

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                dist = math.hypot(dx, dy)
                if dist > radius_cells:
                    continue
                kernel[dy + radius_cells, dx + radius_cells] = 1.0 - (dist / max(1.0, radius_cells))

        weight_map = np.zeros_like(occupied, dtype=np.float32)
        ys, xs = np.nonzero(occupied)
        for y, x in zip(ys, xs):
            y_min = max(0, y - radius_cells)
            y_max = min(weight_map.shape[0], y + radius_cells + 1)
            x_min = max(0, x - radius_cells)
            x_max = min(weight_map.shape[1], x + radius_cells + 1)

            kernel_y_min = y_min - (y - radius_cells)
            kernel_y_max = kernel_y_min + (y_max - y_min)
            kernel_x_min = x_min - (x - radius_cells)
            kernel_x_max = kernel_x_min + (x_max - x_min)

            region = weight_map[y_min:y_max, x_min:x_max]
            region_kernel = kernel[kernel_y_min:kernel_y_max, kernel_x_min:kernel_x_max]
            np.maximum(region, region_kernel, out=region)

        if not np.any(weight_map):
            weight_map = occupied.astype(np.float32)
        return weight_map

    def odom_callback(self, msg: Odometry):
        yaw = quat_to_yaw(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.current_odom_pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw,
        )

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        yaw = quat_to_yaw(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.map_to_base_pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw,
        )
        self.last_odom_pose = self.current_odom_pose
        self.have_initial_pose = True
        self.initial_pose_time_sec = self.get_clock().now().nanoseconds / 1e9
        if self.publish_tf:
            self.publish_transform(msg.header.stamp)
        self.get_logger().info(
            f'Seeded lidar localization from /initialpose: '
            f'x={self.map_to_base_pose.x:.3f} y={self.map_to_base_pose.y:.3f} yaw={self.map_to_base_pose.yaw:.3f}'
        )

    def scan_callback(self, msg: LaserScan):
        if self.weight_map is None or self.current_odom_pose is None or not self.have_initial_pose:
            return

        if (
            self.max_refinement_time_sec > 0.0
            and self.initial_pose_time_sec is not None
            and (self.get_clock().now().nanoseconds / 1e9 - self.initial_pose_time_sec)
            > self.max_refinement_time_sec
        ):
            return

        beams = self.scan_to_points(msg)
        if beams is None:
            return

        predicted_pose = self.predict_pose()
        coarse_pose, coarse_score = self.search_pose(
            predicted_pose,
            beams,
            self.coarse_xy_window,
            self.coarse_xy_step,
            self.coarse_yaw_window,
            self.coarse_yaw_step,
        )
        fine_pose, fine_score = self.search_pose(
            coarse_pose,
            beams,
            self.fine_xy_window,
            self.fine_xy_step,
            self.fine_yaw_window,
            self.fine_yaw_step,
        )

        if fine_score <= -1e8:
            self.get_logger().warn('No valid lidar localization candidate found', throttle_duration_sec=2.0)
            return

        self.map_to_base_pose = fine_pose
        self.last_odom_pose = self.current_odom_pose
        if self.publish_tf:
            self.publish_transform(msg.header.stamp)
        self.publish_pose(msg.header.stamp, fine_score)

    def scan_to_points(self, msg: LaserScan):
        valid_points = []
        range_cap = min(self.max_range, msg.range_max)
        laser_offset = self.lookup_laser_offset(msg.header.frame_id)
        if laser_offset is None:
            return None

        offset_x, offset_y, offset_yaw = laser_offset
        cos_offset = math.cos(offset_yaw)
        sin_offset = math.sin(offset_yaw)
        start_index = 0
        for index in range(start_index, len(msg.ranges), self.beam_stride):
            r = float(msg.ranges[index])
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > range_cap:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            laser_x = r * math.cos(angle)
            laser_y = r * math.sin(angle)
            base_x = offset_x + cos_offset * laser_x - sin_offset * laser_y
            base_y = offset_y + sin_offset * laser_x + cos_offset * laser_y
            valid_points.append((base_x, base_y))
            if len(valid_points) >= self.max_beams:
                break

        if len(valid_points) < self.min_valid_beams:
            return None

        return np.asarray(valid_points, dtype=np.float32)

    def lookup_laser_offset(self, scan_frame: str):
        if scan_frame in self._scan_frame_cache:
            return self._scan_frame_cache[scan_frame]

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                scan_frame,
                Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        yaw = quat_to_yaw(
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        offset = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw,
        )
        self._scan_frame_cache[scan_frame] = offset
        self.get_logger().info(
            f'Using scan frame offset {scan_frame} -> {self.base_frame}: '
            f'x={offset[0]:.3f} y={offset[1]:.3f} yaw={offset[2]:.3f}'
        )
        return offset

    def predict_pose(self) -> Pose2D:
        if self.last_odom_pose is None:
            self.last_odom_pose = self.current_odom_pose
            return self.map_to_base_pose

        delta = pose_delta(self.last_odom_pose, self.current_odom_pose)
        return pose_compose(self.map_to_base_pose, delta)

    def search_pose(
        self,
        seed_pose: Pose2D,
        beams: np.ndarray,
        xy_window: float,
        xy_step: float,
        yaw_window: float,
        yaw_step: float,
    ):
        best_pose = seed_pose
        best_score = -1e9
        xy_offsets = np.arange(-xy_window, xy_window + 0.5 * xy_step, xy_step)
        yaw_offsets = np.arange(-yaw_window, yaw_window + 0.5 * yaw_step, yaw_step)

        for dyaw in yaw_offsets:
            yaw = wrap_angle(seed_pose.yaw + dyaw)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            rot_x = cos_yaw * beams[:, 0] - sin_yaw * beams[:, 1]
            rot_y = sin_yaw * beams[:, 0] + cos_yaw * beams[:, 1]

            for dx in xy_offsets:
                world_x = seed_pose.x + dx + rot_x
                for dy in xy_offsets:
                    world_y = seed_pose.y + dy + rot_y
                    score = self.score_points(world_x, world_y)
                    score -= self.translation_penalty * (dx * dx + dy * dy)
                    score -= self.rotation_penalty * (dyaw * dyaw)
                    if score > best_score:
                        best_score = score
                        best_pose = Pose2D(
                            x=seed_pose.x + dx,
                            y=seed_pose.y + dy,
                            yaw=yaw,
                        )

        return best_pose, best_score

    def score_points(self, world_x: np.ndarray, world_y: np.ndarray) -> float:
        map_x = ((world_x - self.map_origin_x) / self.map_resolution).astype(np.int32)
        map_y = ((world_y - self.map_origin_y) / self.map_resolution).astype(np.int32)
        inside = (
            (map_x >= 0)
            & (map_x < self.map_width)
            & (map_y >= 0)
            & (map_y < self.map_height)
        )
        if not np.any(inside):
            return -1e9

        weights = self.weight_map[map_y[inside], map_x[inside]]
        return float(weights.sum() - 0.2 * (len(world_x) - np.count_nonzero(inside)))

    def publish_transform(self, stamp):
        if self.current_odom_pose is None or self.map_to_base_pose is None:
            return

        map_to_odom = pose_compose(self.map_to_base_pose, pose_inverse(self.current_odom_pose))
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = map_to_odom.x
        transform.transform.translation.y = map_to_odom.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(map_to_odom.yaw / 2.0)
        transform.transform.rotation.w = math.cos(map_to_odom.yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def republish_transform(self):
        if self.current_odom_pose is None or self.map_to_base_pose is None:
            return
        self.publish_transform(self.get_clock().now().to_msg())

    def publish_pose(self, stamp, score: float):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = self.map_to_base_pose.x
        msg.pose.pose.position.y = self.map_to_base_pose.y
        msg.pose.pose.orientation.z = math.sin(self.map_to_base_pose.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.map_to_base_pose.yaw / 2.0)
        covariance_xy = max(0.02, 0.20 - min(0.18, 0.001 * max(score, 0.0)))
        covariance_yaw = max(0.02, 0.10 - min(0.08, 0.0005 * max(score, 0.0)))
        msg.pose.covariance = [
            covariance_xy, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, covariance_xy, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, covariance_yaw,
        ]
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarLocalizer()
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
