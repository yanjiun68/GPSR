#!/usr/bin/env python3
import math
from typing import Optional

from cv_bridge import CvBridge
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401

from putrabot_interfaces.msg import CamDetection


class SelectedBagCloudClusterNode(Node):
    def __init__(self):
        super().__init__("selected_bag_cloud_cluster_node")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("bbox_shrink_factor", 0.12)
        self.declare_parameter("min_depth_m", 0.20)
        self.declare_parameter("max_depth_m", 3.50)
        self.declare_parameter("depth_sampling_stride_px", 2)
        self.declare_parameter("min_valid_depth_points", 40)
        self.declare_parameter("camera_cluster_min_points", 20)
        self.declare_parameter("camera_cluster_target_points", 140)
        self.declare_parameter("point_cloud_cluster_tolerance_m", 0.08)
        self.declare_parameter("camera_cluster_variance_scale_m2", 0.015)
        self.declare_parameter("camera_cluster_stability_scale_m", 0.15)
        self.declare_parameter("max_bbox_age_sec", 0.6)
        self.declare_parameter("max_depth_age_sec", 0.6)
        self.declare_parameter("max_cross_sensor_skew_sec", 0.35)

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.depth_msg: Optional[Image] = None
        self.selected_bbox: Optional[CamDetection] = None
        self.selected_bbox_received_at: Optional[Time] = None
        self.last_base_centroid: Optional[PointStamped] = None
        self.last_publish_time: Optional[Time] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.get_parameter("depth_topic").value, self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value, self.camera_info_callback, 10
        )
        self.create_subscription(CamDetection, "/selected_bag_bbox", self.selected_bbox_callback, 10)

        self.camera_pub = self.create_publisher(PointStamped, "/selected_bag_centroid_camera", 10)
        self.base_pub = self.create_publisher(PointStamped, "/selected_bag_centroid_base", 10)
        self.confidence_pub = self.create_publisher(Float32, "/selected_bag_cluster_confidence", 10)

    def depth_callback(self, msg: Image) -> None:
        self.depth_msg = msg
        self.try_publish()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def selected_bbox_callback(self, msg: CamDetection) -> None:
        self.selected_bbox = msg
        self.selected_bbox_received_at = self.get_clock().now()
        self.try_publish()

    def _depth_to_meters(self, value, encoding: str) -> float:
        if encoding in ("16UC1", "mono16"):
            return float(value) / 1000.0
        return float(value)

    def _stamp_age_sec(self, stamp) -> Optional[float]:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        msg_time = Time.from_msg(stamp)
        return max(0.0, (self.get_clock().now() - msg_time).nanoseconds / 1e9)

    def _bbox_bounds(self, width: int, height: int) -> Optional[tuple[int, int, int, int]]:
        if self.selected_bbox is None:
            return None

        x1 = max(0, min(width - 1, int(self.selected_bbox.x1)))
        x2 = max(0, min(width, int(self.selected_bbox.x2)))
        y1 = max(0, min(height - 1, int(self.selected_bbox.y1)))
        y2 = max(0, min(height, int(self.selected_bbox.y2)))
        if x2 <= x1 or y2 <= y1:
            return None

        shrink = min(0.45, max(0.0, float(self.get_parameter("bbox_shrink_factor").value)))
        dx = int((x2 - x1) * shrink * 0.5)
        dy = int((y2 - y1) * shrink * 0.5)
        left = max(0, x1 + dx)
        right = min(width, x2 - dx)
        top = max(0, y1 + dy)
        bottom = min(height, y2 - dy)
        if right - left < 2 or bottom - top < 2:
            return None
        return left, top, right, bottom

    def _extract_grid(self, depth_img: np.ndarray, bounds: tuple[int, int, int, int]) -> Optional[dict]:
        if self.camera_info is None or self.depth_msg is None:
            return None

        left, top, right, bottom = bounds
        stride = max(1, int(self.get_parameter("depth_sampling_stride_px").value))
        xs = list(range(left, right, stride))
        ys = list(range(top, bottom, stride))
        if not xs or not ys:
            return None
        if xs[-1] != right - 1:
            xs.append(right - 1)
        if ys[-1] != bottom - 1:
            ys.append(bottom - 1)

        points = np.full((len(ys), len(xs), 3), np.nan, dtype=np.float32)
        k = self.camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        cx0 = float(k[2])
        cy0 = float(k[5])
        min_depth_m = float(self.get_parameter("min_depth_m").value)
        max_depth_m = float(self.get_parameter("max_depth_m").value)

        for row_idx, v in enumerate(ys):
            for col_idx, u in enumerate(xs):
                depth_m = self._depth_to_meters(depth_img[v, u], self.depth_msg.encoding)
                if not math.isfinite(depth_m) or depth_m < min_depth_m or depth_m > max_depth_m:
                    continue
                x = (float(u) - cx0) * depth_m / fx
                y = (float(v) - cy0) * depth_m / fy
                points[row_idx, col_idx] = (x, y, depth_m)

        valid_mask = np.isfinite(points[..., 2])
        if int(valid_mask.sum()) < int(self.get_parameter("min_valid_depth_points").value):
            return None
        return {"points": points, "valid_mask": valid_mask, "xs": xs, "ys": ys}

    def _best_cluster(self, grid: dict, bbox_center: tuple[float, float], bbox_extent: float) -> Optional[dict]:
        points = grid["points"]
        valid_mask = grid["valid_mask"]
        xs = grid["xs"]
        ys = grid["ys"]
        tolerance = float(self.get_parameter("point_cloud_cluster_tolerance_m").value)
        min_cluster_points = max(3, int(self.get_parameter("camera_cluster_min_points").value))
        variance_scale = max(1e-6, float(self.get_parameter("camera_cluster_variance_scale_m2").value))
        visited = np.zeros(valid_mask.shape, dtype=bool)

        best = None
        rows, cols = valid_mask.shape
        neighbors = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]

        # Small Euclidean connected-components pass over the sampled depth grid keeps
        # the centroid tied to the selected bag surface instead of a raw ROI median.
        for row in range(rows):
            for col in range(cols):
                if not valid_mask[row, col] or visited[row, col]:
                    continue
                stack = [(row, col)]
                visited[row, col] = True
                members = []
                while stack:
                    current_row, current_col = stack.pop()
                    members.append((current_row, current_col))
                    current_point = points[current_row, current_col]
                    for d_row, d_col in neighbors:
                        next_row = current_row + d_row
                        next_col = current_col + d_col
                        if (
                            next_row < 0
                            or next_col < 0
                            or next_row >= rows
                            or next_col >= cols
                            or visited[next_row, next_col]
                            or not valid_mask[next_row, next_col]
                        ):
                            continue
                        if np.linalg.norm(points[next_row, next_col] - current_point) > tolerance:
                            continue
                        visited[next_row, next_col] = True
                        stack.append((next_row, next_col))

                if len(members) < min_cluster_points:
                    continue

                cluster_points = np.array([points[row_idx, col_idx] for row_idx, col_idx in members], dtype=np.float32)
                centroid = cluster_points.mean(axis=0)
                variance = float(np.mean(np.sum((cluster_points - centroid) ** 2, axis=1)))
                mean_u = float(np.mean([xs[col_idx] for _, col_idx in members]))
                mean_v = float(np.mean([ys[row_idx] for row_idx, _ in members]))
                center_error = math.hypot(mean_u - bbox_center[0], mean_v - bbox_center[1]) / max(1.0, bbox_extent)
                score = len(members) / (1.0 + center_error + variance / variance_scale)

                if best is None or score > best["score"]:
                    best = {
                        "centroid": centroid,
                        "variance": variance,
                        "count": len(members),
                        "score": score,
                    }
        return best

    def _transform_to_base(self, camera_point: PointStamped) -> Optional[PointStamped]:
        target_frame = str(self.get_parameter("base_frame").value)
        try:
            return self.tf_buffer.transform(camera_point, target_frame, timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().warn(f"Camera centroid transform failed: {exc}", throttle_duration_sec=2.0)
            return None

    def _confidence(self, cluster: dict, base_point: PointStamped) -> float:
        count_target = max(1, int(self.get_parameter("camera_cluster_target_points").value))
        point_score = min(1.0, float(cluster["count"]) / float(count_target))

        variance_scale = max(1e-6, float(self.get_parameter("camera_cluster_variance_scale_m2").value))
        compactness = 1.0 / (1.0 + float(cluster["variance"]) / variance_scale)

        stability = 1.0
        if self.last_base_centroid is not None and self.last_publish_time is not None:
            dt = (self.get_clock().now() - self.last_publish_time).nanoseconds / 1e9
            if dt <= 1.0:
                distance = math.sqrt(
                    (base_point.point.x - self.last_base_centroid.point.x) ** 2
                    + (base_point.point.y - self.last_base_centroid.point.y) ** 2
                    + (base_point.point.z - self.last_base_centroid.point.z) ** 2
                )
                scale = max(1e-6, float(self.get_parameter("camera_cluster_stability_scale_m").value))
                stability = math.exp(-distance / scale)

        confidence = 0.45 * point_score + 0.35 * compactness + 0.20 * stability
        return max(0.0, min(1.0, confidence))

    def try_publish(self) -> None:
        if self.selected_bbox is None or self.depth_msg is None or self.camera_info is None:
            return
        if self.selected_bbox_received_at is None:
            return

        bbox_age = max(0.0, (self.get_clock().now() - self.selected_bbox_received_at).nanoseconds / 1e9)
        if bbox_age > float(self.get_parameter("max_bbox_age_sec").value):
            self.get_logger().warn("Skipping camera clustering due to stale bbox.", throttle_duration_sec=2.0)
            return

        depth_age = self._stamp_age_sec(self.depth_msg.header.stamp)
        if depth_age is not None and depth_age > float(self.get_parameter("max_depth_age_sec").value):
            self.get_logger().warn("Skipping camera clustering due to stale depth frame.", throttle_duration_sec=2.0)
            return

        if depth_age is not None:
            skew = abs((self.selected_bbox_received_at - Time.from_msg(self.depth_msg.header.stamp)).nanoseconds) / 1e9
            if skew > float(self.get_parameter("max_cross_sensor_skew_sec").value):
                self.get_logger().warn("Skipping camera clustering due to bbox/depth skew.", throttle_duration_sec=2.0)
                return

        depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding="passthrough")
        height, width = depth_img.shape[:2]
        bounds = self._bbox_bounds(width, height)
        if bounds is None:
            return

        grid = self._extract_grid(depth_img, bounds)
        if grid is None:
            self.get_logger().warn("Not enough valid depth points inside selected bbox.", throttle_duration_sec=2.0)
            return

        left, top, right, bottom = bounds
        bbox_center = (0.5 * (left + right), 0.5 * (top + bottom))
        bbox_extent = 0.5 * max(1, max(right - left, bottom - top))
        cluster = self._best_cluster(grid, bbox_center, bbox_extent)
        if cluster is None:
            self.get_logger().warn("No valid camera cluster found inside selected bbox.", throttle_duration_sec=2.0)
            return

        camera_point = PointStamped()
        camera_point.header = self.depth_msg.header
        if not camera_point.header.frame_id:
            camera_point.header.frame_id = str(self.get_parameter("camera_frame").value)
        camera_point.point.x = float(cluster["centroid"][0])
        camera_point.point.y = float(cluster["centroid"][1])
        camera_point.point.z = float(cluster["centroid"][2])

        base_point = self._transform_to_base(camera_point)
        if base_point is None:
            return

        confidence = self._confidence(cluster, base_point)
        self.camera_pub.publish(camera_point)
        self.base_pub.publish(base_point)
        self.confidence_pub.publish(Float32(data=float(confidence)))

        self.last_base_centroid = base_point
        self.last_publish_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = SelectedBagCloudClusterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
