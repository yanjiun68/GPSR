#!/usr/bin/env python3

import os
import threading

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO


def _default_model_path() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(script_dir)
    candidates = [
        os.path.join(package_root, 'models', 'yolo11n.pt'),
    ]

    try:
        share_dir = get_package_share_directory('person_following')
        candidates.append(os.path.join(share_dir, 'models', 'yolo11n.pt'))
    except PackageNotFoundError:
        pass

    candidates.append('yolov8n.pt')

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate

    return candidates[0]


def _param_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 't', 'yes', 'y', 'on')
    return bool(value)


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')
        self.declare_parameter('detected_image_topic', '/detected_image')
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('target_distance_mm', 1000.0)
        self.declare_parameter('x_p', 0.003)
        self.declare_parameter('x_d', 0.0005)
        self.declare_parameter('z_p', 0.003)
        self.declare_parameter('z_d', 0.0005)
        self.declare_parameter('min_depth_mm', 0)
        self.declare_parameter('max_depth_mm', 3000)
        self.declare_parameter('linear_max_mps', 0.2)
        self.declare_parameter('linear_min_mps', -0.1)
        self.declare_parameter('angular_limit_rps', 0.3)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop_sec', 0.08)
        self.declare_parameter('show_debug_window', False)

        self.model_path = str(self.get_parameter('model_path').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.detected_image_topic = str(self.get_parameter('detected_image_topic').value)
        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.target_distance = float(self.get_parameter('target_distance_mm').value)
        self.x_p = float(self.get_parameter('x_p').value)
        self.x_d = float(self.get_parameter('x_d').value)
        self.z_p = float(self.get_parameter('z_p').value)
        self.z_d = float(self.get_parameter('z_d').value)
        self.min_depth = float(self.get_parameter('min_depth_mm').value)
        self.max_depth = float(self.get_parameter('max_depth_mm').value)
        self.linear_max = float(self.get_parameter('linear_max_mps').value)
        self.linear_min = float(self.get_parameter('linear_min_mps').value)
        self.angular_limit = float(self.get_parameter('angular_limit_rps').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.sync_queue_size = int(self.get_parameter('sync_queue_size').value)
        self.sync_slop_sec = float(self.get_parameter('sync_slop_sec').value)
        self.show_debug_window = _param_bool(self.get_parameter('show_debug_window').value)

        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        self.vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.image_pub = self.create_publisher(Image, self.detected_image_topic, 10)

        self.last_distance_error = 0.0
        self.last_x_center_error = 0.0
        self.tracker = None
        self.tracking = False
        self._shutdown_requested = False
        self._camera_info_received = False
        self._latest_rgb_msg = None
        self._latest_depth_msg = None
        self._latest_stamp = None

        self._realsense_launch_service = None
        self._realsense_launch_thread = None

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub_info = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_cb, qos_sensor
        )
        self.rgb_sub = Subscriber(self, Image, self.rgb_topic, qos_profile=qos_sensor)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos_sensor)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
        )
        self.sync.registerCallback(self.synced_cb)

        timer_period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.process_frame)

        self.get_logger().info(f'Loaded YOLO model: {self.model_path}')
        self.get_logger().info(f'RGB topic: {self.rgb_topic}')
        self.get_logger().info(f'Depth topic: {self.depth_topic}')
        self.get_logger().info(f'CameraInfo topic: {self.camera_info_topic}')

    def info_cb(self, _msg: CameraInfo):
        if not self._camera_info_received:
            self._camera_info_received = True
            self.get_logger().info('Received CameraInfo from RealSense wrapper')

    def synced_cb(self, rgb_msg: Image, depth_msg: Image):
        self._latest_rgb_msg = rgb_msg
        self._latest_depth_msg = depth_msg
        self._latest_stamp = rgb_msg.header.stamp

    def process_frame(self):
        if self._shutdown_requested:
            return

        if self._latest_rgb_msg is None or self._latest_depth_msg is None:
            return

        try:
            bgr_frame = self.bridge.imgmsg_to_cv2(self._latest_rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(self._latest_depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Failed to convert synchronized camera frames: {exc}')
            return

        if self.tracking:
            self.update_tracking(bgr_frame, depth_image)
        else:
            self.detect_person(bgr_frame, depth_image)

        image_msg = self.bridge.cv2_to_imgmsg(bgr_frame, encoding='bgr8')
        image_msg.header = self._latest_rgb_msg.header
        self.image_pub.publish(image_msg)

        if self.show_debug_window:
            cv2.imshow('BGR Frame', bgr_frame)
            if cv2.waitKey(1) == 27:
                self.get_logger().info('Escape pressed, shutting down')
                self._shutdown_requested = True
                self.publish_stop()
                rclpy.shutdown()

    def update_tracking(self, bgr_frame, depth_image):
        if self.tracker is None:
            self.reset_tracking('Tracker is not initialized')
            return

        success, bbox = self.tracker.update(bgr_frame)
        if not success:
            self.reset_tracking('Tracker lost target')
            return

        x1, y1, w, h = map(int, bbox)
        x2, y2 = x1 + w, y1 + h
        x_center = (x1 + x2) // 2
        y_center = (y1 + y2) // 2

        if not self._pixel_in_bounds(depth_image, x_center, y_center):
            self.reset_tracking('Tracked bbox left image bounds')
            return

        distance = float(depth_image[y_center, x_center])
        if not (self.min_depth < distance < self.max_depth):
            self.reset_tracking('Tracked person depth is outside the valid range')
            return

        cv2.rectangle(bgr_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            bgr_frame,
            f'Tracking Person: {distance:.2f} mm',
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        distance_error = distance - self.target_distance
        distance_derivative = distance_error - self.last_distance_error
        control_linear = self.x_p * distance_error + self.x_d * distance_derivative
        control_linear = max(min(control_linear, self.linear_max), self.linear_min)

        frame_center_x = bgr_frame.shape[1] // 2
        x_center_error = frame_center_x - x_center
        x_center_derivative = x_center_error - self.last_x_center_error
        control_angular = self.z_p * x_center_error + self.z_d * x_center_derivative
        control_angular = max(min(control_angular, self.angular_limit), -self.angular_limit)

        self.last_distance_error = distance_error
        self.last_x_center_error = x_center_error

        vel_msg = Twist()
        vel_msg.linear.x = control_linear
        vel_msg.angular.z = control_angular
        self.vel_pub.publish(vel_msg)

    def detect_person(self, bgr_frame, depth_image):
        results = self.model(bgr_frame, verbose=False)
        if not results:
            return

        for box in results[0].boxes:
            if self._box_class_id(box) != 0:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            x_center = (x1 + x2) // 2
            y_center = (y1 + y2) // 2

            if not self._pixel_in_bounds(depth_image, x_center, y_center):
                continue

            distance = float(depth_image[y_center, x_center])
            if not (self.min_depth < distance < self.max_depth):
                continue

            tracker_name, tracker = self.create_tracker()
            if tracker is None:
                self.get_logger().error('No supported OpenCV tracker is available')
                return

            tracker.init(bgr_frame, (x1, y1, x2 - x1, y2 - y1))
            self.tracker = tracker
            self.tracking = True
            self.last_distance_error = 0.0
            self.last_x_center_error = 0.0
            self.get_logger().info(
                f'Locked onto person at ({x_center}, {y_center}), depth={distance:.0f} mm using {tracker_name}'
            )
            return

    def reset_tracking(self, reason: str):
        if self.tracking:
            self.get_logger().info(reason)
        self.tracker = None
        self.tracking = False
        self.last_distance_error = 0.0
        self.last_x_center_error = 0.0
        self.publish_stop()

    def publish_stop(self):
        self.vel_pub.publish(Twist())

    @staticmethod
    def _box_class_id(box) -> int:
        cls_value = box.cls
        if hasattr(cls_value, '__len__'):
            cls_value = cls_value[0]
        if hasattr(cls_value, 'item'):
            cls_value = cls_value.item()
        return int(cls_value)

    @staticmethod
    def _pixel_in_bounds(image, x: int, y: int) -> bool:
        return 0 <= y < image.shape[0] and 0 <= x < image.shape[1]

    @staticmethod
    def create_tracker():
        tracker_candidates = [
            ('legacy.TrackerKCF', lambda: cv2.legacy.TrackerKCF_create(), hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create')),
            ('TrackerKCF', lambda: cv2.TrackerKCF_create(), hasattr(cv2, 'TrackerKCF_create')),
            ('legacy.TrackerMOSSE', lambda: cv2.legacy.TrackerMOSSE_create(), hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerMOSSE_create')),
            ('TrackerMOSSE', lambda: cv2.TrackerMOSSE_create(), hasattr(cv2, 'TrackerMOSSE_create')),
            ('TrackerMIL', lambda: cv2.TrackerMIL_create(), hasattr(cv2, 'TrackerMIL_create')),
        ]

        for tracker_name, factory, is_available in tracker_candidates:
            if not is_available:
                continue
            try:
                return tracker_name, factory()
            except Exception:
                continue

        return None, None

    def destroy_node(self):
        self._shutdown_requested = True

        try:
            self.publish_stop()
        except Exception:
            pass

        if self._realsense_launch_service is not None:
            try:
                self._realsense_launch_service.shutdown()
            except Exception:
                pass
            self._realsense_launch_service = None

        if self._realsense_launch_thread is not None:
            self._realsense_launch_thread.join(timeout=2.0)
            self._realsense_launch_thread = None

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    follower = None
    try:
        follower = PersonFollower()
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        if follower is not None:
            follower.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
