#!/usr/bin/env python3
import glob
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


def camera_sort_key(path):
    suffix = Path(path).name.replace('video', '')
    try:
        return int(suffix)
    except ValueError:
        return 9999


def normalize_camera_source(source):
    source_text = str(source)
    return int(source_text) if source_text.isdigit() else source_text


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def camera_candidates(camera_arg):
    if str(camera_arg).lower() != 'auto':
        return [normalize_camera_source(camera_arg)]

    candidates = []
    seen = set()

    def add_candidate(candidate):
        key = str(candidate)
        if key in seen:
            return
        seen.add(key)
        candidates.append(candidate)

    env_camera = os.environ.get('INSIGHTFACE_CAMERA')
    if env_camera:
        add_candidate(normalize_camera_source(env_camera))

    for video_path in sorted(glob.glob('/dev/video*'), key=camera_sort_key):
        add_candidate(video_path)

    for index in range(10):
        add_candidate(index)

    return candidates


def open_webcam(camera_arg, logger):
    checked = []
    for source in camera_candidates(camera_arg):
        checked.append(source)
        logger.info(f'Trying webcam: {source}')
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue

        for _ in range(5):
            ret, _ = cap.read()
            if ret:
                return cap, source
            time.sleep(0.1)

        cap.release()

    checked_sources = ', '.join(str(item) for item in checked)
    logger.error(f'Could not open webcam. Checked sources: {checked_sources}')
    return None, None


class OpenCvCameraSource:
    def __init__(self, cap):
        self.cap = cap

    def read(self):
        ret, frame = self.cap.read()
        return ret, frame, None

    def release(self):
        self.cap.release()


class RealSenseCameraSource:
    def __init__(
        self,
        logger,
        width,
        height,
        fps,
        serial_no='',
        enable_depth=True,
        align_depth_to_color=True,
    ):
        if rs is None:
            raise RuntimeError(
                'pyrealsense2 is not installed. Install librealsense Python bindings '
                'or use camera_backend:=opencv with a valid /dev/videoN.'
            )

        self.logger = logger
        self.enable_depth = bool(enable_depth)
        self.align = rs.align(rs.stream.color) if self.enable_depth and align_depth_to_color else None
        self.logged_missing_depth = False
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        serial_no = str(serial_no or '').strip()
        if serial_no:
            self.config.enable_device(serial_no)

        self.config.enable_stream(
            rs.stream.color,
            int(width),
            int(height),
            rs.format.bgr8,
            int(fps),
        )
        if self.enable_depth:
            self.config.enable_stream(
                rs.stream.depth,
                int(width),
                int(height),
                rs.format.z16,
                int(fps),
            )
        self.pipeline.start(self.config)

    def read(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as exc:
            self.logger.warning(f'Failed to read RealSense frames: {exc}')
            return False, None, None

        if self.align is not None:
            frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            return False, None, None

        depth_image = None
        if self.enable_depth:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
            elif not self.logged_missing_depth:
                self.logger.warning('RealSense depth stream is enabled but no depth frame was received.')
                self.logged_missing_depth = True

        return True, np.asanyarray(color_frame.get_data()), depth_image

    def release(self):
        self.pipeline.stop()


class CameraNode(Node):
    """Special function: Publish standalone webcam or RealSense frames onto ROS topics."""

    def __init__(self):
        """Special function: Configure camera ownership fallback with production debug disabled."""
        super().__init__('camera_node')

        self.declare_parameter('camera_backend', 'realsense')
        self.declare_parameter('camera_index', 'auto')
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('realsense_width', 640)
        self.declare_parameter('realsense_height', 480)
        self.declare_parameter('realsense_serial_no', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('enable_depth', False)
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('align_depth_to_color', True)
        self.declare_parameter('stream_info_topic', '/camera/stream_info')
        self.declare_parameter('stream_info_period_sec', 1.0)
        self.declare_parameter('log_video_stats', False)
        self.declare_parameter('enable_depth_debug_image', False)
        self.declare_parameter('depth_debug_topic', '/camera/depth/debug_image')
        self.declare_parameter('depth_debug_max_mm', 5000.0)

        camera_backend = str(self.get_parameter('camera_backend').value).strip().lower()
        camera_index = self.get_parameter('camera_index').value
        fps = float(self.get_parameter('fps').value)
        realsense_width = int(self.get_parameter('realsense_width').value)
        realsense_height = int(self.get_parameter('realsense_height').value)
        realsense_serial_no = str(self.get_parameter('realsense_serial_no').value)
        image_topic = str(self.get_parameter('image_topic').value)
        enable_depth = as_bool(self.get_parameter('enable_depth').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        align_depth_to_color = as_bool(self.get_parameter('align_depth_to_color').value)
        stream_info_topic = str(self.get_parameter('stream_info_topic').value)
        self.stream_info_period_sec = max(
            0.1,
            float(self.get_parameter('stream_info_period_sec').value),
        )
        self.log_video_stats = as_bool(self.get_parameter('log_video_stats').value)
        enable_depth_debug_image = as_bool(self.get_parameter('enable_depth_debug_image').value)
        depth_debug_topic = str(self.get_parameter('depth_debug_topic').value)
        self.depth_debug_max_mm = max(
            1.0,
            float(self.get_parameter('depth_debug_max_mm').value),
        )

        self.bridge = CvBridge()
        self.camera_backend = camera_backend
        self.camera_index = str(camera_index)
        self.camera_source = ''
        self.image_topic = image_topic
        self.depth_topic = depth_topic
        self.enable_depth = enable_depth
        self.align_depth_to_color = align_depth_to_color
        self.frame_count = 0
        self.depth_frame_count = 0
        self.start_time = time.monotonic()
        self.last_stream_info_time = 0.0
        self.last_stream_info_frame_count = 0
        self.publisher_ = self.create_publisher(Image, image_topic, 10)
        self.depth_publisher = None
        self.depth_debug_publisher = None
        self.stream_info_publisher = self.create_publisher(String, stream_info_topic, 10)

        if camera_backend in ('realsense', 'realsense_sdk', 'pyrealsense2'):
            self.camera = RealSenseCameraSource(
                self.get_logger(),
                realsense_width,
                realsense_height,
                fps,
                realsense_serial_no,
                enable_depth,
                align_depth_to_color,
            )
            if enable_depth:
                self.depth_publisher = self.create_publisher(Image, depth_topic, 10)
                if enable_depth_debug_image:
                    self.depth_debug_publisher = self.create_publisher(Image, depth_debug_topic, 10)
            camera_source = 'pyrealsense2'
            if realsense_serial_no:
                camera_source = f'{camera_source}:{realsense_serial_no}'
        elif camera_backend in ('opencv', 'v4l2', 'webcam'):
            if enable_depth:
                self.get_logger().warning(
                    'enable_depth is true, but depth output is only supported with camera_backend=realsense.'
                )
            cap, camera_source = open_webcam(camera_index, self.get_logger())
            if cap is None:
                raise RuntimeError('Could not open webcam')
            self.camera = OpenCvCameraSource(cap)
        else:
            raise RuntimeError(
                f"Unsupported camera_backend='{camera_backend}'. "
                'Use realsense or opencv.'
            )

        self.camera_source = str(camera_source)
        period = 1.0 / max(fps, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f'Camera node started. backend={camera_backend}, camera_index={camera_index}, '
            f'camera_source={camera_source}, '
            f'fps={fps}, image_topic={image_topic}, '
            f'enable_depth={enable_depth}, depth_topic={depth_topic if enable_depth else ""}, '
            f'stream_info_topic={stream_info_topic}'
        )

    def _build_depth_debug_image(self, depth_frame):
        clipped_depth = np.clip(depth_frame.astype(np.float32), 0.0, self.depth_debug_max_mm)
        depth_8bit = cv2.convertScaleAbs(clipped_depth, alpha=255.0 / self.depth_debug_max_mm)
        return cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

    def _depth_stats(self, depth_frame):
        if depth_frame is None:
            return {
                'available': False,
                'valid_pixel_count': 0,
                'min_mm': 0.0,
                'median_mm': 0.0,
                'max_mm': 0.0,
            }

        valid_depth = depth_frame[depth_frame > 0]
        if valid_depth.size == 0:
            return {
                'available': True,
                'valid_pixel_count': 0,
                'min_mm': 0.0,
                'median_mm': 0.0,
                'max_mm': 0.0,
            }

        return {
            'available': True,
            'valid_pixel_count': int(valid_depth.size),
            'min_mm': float(np.min(valid_depth)),
            'median_mm': float(np.median(valid_depth)),
            'max_mm': float(np.max(valid_depth)),
        }

    def _publish_stream_info(self, frame, depth_frame):
        now = time.monotonic()
        if now - self.last_stream_info_time < self.stream_info_period_sec:
            return

        elapsed = max(now - self.start_time, 1e-6)
        period_elapsed = max(now - self.last_stream_info_time, 1e-6)
        period_frame_count = max(0, self.frame_count - self.last_stream_info_frame_count)
        refresh_rate_hz = float(period_frame_count / period_elapsed)
        payload = {
            'backend': self.camera_backend,
            'camera_index': self.camera_index,
            'camera_source': self.camera_source,
            'color': {
                'topic': self.image_topic,
                'encoding': 'bgr8',
                'width': int(frame.shape[1]),
                'height': int(frame.shape[0]),
                'resolution': f'{int(frame.shape[1])}x{int(frame.shape[0])}',
                'frame_count': int(self.frame_count),
                'refresh_rate_hz': refresh_rate_hz,
                'average_fps': float(self.frame_count / elapsed),
            },
            'depth': {
                'enabled': bool(self.enable_depth),
                'topic': self.depth_topic if self.enable_depth else '',
                'encoding': '16UC1',
                'aligned_to_color': bool(self.align_depth_to_color),
                'frame_count': int(self.depth_frame_count),
                **self._depth_stats(depth_frame),
            },
        }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.stream_info_publisher.publish(msg)

        if self.log_video_stats:
            self.get_logger().info(
                'video feed: '
                f'{payload["color"]["resolution"]} @ {refresh_rate_hz:.2f} Hz '
                f'(avg {payload["color"]["average_fps"]:.2f} FPS), '
                f'topic={self.image_topic}, source={self.camera_source}'
            )

        self.last_stream_info_time = now
        self.last_stream_info_frame_count = self.frame_count

    def timer_callback(self):
        ret, frame, depth_frame = self.camera.read()
        if not ret:
            self.get_logger().warning('Failed to read frame')
            return

        self.frame_count += 1
        stamp = self.get_clock().now().to_msg()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp
        self.publisher_.publish(msg)

        if self.depth_publisher is not None and depth_frame is not None:
            self.depth_frame_count += 1
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='16UC1')
            depth_msg.header.stamp = stamp
            self.depth_publisher.publish(depth_msg)

            if self.depth_debug_publisher is not None:
                depth_debug = self._build_depth_debug_image(depth_frame)
                depth_debug_msg = self.bridge.cv2_to_imgmsg(depth_debug, encoding='bgr8')
                depth_debug_msg.header.stamp = stamp
                self.depth_debug_publisher.publish(depth_debug_msg)

        self._publish_stream_info(frame, depth_frame)

    def destroy_node(self):
        if hasattr(self, 'camera') and self.camera is not None:
            self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
