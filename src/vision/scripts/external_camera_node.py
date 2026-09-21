#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
# Section: ROS 2 external camera publisher entry point.

"""Publish frames from an external OpenCV camera as ROS 2 Image messages."""

# Section: Standard library imports.
from typing import Any

# Section: OpenCV import for camera access.
import cv2

# Section: ROS 2 Python client imports.
import rclpy
from rclpy.node import Node

# Section: ROS message imports.
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

# Section: OpenCV-to-ROS image conversion import.
from cv_bridge import CvBridge


class ExternalCameraNode(Node):
    """Open an external camera and publish its frames as ROS 2 image topics."""

    # Section: Node construction and camera setup.
    def __init__(self) -> None:
        # Special function: initialize the base ROS node with a stable runtime name.
        super().__init__('external_camera_node')

        # Section: Runtime parameters for selecting and configuring the camera.
        self.declare_parameter('camera_device', '')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('backend', 'v4l2')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)

        # Section: Runtime parameters for ROS topic naming and frame metadata.
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'external_camera_link')
        self.declare_parameter('publish_camera_info', True)

        # Section: Parameter values cached for low-overhead timer callbacks.
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.publish_camera_info = bool(self.get_parameter('publish_camera_info').value)
        self.requested_width = int(self.get_parameter('width').value)
        self.requested_height = int(self.get_parameter('height').value)
        self.requested_fps = max(1.0, float(self.get_parameter('fps').value))

        # Section: ROS helpers and publishers.
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        # Section: OpenCV camera creation and validation.
        self.capture = self._open_camera()
        self._configure_camera()
        self._verify_camera_opened()

        # Section: Periodic frame publishing timer.
        self.timer = self.create_timer(1.0 / self.requested_fps, self.publish_frame)

        # Section: Startup diagnostics for operator visibility.
        self.get_logger().info(
            f'Publishing external camera frames on {self.image_topic} '
            f'at {self.requested_width}x{self.requested_height}@{self.requested_fps:.1f}fps'
        )

    # Section: Camera source resolution helper.
    def _camera_source(self) -> Any:
        # Special function: use an explicit device path when provided.
        camera_device = str(self.get_parameter('camera_device').value).strip()

        # Section: Device path selection.
        if camera_device:
            return camera_device

        # Section: Numeric camera index fallback.
        return int(self.get_parameter('camera_index').value)

    # Section: OpenCV backend resolution helper.
    def _backend_id(self) -> int:
        # Special function: map a friendly backend parameter to an OpenCV constant.
        backend = str(self.get_parameter('backend').value).strip().lower()

        # Section: Linux V4L2 backend selection for USB webcams.
        if backend == 'v4l2':
            return cv2.CAP_V4L2

        # Section: Default backend selection for non-Linux or auto detection.
        return cv2.CAP_ANY

    # Section: OpenCV camera creation helper.
    def _open_camera(self) -> cv2.VideoCapture:
        # Special function: create the OpenCV capture handle for the configured source.
        source = self._camera_source()

        # Section: Camera handle creation.
        capture = cv2.VideoCapture(source, self._backend_id())

        # Section: Return the handle so initialization can configure and validate it.
        return capture

    # Section: Camera property configuration helper.
    def _configure_camera(self) -> None:
        # Special function: request camera width when a positive value is configured.
        if self.requested_width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.requested_width)

        # Special function: request camera height when a positive value is configured.
        if self.requested_height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.requested_height)

        # Special function: request camera FPS when a positive value is configured.
        if self.requested_fps > 0:
            self.capture.set(cv2.CAP_PROP_FPS, self.requested_fps)

    # Section: Camera validation helper.
    def _verify_camera_opened(self) -> None:
        # Special function: fail fast if OpenCV could not open the external camera.
        if self.capture.isOpened():
            return

        # Section: Error message includes both supported camera selection parameters.
        source = self._camera_source()
        raise RuntimeError(f'Unable to open external camera source: {source}')

    # Section: CameraInfo message creation helper.
    def _camera_info_msg(self, image_msg: Image) -> CameraInfo:
        # Special function: publish basic camera metadata without calibration data.
        camera_info = CameraInfo()

        # Section: Header mirrors the image timestamp and frame id.
        camera_info.header = image_msg.header

        # Section: Dimensions mirror the actual published image.
        camera_info.width = image_msg.width
        camera_info.height = image_msg.height

        # Section: Distortion model is empty because this node does not load calibration.
        camera_info.distortion_model = ''

        # Section: Return the generated camera info message.
        return camera_info

    # Section: Timer callback for frame capture and publishing.
    def publish_frame(self) -> None:
        # Special function: capture one frame from the external camera.
        ok, frame = self.capture.read()

        # Section: Read failure handling keeps the node alive for transient USB stalls.
        if not ok or frame is None:
            self.get_logger().warn('Failed to read frame from external camera', throttle_duration_sec=2.0)
            return

        # Section: Convert the OpenCV BGR frame into a ROS Image message.
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # Section: Stamp the image with the current ROS time.
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id

        # Section: Publish the raw image frame.
        self.image_pub.publish(image_msg)

        # Section: Optionally publish matching CameraInfo for consumers that require it.
        if self.publish_camera_info:
            self.camera_info_pub.publish(self._camera_info_msg(image_msg))

    # Section: Node shutdown cleanup.
    def destroy_node(self) -> None:
        # Special function: release the camera before tearing down the ROS node.
        if hasattr(self, 'capture') and self.capture is not None:
            self.capture.release()

        # Section: Delegate the remaining cleanup to rclpy.
        return super().destroy_node()


# Section: Process entry point.
def main(args: list[str] | None = None) -> None:
    # Special function: initialize the ROS 2 Python runtime.
    rclpy.init(args=args)

    # Section: Node lifecycle variable allows cleanup from finally.
    node: ExternalCameraNode | None = None

    # Section: Spin the node until shutdown or failure.
    try:
        node = ExternalCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Special function: destroy the node if initialization completed.
        if node is not None:
            node.destroy_node()

        # Special function: shutdown rclpy only while the context is valid.
        if rclpy.ok():
            rclpy.shutdown()


# Section: Script execution guard.
if __name__ == '__main__':
    main()
