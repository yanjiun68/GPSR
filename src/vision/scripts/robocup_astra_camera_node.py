#!/usr/bin/env python3

"""Relay Astra camera topics into the vision package's stable RoboCup camera contract."""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class RobocupAstraCameraNode(Node):
    """Adapt Astra driver topics and publish lightweight stream diagnostics."""

    def __init__(self):
        """Create parameters, relays, and periodic diagnostics."""
        super().__init__("robocup_astra_camera_node")

        # ---------- Parameters: input Astra topics ----------
        self.declare_parameter("rgb_in", "/astra/color/image_raw")
        self.declare_parameter("depth_in", "/astra/depth/image_raw")
        self.declare_parameter("rgb_info_in", "/astra/color/camera_info")
        self.declare_parameter("depth_info_in", "/astra/depth/camera_info")

        # ---------- Parameters: output RoboCup topics ----------
        self.declare_parameter("rgb_out", "/camera/image_raw")
        self.declare_parameter("depth_out", "/camera/depth/image_raw")
        self.declare_parameter("rgb_info_out", "/camera/camera_info")
        self.declare_parameter("depth_info_out", "/camera/depth/camera_info")

        # ---------- Parameters: health monitoring ----------
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("frame_timeout_sec", 2.0)

        # ---------- Parameter values: input Astra topics ----------
        self.rgb_in = self.get_parameter("rgb_in").value
        self.depth_in = self.get_parameter("depth_in").value
        self.rgb_info_in = self.get_parameter("rgb_info_in").value
        self.depth_info_in = self.get_parameter("depth_info_in").value

        # ---------- Parameter values: output RoboCup topics ----------
        self.rgb_out = self.get_parameter("rgb_out").value
        self.depth_out = self.get_parameter("depth_out").value
        self.rgb_info_out = self.get_parameter("rgb_info_out").value
        self.depth_info_out = self.get_parameter("depth_info_out").value

        # ---------- Parameter values: health monitoring ----------
        self.diagnostics_topic = self.get_parameter("diagnostics_topic").value
        self.frame_timeout_sec = float(self.get_parameter("frame_timeout_sec").value)

        # ---------- Runtime state: stream timestamps and counters ----------
        self.last_rgb_time = None
        self.last_depth_time = None
        self.rgb_count = 0
        self.depth_count = 0

        # ---------- Publishers: normalized RoboCup camera outputs ----------
        self.rgb_pub = self.create_publisher(Image, self.rgb_out, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, self.depth_out, qos_profile_sensor_data)
        self.rgb_info_pub = self.create_publisher(CameraInfo, self.rgb_info_out, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, self.depth_info_out, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, self.diagnostics_topic, 10)

        # ---------- Subscribers: Astra inputs and camera-info relays ----------
        self.create_subscription(Image, self.rgb_in, self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_in, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.rgb_info_in, self.rgb_info_pub.publish, 10)
        self.create_subscription(CameraInfo, self.depth_info_in, self.depth_info_pub.publish, 10)

        # ---------- Timer: periodic health status ----------
        self.create_timer(1.0, self.publish_diagnostics)

        # ---------- Startup logs: visible topic contract ----------
        self.get_logger().info(f"RGB relay: {self.rgb_in} -> {self.rgb_out}")
        self.get_logger().info(f"Depth relay: {self.depth_in} -> {self.depth_out}")

    def on_rgb(self, msg):
        """Relay one RGB image and update RGB stream health."""
        self.last_rgb_time = self.get_clock().now()
        self.rgb_count += 1
        self.rgb_pub.publish(msg)

    def on_depth(self, msg):
        """Relay one depth image and update depth stream health."""
        self.last_depth_time = self.get_clock().now()
        self.depth_count += 1
        self.depth_pub.publish(msg)

    def stream_ok(self, last_time):
        """Return whether a stream has delivered a frame recently."""
        if last_time is None:
            return False
        age_sec = (self.get_clock().now() - last_time).nanoseconds / 1e9
        return age_sec <= self.frame_timeout_sec

    def publish_diagnostics(self):
        """Publish a diagnostic summary for the normalized camera feed."""
        rgb_ok = self.stream_ok(self.last_rgb_time)
        depth_ok = self.stream_ok(self.last_depth_time)

        status = DiagnosticStatus()
        status.name = "robocup_astra_camera"
        status.hardware_id = "astra_camera"
        status.level = DiagnosticStatus.OK if rgb_ok and depth_ok else DiagnosticStatus.WARN
        status.message = (
            "RGB and depth active"
            if rgb_ok and depth_ok
            else "Waiting for RGB/depth frames"
        )
        status.values = [
            KeyValue(key="rgb_frames", value=str(self.rgb_count)),
            KeyValue(key="depth_frames", value=str(self.depth_count)),
            KeyValue(key="rgb_ok", value=str(rgb_ok)),
            KeyValue(key="depth_ok", value=str(depth_ok)),
        ]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(status)
        self.diag_pub.publish(msg)


def main(args=None):
    """Run the RoboCup Astra camera adapter node."""
    rclpy.init(args=args)
    node = RobocupAstraCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
