#!/usr/bin/env python3
"""
Static TF Publisher for Camera Transform (Eye-in-Hand Mode)

Publishes the transform from camera_mount_link (on gripper) to camera_link.
The URDF defines camera_mount_link as a child of gripper_link.
This node bridges the mount point to the RealSense camera's TF tree.

Usage:
    ros2 run rx200_handbag_grasp camera_tf_publisher.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import yaml
import os


class CameraTFPublisher(Node):
    """Publishes static camera transform for Eye-in-Hand configuration."""

    def __init__(self):
        super().__init__('camera_tf_publisher')

        # Parameters
        self.declare_parameter('tf_file', '')
        tf_file = self.get_parameter('tf_file').value

        # Default path
        if not tf_file:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('rx200_handbag_grasp')
                tf_file = os.path.join(pkg_share, 'config', 'camera_tf.yaml')
            except Exception:
                pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                tf_file = os.path.join(pkg_dir, 'config', 'camera_tf.yaml')

        self.get_logger().info(f'Loading TF from: {tf_file}')

        if not os.path.exists(tf_file):
            self.get_logger().error(f'TF file not found: {tf_file}')
            self.get_logger().info('Publishing identity transform as default')
            self._publish_default_transform()
            return

        # Load calibration
        with open(tf_file, 'r') as f:
            data = yaml.safe_load(f)

        # Check if using new format (camera_transform) or legacy format (camera_to_lift_base)
        if 'camera_transform' in data:
            # New Eye-in-Hand format
            self.tf_data = data['camera_transform']
            self.parent_frame = self.tf_data['parent_frame']
            self.child_frame = self.tf_data['child_frame']
            self.get_logger().info(f'Eye-in-Hand mode: {self.parent_frame} -> {self.child_frame}')
        elif 'camera_to_lift_base' in data:
            # Legacy Eye-to-Hand format - still support for backwards compatibility
            self.get_logger().warn('Legacy Eye-to-Hand config detected. Consider re-calibrating.')
            self.tf_data = data['camera_to_lift_base']
            self.parent_frame = self.tf_data['parent_frame']
            self.child_frame = 'camera_link'
            self._publish_legacy_transform()
            return
        else:
            self.get_logger().error('Invalid config format')
            self._publish_default_transform()
            return

        # Static broadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish the transform
        self._publish_transform()

    def _publish_transform(self):
        """Publish the camera mount to camera link transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        trans = self.tf_data['translation']
        rot = self.tf_data['rotation_quaternion']

        t.transform.translation.x = float(trans['x'])
        t.transform.translation.y = float(trans['y'])
        t.transform.translation.z = float(trans['z'])

        t.transform.rotation.x = float(rot['x'])
        t.transform.rotation.y = float(rot['y'])
        t.transform.rotation.z = float(rot['z'])
        t.transform.rotation.w = float(rot['w'])

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Published static TF: {self.parent_frame} -> {self.child_frame}'
        )
        self.get_logger().info(
            f'  Translation: [{trans["x"]:.4f}, {trans["y"]:.4f}, {trans["z"]:.4f}]'
        )

    def _publish_default_transform(self):
        """Publish identity transform as default."""
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'rx200/camera_mount_link'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info('Published default identity TF: rx200/camera_mount_link -> camera_link')

    def _publish_legacy_transform(self):
        """Handle legacy Eye-to-Hand configuration."""
        import numpy as np

        # TF Buffer to find connection to root (camera_link)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Timer to retry connecting the tree
        self.timer = self.create_timer(1.0, self._connect_legacy_tree)

    def _connect_legacy_tree(self):
        """Legacy mode: connect camera to lift_base_link."""
        try:
            import numpy as np

            optical_frame = self.tf_data.get('child_frame', 'camera_color_optical_frame')
            camera_root = 'camera_link'

            try:
                t_optical_link = self.tf_buffer.lookup_transform(
                    camera_root,
                    optical_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0)
                )
            except tf2_ros.LookupException:
                self.get_logger().warn(
                    f'Waiting for camera TF: {optical_frame} -> {camera_root}',
                    throttle_duration_sec=2.0
                )
                return

            # Compute and publish transform
            self._publish_legacy_link_transform(t_optical_link)
            self.timer.cancel()
            self.get_logger().info(f'Legacy mode: linked lift_base_link -> {camera_root}')

        except Exception as e:
            self.get_logger().warn(f'TF Connection Error: {e}', throttle_duration_sec=2.0)

    def _publish_legacy_link_transform(self, t_optical_link):
        """Compose and publish legacy transform."""
        import numpy as np

        trans_calib = self.tf_data['translation']
        rot_calib = self.tf_data['rotation_quaternion']

        P_calib = np.array([trans_calib['x'], trans_calib['y'], trans_calib['z']])
        Q_calib = np.array([rot_calib['x'], rot_calib['y'], rot_calib['z'], rot_calib['w']])

        P_ol = np.array([
            t_optical_link.transform.translation.x,
            t_optical_link.transform.translation.y,
            t_optical_link.transform.translation.z
        ])
        Q_ol = np.array([
            t_optical_link.transform.rotation.x,
            t_optical_link.transform.rotation.y,
            t_optical_link.transform.rotation.z,
            t_optical_link.transform.rotation.w
        ])

        P_final = P_calib + self._rotate_vector(Q_calib, P_ol)
        Q_final = self._quaternion_multiply(Q_calib, Q_ol)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.tf_data['parent_frame']
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = P_final[0]
        t.transform.translation.y = P_final[1]
        t.transform.translation.z = P_final[2]
        t.transform.rotation.x = Q_final[0]
        t.transform.rotation.y = Q_final[1]
        t.transform.rotation.z = Q_final[2]
        t.transform.rotation.w = Q_final[3]

        self.static_broadcaster.sendTransform(t)

    def _rotate_vector(self, q, v):
        import numpy as np
        q_vec = q[:3]
        q_w = q[3]
        t = 2.0 * np.cross(q_vec, v)
        return v + q_w * t + np.cross(q_vec, t)

    def _quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ]


def main(args=None):
    rclpy.init(args=args)
    node = CameraTFPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
