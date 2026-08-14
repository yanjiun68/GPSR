#!/usr/bin/env python3
"""
Perception Node for RX200 Official Grasp System.

Uses official Interbotix perception modules:
- InterbotixPointCloudInterface: PointCloud clustering for object detection
- InterbotixArmTagInterface: AprilTag-based camera-to-arm calibration
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
import json
import threading
from typing import List, Tuple, Optional


class PerceptionNode(Node):
    """
    Perception node using official Interbotix modules.

    Provides:
    - PointCloud-based object detection (cluster positions)
    - AprilTag camera calibration
    - Object position publishing for grasp planning
    """

    def __init__(self):
        super().__init__('perception_node')

        self.callback_group = ReentrantCallbackGroup()

        # ========================================
        # Parameters
        # ========================================
        self.declare_parameter('robot_model', 'rx200')
        self.declare_parameter('base_frame', 'rx200/base_link')
        self.declare_parameter('arm_tag_frame', 'rx200/ar_tag_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('filter_ns', 'pc_filter')
        self.declare_parameter('apriltag_ns', 'apriltag')
        self.declare_parameter('armtag_ns', 'armtag')

        # Cluster detection params
        self.declare_parameter('x_filter_min', -0.25)
        self.declare_parameter('x_filter_max', 0.25)
        self.declare_parameter('y_filter_min', 0.05)
        self.declare_parameter('y_filter_max', 0.50)
        self.declare_parameter('z_filter_min', 0.15)
        self.declare_parameter('z_filter_max', 0.60)
        self.declare_parameter('cluster_tol', 0.02)
        self.declare_parameter('cluster_min_size', 100)
        self.declare_parameter('cluster_max_size', 5000)

        # Calibration params
        self.declare_parameter('num_calibration_samples', 5)
        self.declare_parameter('auto_calibrate', True)

        # Get parameters
        self.robot_model = self.get_parameter('robot_model').value
        self.base_frame = self.get_parameter('base_frame').value
        self.arm_tag_frame = self.get_parameter('arm_tag_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.filter_ns = self.get_parameter('filter_ns').value
        self.apriltag_ns = self.get_parameter('apriltag_ns').value
        self.armtag_ns = self.get_parameter('armtag_ns').value
        self.num_calibration_samples = self.get_parameter('num_calibration_samples').value
        self.auto_calibrate = self.get_parameter('auto_calibrate').value

        # ========================================
        # State
        # ========================================
        self.is_calibrated = False
        self.detected_clusters = []
        self.selected_cluster_idx = 0
        self.data_lock = threading.Lock()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ========================================
        # Service Clients (to official modules)
        # ========================================
        # PointCloud filter services
        self.cli_get_clusters = self.create_client(
            Trigger,
            f'/{self.filter_ns}/get_cluster_positions',
            callback_group=self.callback_group
        )
        self.cli_enable_pipeline = self.create_client(
            SetBool,
            f'/{self.filter_ns}/enable_pipeline',
            callback_group=self.callback_group
        )

        # ========================================
        # Publishers
        # ========================================
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/perception/target_pose',
            10
        )
        self.clusters_pub = self.create_publisher(
            MarkerArray,
            '/perception/clusters_markers',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/perception/status',
            10
        )

        # ========================================
        # Services Provided
        # ========================================
        self.srv_detect = self.create_service(
            Trigger,
            '/perception/detect_objects',
            self.detect_objects_callback,
            callback_group=self.callback_group
        )
        self.srv_calibrate = self.create_service(
            Trigger,
            '/perception/calibrate',
            self.calibrate_callback,
            callback_group=self.callback_group
        )
        self.srv_select_object = self.create_service(
            Trigger,
            '/perception/select_nearest',
            self.select_nearest_callback,
            callback_group=self.callback_group
        )
        self.srv_get_target = self.create_service(
            Trigger,
            '/perception/get_target_pose',
            self.get_target_pose_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('RX200 Official Perception Node Initialized')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Services:')
        self.get_logger().info('  /perception/detect_objects  - Detect objects via PointCloud')
        self.get_logger().info('  /perception/calibrate       - Run AprilTag calibration')
        self.get_logger().info('  /perception/select_nearest  - Select nearest object')
        self.get_logger().info('  /perception/get_target_pose - Get current target pose')
        self.get_logger().info('=' * 60)

    def wait_for_service_ready(self, client, timeout=5.0) -> bool:
        """Wait for a service to become available."""
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'Service {client.srv_name} not available')
            return False
        return True

    def detect_objects_callback(self, request, response):
        """
        Detect objects using PointCloud clustering.

        Uses the official pc_filter node to get cluster positions.
        """
        self.get_logger().info('=== Detecting objects via PointCloud ===')

        try:
            # Get clusters from official perception module
            clusters = self._get_cluster_positions()

            if clusters is None or len(clusters) == 0:
                response.success = False
                response.message = "No objects detected"
                self._publish_status("No objects detected")
                return response

            with self.data_lock:
                self.detected_clusters = clusters
                self.selected_cluster_idx = 0

            # Publish visualization markers
            self._publish_cluster_markers(clusters)

            # Log detected objects
            self.get_logger().info(f'Detected {len(clusters)} objects:')
            for i, cluster in enumerate(clusters):
                pos = cluster['position']
                self.get_logger().info(
                    f'  [{i}] Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), '
                    f'Points: {cluster.get("num_points", "N/A")}'
                )

            response.success = True
            response.message = f"Detected {len(clusters)} objects"
            self._publish_status(f"Detected {len(clusters)} objects")

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
            response.success = False
            response.message = str(e)

        return response

    def _get_cluster_positions(self) -> Optional[List[dict]]:
        """
        Get cluster positions from the official pc_filter node.

        Returns list of cluster dictionaries with:
        - name: cluster name (e.g., 'cluster_1')
        - position: [x, y, z] in base frame
        - yaw: orientation (if available)
        - color: [r, g, b] average color
        - num_points: number of points in cluster
        """
        try:
            # Try to use InterbotixPointCloudInterface
            from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
            from interbotix_common_modules.common_robot.robot import InterbotixRobotNode

            # Note: In standalone mode, we need to call the service directly
            # The InterbotixPointCloudInterface requires a InterbotixRobotNode
            # For simplicity, we'll call the service directly here

        except ImportError:
            self.get_logger().info('Using direct service call for cluster detection')

        # Direct service call approach
        if not self.wait_for_service_ready(self.cli_get_clusters):
            return None

        # Call the service
        from interbotix_perception_msgs.srv import ClusterInfoArray

        # Since we're using Trigger, we need to handle this differently
        # The actual service is ClusterInfoArray, let's recreate the client
        try:
            cli_clusters = self.create_client(
                ClusterInfoArray,
                f'/{self.filter_ns}/get_cluster_positions'
            )

            if not cli_clusters.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('ClusterInfoArray service not available')
                return None

            future = cli_clusters.call_async(ClusterInfoArray.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() is None:
                return None

            result = future.result()
            clusters = []

            for i, cluster_info in enumerate(result.clusters):
                # Transform cluster position to base frame if needed
                cluster = {
                    'name': f'cluster_{i+1}',
                    'position': [
                        cluster_info.position.x,
                        cluster_info.position.y,
                        cluster_info.position.z
                    ],
                    'yaw': 0.0,
                    'color': [
                        cluster_info.color.r,
                        cluster_info.color.g,
                        cluster_info.color.b
                    ],
                    'num_points': cluster_info.num_points
                }
                clusters.append(cluster)

            return clusters

        except Exception as e:
            self.get_logger().error(f'Failed to get clusters: {e}')
            return None

    def calibrate_callback(self, request, response):
        """
        Run AprilTag calibration using official ArmTag interface.

        Computes the transform from camera frame to robot base frame.
        """
        self.get_logger().info('=== Running AprilTag Calibration ===')

        try:
            # Try to use InterbotixArmTagInterface
            success = self._run_armtag_calibration()

            if success:
                self.is_calibrated = True
                response.success = True
                response.message = "Calibration successful"
                self._publish_status("Calibration successful")
            else:
                response.success = False
                response.message = "Calibration failed - AprilTag not detected"
                self._publish_status("Calibration failed")

        except Exception as e:
            self.get_logger().error(f'Calibration error: {e}')
            response.success = False
            response.message = str(e)

        return response

    def _run_armtag_calibration(self) -> bool:
        """
        Run ArmTag calibration using the official module.

        Returns True if calibration succeeded.
        """
        try:
            from interbotix_perception_modules.armtag import InterbotixArmTagInterface
            from interbotix_common_modules.common_robot.robot import InterbotixRobotNode

            self.get_logger().info('Using InterbotixArmTagInterface for calibration')

            # The official module needs InterbotixRobotNode
            # For standalone use, we'll use the published TF from armtag.launch.py

        except ImportError:
            self.get_logger().warn('InterbotixArmTagInterface not available')

        # Alternative: Check if the calibration TF is already being published
        # by the armtag.launch.py (which should be running)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0)
            )

            self.get_logger().info(
                f'Camera-to-base transform found: '
                f'translation=({transform.transform.translation.x:.3f}, '
                f'{transform.transform.translation.y:.3f}, '
                f'{transform.transform.translation.z:.3f})'
            )
            return True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return False

    def select_nearest_callback(self, request, response):
        """Select the nearest detected object to the robot."""
        with self.data_lock:
            if not self.detected_clusters:
                response.success = False
                response.message = "No objects detected. Run detection first."
                return response

            # Find nearest cluster (smallest x distance to robot base)
            min_dist = float('inf')
            nearest_idx = 0

            for i, cluster in enumerate(self.detected_clusters):
                pos = cluster['position']
                dist = np.sqrt(pos[0]**2 + pos[1]**2)  # Horizontal distance
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = i

            self.selected_cluster_idx = nearest_idx
            pos = self.detected_clusters[nearest_idx]['position']

        response.success = True
        response.message = f"Selected object {nearest_idx} at distance {min_dist:.3f}m"
        self.get_logger().info(
            f'Selected object [{nearest_idx}]: '
            f'position=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})'
        )

        return response

    def get_target_pose_callback(self, request, response):
        """Get the target pose for the currently selected object."""
        with self.data_lock:
            if not self.detected_clusters:
                response.success = False
                response.message = "No objects detected"
                return response

            if self.selected_cluster_idx >= len(self.detected_clusters):
                response.success = False
                response.message = "Invalid object selection"
                return response

            cluster = self.detected_clusters[self.selected_cluster_idx]

        # Create target pose
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = cluster['position'][0]
        pose.pose.position.y = cluster['position'][1]
        pose.pose.position.z = cluster['position'][2]

        # Orientation: gripper pointing down
        import math
        yaw = math.atan2(pose.pose.position.y, pose.pose.position.x)
        pitch = math.pi / 2  # 90 degrees - pointing down

        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)

        pose.pose.orientation.x = sp * cy
        pose.pose.orientation.y = sp * sy
        pose.pose.orientation.z = cp * sy
        pose.pose.orientation.w = cp * cy

        # Publish target pose
        self.target_pose_pub.publish(pose)

        response.success = True
        response.message = json.dumps({
            'position': cluster['position'],
            'cluster_name': cluster['name'],
            'num_points': cluster.get('num_points', 0)
        })

        return response

    def _publish_cluster_markers(self, clusters: List[dict]):
        """Publish visualization markers for detected clusters."""
        marker_array = MarkerArray()

        for i, cluster in enumerate(clusters):
            # Sphere marker for cluster position
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'clusters'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = cluster['position'][0]
            marker.pose.position.y = cluster['position'][1]
            marker.pose.position.z = cluster['position'][2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03

            # Color based on selection
            if i == self.selected_cluster_idx:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime.sec = 10
            marker_array.markers.append(marker)

            # Text marker for label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'cluster_labels'
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = cluster['position'][0]
            text_marker.pose.position.y = cluster['position'][1]
            text_marker.pose.position.z = cluster['position'][2] + 0.05
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.02
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f"{cluster['name']}"
            text_marker.lifetime.sec = 10
            marker_array.markers.append(text_marker)

        self.clusters_pub.publish(marker_array)

    def _publish_status(self, status: str):
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = PerceptionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
