#!/usr/bin/env python3
"""
Pick and Place Orchestrator Node for RX200 Official Grasp System.

Coordinates the complete pick and place pipeline using official Interbotix modules:
1. AprilTag Calibration (camera-to-arm transform)
2. PointCloud Object Detection (cluster positions)
3. Object Selection (nearest/specific)
4. Grasp Execution (MoveIt)
5. Place Execution
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import time


class PickPlaceNode(Node):
    """
    Main orchestrator for pick and place operations.

    Provides high-level commands that coordinate perception and execution.
    """

    def __init__(self):
        super().__init__('pick_place_node')

        self.callback_group = ReentrantCallbackGroup()

        # ========================================
        # Parameters
        # ========================================
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calibration_interval', 5)
        self.declare_parameter('detection_timeout', 10.0)
        self.declare_parameter('execution_timeout', 60.0)

        self.auto_calibrate = self.get_parameter('auto_calibrate').value
        self.calibration_interval = self.get_parameter('calibration_interval').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.execution_timeout = self.get_parameter('execution_timeout').value

        # ========================================
        # State
        # ========================================
        self.pick_count = 0
        self.is_calibrated = False

        # ========================================
        # Service Clients (to other nodes)
        # ========================================
        # Perception services
        self.cli_detect = self.create_client(
            Trigger, '/perception/detect_objects',
            callback_group=self.callback_group
        )
        self.cli_calibrate = self.create_client(
            Trigger, '/perception/calibrate',
            callback_group=self.callback_group
        )
        self.cli_select_nearest = self.create_client(
            Trigger, '/perception/select_nearest',
            callback_group=self.callback_group
        )
        self.cli_get_target = self.create_client(
            Trigger, '/perception/get_target_pose',
            callback_group=self.callback_group
        )

        # Executor services
        self.cli_go_home = self.create_client(
            Trigger, '/executor/go_home',
            callback_group=self.callback_group
        )
        self.cli_go_observe = self.create_client(
            Trigger, '/executor/go_observe',
            callback_group=self.callback_group
        )
        self.cli_go_sleep = self.create_client(
            Trigger, '/executor/go_sleep',
            callback_group=self.callback_group
        )
        self.cli_open_gripper = self.create_client(
            Trigger, '/executor/open_gripper',
            callback_group=self.callback_group
        )
        self.cli_close_gripper = self.create_client(
            Trigger, '/executor/close_gripper',
            callback_group=self.callback_group
        )
        self.cli_execute_pick = self.create_client(
            Trigger, '/executor/execute_pick',
            callback_group=self.callback_group
        )

        # ========================================
        # Publishers
        # ========================================
        self.status_pub = self.create_publisher(
            String, '/pick_place/status', 10
        )

        # ========================================
        # Services Provided
        # ========================================
        self.srv_run_pick = self.create_service(
            Trigger, '/pick_place/run_pick',
            self.run_pick_callback,
            callback_group=self.callback_group
        )
        self.srv_run_pipeline = self.create_service(
            Trigger, '/pick_place/run_full_pipeline',
            self.run_full_pipeline_callback,
            callback_group=self.callback_group
        )
        self.srv_calibrate = self.create_service(
            Trigger, '/pick_place/calibrate',
            self.calibrate_callback,
            callback_group=self.callback_group
        )
        self.srv_detect = self.create_service(
            Trigger, '/pick_place/detect',
            self.detect_callback,
            callback_group=self.callback_group
        )
        self.srv_prepare = self.create_service(
            Trigger, '/pick_place/prepare',
            self.prepare_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('RX200 Pick and Place Orchestrator Initialized')
        self.get_logger().info('=' * 60)
        self.get_logger().info('High-level services:')
        self.get_logger().info('  /pick_place/run_full_pipeline - Complete pick operation')
        self.get_logger().info('  /pick_place/run_pick          - Pick (needs prior detection)')
        self.get_logger().info('  /pick_place/detect            - Detect objects only')
        self.get_logger().info('  /pick_place/calibrate         - Calibrate camera')
        self.get_logger().info('  /pick_place/prepare           - Prepare for operation')
        self.get_logger().info('=' * 60)

    def call_service(self, client, timeout=10.0):
        """Call a Trigger service and wait for result."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {client.srv_name} not available')
            return False, f"Service {client.srv_name} not available"

        request = Trigger.Request()
        future = client.call_async(request)

        # Wait for result
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            return False, "Service call timeout"

        result = future.result()
        return result.success, result.message

    def publish_status(self, status: str):
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')

    # ========================================
    # Service Callbacks
    # ========================================

    def prepare_callback(self, request, response):
        """Prepare for operation: open gripper and go to observe position."""
        self.publish_status('Preparing for operation...')

        # Open gripper
        success, msg = self.call_service(self.cli_open_gripper)
        if not success:
            self.get_logger().warn(f'Failed to open gripper: {msg}')

        # Go to observe position
        success, msg = self.call_service(self.cli_go_observe)
        if success:
            response.success = True
            response.message = "Ready for operation"
            self.publish_status('Ready for operation')
        else:
            response.success = False
            response.message = f"Failed to prepare: {msg}"

        return response

    def calibrate_callback(self, request, response):
        """Run AprilTag calibration."""
        self.publish_status('Running calibration...')

        success, msg = self.call_service(self.cli_calibrate, timeout=30.0)

        if success:
            self.is_calibrated = True
            response.success = True
            response.message = "Calibration successful"
            self.publish_status('Calibration successful')
        else:
            response.success = False
            response.message = f"Calibration failed: {msg}"
            self.publish_status('Calibration failed')

        return response

    def detect_callback(self, request, response):
        """Run object detection."""
        self.publish_status('Detecting objects...')

        # Detect objects
        success, msg = self.call_service(self.cli_detect, timeout=self.detection_timeout)

        if not success:
            response.success = False
            response.message = f"Detection failed: {msg}"
            return response

        # Select nearest object
        success, msg = self.call_service(self.cli_select_nearest)
        if not success:
            response.success = False
            response.message = f"Object selection failed: {msg}"
            return response

        # Get target pose
        success, msg = self.call_service(self.cli_get_target)
        if not success:
            response.success = False
            response.message = f"Failed to get target pose: {msg}"
            return response

        response.success = True
        response.message = f"Detection complete: {msg}"
        self.publish_status('Detection complete')

        return response

    def run_pick_callback(self, request, response):
        """Run pick operation (assumes detection already done)."""
        self.publish_status('Executing pick...')

        success, msg = self.call_service(self.cli_execute_pick, timeout=self.execution_timeout)

        if success:
            self.pick_count += 1
            response.success = True
            response.message = f"Pick successful (total: {self.pick_count})"
            self.publish_status('Pick successful')
        else:
            response.success = False
            response.message = f"Pick failed: {msg}"
            self.publish_status('Pick failed')

        return response

    def run_full_pipeline_callback(self, request, response):
        """
        Run the complete pick pipeline:
        1. Calibration (if needed)
        2. Prepare (go to observe, open gripper)
        3. Detect objects
        4. Execute pick
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('=== RUNNING FULL PICK PIPELINE ===')
        self.get_logger().info('=' * 60)

        # Step 0: Calibration (if needed)
        need_calibration = (
            self.auto_calibrate and
            (not self.is_calibrated or
             self.pick_count % self.calibration_interval == 0)
        )

        if need_calibration:
            self.publish_status('Phase 0: Calibration')
            self.get_logger().info('\n>>> PHASE 0: AprilTag Calibration')

            success, msg = self.call_service(self.cli_calibrate, timeout=30.0)
            if success:
                self.is_calibrated = True
            else:
                self.get_logger().warn(f'Calibration issue: {msg}')
                # Continue anyway - may work with existing TF

        # Step 1: Prepare
        self.publish_status('Phase 1: Preparation')
        self.get_logger().info('\n>>> PHASE 1: Preparation')

        success, msg = self.call_service(self.cli_open_gripper)
        if not success:
            self.get_logger().warn(f'Failed to open gripper: {msg}')

        success, msg = self.call_service(self.cli_go_observe)
        if not success:
            response.success = False
            response.message = f"Preparation failed: {msg}"
            return response

        time.sleep(1.0)  # Allow camera to stabilize

        # Step 2: Detection
        self.publish_status('Phase 2: Detection')
        self.get_logger().info('\n>>> PHASE 2: Object Detection')

        success, msg = self.call_service(self.cli_detect, timeout=self.detection_timeout)
        if not success:
            response.success = False
            response.message = f"Detection failed: {msg}"
            return response

        # Select nearest
        success, msg = self.call_service(self.cli_select_nearest)
        if not success:
            response.success = False
            response.message = f"Object selection failed: {msg}"
            return response

        # Get target pose
        success, msg = self.call_service(self.cli_get_target)
        if not success:
            response.success = False
            response.message = f"Target pose failed: {msg}"
            return response

        # Step 3: Execute pick
        self.publish_status('Phase 3: Execution')
        self.get_logger().info('\n>>> PHASE 3: Pick Execution')

        success, msg = self.call_service(self.cli_execute_pick, timeout=self.execution_timeout)
        if not success:
            response.success = False
            response.message = f"Pick execution failed: {msg}"
            return response

        self.pick_count += 1

        self.get_logger().info('=' * 60)
        self.get_logger().info('=== PICK PIPELINE COMPLETED ===')
        self.get_logger().info(f'Total picks: {self.pick_count}')
        self.get_logger().info('=' * 60)

        self.publish_status('Pipeline complete')

        response.success = True
        response.message = f"Pick pipeline completed successfully (pick #{self.pick_count})"
        return response


def main(args=None):
    rclpy.init(args=args)

    node = PickPlaceNode()
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
