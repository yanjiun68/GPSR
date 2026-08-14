#!/usr/bin/env python3
"""
Grasp Executor Node for RX200 Official Grasp System.

Uses MoveIt for motion planning and the Interbotix gripper interface.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
    BoundingVolume
)
from shape_msgs.msg import SolidPrimitive

import time
import threading


class GraspExecutorNode(Node):
    """
    MoveIt-based grasp execution for RX200.

    Provides motion primitives:
    - Move to joint positions (home, sleep, observe)
    - Move to Cartesian pose
    - Gripper control (open, close)
    - Full grasp sequence
    """

    def __init__(self):
        super().__init__('grasp_executor')

        self.callback_group = ReentrantCallbackGroup()

        # ========================================
        # Parameters
        # ========================================
        self.declare_parameter('arm_group', 'interbotix_arm')
        self.declare_parameter('gripper_group', 'interbotix_gripper')
        self.declare_parameter('ee_link', 'rx200/ee_gripper_link')
        self.declare_parameter('base_frame', 'rx200/base_link')

        # Motion parameters
        self.declare_parameter('max_velocity_scaling', 0.3)
        self.declare_parameter('max_acceleration_scaling', 0.3)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 10)

        # Gripper parameters
        self.declare_parameter('gripper_open', 0.037)
        self.declare_parameter('gripper_close', 0.015)
        self.declare_parameter('gripper_grasp', 0.020)

        # Grasp parameters
        self.declare_parameter('approach_distance', 0.08)
        self.declare_parameter('lift_height', 0.10)

        # Get parameters
        self.arm_group = self.get_parameter('arm_group').value
        self.gripper_group = self.get_parameter('gripper_group').value
        self.ee_link = self.get_parameter('ee_link').value
        self.base_frame = self.get_parameter('base_frame').value

        self.max_vel_scale = self.get_parameter('max_velocity_scaling').value
        self.max_acc_scale = self.get_parameter('max_acceleration_scaling').value
        self.planning_time = self.get_parameter('planning_time').value
        self.num_attempts = self.get_parameter('num_planning_attempts').value

        self.gripper_open_pos = self.get_parameter('gripper_open').value
        self.gripper_close_pos = self.get_parameter('gripper_close').value
        self.gripper_grasp_pos = self.get_parameter('gripper_grasp').value

        self.approach_distance = self.get_parameter('approach_distance').value
        self.lift_height = self.get_parameter('lift_height').value

        # ========================================
        # Joint configurations
        # ========================================
        self.arm_joints = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        self.gripper_joint = 'left_finger'

        # Predefined poses [waist, shoulder, elbow, wrist_angle, wrist_rotate]
        self.poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],
            'sleep': [0.0, -1.8, 1.55, 0.8, 0.0],
            'observe': [0.0, 0.3, 0.3, -0.3, 0.0],
            'upright': [0.0, 0.0, -1.57, 0.0, 0.0],
        }

        # ========================================
        # State
        # ========================================
        self._goal_complete = False
        self._goal_success = False
        self._lock = threading.Lock()

        # Target pose storage
        self.target_pose = None
        self.pregrasp_pose = None

        # ========================================
        # MoveIt Action Client
        # ========================================
        self._move_action_client = ActionClient(
            self, MoveGroup, 'move_action',
            callback_group=self.callback_group
        )

        # ========================================
        # Subscribers
        # ========================================
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/perception/target_pose',
            self.target_pose_callback,
            10
        )

        # ========================================
        # Services
        # ========================================
        self.srv_go_home = self.create_service(
            Trigger, '/executor/go_home', self.go_home_callback,
            callback_group=self.callback_group
        )
        self.srv_go_sleep = self.create_service(
            Trigger, '/executor/go_sleep', self.go_sleep_callback,
            callback_group=self.callback_group
        )
        self.srv_go_observe = self.create_service(
            Trigger, '/executor/go_observe', self.go_observe_callback,
            callback_group=self.callback_group
        )
        self.srv_open_gripper = self.create_service(
            Trigger, '/executor/open_gripper', self.open_gripper_callback,
            callback_group=self.callback_group
        )
        self.srv_close_gripper = self.create_service(
            Trigger, '/executor/close_gripper', self.close_gripper_callback,
            callback_group=self.callback_group
        )
        self.srv_execute_grasp = self.create_service(
            Trigger, '/executor/execute_grasp', self.execute_grasp_callback,
            callback_group=self.callback_group
        )
        self.srv_execute_pick = self.create_service(
            Trigger, '/executor/execute_pick', self.execute_pick_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('RX200 Grasp Executor Node Initialized')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for MoveGroup action server...')

        if self._move_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('MoveGroup action server available!')
        else:
            self.get_logger().warn('MoveGroup server not available')

    def target_pose_callback(self, msg: PoseStamped):
        """Store the target pose from perception."""
        self.target_pose = msg

        # Compute pre-grasp pose (above target)
        self.pregrasp_pose = PoseStamped()
        self.pregrasp_pose.header = msg.header
        self.pregrasp_pose.pose = msg.pose
        self.pregrasp_pose.pose.position.z += self.approach_distance

        self.get_logger().info(
            f'Received target pose: ({msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})'
        )

    # ========================================
    # Motion Primitives
    # ========================================

    def move_to_joint_positions(self, joint_positions, timeout=30.0) -> bool:
        """Move arm to specified joint positions."""
        if not self._move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        self._goal_complete = False
        self._goal_success = False

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.arm_group
        request.num_planning_attempts = self.num_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.max_vel_scale
        request.max_acceleration_scaling_factor = self.max_acc_scale

        # Joint constraints
        constraints = Constraints()
        for name, value in zip(self.arm_joints, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False

        future = self._move_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        # Wait for completion
        start = time.time()
        while not self._goal_complete and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._goal_success

    def move_gripper(self, position, timeout=10.0) -> bool:
        """Move gripper to specified position."""
        if not self._move_action_client.wait_for_server(timeout_sec=5.0):
            return False

        self._goal_complete = False
        self._goal_success = False

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.gripper_group
        request.num_planning_attempts = 5
        request.allowed_planning_time = 2.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = position
        jc.tolerance_above = 0.005
        jc.tolerance_below = 0.005
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False

        future = self._move_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        start = time.time()
        while not self._goal_complete and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._goal_success

    def move_to_pose(self, pose: PoseStamped, timeout=30.0) -> bool:
        """Move arm end-effector to specified pose."""
        if not self._move_action_client.wait_for_server(timeout_sec=5.0):
            return False

        self._goal_complete = False
        self._goal_success = False

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.arm_group
        request.num_planning_attempts = self.num_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.max_vel_scale
        request.max_acceleration_scaling_factor = self.max_acc_scale

        # Position constraint
        constraints = Constraints()

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = self.ee_link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        # Bounding region (sphere)
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm tolerance
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        request.goal_constraints.append(constraints)
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False

        future = self._move_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        start = time.time()
        while not self._goal_complete and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._goal_success

    def _goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self._goal_complete = True
            self._goal_success = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Handle result."""
        result = future.result()
        error_code = result.result.error_code.val

        self._goal_complete = True
        self._goal_success = (error_code == MoveItErrorCodes.SUCCESS)

        if not self._goal_success:
            self.get_logger().warn(f'Motion failed with error code: {error_code}')

    # ========================================
    # Service Callbacks
    # ========================================

    def go_home_callback(self, request, response):
        """Go to home position."""
        self.get_logger().info('Moving to home position...')
        success = self.move_to_joint_positions(self.poses['home'])
        response.success = success
        response.message = "Moved to home" if success else "Failed to move to home"
        return response

    def go_sleep_callback(self, request, response):
        """Go to sleep position."""
        self.get_logger().info('Moving to sleep position...')
        success = self.move_to_joint_positions(self.poses['sleep'])
        response.success = success
        response.message = "Moved to sleep" if success else "Failed"
        return response

    def go_observe_callback(self, request, response):
        """Go to observe position."""
        self.get_logger().info('Moving to observe position...')
        success = self.move_to_joint_positions(self.poses['observe'])
        response.success = success
        response.message = "Moved to observe" if success else "Failed"
        return response

    def open_gripper_callback(self, request, response):
        """Open gripper."""
        self.get_logger().info('Opening gripper...')
        success = self.move_gripper(self.gripper_open_pos)
        response.success = success
        response.message = "Gripper opened" if success else "Failed"
        return response

    def close_gripper_callback(self, request, response):
        """Close gripper."""
        self.get_logger().info('Closing gripper...')
        success = self.move_gripper(self.gripper_close_pos)
        response.success = success
        response.message = "Gripper closed" if success else "Failed"
        return response

    def execute_grasp_callback(self, request, response):
        """Execute grasp at current target pose."""
        if self.target_pose is None:
            response.success = False
            response.message = "No target pose available"
            return response

        self.get_logger().info('=== Executing Grasp ===')

        # Move to grasp pose
        self.get_logger().info('Moving to grasp pose...')
        if not self.move_to_pose(self.target_pose):
            response.success = False
            response.message = "Failed to move to grasp pose"
            return response

        # Close gripper
        self.get_logger().info('Closing gripper...')
        if not self.move_gripper(self.gripper_grasp_pos):
            response.success = False
            response.message = "Failed to close gripper"
            return response

        response.success = True
        response.message = "Grasp executed"
        return response

    def execute_pick_callback(self, request, response):
        """Execute full pick sequence: approach -> grasp -> lift."""
        if self.target_pose is None or self.pregrasp_pose is None:
            response.success = False
            response.message = "No target pose available"
            return response

        self.get_logger().info('=' * 50)
        self.get_logger().info('=== Executing Pick Sequence ===')
        self.get_logger().info('=' * 50)

        # Step 1: Open gripper
        self.get_logger().info('Step 1: Opening gripper...')
        if not self.move_gripper(self.gripper_open_pos):
            response.success = False
            response.message = "Failed to open gripper"
            return response

        # Step 2: Move to pre-grasp
        self.get_logger().info('Step 2: Moving to pre-grasp pose...')
        if not self.move_to_pose(self.pregrasp_pose):
            response.success = False
            response.message = "Failed to move to pre-grasp"
            return response

        # Step 3: Move to grasp
        self.get_logger().info('Step 3: Moving to grasp pose...')
        if not self.move_to_pose(self.target_pose):
            response.success = False
            response.message = "Failed to move to grasp pose"
            return response

        # Step 4: Close gripper
        self.get_logger().info('Step 4: Closing gripper...')
        if not self.move_gripper(self.gripper_grasp_pos):
            response.success = False
            response.message = "Failed to close gripper"
            return response

        time.sleep(0.5)  # Brief pause

        # Step 5: Lift
        self.get_logger().info('Step 5: Lifting...')
        lift_pose = PoseStamped()
        lift_pose.header = self.target_pose.header
        lift_pose.pose = self.target_pose.pose
        lift_pose.pose.position.z += self.lift_height

        if not self.move_to_pose(lift_pose):
            response.success = False
            response.message = "Failed to lift"
            return response

        # Step 6: Return to home
        self.get_logger().info('Step 6: Returning to home...')
        if not self.move_to_joint_positions(self.poses['home']):
            response.success = False
            response.message = "Failed to return to home"
            return response

        self.get_logger().info('=' * 50)
        self.get_logger().info('=== Pick Sequence Complete! ===')
        self.get_logger().info('=' * 50)

        response.success = True
        response.message = "Pick sequence completed successfully"
        return response


def main(args=None):
    rclpy.init(args=args)

    node = GraspExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
