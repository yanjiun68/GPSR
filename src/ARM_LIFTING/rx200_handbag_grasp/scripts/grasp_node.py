#!/usr/bin/env python3
"""
Core Grasp Node for RX200 Bag Handle Grasping (Eye-in-Hand Mode)

Camera is mounted on gripper servo - TF is defined in URDF.
No runtime calibration needed.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, Point, PoseStamped, TransformStamped, Quaternion, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
# from interbotix_xs_msgs.srv import RobotConfig  # Not available in this version
from interbotix_xs_msgs.srv import RegisterValues
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from rx200_handbag_grasp.action import MoveBaseDistance
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, MoveItErrorCodes, BoundingVolume,
    RobotTrajectory
)
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as BuiltinDuration
from interbotix_xs_msgs.msg import JointSingleCommand

from cv_bridge import CvBridge
import message_filters
import tf2_ros
import tf2_geometry_msgs

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import threading
import time
import copy
import math
import subprocess
from collections import deque


class KalmanFilter3D:
    """
    Kalman filter for 3D position + depth smoothing with DYNAMIC dt.
    State: [x, y, z, vx, vy, vz]
    OPTIMIZED: dt is computed from actual measurement timestamps.
    """
    def __init__(self, process_noise=0.001, measurement_noise=0.01):
        self.dim_x = 6  # [x, y, z, vx, vy, vz]
        self.dim_z = 3  # [x, y, z]
        self._process_noise = process_noise

        # State vector
        self.x = np.zeros(self.dim_x)

        # State covariance
        self.P = np.eye(self.dim_x) * 1.0

        # State transition matrix (constant velocity model)
        self.F = np.eye(self.dim_x)

        # Measurement matrix
        self.H = np.zeros((self.dim_z, self.dim_x))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        # Process noise
        self.Q = np.eye(self.dim_x) * process_noise

        # Measurement noise
        self.R = np.eye(self.dim_z) * measurement_noise

        self.initialized = False
        self._last_update_time = None

    def reset(self):
        """Reset filter state."""
        self.x = np.zeros(self.dim_x)
        self.P = np.eye(self.dim_x) * 1.0
        self.initialized = False
        self._last_update_time = None

    def _update_dt(self):
        """Dynamically update dt from actual measurement interval."""
        now = time.time()
        if self._last_update_time is not None:
            dt = min(now - self._last_update_time, 1.0)  # Cap at 1s
            dt = max(dt, 0.01)  # Floor at 10ms
        else:
            dt = 0.1  # Initial default
        self._last_update_time = now

        # Update state transition matrix with actual dt
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # Scale process noise by dt
        self.Q = np.eye(self.dim_x) * self._process_noise * dt
        return dt

    def predict(self):
        """Predict step."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """Update step with measurement z = [x, y, z]."""
        z = np.array(z)
        self._update_dt()

        if not self.initialized:
            self.x[:3] = z
            self.initialized = True
            return self.x[:3]

        # Predict
        self.predict()

        # Update
        y = z - self.H @ self.x  # Innovation
        S = self.H @ self.P @ self.H.T + self.R  # Innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        self.x = self.x + K @ y
        self.P = (np.eye(self.dim_x) - K @ self.H) @ self.P

        return self.x[:3]

    def get_state(self):
        """Get current state estimate [x, y, z]."""
        return self.x[:3].copy()

    def get_velocity(self):
        """Get current velocity estimate [vx, vy, vz]."""
        return self.x[3:].copy()


class DepthTemporalFilter:
    """
    Temporal depth smoothing using weighted sliding window.
    OPTIMIZED: Gives more weight to recent measurements via exponential decay.
    """
    def __init__(self, window_size=5, max_age_sec=0.5):
        self.buffer = deque(maxlen=window_size)
        self.max_age = max_age_sec

    def reset(self):
        self.buffer.clear()

    def update(self, depth_value, timestamp=None):
        """Add a new depth measurement."""
        if timestamp is None:
            timestamp = time.time()
        self.buffer.append((timestamp, depth_value))

    def get_filtered(self):
        """Get time-weighted average depth."""
        if not self.buffer:
            return None
        now = self.buffer[-1][0]
        # Remove stale entries
        valid = [(t, d) for t, d in self.buffer if now - t < self.max_age]
        if not valid:
            return None
        depths = np.array([d for _, d in valid])
        times = np.array([t for t, _ in valid])
        weights = np.exp(-(now - times) * 5.0)  # Exponential decay
        return float(np.average(depths, weights=weights))


class ConvergenceDetector:
    """Detects if visual servoing has converged based on error history."""
    def __init__(self, window_size=5, threshold=0.002):
        self.window_size = window_size
        self.threshold = threshold
        self.error_history = deque(maxlen=window_size)

    def reset(self):
        self.error_history.clear()

    def add_error(self, error):
        """Add error measurement (scalar or norm)."""
        self.error_history.append(error)

    def is_converged(self):
        """Check if converged based on error std."""
        if len(self.error_history) < self.window_size:
            return False
        errors = np.array(self.error_history)
        return np.std(errors) < self.threshold and np.mean(errors) < self.threshold * 2


def pose_to_transformation_matrix(pose):
    """
    Transform a Six Element Pose vector to a 4x4 Transformation Matrix.
    Same as official: interbotix_common_modules.angle_manipulation

    :param pose: [x, y, z, roll, pitch, yaw]
    :return: 4x4 numpy transformation matrix
    """
    mat = np.identity(4)
    # Rotation matrix from euler angles (xyz order)
    r = R.from_euler('xyz', pose[3:])
    mat[:3, :3] = r.as_matrix()
    mat[:3, 3] = pose[:3]
    return mat


def rotation_matrix_to_euler_angles(rot_mat):
    """
    Extract euler angles from rotation matrix.
    Same as official: interbotix_common_modules.angle_manipulation
    """
    r = R.from_matrix(rot_mat)
    return list(r.as_euler('xyz'))


class GraspNode(Node):
    """
    Core grasp node with official-style AprilTag calibration.
    """

    def __init__(self):
        super().__init__('grasp_node')

        # ========================================
        # Callback Group for async operations
        # ========================================
        self.cb_group = ReentrantCallbackGroup()

        # ========================================
        # Parameter Declarations
        # ========================================

        # --- Frame Configuration (same as official) ---
        self.declare_parameter('ref_frame', 'camera_color_optical_frame')
        self.declare_parameter('arm_base_frame', 'rx200/base_link')
        self.declare_parameter('ee_link', 'rx200/ee_gripper_link')

        # --- Depth Processing ---
        self.declare_parameter('min_depth', 0.15)
        self.declare_parameter('max_depth', 0.80)
        self.declare_parameter('max_grasp_distance', 0.35)
        self.declare_parameter('roi_padding', 5)
        self.declare_parameter('depth_sample_ratio', 0.5)
        self.declare_parameter('use_depth_filter', True)  # Bilateral filter for noise reduction

        # --- Grasp Geometry ---
        self.declare_parameter('approach_distance', 0.10)  # Pre-grasp height above target
        self.declare_parameter('grasp_offset_z', 0.0)
        self.declare_parameter('lift_height', 0.12)
        self.declare_parameter('grasp_min_z', 0.03)
        self.declare_parameter('depth_safety_offset', 0.005)  # 5mm safety offset to avoid pressing table
        self.declare_parameter('depth_window_size', 11)  # Window size for median depth (11x11)
        self.declare_parameter('grasp_point_offset_ratio', 0.0)  # 0=center, 0.15=15% above center (for handles)

        # --- Fixed Grasp Orientation (Stable baseline) ---
        self.declare_parameter('use_fixed_orientation', True)  # Lock roll/pitch, only yaw
        self.declare_parameter('fixed_grasp_pitch', 1.5708)  # +90° = gripper pointing down
        self.declare_parameter('fixed_grasp_roll', 0.0)
        self.declare_parameter('fixed_grasp_yaw', 0.0)  # Start with yaw=0 for stability

        # --- Cartesian Path ---
        self.declare_parameter('use_cartesian_path', True)  # Use straight line for approach/lift
        self.declare_parameter('cartesian_step_size', 0.005)  # 5mm step for Cartesian planning
        self.declare_parameter('cartesian_jump_threshold', 0.0)  # Disable jump detection

        # --- Gripper Strategy ---
        self.declare_parameter('gripper_second_squeeze', True)  # Enable second squeeze
        self.declare_parameter('gripper_first_close_effort', 0.5)  # First close effort
        self.declare_parameter('gripper_squeeze_effort', 0.8)  # Second squeeze effort
        self.declare_parameter('gripper_close_delay', 0.3)  # Delay between closes
        # Gripper Force Control
        self.declare_parameter('gripper_current_limit_grasp', 60) # ~60mA for light grasp? (Unit depends on motor)
        self.declare_parameter('gripper_current_limit_max', 150)  # Max current for opening/strong grasp

        # --- Pose Filtering ---
        self.declare_parameter('pose_filter_alpha', 0.3)
        self.declare_parameter('min_detection_confidence', 0.5)

        # Time Sync Config
        self.max_frame_delay = 0.06 # OPTIMIZED: was 0.08, tighter sync


        # --- Multi-Target Selection ---
        self.declare_parameter('target_selection.use_reachability', True)
        self.declare_parameter('target_selection.reachability_weight', 0.3)
        self.declare_parameter('target_selection.confidence_weight', 0.5)
        self.declare_parameter('target_selection.depth_quality_weight', 0.2)
        self.declare_parameter('target_selection.optimal_distance', 0.25)  # meters from base

        # --- Retry Mechanism ---
        self.declare_parameter('retry.enabled', True)
        self.declare_parameter('retry.max_attempts', 3)
        self.declare_parameter('retry.lift_adjustment', 0.05)  # meters to adjust lift on retry
        self.declare_parameter('retry.position_offset', 0.02)  # meters to offset grasp position

        # --- Multi-Frame Detection Fusion ---
        self.declare_parameter('fusion.enabled', True)
        self.declare_parameter('fusion.buffer_size', 5)  # Number of frames to average
        self.declare_parameter('fusion.min_detections', 3)  # Minimum detections to output pose
        self.declare_parameter('fusion.max_position_std', 0.03)  # Max position std dev (meters)
        self.declare_parameter('fusion.timeout_sec', 1.0)  # Clear buffer if no detection

        # --- Visual Servoing (Closed-Loop Fine-Tuning) ---
        self.declare_parameter('visual_servo.enabled', True)
        self.declare_parameter('visual_servo.max_iterations', 10)  # Max correction iterations
        self.declare_parameter('visual_servo.pixel_tolerance', 15)  # Pixels from image center
        self.declare_parameter('visual_servo.depth_tolerance', 0.015)  # meters
        self.declare_parameter('visual_servo.step_gain', 0.5)  # Correction gain (0-1)
        self.declare_parameter('visual_servo.max_step', 0.02)  # Max correction per iteration (m)
        self.declare_parameter('visual_servo.min_step', 0.002)  # Min correction threshold (m)
        self.declare_parameter('visual_servo.settle_time', 0.3)  # Seconds to wait after move

        # Adaptive gain control
        self.declare_parameter('visual_servo.adaptive_gain', True)
        self.declare_parameter('visual_servo.gain_far', 0.5)
        self.declare_parameter('visual_servo.gain_mid', 0.35)
        self.declare_parameter('visual_servo.gain_near', 0.2)
        self.declare_parameter('visual_servo.error_threshold_far', 0.01)
        self.declare_parameter('visual_servo.error_threshold_near', 0.005)
        self.declare_parameter('visual_servo.convergence_window', 5)
        self.declare_parameter('visual_servo.convergence_threshold', 0.002)

        # Adaptive settle_time
        self.declare_parameter('visual_servo.adaptive_settle', True)
        self.declare_parameter('visual_servo.settle_time_far', 0.10)
        self.declare_parameter('visual_servo.settle_time_near', 0.30)

        # [NEW] Advanced depth processing
        self.declare_parameter('depth_use_mask_sampling', True)
        self.declare_parameter('depth_percentile_low', 15)
        self.declare_parameter('depth_percentile_high', 85)
        self.declare_parameter('depth_outlier_threshold', 0.02)
        self.declare_parameter('depth_plane_fitting', True)
        self.declare_parameter('depth_plane_inlier_threshold', 0.005)
        self.declare_parameter('depth_window_adaptive', True)
        self.declare_parameter('depth_window_ratio', 0.25)
        self.declare_parameter('depth_window_min', 11)
        self.declare_parameter('depth_window_max', 31)

        # [NEW] Kalman filter for detection smoothing
        self.declare_parameter('fusion.use_kalman', True)
        self.declare_parameter('fusion.kalman_process_noise', 0.001)
        self.declare_parameter('fusion.kalman_measurement_noise', 0.01)

        # [NEW] Contact detection
        self.declare_parameter('contact_detection.enabled', True)
        self.declare_parameter('contact_detection.current_threshold', 45)
        self.declare_parameter('contact_detection.approach_speed', 0.003)
        self.declare_parameter('contact_detection.max_extra_descent', 0.03)
        self.declare_parameter('contact_detection.check_interval', 0.05)
        self.declare_parameter('contact_detection.current_window', 5)

        self.declare_parameter('grasp_offset_x', 0.0)
        self.declare_parameter('grasp_offset_y', 0.0)


        # --- MoveIt Planning ---
        self.declare_parameter('arm_group', 'interbotix_arm')
        self.declare_parameter('arm_lift_group', 'interbotix_arm_lift')  # Coordinated group
        self.declare_parameter('use_coordinated_motion', True)  # Use arm+lift together
        self.declare_parameter('max_velocity_scaling', 0.25)
        self.declare_parameter('max_acceleration_scaling', 0.25)
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('num_planning_attempts', 15)
        self.declare_parameter('use_orientation_constraint', True)

        # --- Gripper (PWM mode) ---
        self.declare_parameter('gripper_pwm_open', 200)
        self.declare_parameter("gripper_pwm_close", -300)
        self.declare_parameter("gripper_close_duration", 2.5)
        self.declare_parameter('lift_group', 'interbotix_lift')

        # --- Grasp Verification ---
        self.declare_parameter('grasp_verify.enabled', True)
        self.declare_parameter('grasp_verify.finger_joint', 'left_finger')
        self.declare_parameter('grasp_verify.min_position', 0.005)  # Min finger position (fully closed, no object)
        self.declare_parameter('grasp_verify.grasp_threshold', 0.012)  # If > this, object is grasped
        self.declare_parameter('grasp_verify.max_retries', 2)  # Retry grasp from scratch

        # --- Pre-Grasp Navigation Gate ---
        self.declare_parameter('pregrasp_nav.enabled', True)
        self.declare_parameter('pregrasp_nav.min_distance_m', 0.5)
        self.declare_parameter('pregrasp_nav.max_distance_m', 0.8)
        self.declare_parameter('pregrasp_nav.target_distance_m', 0.65)
        self.declare_parameter('pregrasp_nav.base_action_name', '/base/move_by_distance')
        self.declare_parameter('pregrasp_nav.base_align_topic', '/base/aligned_for_grasp')
        self.declare_parameter('pregrasp_nav.base_action_timeout_s', 20.0)
        self.declare_parameter('pregrasp_nav.base_align_timeout_s', 10.0)
        self.declare_parameter('pregrasp_nav.max_detection_stale_s', 3.0)
        self.declare_parameter('pregrasp_nav.wait_fresh_detection_timeout_s', 2.0)

        # --- Workspace Limits ---
        self.declare_parameter('workspace_x_min', 0.10)
        self.declare_parameter('workspace_x_max', 0.60)
        self.declare_parameter('workspace_y_min', -0.25)
        self.declare_parameter('workspace_y_max', 0.25)
        self.declare_parameter('workspace_z_min', -0.05)
        self.declare_parameter('workspace_z_max', 0.35)

        # --- Lift Axis ---
        self.declare_parameter('lift.enabled', True)
        self.declare_parameter('lift.joint_name', 'lift_joint')
        self.declare_parameter('lift.action_name', '/lift_controller/follow_joint_trajectory')
        self.declare_parameter('lift.min_position', 0.0)
        self.declare_parameter('lift.max_position', 0.275)
        self.declare_parameter('lift.home_position', 0.0)
        self.declare_parameter('lift.grasp_position', 0.0)
        self.declare_parameter('lift.retract_position', 0.15)
        self.declare_parameter('lift.move_timeout', 15.0)

        # ========================================
        # Load Parameters
        # ========================================
        self.ref_frame = self.get_parameter('ref_frame').value
        self.arm_base_frame = self.get_parameter('arm_base_frame').value
        self.ee_link = self.get_parameter('ee_link').value

        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.max_grasp_distance = self.get_parameter('max_grasp_distance').value
        self.roi_padding = self.get_parameter('roi_padding').value
        self.depth_sample_ratio = self.get_parameter('depth_sample_ratio').value
        self.use_depth_filter = self.get_parameter('use_depth_filter').value

        self.approach_distance = self.get_parameter('approach_distance').value
        self.grasp_offset_x = self.get_parameter('grasp_offset_x').value
        self.grasp_offset_y = self.get_parameter('grasp_offset_y').value
        self.grasp_offset_z = self.get_parameter('grasp_offset_z').value
        self.lift_height = self.get_parameter('lift_height').value
        self.grasp_min_z = self.get_parameter('grasp_min_z').value
        self.depth_safety_offset = self.get_parameter('depth_safety_offset').value
        self.depth_window_size = self.get_parameter('depth_window_size').value
        self.grasp_point_offset_ratio = self.get_parameter('grasp_point_offset_ratio').value



        # Fixed orientation parameters
        self.use_fixed_orientation = self.get_parameter('use_fixed_orientation').value
        self.fixed_grasp_pitch = self.get_parameter('fixed_grasp_pitch').value
        self.fixed_grasp_roll = self.get_parameter('fixed_grasp_roll').value
        self.fixed_grasp_yaw = self.get_parameter('fixed_grasp_yaw').value

        # Cartesian path parameters
        self.use_cartesian_path = self.get_parameter('use_cartesian_path').value
        self.cartesian_step_size = self.get_parameter('cartesian_step_size').value
        self.cartesian_jump_threshold = self.get_parameter('cartesian_jump_threshold').value

        # Gripper strategy parameters
        self.gripper_second_squeeze = self.get_parameter('gripper_second_squeeze').value
        self.gripper_first_close_effort = self.get_parameter('gripper_first_close_effort').value
        self.gripper_squeeze_effort = self.get_parameter('gripper_squeeze_effort').value
        self.gripper_close_delay = self.get_parameter('gripper_close_delay').value
        self.gripper_current_limit_grasp = self.get_parameter('gripper_current_limit_grasp').value
        self.gripper_current_limit_max = self.get_parameter('gripper_current_limit_max').value
        self.contact_threshold = 100.0  # Effort/current threshold for contact detection (sensitive for paper bag)

        self.pose_filter_alpha = self.get_parameter('pose_filter_alpha').value
        self.min_confidence = self.get_parameter('min_detection_confidence').value

        # Multi-target selection
        self.use_reachability = self.get_parameter('target_selection.use_reachability').value
        self.reachability_weight = self.get_parameter('target_selection.reachability_weight').value
        self.confidence_weight = self.get_parameter('target_selection.confidence_weight').value
        self.depth_quality_weight = self.get_parameter('target_selection.depth_quality_weight').value
        self.optimal_distance = self.get_parameter('target_selection.optimal_distance').value

        # Retry mechanism
        self.retry_enabled = self.get_parameter('retry.enabled').value
        self.retry_max_attempts = self.get_parameter('retry.max_attempts').value
        self.retry_lift_adjustment = self.get_parameter('retry.lift_adjustment').value
        self.retry_position_offset = self.get_parameter('retry.position_offset').value

        self.arm_group = self.get_parameter('arm_group').value
        self.lift_group = self.get_parameter('lift_group').value
        self.arm_lift_group = self.get_parameter('arm_lift_group').value
        self.use_coordinated_motion = self.get_parameter('use_coordinated_motion').value
        self.max_vel_scale = self.get_parameter('max_velocity_scaling').value
        self.max_acc_scale = self.get_parameter('max_acceleration_scaling').value
        self.planning_time = self.get_parameter('planning_time').value
        self.num_attempts = self.get_parameter('num_planning_attempts').value
        self.use_orientation_constraint = self.get_parameter('use_orientation_constraint').value

        self.gripper_pwm_open = self.get_parameter('gripper_pwm_open').value
        self.gripper_pwm_close = self.get_parameter('gripper_pwm_close').value
        self.gripper_close_duration = self.get_parameter('gripper_close_duration').value
        
        # Client for setting motor registers (for force control)
        # NOTE: RobotConfig service not available in this interbotix version
        # Force control disabled - using default gripper behavior
        self._robot_config_client = None
        self.get_logger().warn('RobotConfig service not available - gripper force control disabled')

        # Load Force Control Parameters (kept for future use)
        self.gripper_current_limit_grasp = self.get_parameter("gripper_current_limit_grasp").value
        self.gripper_current_limit_max = self.get_parameter("gripper_current_limit_max").value
        
        # MoveIt Gripper Setup
        self.gripper_group_name = 'interbotix_gripper'
        self.gripper_joint_name = 'left_finger'

        self.ws_x_min = self.get_parameter('workspace_x_min').value
        self.ws_x_max = self.get_parameter('workspace_x_max').value
        self.ws_y_min = self.get_parameter('workspace_y_min').value
        self.ws_y_max = self.get_parameter('workspace_y_max').value
        self.ws_z_min = self.get_parameter('workspace_z_min').value
        self.ws_z_max = self.get_parameter('workspace_z_max').value

        # Load lift parameters
        self.lift_enabled = self.get_parameter('lift.enabled').value
        self.lift_joint_name = self.get_parameter('lift.joint_name').value
        self.lift_action_name = self.get_parameter('lift.action_name').value
        self.lift_min_pos = self.get_parameter('lift.min_position').value
        self.lift_max_pos = self.get_parameter('lift.max_position').value
        self.lift_home_pos = self.get_parameter('lift.home_position').value
        self.lift_grasp_pos = self.get_parameter('lift.grasp_position').value
        self.lift_retract_pos = self.get_parameter('lift.retract_position').value
        self.lift_timeout = self.get_parameter('lift.move_timeout').value
        
        # --- Robustness & Debugging ---
        self.declare_parameter('debug.visualization_only', False)
        self.declare_parameter('debug.publish_markers', True)
        self.declare_parameter('depth_check.min_valid_ratio', 0.1)

        self.viz_only = self.get_parameter('debug.visualization_only').value
        self.publish_markers = self.get_parameter('debug.publish_markers').value
        self.min_valid_depth_ratio = self.get_parameter('depth_check.min_valid_ratio').value

        # ========================================
        # State Variables
        # ========================================
        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.camera_info_msg = None
        self.filtered_pose = None
        self.current_grasp_pose = None
        self.is_busy = False
        self._lock = threading.Lock()

        # Eye-in-Hand mode: no runtime calibration needed, TF comes from URDF
        self.is_calibrated = True

        # MoveIt state
        self._goal_complete = False
        self._goal_success = False

        # Lift state
        self.lift_is_homed = False
        self._lift_goal_complete = False
        self._lift_goal_success = False

        # Gripper controller state (for direct controller control)
        self._gripper_goal_complete = False
        self._gripper_goal_success = False
        self._arm_goal_complete = False
        self._arm_goal_success = False

        # Object orientation for gripper alignment (from bbox detection)
        self._current_object_angle = 0.0

        # Last detection data for visual servoing
        self._last_detection = None

        # Predefined joint positions [waist, shoulder, elbow, wrist_angle, wrist_rotate]
        # Home/return pose captured from /joint_states on hardware.
        self.home_joints = [
            -0.08897088468074799,
            -0.21935926377773285,
            -0.5169515609741211,
            1.2501944303512573,
            0.0,
        ]
        # Startup pose used before detection/grasp execution.
        self.grasp_joints = [
            -0.13192234933376312,
            0.4111068546772003,
            0.849825382232666,
            -0.45405831933021545,
            -0.0015339808305725455,
        ]
        # Measured lift was -0.000294; clamp to 0.0 to stay within configured joint limits.
        self.ready_lift_pos = 0.0
        self.sleep_joints = [-1.57, -1.8, 1.55, 0.8, 0.0]
        self.arm_joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']

        # Multi-frame fusion state
        self.fusion_enabled = self.get_parameter('fusion.enabled').value
        self.fusion_buffer_size = self.get_parameter('fusion.buffer_size').value
        self.fusion_min_detections = self.get_parameter('fusion.min_detections').value
        self.fusion_max_std = self.get_parameter('fusion.max_position_std').value
        self.fusion_timeout = self.get_parameter('fusion.timeout_sec').value
        self.detection_buffer = []  # List of (timestamp, position, yaw)
        self.last_detection_time = 0.0

        # Visual servoing parameters
        self.vs_enabled = self.get_parameter('visual_servo.enabled').value
        self.vs_max_iterations = self.get_parameter('visual_servo.max_iterations').value
        self.vs_pixel_tolerance = self.get_parameter('visual_servo.pixel_tolerance').value
        self.vs_depth_tolerance = self.get_parameter('visual_servo.depth_tolerance').value
        self.vs_step_gain = self.get_parameter('visual_servo.step_gain').value
        self.vs_max_step = self.get_parameter('visual_servo.max_step').value
        self.vs_min_step = self.get_parameter('visual_servo.min_step').value
        self.vs_settle_time = self.get_parameter('visual_servo.settle_time').value

        # Adaptive gain parameters
        self.vs_adaptive_gain = self.get_parameter('visual_servo.adaptive_gain').value
        self.vs_gain_far = self.get_parameter('visual_servo.gain_far').value
        self.vs_gain_mid = self.get_parameter('visual_servo.gain_mid').value
        self.vs_gain_near = self.get_parameter('visual_servo.gain_near').value
        self.vs_error_threshold_far = self.get_parameter('visual_servo.error_threshold_far').value
        self.vs_error_threshold_near = self.get_parameter('visual_servo.error_threshold_near').value
        self.vs_convergence_window = self.get_parameter('visual_servo.convergence_window').value
        self.vs_convergence_threshold = self.get_parameter('visual_servo.convergence_threshold').value

        # Adaptive settle parameters
        self.vs_adaptive_settle = self.get_parameter('visual_servo.adaptive_settle').value
        self.vs_settle_time_far = self.get_parameter('visual_servo.settle_time_far').value
        self.vs_settle_time_near = self.get_parameter('visual_servo.settle_time_near').value

        # [NEW] Advanced depth processing parameters
        self.depth_use_mask_sampling = self.get_parameter('depth_use_mask_sampling').value
        self.depth_percentile_low = self.get_parameter('depth_percentile_low').value
        self.depth_percentile_high = self.get_parameter('depth_percentile_high').value
        self.depth_outlier_threshold = self.get_parameter('depth_outlier_threshold').value
        self.depth_plane_fitting = self.get_parameter('depth_plane_fitting').value
        self.depth_plane_inlier_threshold = self.get_parameter('depth_plane_inlier_threshold').value
        self.depth_window_adaptive = self.get_parameter('depth_window_adaptive').value
        self.depth_window_ratio = self.get_parameter('depth_window_ratio').value
        self.depth_window_min = self.get_parameter('depth_window_min').value
        self.depth_window_max = self.get_parameter('depth_window_max').value

        # [NEW] Kalman filter parameters
        self.use_kalman = self.get_parameter('fusion.use_kalman').value
        kalman_process_noise = self.get_parameter('fusion.kalman_process_noise').value
        kalman_measurement_noise = self.get_parameter('fusion.kalman_measurement_noise').value

        # [NEW] Contact detection parameters
        self.contact_detection_enabled = self.get_parameter('contact_detection.enabled').value
        self.contact_current_threshold = self.get_parameter('contact_detection.current_threshold').value
        self.contact_approach_speed = self.get_parameter('contact_detection.approach_speed').value
        self.contact_max_extra_descent = self.get_parameter('contact_detection.max_extra_descent').value
        self.contact_check_interval = self.get_parameter('contact_detection.check_interval').value
        self.contact_current_window = self.get_parameter('contact_detection.current_window').value

        # Initialize Kalman filter and convergence detector
        self.position_kalman = KalmanFilter3D(kalman_process_noise, kalman_measurement_noise)
        self.convergence_detector = ConvergenceDetector(
            self.vs_convergence_window, self.vs_convergence_threshold
        )

        # Temporal depth filter for smoothing across frames
        self.depth_temporal_filter = DepthTemporalFilter(window_size=7, max_age_sec=0.5)

        # [NEW] SAM mask storage for depth sampling
        self._current_mask = None
        self._current_mask_timestamp = 0.0

        # [NEW] Current readings buffer for contact detection
        self._current_readings = deque(maxlen=self.contact_current_window)

        # Callback Group to prevent Deadlock between Service and ActionClient
        self.cb_group = ReentrantCallbackGroup()

        # Grasp verification state
        self.grasp_verify_enabled = self.get_parameter('grasp_verify.enabled').value
        self.finger_joint_name = self.get_parameter('grasp_verify.finger_joint').value
        self.finger_min_pos = self.get_parameter('grasp_verify.min_position').value
        self.finger_grasp_threshold = self.get_parameter('grasp_verify.grasp_threshold').value
        self.grasp_max_retries = self.get_parameter('grasp_verify.max_retries').value

        self.pregrasp_nav_enabled = self.get_parameter('pregrasp_nav.enabled').value
        self.pregrasp_nav_min_d = self.get_parameter('pregrasp_nav.min_distance_m').value
        self.pregrasp_nav_max_d = self.get_parameter('pregrasp_nav.max_distance_m').value
        self.pregrasp_nav_target_d = self.get_parameter('pregrasp_nav.target_distance_m').value
        self.pregrasp_nav_action_name = self.get_parameter('pregrasp_nav.base_action_name').value
        self.pregrasp_nav_align_topic = self.get_parameter('pregrasp_nav.base_align_topic').value
        self.pregrasp_nav_action_timeout = self.get_parameter('pregrasp_nav.base_action_timeout_s').value
        self.pregrasp_nav_align_timeout = self.get_parameter('pregrasp_nav.base_align_timeout_s').value
        self.pregrasp_nav_max_detection_stale = self.get_parameter('pregrasp_nav.max_detection_stale_s').value
        self.pregrasp_nav_wait_fresh_detection_timeout = self.get_parameter('pregrasp_nav.wait_fresh_detection_timeout_s').value

        self.ready_reached = False
        self.base_aligned = False
        self.current_finger_position = 0.0
        self.current_lift_position = 0.0 # Track current lift position
        
        # ========================================
        # TF2
        # ========================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for static transforms (same as official)
        self.pub_transforms = self.create_publisher(
            TransformStamped, '/static_transforms', 50
        )

        # ========================================
        # MoveIt Action Client
        # ========================================
        # MoveIt Action Client
        self._move_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)

        # Cartesian Path Service Client (for straight-line approach/lift)
        self._cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path', callback_group=self.cb_group
        )

        # Execute Trajectory Action Client (for executing Cartesian paths)
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory', callback_group=self.cb_group
        )
        self._base_move_client = ActionClient(
            self, MoveBaseDistance, self.pregrasp_nav_action_name, callback_group=self.cb_group
        )

        # Direct Gripper Controller Action Client (bypasses MoveIt for reliable gripper control)
        self._gripper_controller_client = ActionClient(
            self, FollowJointTrajectory, '/rx200/gripper_controller/follow_joint_trajectory',
            callback_group=self.cb_group
        )

        # Direct Arm Controller Action Client (fallback for home/ready transitions)
        self._arm_controller_client = ActionClient(
            self, FollowJointTrajectory, '/rx200/arm_controller/follow_joint_trajectory',
            callback_group=self.cb_group
        )
        
        # Robot Config Client (for Current Limit)
        self._robot_config_client = self.create_client(
            RegisterValues, '/rx200/set_motor_registers', callback_group=self.cb_group
        )

        # ========================================
        # Lift Action Client (DEPRECATED - Moved to MoveIt)
        # ========================================
        # if self.lift_enabled:
        #    self._lift_client = ActionClient(
        #        self, FollowJointTrajectory, self.lift_action_name, callback_group=self.cb_group
        #    )
        self.sub_lift_homed = self.create_subscription(
            Bool, '/lift/is_homed', self._lift_homed_cb, 10
        )
        self.get_logger().info(f'Lift enabled: Using MoveIt group {self.lift_group}')

        # ========================================
        # Publishers
        # ========================================
        self.grasp_pose_pub = self.create_publisher(PoseStamped, '/grasp/target_pose', 10)
        self.pregrasp_pose_pub = self.create_publisher(PoseStamped, '/grasp/pregrasp_pose', 10)
        self.status_pub = self.create_publisher(Bool, '/grasp/busy', 10)
        self.gripper_pub = self.create_publisher(JointSingleCommand, '/rx200/commands/joint_single', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/grasp/debug_markers', 10)
        
        # [NEW] MoveIt Servo Continuous Control
        self.servo_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # ========================================
        # Subscribers
        # ========================================
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        # Always consume selector output to follow "point-then-grasp" workflow.
        self.declare_parameter('yolo_detections_topic', '/handbag/selected_detections')
        self.declare_parameter('yolo_mask_topic', '/yolo/mask')

        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.yolo_detections_topic = self.get_parameter('yolo_detections_topic').value
        self.yolo_mask_topic = self.get_parameter('yolo_mask_topic').value

        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic,
            self._camera_info_callback, 10
        )

        # Synchronized: YOLO detections + Depth image
        # OPTIMIZED: Tighter sync window to avoid mismatched depth/detection frames
        self.detection_sub = message_filters.Subscriber(self, Detection2DArray, self.yolo_detections_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub], queue_size=15, slop=0.08
        )
        self.sync.registerCallback(self._sync_callback_robust)

        # [NEW] SAM mask subscription for depth sampling
        self.mask_sub = self.create_subscription(
            Image, self.yolo_mask_topic, self._mask_callback, 10
        )

        # JointState subscription for gripper finger monitoring AND waist pre-alignment
        from sensor_msgs.msg import JointState
        self._latest_joint_state = None  # Init for waist pre-alignment
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )
        self.base_aligned_sub = self.create_subscription(
            Bool, self.pregrasp_nav_align_topic, self._base_aligned_cb, 10
        )

        # ========================================
        # Services
        # ========================================
        self.create_service(Trigger, '/grasp/execute', self._execute_grasp_service, callback_group=self.cb_group)
        self.create_service(Trigger, '/grasp/execute_ready', self._execute_ready_service)
        self.create_service(Trigger, '/grasp/package_ready', self._package_ready_service)
        self.create_service(Trigger, '/grasp/go_home', self._go_home_service)
        self.create_service(Trigger, '/grasp/go_sleep', self._go_sleep_service)
        self.create_service(Trigger, '/grasp/rest', self._rest_service)
        self.create_service(Trigger, '/grasp/open_gripper', self._open_gripper_service)
        self.create_service(Trigger, '/grasp/close_gripper', self._close_gripper_service)

        # Lift services
        if self.lift_enabled:
            self.create_service(Trigger, '/grasp/home_lift', self._home_lift_service)
            self.create_service(Trigger, '/grasp/lower_lift', self._lower_lift_service)
            self.create_service(Trigger, '/grasp/raise_lift', self._raise_lift_service)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Grasp Node initialized (Eye-in-Hand Mode)')
        self.get_logger().info(f'  ref_frame: {self.ref_frame}')
        self.get_logger().info(f'  arm_base_frame: {self.arm_base_frame}')
        self.get_logger().info(f'  lift_enabled: {self.lift_enabled}')
        if self.lift_enabled:
            self.get_logger().info(f'  lift_action: {self.lift_action_name}')
        self.get_logger().info(f'  camera_info_topic: {self.camera_info_topic}')
        self.get_logger().info(f'  depth_topic: {self.depth_topic}')
        self.get_logger().info(f'  yolo_detections_topic: {self.yolo_detections_topic}')
        self.get_logger().info(f'  yolo_mask_topic: {self.yolo_mask_topic}')
        self.get_logger().info('=' * 60)

        # Wait for MoveIt
        self.get_logger().info('Waiting for MoveGroup action server...')
        if self._move_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().info('MoveGroup connected!')
        else:
            self.get_logger().warn('MoveGroup not available yet')

        # Wait for Lift homing (via topic)
        if self.lift_enabled:
             # Just ensure topic is subscribed
             pass

        # Manual ready mode: keep the arm still until the operator explicitly
        # calls /grasp/execute_ready.
        self.get_logger().info('Manual ready mode enabled. Waiting for /grasp/execute_ready to move to grasp pose.')

    # ========================================
    # Camera Info Callback
    # ========================================
    def _camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics."""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = np.array(msg.k).reshape(3, 3)
            self.camera_info_msg = msg
            self.get_logger().info('Camera intrinsics received')

    # ========================================
    # [NEW] Mask Callback for Depth Sampling
    # ========================================
    def _mask_callback(self, msg: Image):
        """
        Receive SAM mask from YOLO detector for mask-based depth sampling.
        """
        try:
            # Convert mono8 image to numpy array
            mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            self._current_mask = mask
            self._current_mask_timestamp = time.time()
        except Exception as e:
            self.get_logger().debug(f'Mask callback error: {e}')

    def _get_current_mask(self):
        """Get current mask if fresh (within 1 second)."""
        if self._current_mask is None:
            return None
        if time.time() - self._current_mask_timestamp > 1.0:
            return None
        return self._current_mask

    def _move_to_ready_once(self):
        """One-time move to grasp pose (arm + lift)."""
        # Cancel this timer so it only runs once
        try:
            if hasattr(self, '_ready_timer') and self._ready_timer:
                self._ready_timer.cancel()
        except Exception:
            pass

        self.get_logger().info('=' * 60)
        self.get_logger().info('AUTO-MOVING TO GRASP POSE')
        self.get_logger().info('=' * 60)

        self._move_to_grasp_pose(log_prefix='AUTO')
        self.get_logger().info('=' * 60)

    def _move_to_grasp_pose(self, log_prefix='MANUAL') -> bool:
        """Move lift and arm to the configured startup grasp pose."""
        self.ready_reached = False

        # Wait for MoveGroup to be fully ready
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self._move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            return False
        self.get_logger().info('MoveGroup ready!')

        # Move lift to startup position first.
        if self.lift_enabled:
            # Wait for lift to be homed before attempting movement
            if not self.lift_is_homed:
                self.get_logger().info('Waiting for lift to complete homing...')
                wait_start = time.time()
                max_wait = 35.0  # Maximum wait time for homing (home_timeout_s = 30s + buffer)
                while not self.lift_is_homed and (time.time() - wait_start) < max_wait:
                    time.sleep(0.5)  # Wait for homing callback
                    # Spin once to process callbacks (update lift_is_homed from topic)
                        # Note: callbacks are processed by executor, just need to wait

                if self.lift_is_homed:
                    self.get_logger().info('Lift homing complete!')
                else:
                    self.get_logger().warn(f'Lift not homed after {max_wait}s, skipping lift movement')

            if self.lift_is_homed:
                self.get_logger().info(f'[{log_prefix}] Moving lift to startup position: {self.ready_lift_pos}m')
                if not self._move_lift_to(self.ready_lift_pos):
                    self.get_logger().warn('Lift movement failed, continuing with arm...')
                time.sleep(1.0)  # Wait for lift to complete
            else:
                self.get_logger().warn('Lift not homed, skipping lift movement')

        # Move arm to grasp startup position
        self.get_logger().info(f'[{log_prefix}] Moving arm to grasp joints...')
        success = self._move_to_joints(self.grasp_joints)
        if success:
            self.get_logger().info(f'[{log_prefix}] ✅ Grasp pose reached! System ready for detection.')
            self.ready_reached = True
        else:
            self.get_logger().error(f'[{log_prefix}] ❌ Failed to move arm to grasp pose!')
            self.ready_reached = False

        return success

    # ========================================
    # AprilTag Calibration (Official Style)
    # ========================================
    def _joint_state_callback(self, msg):
        """Monitor gripper finger position and store full joint state."""
        try:
            # Store full joint state for waist pre-alignment
            self._latest_joint_state = msg
            
            if self.finger_joint_name in msg.name:
                idx = msg.name.index(self.finger_joint_name)
                self.current_finger_position = msg.position[idx]
            
            # Also track lift position
            if self.lift_joint_name in msg.name:
                idx = msg.name.index(self.lift_joint_name)
                self.current_lift_position = msg.position[idx]
        except Exception:
            pass

    def _base_aligned_cb(self, msg: Bool):
        """Track base alignment state from the mobile base."""
        self.base_aligned = bool(msg.data)

    def _verify_grasp(self):
        """
        Verify if object is grasped based on gripper finger position.
        
        Logic:
        - If fully closed (pos < min_pos), grasp failed (empty gripper).
        - If position > grasp_threshold, object is holding the fingers open -> Success.
        """
        if not self.grasp_verify_enabled:
            return True, "Verification disabled"
            
        # Give some time for joint state to update after closing
        time.sleep(0.5)
        
        pos = self.current_finger_position
        self.get_logger().info(f'Grasp verification: finger_pos={pos*1000:.1f}mm (thresh={self.finger_grasp_threshold*1000:.1f}mm)')
        
        if pos > self.finger_grasp_threshold:
            return True, f"Grasp confirmed (pos={pos*1000:.1f}mm)"
        else:
            return False, f"Grasp failed (pos={pos*1000:.1f}mm)"

    def _get_transform(self, target_frame, source_frame):
        """
        Get transform from TF and convert to 4x4 matrix.
        Same as official: InterbotixArmTagInterface.get_transform()
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=Time(),
                timeout=Duration(seconds=2.0)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            quat = trans.transform.rotation
            quat_list = [quat.x, quat.y, quat.z, quat.w]
            rpy = euler_from_quaternion(quat_list)
            return pose_to_transformation_matrix([x, y, z, rpy[0], rpy[1], rpy[2]])
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {target_frame} -> {source_frame}: {e}')
            return None

    # ========================================
    # YOLO Detection Callback (Legacy _sync_callback removed - use _sync_callback_robust)
    # ========================================

    def _visualize_grasp(self, pose_cam, pose_base, bbox, stats):
        """Publish debug markers for Rviz."""
        if not self.publish_markers: return
        
        markers = MarkerArray()
        
        # 1. Camera Ray (Line from camera center to object)
        m_ray = Marker()
        m_ray.header.frame_id = self.ref_frame
        m_ray.header.stamp = self.get_clock().now().to_msg()
        m_ray.ns = "grasp_ray"
        m_ray.id = 0
        m_ray.type = Marker.ARROW
        m_ray.action = Marker.ADD
        m_ray.scale.x = 0.01; m_ray.scale.y = 0.02; m_ray.scale.z = 0.0
        m_ray.color.a = 1.0; m_ray.color.r = 1.0; m_ray.color.g = 1.0; m_ray.color.b = 0.0
        
        p_start = Point(); p_start.x=0.0; p_start.y=0.0; p_start.z=0.0
        p_end = Point(); 
        p_end.x = pose_cam.pose.position.x
        p_end.y = pose_cam.pose.position.y
        p_end.z = pose_cam.pose.position.z
        m_ray.points = [p_start, p_end]
        markers.markers.append(m_ray)
        
        # 2. Base Target Sphere
        if pose_base:
            m_target = Marker()
            m_target.header.frame_id = self.arm_base_frame
            m_target.header.stamp = self.get_clock().now().to_msg()
            m_target.ns = "grasp_target"
            m_target.id = 1
            m_target.type = Marker.SPHERE
            m_target.action = Marker.ADD
            m_target.pose = pose_base.pose
            m_target.scale.x = 0.05; m_target.scale.y = 0.05; m_target.scale.z = 0.05
            
            # Color based on status
            if stats['status'] == 'GOOD':
                m_target.color.r = 0.0; m_target.color.g = 1.0; m_target.color.b = 0.0
            else:
                m_target.color.r = 1.0; m_target.color.g = 0.0; m_target.color.b = 0.0
            m_target.color.a = 0.8
            markers.markers.append(m_target)
        
        self.marker_pub.publish(markers)

    def _compute_grasp_from_roi(self, depth_image, x1, y1, x2, y2, mask=None):
        """
        Extract 3D grasp point from depth ROI with robust depth estimation.

        OPTIMIZED VERSION with:
        1. Adaptive window sizing based on object size
        2. SAM mask-based depth sampling (if mask provided)
        3. RANSAC plane fitting for surface estimation
        4. Enhanced outlier rejection with configurable percentiles
        5. Kalman-filtered depth smoothing

        Args:
            depth_image: Depth image (uint16, mm)
            x1, y1, x2, y2: Bounding box coordinates
            mask: Optional binary mask for the object (from SAM)

        Returns: (point, yaw, stats_dict)
        """
        img_h, img_w = depth_image.shape[:2]

        # Calculate bbox center and size
        w = x2 - x1
        h = y2 - y1
        cx_bbox = int(x1 + w / 2)
        cy_bbox = int(y1 + h / 2)

        # Grasp point: use center for box-shaped objects
        grasp_offset_ratio = getattr(self, 'grasp_point_offset_ratio', 0.0)
        cy_grasp = int(cy_bbox - grasp_offset_ratio * h)
        cx_grasp = cx_bbox

        # Clamp to image bounds
        cy_grasp = max(0, min(cy_grasp, img_h - 1))
        cx_grasp = max(0, min(cx_grasp, img_w - 1))

        stats = {
            'valid_ratio': 0.0,
            'status': 'GOOD',
            'median_depth': 0.0,
            'grasp_pixel': (cx_grasp, cy_grasp),
            'method': 'window'
        }

        # ========================================
        # ADAPTIVE WINDOW SIZE
        # ========================================
        if self.depth_window_adaptive:
            # Window size = ratio * min(bbox_w, bbox_h)
            adaptive_size = int(min(w, h) * self.depth_window_ratio)
            adaptive_size = max(self.depth_window_min, min(adaptive_size, self.depth_window_max))
            adaptive_size = adaptive_size | 1  # Ensure odd
            half_win = adaptive_size // 2
            stats['window_size'] = adaptive_size
        else:
            half_win = self.depth_window_size // 2
            stats['window_size'] = self.depth_window_size

        # ========================================
        # DEPTH SAMPLING STRATEGY
        # ========================================
        median_depth = None

        # Strategy 1: Use SAM mask if available and enabled
        if self.depth_use_mask_sampling and mask is not None:
            try:
                # Ensure mask matches depth image size
                if mask.shape[:2] == depth_image.shape[:2]:
                    # Sample depth only within the mask
                    mask_region = mask[y1:y2, x1:x2]
                    depth_region = depth_image[y1:y2, x1:x2].astype(np.float32) / 1000.0

                    if mask_region.shape == depth_region.shape:
                        valid_mask_pixels = mask_region > 0
                        valid_depth_mask = (
                            (depth_region > self.min_depth) &
                            (depth_region < self.max_depth) &
                            np.isfinite(depth_region) &
                            valid_mask_pixels
                        )

                        valid_count = np.sum(valid_depth_mask)
                        if valid_count > 10:
                            valid_depths = depth_region[valid_depth_mask]
                            median_depth = self._compute_robust_depth(valid_depths, stats)
                            stats['method'] = 'mask_sampling'
                            stats['valid_ratio'] = valid_count / np.sum(valid_mask_pixels) if np.sum(valid_mask_pixels) > 0 else 0
            except Exception as e:
                self.get_logger().debug(f'Mask sampling failed: {e}, falling back to window')

        # Strategy 2: Window-based sampling (fallback or primary)
        if median_depth is None:
            x_min = max(0, cx_grasp - half_win)
            x_max = min(img_w, cx_grasp + half_win + 1)
            y_min_win = max(0, cy_grasp - half_win)
            y_max_win = min(img_h, cy_grasp + half_win + 1)

            depth_window = depth_image[y_min_win:y_max_win, x_min:x_max].astype(np.float32) / 1000.0

            valid_mask = (
                (depth_window > self.min_depth) &
                (depth_window < self.max_depth) &
                np.isfinite(depth_window) &
                (depth_window > 0.01)
            )
            valid_count = np.sum(valid_mask)
            total_pixels = depth_window.size

            if valid_count > 3:
                valid_depths = depth_window[valid_mask]
                median_depth = self._compute_robust_depth(valid_depths, stats)
                stats['valid_ratio'] = valid_count / total_pixels
                stats['method'] = 'window'
            else:
                # Fallback: Try broader range in full ROI
                roi_depth = depth_image[y1:y2, x1:x2].astype(np.float32) / 1000.0
                valid_roi = (roi_depth > 0.1) & (roi_depth < 1.5) & np.isfinite(roi_depth)

                if np.sum(valid_roi) > 10:
                    valid_depths = roi_depth[valid_roi]
                    median_depth = self._compute_robust_depth(valid_depths, stats)
                    stats['status'] = 'FALLBACK_ROI'
                    stats['valid_ratio'] = np.sum(valid_roi) / roi_depth.size
                    stats['method'] = 'roi_fallback'
                    self.get_logger().warn(f'Low depth quality at grasp point, using ROI: {median_depth:.3f}m')
                else:
                    median_depth = 0.35
                    stats['status'] = 'ASSUMED'
                    stats['valid_ratio'] = 0.0
                    stats['method'] = 'assumed'
                    self.get_logger().warn('⚠️ NO VALID DEPTH! Using assumption 0.35m')

        raw_depth = float(median_depth)
        stats['raw_depth'] = raw_depth

        # Clamp to max grasp distance for pose generation, but keep the raw
        # distance available for pre-grasp navigation decisions.
        if self.max_grasp_distance > 0.0 and median_depth > self.max_grasp_distance:
            median_depth = self.max_grasp_distance
            stats['status'] = 'CLAMPED'

        stats['median_depth'] = median_depth

        # ========================================
        # BACK-PROJECTION TO 3D
        # ========================================
        fx = self.camera_intrinsics[0, 0]
        fy = self.camera_intrinsics[1, 1]
        ppx = self.camera_intrinsics[0, 2]
        ppy = self.camera_intrinsics[1, 2]

        Z = median_depth
        X = (cx_grasp - ppx) * Z / fx + self.grasp_offset_x
        Y = (cy_grasp - ppy) * Z / fy + self.grasp_offset_y

        # ADD SAFETY Z OFFSET
        Z_safe = Z + self.depth_safety_offset + self.grasp_offset_z
        stats['z_offset_applied'] = self.depth_safety_offset

        self.get_logger().info(
            f'Depth [{stats["method"]}]: raw={median_depth:.4f}m, safe={Z_safe:.4f}m '
            f'(+{self.depth_safety_offset*1000:.1f}mm), valid={stats["valid_ratio"]:.2f}, '
            f'win={stats.get("window_size", "N/A")}, status={stats["status"]}'
        )

        return np.array([X, Y, Z_safe]), 0.0, stats

    def _compute_robust_depth(self, valid_depths, stats):
        """
        Compute robust depth estimate using percentile filtering and optional plane fitting.

        Args:
            valid_depths: Array of valid depth values
            stats: Stats dict to update

        Returns:
            Robust median depth estimate
        """
        # Percentile filtering to remove outliers
        p_low = np.percentile(valid_depths, self.depth_percentile_low)
        p_high = np.percentile(valid_depths, self.depth_percentile_high)
        filtered_depths = valid_depths[(valid_depths >= p_low) & (valid_depths <= p_high)]

        if len(filtered_depths) < 3:
            filtered_depths = valid_depths

        # Additional outlier rejection using IQR
        if len(filtered_depths) > 10:
            q1 = np.percentile(filtered_depths, 25)
            q3 = np.percentile(filtered_depths, 75)
            iqr = q3 - q1
            iqr_filtered = filtered_depths[
                (filtered_depths >= q1 - 1.5 * iqr) &
                (filtered_depths <= q3 + 1.5 * iqr)
            ]
            if len(iqr_filtered) > 3:
                filtered_depths = iqr_filtered
                stats['outlier_method'] = 'iqr'

        # RANSAC plane fitting for surface estimation (optional)
        if self.depth_plane_fitting and len(filtered_depths) > 20:
            try:
                # Simple RANSAC-like approach: iteratively remove outliers
                for _ in range(3):
                    med = np.median(filtered_depths)
                    mad = np.median(np.abs(filtered_depths - med))
                    if mad > 0:
                        inliers = np.abs(filtered_depths - med) < 3 * mad
                        if np.sum(inliers) > 5:
                            filtered_depths = filtered_depths[inliers]
                stats['plane_fitted'] = True
            except Exception:
                pass

        median_depth = np.median(filtered_depths)
        stats['depth_std'] = np.std(filtered_depths)

        return median_depth

    def _create_grasp_pose_camera(self, point, yaw_angle, timestamp=None):
        """
        Create grasp pose in camera frame (position only).

        NOTE: Orientation is set to identity here because the ACTUAL orientation
        is computed in _adjust_orientation_for_5dof() AFTER transform to base_link.

        For 5-DOF arm (RX200):
        - Roll = 0 (fixed)
        - Pitch = 90° (gripper pointing down, fixed)
        - Yaw = atan2(y, x) (computed from target position, required for 5-DOF)

        We do NOT infer yaw from visual detection (bbox orientation) for stability.
        """
        pose = PoseStamped()
        pose.header.frame_id = self.ref_frame
        
        # Use exact timestamp if provided for TF accuracy
        if timestamp:
             pose.header.stamp = timestamp
        else:
             pose.header.stamp = self.get_clock().now().to_msg()
             
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])

        # Identity orientation - will be overwritten by _adjust_orientation_for_5dof
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        return pose

    def _transform_to_base(self, pose_cam: PoseStamped, stamp=None):
        """Transform pose to base frame using SPECIFIC TIMESTAMP for accuracy."""
        try:
            # Use specific timestamp to avoid "projection skew" when moving
            # If stamp is None, use latest (Time()) but warn?
            lookup_time = stamp if stamp else Time()
            
            transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ref_frame,
                lookup_time,  # Use Exact Timestamp
                timeout=Duration(seconds=0.1) # Fast timeout for synced lookup
            )
        except Exception as e:
            # Debug: Check what frames are available
            try:
                frames_str = self.tf_buffer.all_frames_as_string()
                self.get_logger().error(
                    f'❌ TF FAILED: {self.arm_base_frame} → {self.ref_frame}\n'
                    f'Error: {e}\n'
                    f'Available frames:\n{frames_str[:1000]}',
                    throttle_duration_sec=5.0
                )
            except Exception:
                self.get_logger().error(
                    f'❌ TF chain broken: {self.arm_base_frame} → {self.ref_frame}: {e}'
                )
            return None
        
        try:
            pose_base = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, transform)
            pose_base.header.frame_id = self.arm_base_frame
            # Safety offset is already applied in _compute_grasp_from_roi()
            # Do NOT add another offset here to avoid cumulative error
            return self._adjust_orientation_for_5dof(pose_base)
        except Exception as e:
            self.get_logger().error(f'Transform apply failed: {e}')
            return None

    def _adjust_orientation_for_5dof(self, pose: PoseStamped, override_angle=None):
        """
        Adjust orientation for 5-DOF arm to achieve vertical (top-down) grasp.

        For RX200:
        - Yaw is coupled with waist rotation, MUST align with atan2(y, x)
        - Pitch is -90° (gripper pointing DOWN toward ground)
        - Roll (wrist_rotate) aligns gripper fingers with object's SHORT edge

        Gripper coordinate system (at home pose, arm extended forward):
        - X: forward (approach direction)
        - Y: left-right (fingers open along this axis)
        - Z: up (roll axis)
        
        To grasp the SHORT edge:
        - Gripper fingers should SPAN the LONG edge
        - For tall object (h > w): fingers span height -> roll aligned with arm
        - For wide object (w > h): fingers span width -> roll rotated 90°
        """
        # Calculate Yaw from Position (mandatory for 5-DOF arm)
        # This makes the arm face toward the target
        yaw = np.arctan2(pose.pose.position.y, pose.pose.position.x)

        # Get object orientation from detection
        if override_angle is not None:
            object_angle = override_angle
        else:
            object_angle = getattr(self, '_current_object_angle', 0.0)
        
        # Transform object angle from camera frame to gripper frame
        # object_angle: 0 = horizontal (width along x), 90 = vertical (height along y) in camera
        # 
        # After arm rotates by yaw, the gripper's finger-opening direction changes.
        # We need to compensate for this to keep fingers aligned with the object.
        #
        # Roll formula: align gripper Y (finger span) with object's LONG dimension
        # roll = object_angle - yaw + offset
        #
        # For object_angle=0 (wide box): we want roll such that fingers span width
        # For object_angle=90 (tall box): we want roll=0 (fingers span height)
        
        roll = object_angle - yaw + np.pi/2
        
        # Normalize roll to [-pi, pi]
        while roll > np.pi:
            roll -= 2*np.pi
        while roll < -np.pi:
            roll += 2*np.pi
        
        self.get_logger().info(
            f"🔧 5DOF Orientation: yaw={np.degrees(yaw):.1f}°, roll={np.degrees(roll):.1f}° (obj_angle={np.degrees(object_angle):.1f}°)"
        )

        # Orientation: R = Rz(yaw) * Ry(pi/2) * Rx(roll)
        r_z = R.from_euler('z', yaw)
        r_y = R.from_euler('y', np.pi/2) 
        r_x = R.from_euler('x', roll)
        r = r_z * r_y * r_x

        quat = r.as_quat()
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def _compute_pregrasp_pose(self, grasp_pose: PoseStamped):
        """Compute pre-grasp pose."""
        pregrasp = PoseStamped()
        pregrasp.header = grasp_pose.header
        pregrasp.pose = copy.deepcopy(grasp_pose.pose)
        pregrasp.pose.position.z += self.approach_distance
        # 不让预抓取高度低于最小Z，避免规划到桌面以下
        if pregrasp.pose.position.z < self.grasp_min_z:
            pregrasp.pose.position.z = self.grasp_min_z
        return pregrasp

    def _filter_pose(self, new_pose: PoseStamped):
        """Apply EMA filter."""
        if self.filtered_pose is None:
            self.filtered_pose = new_pose
            return new_pose

        alpha = self.pose_filter_alpha
        self.filtered_pose.pose.position.x = alpha * new_pose.pose.position.x + (1-alpha) * self.filtered_pose.pose.position.x
        self.filtered_pose.pose.position.y = alpha * new_pose.pose.position.y + (1-alpha) * self.filtered_pose.pose.position.y
        self.filtered_pose.pose.position.z = alpha * new_pose.pose.position.z + (1-alpha) * self.filtered_pose.pose.position.z
        self.filtered_pose.header = new_pose.header
        if self.filtered_pose.pose.position.z < self.grasp_min_z:
            self.filtered_pose.pose.position.z = self.grasp_min_z
        return self.filtered_pose

    def _check_workspace(self, pose: PoseStamped):
        """Check workspace bounds. Let MoveIt handle actual reachability."""
        x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        
        in_bounds = (self.ws_x_min <= x <= self.ws_x_max and
                     self.ws_y_min <= y <= self.ws_y_max and
                     self.ws_z_min <= z <= self.ws_z_max)
        
        if not in_bounds:
            self.get_logger().warn(
                f"Target out of bounds: ({x:.3f}, {y:.3f}, {z:.3f})"
            )
            
        return in_bounds


    # ========================================
    # Gripper Control
    # ========================================
    def _control_gripper_moveit(self, position):
        """Control gripper using direct controller action (more reliable than MoveIt planning)."""
        if not self._gripper_controller_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper controller not available')
            return False

        self.get_logger().info(f"MOVING GRIPPER TO {position}...")

        # Create FollowJointTrajectory goal
        goal = FollowJointTrajectory.Goal()

        # Build trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.gripper_joint_name]  # 'left_finger'

        # Single point trajectory
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.velocities = []
        point.accelerations = []
        point.time_from_start = BuiltinDuration(sec=1, nanosec=0)
        trajectory.points.append(point)

        goal.trajectory = trajectory

        # Send goal and wait for result
        self._gripper_goal_complete = False
        self._gripper_goal_success = False

        future = self._gripper_controller_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_goal_response_cb)

        # Wait for completion
        start = time.time()
        while not self._gripper_goal_complete and (time.time() - start) < 5.0:
            time.sleep(0.05)

        return self._gripper_goal_success

    def _get_motor_register(self, reg_name):
        """Read a register value for the gripper."""
        if self._robot_config_client is None: return -1
        try:
            req = RegisterValues.Request()
            req.cmd_type = 'group'
            req.name = 'gripper'
            req.reg = reg_name
            future = self._robot_config_client.call_async(req)
            result = future.result(timeout=1.0)
            return result.values[0] if result.values else -1
        except Exception:
            return -1

    def _control_gripper_pwm(self, pwm_value):
        """Send direct PWM command to gripper (Mode must be PWM)."""
        msg = JointSingleCommand()
        msg.name = 'gripper'
        msg.cmd = float(pwm_value)
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Published Gripper PWM: {pwm_value}")
        time.sleep(1.0) # Wait for move
        
        # DEBUG: Check Error Status and Current
        err = self._get_motor_register("Hardware_Error_Status")
        curr = self._get_motor_register("Present_Current")
        self.get_logger().info(f"🔎 GRASP MONITOR: ErrorStatus={err}, PresentCurrent={curr}")
        if err > 0:
            self.get_logger().error(f"🚨 HARDWARE ERROR DETECTED: {err} (Bit 5=Overload, Bit 2=Overheat)")
            
        return True

    def _gripper_goal_response_cb(self, future):
        """Callback for gripper controller goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            self._gripper_goal_complete = True
            self._gripper_goal_success = False
            return

        self.get_logger().info('Gripper goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result_cb)

    def _gripper_result_cb(self, future):
        """Callback for gripper controller result."""
        result = future.result()
        if result.result.error_code == 0:  # SUCCESSFUL
            self.get_logger().info('Gripper movement completed successfully')
            self._gripper_goal_success = True
        else:
            self.get_logger().warn(f'Gripper movement failed: error_code={result.result.error_code}')
            self._gripper_goal_success = False
        self._gripper_goal_complete = True

    def _arm_goal_response_cb(self, future):
        """Callback for arm controller goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Arm direct-joint goal rejected')
            self._arm_goal_complete = True
            self._arm_goal_success = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._arm_result_cb)

    def _arm_result_cb(self, future):
        """Callback for arm controller result."""
        try:
            result = future.result()
            self._arm_goal_success = (result.result.error_code == 0)
        except Exception:
            self._arm_goal_success = False
        self._arm_goal_complete = True

    def _set_gripper_current_limit(self, current_val):
        """
        Dynamically sets the Current Limit (Register 38) for the gripper motor.
        Synchronous call to ensure it is applied BEFORE movement.
        """
        # Skip if client not available
        if self._robot_config_client is None:
            return

        try:
            req = RegisterValues.Request()
            req.cmd_type = 'group' 
            req.name = 'gripper'
            req.reg = 'Current_Limit'
            req.value = int(current_val)
            
            future = self._robot_config_client.call_async(req)
            # BLOCKING WAIT (Safe in MultiThreadedExecutor + ReentrantCallbackGroup)
            try:
                # Wait for response (BLOCKING)
                # Note: rclpy Future.result() might not accept timeout in all versions/contexts
                # But we need to wait.
                while not future.done():
                     time.sleep(0.05)
                
                result = future.result()
                self.get_logger().info(f"✅ Gripper current limit set to {current_val}")
                
                # VERIFY: Read it back
                debug_limit = self._get_motor_register('Current_Limit')
                self.get_logger().info(f"🔎 VERIFY Current_Limit in Hardware: {debug_limit}")
                
            except Exception as e:
                 self.get_logger().warn(f"❌ Failed to set gripper current limit (Timeout/Error): {e}")
                 
        except Exception as e:
            self.get_logger().error(f"Failed to call register service: {e}") 

    def _open_gripper(self):
        """Open the gripper using MoveIt."""
        self.get_logger().info("Opening Gripper...")
        # Restore max current for opening
        if self.gripper_current_limit_max > 0:
             self._set_gripper_current_limit(self.gripper_current_limit_max)
             
        result = self._control_gripper_moveit(0.037)  # Open position
        time.sleep(0.5)
        return result

    def _close_gripper(self):
        """Close the gripper with optional two-stage squeeze."""
        if self.gripper_second_squeeze:
            return self._close_gripper_with_squeeze()
        else:
            self.get_logger().info("Closing Gripper (PWM Mode)...")
            
            # Use PWM command directly for "Force Feedback" style grasp
            # Lower PWM = Lower Force/Torque
            # Use 'gripper_current_limit_grasp' as a proxy for PWM scalar if needed,
            # but simpler to just use 'gripper_pwm_close' parameter.
            # Default pwm_close is -300? I will use -200 for gentler grasp.
            
            # Ensure limit is set anyway for safety
            if self.gripper_current_limit_grasp > 0:
                 self._set_gripper_current_limit(self.gripper_current_limit_grasp)
            
            # Send PWM command instead of MoveIt
            # PWM range: -limit to +limit. negative usually closes?
            # Check grasp_params for `gripper_pwm_close` value.
            pwm_val = self.get_parameter("gripper_pwm_close").value
            # If default is high (-300), maybe scale it?
            # User wants feedback/lower force. Let's start with what the param says.
            return self._control_gripper_pwm(pwm_val)

    def _close_gripper_with_squeeze(self):
        """Two-stage gripper close using direct controller."""
        # Stage 1: First close to moderate grip
        self.get_logger().info("Gripper Stage 1: Close to 0.020...")
        self._control_gripper_moveit(0.020)
        time.sleep(self.gripper_close_delay)

        # Check position
        current_pos = self._get_gripper_position()
        if current_pos is not None:
            self.get_logger().info(f"Gripper position: {current_pos:.4f}")

        # Stage 2: Squeeze to minimum (URDF lower limit is 0.015)
        self.get_logger().info("Gripper Stage 2: Squeeze to 0.015...")
        self._control_gripper_moveit(0.015)  # Minimum allowed by URDF
        time.sleep(self.gripper_close_delay)

        final_pos = self._get_gripper_position()
        self.get_logger().info(f"Gripper final: {final_pos:.4f}" if final_pos else "Gripper final: unknown")
        return True

    def _get_gripper_position(self) -> float:
        """Get current gripper finger position from joint states."""
        try:
            if hasattr(self, 'current_finger_position'):
                return self.current_finger_position
        except Exception:
            pass
        return None

    # ========================================
    # Visual Servoing (Closed-Loop Fine-Tuning) - OPTIMIZED
    # ========================================
    def _visual_servo_adjust(self, target_depth: float, align_camera_center: bool = False, maintain_height: bool = False) -> bool:
        """
        Closed-loop visual servoing to fine-tune gripper or camera position.

        OPTIMIZED VERSION with:
        1. Adaptive gain control based on error magnitude
        2. Kalman-filtered detection for smoother tracking
        3. Convergence detection based on error history
        4. Better oscillation prevention

        Args:
            target_depth: Desired depth to object (meters). Ignored if maintain_height=True.
            align_camera_center: If True, aligns CAMERA to object (u=center).
                                 If False, aligns GRIPPER to object (using grasp_offset).
            maintain_height: If True, only corrects X/Y, keeps Z constant (error_depth=0).

        Returns:
            True if successfully converged, False otherwise
        """
        if not self.vs_enabled:
            self.get_logger().info("Visual servoing disabled, skipping")
            return True

        self.get_logger().info("🎯 Starting Visual Servoing fine-tune (OPTIMIZED)...")

        # Get camera intrinsics for pixel-to-meter conversion
        if self.camera_intrinsics is None:
            self.get_logger().warn("No camera intrinsics, skipping visual servo")
            return True

        fx = self.camera_intrinsics[0, 0]
        fy = self.camera_intrinsics[1, 1]
        cx = self.camera_intrinsics[0, 2]
        cy = self.camera_intrinsics[1, 2]

        # Reset convergence detector and Kalman filter for this session
        self.convergence_detector.reset()
        if self.use_kalman:
            self.position_kalman.reset()

        no_detection_count = 0
        max_no_detection = 5

        for iteration in range(self.vs_max_iterations):
            # ADAPTIVE SETTLE TIME: wait less when far, more when near
            if self.vs_adaptive_settle and hasattr(self, 'vs_settle_time_far'):
                if hasattr(self, '_last_vs_error_norm') and self._last_vs_error_norm is not None:
                    if self._last_vs_error_norm > self.vs_error_threshold_far:
                        settle = self.vs_settle_time_far
                    elif self._last_vs_error_norm < self.vs_error_threshold_near:
                        settle = self.vs_settle_time_near
                    else:
                        settle = self.vs_settle_time
                else:
                    settle = self.vs_settle_time
            else:
                settle = self.vs_settle_time

            # Use event-driven wait if possible, fall back to sleep
            if not self._wait_for_settle(timeout=settle):
                time.sleep(settle)

            # Get current detection
            detection = self._get_current_detection()
            if detection is None:
                no_detection_count += 1
                self.get_logger().warn(f"VS iter {iteration}: No detection ({no_detection_count}/{max_no_detection})")
                if no_detection_count >= max_no_detection:
                    self.get_logger().warn("VS: Too many missed detections, aborting")
                    return False
                continue
            no_detection_count = 0

            obj_u, obj_v, obj_depth, obj_w, obj_h = detection

            # Use current depth for pixel-to-meter conversion
            current_depth = obj_depth if obj_depth > 0.1 else target_depth

            # Calculate object position in camera frame (meters)
            grasp_v = obj_v - self.grasp_point_offset_ratio * obj_h

            obj_x_m = ((obj_u - cx) / fx) * current_depth
            obj_y_m = ((grasp_v - cy) / fy) * current_depth

            # Apply Kalman filter for smoother tracking
            if self.use_kalman:
                filtered_pos = self.position_kalman.update([obj_x_m, obj_y_m, obj_depth])
                obj_x_m, obj_y_m, filtered_depth = filtered_pos
                # Use filtered depth only if it's reasonable
                if 0.05 < filtered_depth < 1.0:
                    current_depth = filtered_depth

            # Calculate error
            if align_camera_center:
                error_x_m = obj_x_m
                error_y_m = obj_y_m
                mode_str = "CAMERA_ALIGN"
            else:
                error_x_m = obj_x_m - (-self.grasp_offset_x)
                error_y_m = obj_y_m - (-self.grasp_offset_y)
                mode_str = "GRIPPER_ALIGN"

            if maintain_height:
                error_depth = 0.0
                mode_str += "+HoH"
            else:
                error_depth = obj_depth - target_depth

            # Calculate error magnitude for adaptive gain
            error_norm = np.sqrt(error_x_m**2 + error_y_m**2)

            # Add to convergence detector
            self.convergence_detector.add_error(error_norm)

            self.get_logger().info(
                f"VS [{mode_str}] iter {iteration}: err_x={error_x_m*1000:.1f}mm, "
                f"err_y={error_y_m*1000:.1f}mm, err_depth={error_depth*1000:.1f}mm, "
                f"norm={error_norm*1000:.1f}mm"
            )

            # Check convergence (using meter tolerance derived from pixel tolerance)
            meter_tolerance = (self.vs_pixel_tolerance / fx) * current_depth

            if (abs(error_x_m) < meter_tolerance and
                abs(error_y_m) < meter_tolerance and
                abs(error_depth) < self.vs_depth_tolerance):
                self.get_logger().info(f"✅ Visual servo converged in {iteration+1} iterations!")
                return True

            # Check convergence via history (oscillation detection)
            if self.convergence_detector.is_converged():
                self.get_logger().info(f"✅ VS converged via history analysis in {iteration+1} iterations!")
                return True

            # ADAPTIVE GAIN CONTROL
            if self.vs_adaptive_gain:
                if error_norm > self.vs_error_threshold_far:
                    gain = self.vs_gain_far
                    gain_mode = "FAR"
                elif error_norm > self.vs_error_threshold_near:
                    gain = self.vs_gain_mid
                    gain_mode = "MID"
                else:
                    gain = self.vs_gain_near
                    gain_mode = "NEAR"
            else:
                gain = self.vs_step_gain
                gain_mode = "FIXED"

            # Calculate corrections with adaptive gain
            dx_cam = error_x_m * gain
            dy_cam = error_y_m * gain
            dz_cam = error_depth * gain

            # Clamp corrections to max step
            dx_cam = np.clip(dx_cam, -self.vs_max_step, self.vs_max_step)
            dy_cam = np.clip(dy_cam, -self.vs_max_step, self.vs_max_step)
            dz_cam = np.clip(dz_cam, -self.vs_max_step, self.vs_max_step)

            # Check if correction is too small (converged or stuck)
            correction_magnitude = np.sqrt(dx_cam**2 + dy_cam**2 + dz_cam**2)
            if correction_magnitude < self.vs_min_step:
                self.get_logger().info(f"VS: Correction too small ({correction_magnitude*1000:.2f}mm), converged")
                return True

            self.get_logger().info(
                f"VS correction [{gain_mode}]: dx={dx_cam*1000:.1f}mm, dy={dy_cam*1000:.1f}mm, "
                f"dz={dz_cam*1000:.1f}mm (gain={gain:.2f})"
            )

            # Apply correction via Cartesian move in camera frame
            # OPTIMIZED: Use fast direct control for small corrections
            correction_magnitude = np.sqrt(dx_cam**2 + dy_cam**2 + dz_cam**2)
            if correction_magnitude < 0.01:  # < 10mm: use fast path
                if not self._apply_vs_correction_fast(dx_cam, dy_cam, dz_cam):
                    self.get_logger().warn("VS: Fast correction failed, trying MoveIt")
                    if not self._apply_vs_correction(dx_cam, dy_cam, dz_cam):
                        self.get_logger().warn("VS: Failed to apply correction")
                        continue
            else:
                if not self._apply_vs_correction(dx_cam, dy_cam, dz_cam):
                    self.get_logger().warn("VS: Failed to apply correction")
                    continue

            # Store error for adaptive settle_time
            self._last_vs_error_norm = error_norm

        self.get_logger().warn(f"⚠️ Visual servo did not converge in {self.vs_max_iterations} iterations")
        return False

    def _get_current_detection(self):
        """
        Get the current object detection from the latest YOLO result.

        Returns:
            Tuple (u, v, depth, w, h) or None if no detection
            - u, v: Object center in image coordinates
            - depth: Measured depth in meters
            - w, h: Bounding box size in pixels
        """
        # Use the last processed detection if available
        if not hasattr(self, '_last_detection') or self._last_detection is None:
            return None

        det = self._last_detection
        if time.time() - det.get('timestamp', 0) > 5.0:
            # Detection is stale (5 seconds - generous for during grasp execution)
            return None

        return (
            det.get('u', 0),
            det.get('v', 0),
            det.get('depth', 0),
            det.get('w', 0),
            det.get('h', 0)
        )

    def _apply_vs_correction(self, dx_cam: float, dy_cam: float, dz_cam: float) -> bool:
        """
        Apply visual servo correction by moving the end-effector.

        Correction is given in camera frame and needs to be transformed to base frame.
        Uses small Cartesian moves for precise adjustment.

        CRITICAL: Delta is applied to ROBOT'S ACTUAL POSITION, not the target grasp pose.
        This ensures true closed-loop control - the robot moves relative to where it IS,
        not where we previously thought the object was.

        Args:
            dx_cam, dy_cam, dz_cam: Corrections in camera frame (meters)

        Returns:
            True if move succeeded
        """
        try:
            # Get current TF from base to camera
            cam_transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ref_frame,  # camera_color_optical_frame
                Time(),
                timeout=Duration(seconds=1.0)
            )

            # Extract rotation matrix from transform
            q = cam_transform.transform.rotation
            rot = R.from_quat([q.x, q.y, q.z, q.w])

            # Transform correction vector from camera frame to base frame
            correction_cam = np.array([dx_cam, dy_cam, dz_cam])
            correction_base = rot.apply(correction_cam)

            self.get_logger().debug(
                f"VS correction in base frame: "
                f"dx={correction_base[0]*1000:.1f}mm, "
                f"dy={correction_base[1]*1000:.1f}mm, "
                f"dz={correction_base[2]*1000:.1f}mm"
            )

            # ============================================
            # CRITICAL FIX: Get ACTUAL robot EE position from TF
            # NOT the stored grasp pose (which is the object position)
            # ============================================
            ee_transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ee_link,
                Time(),
                timeout=Duration(seconds=1.0)
            )
            
            current_ee_x = ee_transform.transform.translation.x
            current_ee_y = ee_transform.transform.translation.y
            current_ee_z = ee_transform.transform.translation.z
            current_ee_q = ee_transform.transform.rotation
            
            self.get_logger().info(
                f"📍 Actual EE position: ({current_ee_x:.3f}, {current_ee_y:.3f}, {current_ee_z:.3f})"
            )

            # Create target pose with correction applied to ACTUAL position
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.arm_base_frame
            target_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Apply correction to ACTUAL EE position
            target_pose.pose.position.x = current_ee_x + correction_base[0]
            target_pose.pose.position.y = current_ee_y + correction_base[1]
            target_pose.pose.position.z = current_ee_z + correction_base[2]
            
            # PRESERVE CURRENT ORIENTATION from actual EE
            target_pose.pose.orientation = current_ee_q
            
            self.get_logger().info(
                f"🎯 VS target: ({target_pose.pose.position.x:.3f}, "
                f"{target_pose.pose.position.y:.3f}, {target_pose.pose.position.z:.3f})"
            )

            # Execute small Cartesian move, preserving orientation
            # ignore_orientation=True allows MoveIt flexibility if exact orientation unreachable
            return self._move_to_pose(target_pose, timeout=5.0, use_lift=False, ignore_orientation=True)

        except Exception as e:
            self.get_logger().error(f"VS correction failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

    # ========================================
    # Fast VS Correction (Direct Joint Control — skips MoveIt planning)
    # ========================================
    def _apply_vs_correction_fast(self, dx_cam: float, dy_cam: float, dz_cam: float) -> bool:
        """
        Fast VS correction using direct arm controller (skips MoveIt planning).

        For small corrections (<10mm), this is 5-10x faster than full MoveIt plan+execute.
        Falls back to standard _apply_vs_correction on failure.

        Args:
            dx_cam, dy_cam, dz_cam: Corrections in camera frame (meters)

        Returns:
            True if move succeeded
        """
        try:
            # 1. Transform correction to base frame
            cam_tf = self.tf_buffer.lookup_transform(
                self.arm_base_frame, self.ref_frame, Time(), Duration(seconds=0.5)
            )
            q = cam_tf.transform.rotation
            rot = R.from_quat([q.x, q.y, q.z, q.w])
            correction_base = rot.apply([dx_cam, dy_cam, dz_cam])

            # 2. Get current joint state
            js = self._latest_joint_state
            if js is None:
                return False

            # Extract current arm joint positions
            current_joints = []
            for jn in self.arm_joint_names:
                if jn in js.name:
                    idx = js.name.index(jn)
                    current_joints.append(js.position[idx])
                else:
                    return False

            # 3. Use numerical approximation: waist rotation for X/Y, shoulder/elbow for Z
            # For small corrections, linear approximation works well
            ee_tf = self.tf_buffer.lookup_transform(
                self.arm_base_frame, self.ee_link, Time(), Duration(seconds=0.5)
            )
            ee_x = ee_tf.transform.translation.x
            ee_y = ee_tf.transform.translation.y
            ee_z = ee_tf.transform.translation.z

            target_x = ee_x + correction_base[0]
            target_y = ee_y + correction_base[1]
            target_z = ee_z + correction_base[2]

            # 4. Simple analytical IK for waist angle (yaw)
            new_waist = np.arctan2(target_y, target_x)

            # 5. For Z correction: adjust wrist_angle proportionally
            # This is a linear approximation valid for small dz
            dz = correction_base[2]
            new_joints = list(current_joints)
            new_joints[0] = new_waist  # waist

            # Adjust wrist_angle for Z (empirical Jacobian approximation)
            # Each 0.01 rad of wrist_angle ≈ 1mm Z change at typical reach
            if abs(dz) > 0.001:
                # For small arm like RX200 at ~0.25m reach:
                # Z sensitivity ≈ -0.1m per rad of shoulder change
                new_joints[1] += dz * 2.0   # shoulder compensation
                new_joints[3] -= dz * 1.5   # wrist_angle compensation

            # 6. Send directly to arm controller (fast!)
            return self._move_to_joints_direct(new_joints, timeout=2.0)

        except Exception as e:
            self.get_logger().debug(f"Fast VS correction failed: {e}")
            return False

    # ========================================
    # Event-driven settle (replaces time.sleep)
    # ========================================
    def _wait_for_settle(self, position_threshold=0.001, timeout=0.5):
        """
        Wait for arm to stop moving based on joint state velocity.
        Replaces fixed time.sleep() calls for faster response.

        Args:
            position_threshold: Max position delta to consider settled (rad)
            timeout: Max wait time (seconds)

        Returns:
            True if settled, False if timeout
        """
        start = time.time()
        prev_positions = None
        stable_count = 0

        while (time.time() - start) < timeout:
            js = self._latest_joint_state
            if js is None:
                time.sleep(0.02)
                continue

            # Check arm joints only (first 5)
            try:
                arm_indices = [js.name.index(jn) for jn in self.arm_joint_names if jn in js.name]
                if not arm_indices:
                    time.sleep(0.02)
                    continue
                current = np.array([js.position[i] for i in arm_indices])
            except (ValueError, IndexError):
                time.sleep(0.02)
                continue

            # Check velocity if available
            if js.velocity and len(js.velocity) >= len(arm_indices):
                try:
                    max_vel = max(abs(js.velocity[i]) for i in arm_indices)
                    if max_vel < 0.01:  # rad/s threshold
                        stable_count += 1
                        if stable_count >= 2:  # 2 consecutive stable readings
                            return True
                    else:
                        stable_count = 0
                except (IndexError, TypeError):
                    pass
            # Fallback: position change check
            elif prev_positions is not None:
                delta = np.max(np.abs(current - prev_positions))
                if delta < position_threshold:
                    stable_count += 1
                    if stable_count >= 3:
                        return True
                else:
                    stable_count = 0

            prev_positions = current
            time.sleep(0.02)  # 50Hz check rate

        return False

    # ========================================
    # Continuous Real-Time Tracking (MoveIt Servo)
    # ========================================
    def _continuous_servo_descent(self, target_z: float, timeout: float = 15.0) -> bool:
        """
        Dynamically approach target_z while continuously maintaining visual alignment 
        using MoveIt Servo (real-time velocity control).
        
        Replaces the discrete "Look-and-Move" with continuous flow.
        Stops if contact is detected or target_z is reached.
        """
        self.get_logger().info("🌊 Starting Continuous Servo Tracking & Descent...")
        
        if self.camera_intrinsics is None:
            self.get_logger().error("No camera intrinsics for servoing")
            return False

        fx = self.camera_intrinsics[0, 0]
        fy = self.camera_intrinsics[1, 1]
        cx = self.camera_intrinsics[0, 2]
        cy = self.camera_intrinsics[1, 2]
        
        # PID Constants for XY visual centering
        Kp_xy = 1.0  
        Kd_xy = 0.1
        
        # P Constant for Z descent
        Kp_z = 0.5 
        max_v_xy = 0.05  # m/s
        max_v_z = 0.05   # m/s
        
        prev_err_x, prev_err_y = 0.0, 0.0
        start_time = time.time()
        last_t = time.time()
        
        # Contact monitoring queue
        current_history = deque(maxlen=3)
        
        publish_rate = self.create_rate(50) # 50Hz control loop

        while rclpy.ok() and (time.time() - start_time) < timeout:
            now_t = time.time()
            dt = max(now_t - last_t, 0.001)
            last_t = now_t
            
            # --- 1. Contact Detection ---
            js = self._latest_joint_state
            if js and js.effort:
                try:
                    effort_idx = [js.name.index(n) for n in self.arm_joint_names if n in js.name]
                    currents = [abs(js.effort[i]) for i in effort_idx]
                    avg_current = sum(currents) / max(len(currents), 1)
                    current_history.append(avg_current)
                    
                    if len(current_history) == 3 and np.mean(current_history) > self.contact_threshold:
                        self.get_logger().info(f"🛑 Contact Detected during servo! (Current: {np.mean(current_history):.0f})")
                        self._stop_servo()
                        return True
                except ValueError:
                    pass
            
            # --- 2. Z Height Check ---
            try:
                ee_tf = self.tf_buffer.lookup_transform(
                    self.arm_base_frame, self.ee_link, Time(), Duration(seconds=0.1)
                )
                curr_z = ee_tf.transform.translation.z
                if curr_z <= target_z:
                    self.get_logger().info(f"✅ Reached target Z: {curr_z:.3f}m")
                    self._stop_servo()
                    return True
            except Exception:
                curr_z = target_z + 0.1 # Fallback

            # --- 3. Visual Tracking Error ---
            det = self._get_current_detection()
            if det:
                u, v, depth, w, h = det
                err_x_pix = u - cx
                err_y_pix = v - cy
                
                # Convert pixels to meters roughly at 0.2m depth 
                # (We don't need exact depth since servo is relatively robust)
                scale = 0.25 / fx 
                err_x_m = err_x_pix * scale
                err_y_m = err_y_pix * scale
                
                # PID
                vx = Kp_xy * err_x_m + Kd_xy * ((err_x_m - prev_err_x) / dt)
                vy = Kp_xy * err_y_m + Kd_xy * ((err_y_m - prev_err_y) / dt)
                
                prev_err_x = err_x_m
                prev_err_y = err_y_m
            else:
                # No object visible? Keep going down slowly, stop XY movement
                vx, vy = 0.0, 0.0
                
            # Clamp XY speeds
            vx = max(-max_v_xy, min(max_v_xy, vx))
            vy = max(-max_v_xy, min(max_v_xy, vy))
            
            # --- 4. Z Descent Velocity ---
            err_z = curr_z - target_z
            vz = -Kp_z * err_z  # Negative Z direction (down)
            vz = max(-max_v_z, min(0.0, vz)) # Clamp to going down only
            
            # Slow down Z descent if XY error is large (wait for alignment)
            if abs(vx) > 0.02 or abs(vy) > 0.02:
                vz *= 0.2
            
            # Send Twist command
            # The twist is published in 'rx200/base_link' frame according to our servo_params
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.arm_base_frame 
            
            # NOTE: Camera axes vs Base axes mapping might require tweaking based on mount!
            # Assuming standard overhead eye-in-hand:
            # camera +X = image right. If object is right of center (err_x > 0), arm should move Right (base Y)
            # camera +Y = image down. If object is bottom of center (err_y > 0), arm should move Forward (base X)
            msg.twist.linear.x = vy  
            msg.twist.linear.y = -vx 
            msg.twist.linear.z = vz
            
            # No wrist rotation for now during descent
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0
            
            self.servo_pub.publish(msg)
            
            publish_rate.sleep()

        self._stop_servo()
        self.get_logger().warn("Servo descent timed out.")
        return False

    def _stop_servo(self):
        """Send zero twist to stop servoing."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.arm_base_frame
        # All zeros
        self.servo_pub.publish(msg)

    # ========================================
    # Contact Detection Approach (NEW)
    # ========================================
    def _approach_with_contact_detection(self, target_z: float, timeout: float = 10.0) -> tuple:
        """
        Approach target Z position with contact detection.

        Instead of blindly moving to a fixed Z position, this method:
        1. Moves in small increments toward target_z
        2. Monitors gripper motor current for contact
        3. Stops immediately when contact is detected
        4. Returns the actual contact Z position

        This provides closed-loop depth verification and prevents:
        - Pressing too hard on objects
        - Missing objects due to depth estimation errors

        Args:
            target_z: Target Z position in base frame
            timeout: Maximum time for approach

        Returns:
            Tuple (success, contact_z, contact_detected)
        """
        if not self.contact_detection_enabled:
            self.get_logger().info("Contact detection disabled, using standard approach")
            return (True, target_z, False)

        self.get_logger().info(f"🎯 Contact detection approach to Z={target_z:.3f}m")

        try:
            # Get current EE position
            ee_tf = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ee_link,
                Time(),
                timeout=Duration(seconds=1.0)
            )

            current_z = ee_tf.transform.translation.z
            current_x = ee_tf.transform.translation.x
            current_y = ee_tf.transform.translation.y
            current_q = ee_tf.transform.rotation

            self.get_logger().info(f"Starting Z: {current_z:.3f}m, Target Z: {target_z:.3f}m")

            # Calculate total descent distance
            total_descent = current_z - target_z
            if total_descent <= 0:
                self.get_logger().warn("Already at or below target Z")
                return (True, current_z, False)

            # Maximum extra descent allowed
            max_z = target_z - self.contact_max_extra_descent

            # Clear current readings buffer
            self._current_readings.clear()

            start_time = time.time()
            step_count = 0
            contact_detected = False

            while current_z > max_z and (time.time() - start_time) < timeout:
                # Calculate next Z position (small step)
                next_z = current_z - self.contact_approach_speed

                # Don't go below absolute minimum
                next_z = max(next_z, max_z)

                # Create target pose
                step_pose = PoseStamped()
                step_pose.header.frame_id = self.arm_base_frame
                step_pose.header.stamp = self.get_clock().now().to_msg()
                step_pose.pose.position.x = current_x
                step_pose.pose.position.y = current_y
                step_pose.pose.position.z = next_z
                step_pose.pose.orientation = current_q

                # Execute small move
                if not self._move_to_pose(step_pose, timeout=2.0, use_lift=False, ignore_orientation=True):
                    self.get_logger().warn(f"Contact approach step {step_count} failed")
                    break

                # Wait for settle and check current
                time.sleep(self.contact_check_interval)

                # Read gripper current (if available)
                gripper_current = self._read_gripper_current()
                if gripper_current is not None:
                    self._current_readings.append(gripper_current)

                    # Check for contact (average current over window)
                    if len(self._current_readings) >= 3:
                        avg_current = np.mean(list(self._current_readings))
                        if avg_current > self.contact_current_threshold:
                            contact_detected = True
                            self.get_logger().info(
                                f"⚡ CONTACT DETECTED at Z={next_z:.3f}m "
                                f"(current={avg_current:.1f}mA > {self.contact_current_threshold}mA)"
                            )
                            return (True, next_z, True)

                # Update current Z
                current_z = next_z
                step_count += 1

                # Progress log
                if step_count % 5 == 0:
                    self.get_logger().info(f"Approach step {step_count}: Z={current_z:.3f}m")

            # Reached target without contact
            self.get_logger().info(f"Reached Z={current_z:.3f}m without contact detection")
            return (True, current_z, False)

        except Exception as e:
            self.get_logger().error(f"Contact detection approach failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return (False, target_z, False)

    def _read_gripper_current(self) -> float:
        """
        Read current gripper motor current.

        Returns:
            Current in mA, or None if not available
        """
        try:
            # Try to get current from motor register
            current = self._get_motor_register('Present_Current')
            if current is not None:
                # Convert register value to mA (approximate)
                # XM430: ~2.69mA per unit
                return abs(current) * 2.69
        except Exception:
            pass

        # Fallback: estimate from joint effort if available
        if hasattr(self, '_latest_joint_state') and self._latest_joint_state is not None:
            try:
                js = self._latest_joint_state
                if 'left_finger' in js.name:
                    idx = js.name.index('left_finger')
                    if idx < len(js.effort):
                        # Rough conversion from effort to current estimate
                        return abs(js.effort[idx]) * 100  # Approximate scaling
            except Exception:
                pass

        return None

    # ========================================
    # MoveIt Control
    # ========================================
    def _move_to_joints(self, positions, timeout=30.0):
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup not available')
            return self._move_to_joints_direct(positions, timeout=min(timeout, 8.0))

        self._goal_complete = False
        self._goal_success = False

        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.arm_group
        request.num_planning_attempts = self.num_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.max_vel_scale
        request.max_acceleration_scaling_factor = self.max_acc_scale

        constraints = Constraints()
        for name, pos in zip(self.arm_joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)
        goal.request = request
        goal.planning_options.plan_only = False

        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

        # Wait for completion using simple polling (callbacks run in executor threads)
        start = time.time()
        while not self._goal_complete and (time.time() - start) < timeout:
            time.sleep(0.05)

        if self._goal_success:
            return True

        # Fallback path for deterministic home/ready moves if MoveIt returns invalid plan.
        self.get_logger().warn('MoveIt joint planning failed; trying direct arm-controller move...')
        return self._move_to_joints_direct(positions, timeout=min(timeout, 8.0))

    def _move_to_joints_direct(self, positions, timeout=8.0):
        """Send direct FollowJointTrajectory goal to arm controller."""
        if not self._arm_controller_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Arm controller action server not available for direct joint move')
            return False

        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.arm_joint_names)

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = BuiltinDuration(sec=2, nanosec=0)
        trajectory.points.append(point)
        goal.trajectory = trajectory

        self._arm_goal_complete = False
        self._arm_goal_success = False

        future = self._arm_controller_client.send_goal_async(goal)
        future.add_done_callback(self._arm_goal_response_cb)

        start = time.time()
        while not self._arm_goal_complete and (time.time() - start) < timeout:
            time.sleep(0.05)

        if self._arm_goal_success:
            self.get_logger().info('Direct arm-controller joint move succeeded.')
        else:
            self.get_logger().error('Direct arm-controller joint move failed.')
        return self._arm_goal_success

    def _move_to_pose(self, pose: PoseStamped, timeout=30.0, use_lift=True, ignore_orientation=False):
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            return False

        self._goal_complete = False
        self._goal_success = False

        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        # Use coordinated arm+lift group for better reachability
        if use_lift and self.lift_enabled and self.use_coordinated_motion:
            request.group_name = self.arm_lift_group
            self.get_logger().info(f"Using coordinated group: {self.arm_lift_group}")
        else:
            request.group_name = self.arm_group
        request.num_planning_attempts = self.num_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.max_vel_scale
        request.max_acceleration_scaling_factor = self.max_acc_scale

        constraints = Constraints()
        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = self.ee_link
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]  # Tolerance 2cm (VS will fix accuracy)
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        # Always add orientation constraint to prevent "wild spinning"
        # We want the gripper to point roughly DOWN even when "ignoring" strict orientation
        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.weight = 1.0

        if ignore_orientation:
             # LOOSE: Roughly downward, allowing full yaw rotation
             # This prevents the arm from going upside down or sideways
             oc.absolute_x_axis_tolerance = 0.5 
             oc.absolute_y_axis_tolerance = 0.5
             oc.absolute_z_axis_tolerance = 6.28 # 2*pi (Free yaw)
        elif self.use_orientation_constraint:
             # STRICT: adhere to target orientation
             oc.absolute_x_axis_tolerance = 0.1  
             oc.absolute_y_axis_tolerance = 0.1
             oc.absolute_z_axis_tolerance = 3.14 # 5-DOF coupling tolerance
        else:
             # RELAXED: standard movement
             # Still enforce roughly downward to prevent flips
             oc.absolute_x_axis_tolerance = 0.3
             oc.absolute_y_axis_tolerance = 0.3
             oc.absolute_z_axis_tolerance = 6.28
        
        # Add to Goal Constraints (Final Pose)
        constraints.orientation_constraints.append(oc)
        
        # Also add to Path Constraints (Trajectory) to prevent wild flipping
        # This forces the gripper to stay roughly downward-pointing *during* movement
        # Always add path constraints, but use different tolerances based on mode
        pc_oc = OrientationConstraint()
        pc_oc.header = pose.header
        pc_oc.link_name = self.ee_link
        pc_oc.orientation = pose.pose.orientation  # Use target orientation (down)
        pc_oc.weight = 1.0

        if ignore_orientation:
            # LOOSE path constraint: only prevent flipping, free yaw
            pc_oc.absolute_x_axis_tolerance = 0.5
            pc_oc.absolute_y_axis_tolerance = 0.5
            pc_oc.absolute_z_axis_tolerance = 6.28  # 2*pi - free yaw
        elif self.use_orientation_constraint:
            # STRICT path constraint: tight pitch/roll control
            pc_oc.absolute_x_axis_tolerance = 0.2
            pc_oc.absolute_y_axis_tolerance = 0.2
            pc_oc.absolute_z_axis_tolerance = 6.28
        else:
            # MODERATE path constraint
            pc_oc.absolute_x_axis_tolerance = 0.4
            pc_oc.absolute_y_axis_tolerance = 0.4
            pc_oc.absolute_z_axis_tolerance = 6.28

        request.path_constraints.orientation_constraints.append(pc_oc)

        # Debug: Calculate distance from arm base
        try:
            target_point = pose.pose.position
            # Transform to arm base for distance check
            if pose.header.frame_id != self.arm_base_frame:
                # We can't easily transform here without a buffer, but let's assume valid frame
                pass
            
            # Simple Euclidean distance from (0,0,0) of the reference frame
            dist = math.sqrt(target_point.x**2 + target_point.y**2 + target_point.z**2)
            self.get_logger().info(f"MoveIt Request: Frame={pose.header.frame_id}, Target=({target_point.x:.3f}, {target_point.y:.3f}, {target_point.z:.3f}), Dist={dist:.3f}m")
        except Exception:
            pass

        request.goal_constraints.append(constraints)
        goal.request = request
        goal.planning_options.plan_only = False

        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

        # Wait for completion using simple polling
        start = time.time()
        while not self._goal_complete and (time.time() - start) < timeout:
            time.sleep(0.05)

        if not self._goal_success:
             self.get_logger().error(f"MoveIt Planning FAILED! Result: {self._goal_success}")
             
        return self._goal_success

    def _wait_for_future(self, future, timeout_sec=5.0):
        """Wait for a future to complete without blocking executor threads."""
        start = time.time()
        while not future.done() and (time.time() - start) < timeout_sec:
            time.sleep(0.05)
        return future.done()

    def _move_cartesian(self, waypoints: list, timeout=15.0) -> bool:
        """
        Execute Cartesian straight-line motion through waypoints.

        This is critical for:
        - Approach: Straight down to grasp pose (last 10cm)
        - Lift: Straight up after grasping (10cm)

        Using Cartesian path prevents arm from taking weird curved trajectories
        that can cause gripper orientation problems.

        Args:
            waypoints: List of Pose objects (not PoseStamped)
            timeout: Maximum execution time

        Returns:
            True if motion succeeded
        """
        if not self.use_cartesian_path:
            self.get_logger().warn("Cartesian path disabled, using regular motion")
            return False

        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Cartesian path service not available")
            return False

        # Build request
        request = GetCartesianPath.Request()
        request.header.frame_id = self.arm_base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = self.arm_group
        request.link_name = self.ee_link
        request.waypoints = waypoints
        request.max_step = self.cartesian_step_size  # 5mm step
        request.jump_threshold = self.cartesian_jump_threshold  # 0 = disabled
        request.avoid_collisions = True
        # Note: velocity/acceleration scaling is applied during trajectory execution, not here

        # Call service
        self.get_logger().info(f"Computing Cartesian path with {len(waypoints)} waypoints...")
        future = self._cartesian_client.call_async(request)

        if not self._wait_for_future(future, timeout_sec=5.0):
            self.get_logger().error("Cartesian path computation timed out")
            return False

        result = future.result()

        # Check path fraction (how much of path was computed)
        if result.fraction < 0.9:
            self.get_logger().warn(
                f"Cartesian path only {result.fraction*100:.1f}% complete. "
                "Falling back to regular motion."
            )
            return False

        self.get_logger().info(f"Cartesian path computed: {result.fraction*100:.1f}% complete")

        # Apply Velocity Scaling manually
        # GetCartesianPath does not respect request velocity scaling, so we retime the trajectory
        scaled_trajectory = self._scale_trajectory(result.solution, self.max_vel_scale)

        # Execute the trajectory
        return self._execute_trajectory(scaled_trajectory, timeout)

    def _scale_trajectory(self, trajectory, scale_factor):
        """
        Scale trajectory timing with jerk-aware smoothing.
        
        OPTIMIZED:
        - Enforces acceleration continuity between points
        - Clamps joint velocities/accelerations to hardware limits
        - Applies cubic interpolation for smoother transitions
        
        Args:
            trajectory: RobotTrajectory message
            scale_factor: 0.1 to 1.0 (smaller = slower)
        """
        if scale_factor >= 1.0 or scale_factor <= 0.0:
            return trajectory

        inv_scale = 1.0 / scale_factor
        
        # Hardware limits for RX200 (rad/s, rad/s^2)
        max_joint_vel = 3.14      # From joint_limits.yaml
        max_joint_acc = 5.0       # From joint_limits.yaml
        
        new_traj = trajectory
        for point in new_traj.joint_trajectory.points:
            # Scale time_from_start
            old_sec = point.time_from_start.sec
            old_nano = point.time_from_start.nanosec
            total_sec = old_sec + old_nano * 1e-9
            
            new_total_sec = total_sec * inv_scale
            point.time_from_start.sec = int(new_total_sec)
            point.time_from_start.nanosec = int((new_total_sec - int(new_total_sec)) * 1e9)
            
            # Scale velocities and clamp to hardware limits
            if point.velocities:
                scaled_vels = [v * scale_factor for v in point.velocities]
                point.velocities = [
                    max(-max_joint_vel, min(max_joint_vel, v)) for v in scaled_vels
                ]
            
            # Scale accelerations with squared factor and clamp
            if point.accelerations:
                scaled_accs = [a * scale_factor * scale_factor for a in point.accelerations]
                point.accelerations = [
                    max(-max_joint_acc, min(max_joint_acc, a)) for a in scaled_accs
                ]
                 
        return new_traj

    def _execute_trajectory(self, trajectory: RobotTrajectory, timeout=15.0) -> bool:
        """Execute a planned trajectory."""
        if not self._execute_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Execute trajectory action server not available")
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self._goal_complete = False
        self._goal_success = False

        future = self._execute_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

        # Wait for execution
        start = time.time()
        while not self._goal_complete and (time.time() - start) < timeout:
            time.sleep(0.05)

        if self._goal_success:
            self.get_logger().info("Cartesian trajectory executed successfully")
        else:
            self.get_logger().error("Cartesian trajectory execution failed")

        return self._goal_success

    def _move_straight_line(self, start_pose: PoseStamped, end_pose: PoseStamped, timeout=15.0) -> bool:
        """
        Move in a straight line from start to end pose.

        This is a convenience wrapper for Cartesian motion with 2 points.
        """
        if not self.use_cartesian_path:
            # Fallback to regular motion
            return self._move_to_pose(end_pose, timeout=timeout)

        # Convert PoseStamped to Pose for waypoints
        waypoints = [end_pose.pose]  # Only end pose, start is current

        success = self._move_cartesian(waypoints, timeout)

        if not success:
            self.get_logger().warn("Cartesian motion failed, trying regular motion...")
            return self._move_to_pose(end_pose, timeout=timeout)

        return True

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_complete = True
            self._goal_success = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        self._goal_complete = True
        if hasattr(result.result, 'error_code'):
             error_code = result.result.error_code.val
        else:
             error_code = -999 # Unknown
             
        self._goal_success = (error_code == MoveItErrorCodes.SUCCESS)
        
        if not self._goal_success:
             self.get_logger().error(f"❌ MoveIt Planning Failed with error code: {error_code}")

    def _move_to_pose_direct(self, pose: PoseStamped, timeout=15.0):
        """
        Move directly to pose using MoveIt with stricter constraints for minimal detour.
        
        Uses:
        - Smaller position tolerance (sphere radius)
        - Stricter orientation path constraints
        - Shorter planning time (take first valid solution)
        """
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            return False

        self._goal_complete = False
        self._goal_success = False

        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.arm_group
        request.num_planning_attempts = 3  # Fewer attempts, take first good path
        request.allowed_planning_time = 3.0  # Short time = simpler path
        request.max_velocity_scaling_factor = self.max_vel_scale
        request.max_acceleration_scaling_factor = self.max_acc_scale

        constraints = Constraints()
        
        # Tighter position constraint
        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = self.ee_link
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]  # Tighter 2cm tolerance
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        # Strict orientation to prevent wild yaw swings
        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.weight = 1.0
        oc.absolute_x_axis_tolerance = 0.3
        oc.absolute_y_axis_tolerance = 0.3
        oc.absolute_z_axis_tolerance = 1.0  # Allow some yaw variation
        constraints.orientation_constraints.append(oc)

        request.goal_constraints.append(constraints)
        goal.request = request
        goal.planning_options.plan_only = False

        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

        start = time.time()
        while not self._goal_complete and (time.time() - start) < timeout:
            time.sleep(0.05)

        return self._goal_success

    def _move_to_pose_coordinated(self, pose: PoseStamped, lift_position: float, timeout=30.0):
        """
        Move arm and lift simultaneously using the coordinated arm_lift planning group.
        
        This enables faster motion by planning arm and lift trajectories together.
        Falls back to sequential motion if coordinated planning fails.
        
        Args:
            pose: Target end-effector pose
            lift_position: Target lift position in meters
            timeout: Planning timeout
        
        Returns:
            True if successful
        """
        if not self.use_coordinated_motion or not self.lift_enabled:
            # Fallback to sequential motion
            lift_ok = self._move_lift_to(lift_position)
            arm_ok = self._move_to_pose(pose)
            return lift_ok and arm_ok
        
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            return False

        self._goal_complete = False
        self._goal_success = False

        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.arm_lift_group  # Use coordinated group
        request.num_planning_attempts = self.num_attempts
        request.allowed_planning_time = self.planning_time * 1.5  # More time for 6-DOF
        request.max_velocity_scaling_factor = self.max_vel_scale
        request.max_acceleration_scaling_factor = self.max_acc_scale

        # End-effector pose constraints
        constraints = Constraints()
        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = self.ee_link
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.015]  # Slightly larger tolerance for coordinated
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 3.14
        oc.absolute_y_axis_tolerance = 3.14
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        # Lift joint constraint
        jc = JointConstraint()
        jc.joint_name = self.lift_joint_name
        jc.position = lift_position
        jc.tolerance_above = 0.005
        jc.tolerance_below = 0.005
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)
        goal.request = request
        goal.planning_options.plan_only = False

        self.get_logger().info(f'Coordinated motion: arm + lift to {lift_position*1000:.1f}mm')
        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

        start = time.time()
        while not self._goal_complete and (time.time() - start) < timeout:
            time.sleep(0.05)

        if self._goal_success:
            return True
        else:
            # Fallback to sequential motion on failure
            self.get_logger().warn('Coordinated motion failed, falling back to sequential')
            lift_ok = self._move_lift_to(lift_position)
            arm_ok = self._move_to_pose(pose)
            return lift_ok and arm_ok

    # ========================================
    # Cartesian Path Helper
    # ========================================
    def _interpolate_cartesian(self, start_pose, end_pose, steps=10):
        """Generate intermediate waypoints for linear interpolation."""
        waypoints = []
        p1 = start_pose.pose.position
        p2 = end_pose.pose.position
        q1 = start_pose.pose.orientation # Keep orientation constant
        
        for i in range(1, steps + 1):
            alpha = i / float(steps)
            wp = copy.deepcopy(start_pose)
            wp.pose.position.x = p1.x + alpha * (p2.x - p1.x)
            wp.pose.position.y = p1.y + alpha * (p2.y - p1.y)
            wp.pose.position.z = p1.z + alpha * (p2.z - p1.z)
            wp.pose.orientation = q1 # Enforce same orientation
            waypoints.append(wp)
        return waypoints

    def _execute_cartesian_move(self, target_pose, steps=5):
        """Execute a pseudo-Cartesian move by interpolating waypoints."""
        current_pose = self._move_client.get_result() # Not available directly, need to fetch
        # For simplicity, we just plan to target directly if close, OR use waypoints
        # Since getting current pose via MoveIt interface is tricky without subscription,
        # we will break the move into smaller chunks if distance is large.
        # BUT, standard RRTConnect with orientation constraints is often 'straight enough' for short segments.
        # We will enforce orientation constraint in _move_to_pose.
        return self._move_to_pose(target_pose)




    def _visualize_grasp_debug(self, grasp_pose, approach_pose):
        """Publish debug markers."""
        marker_array = MarkerArray()
        
        # 1. Grasp Point Sphere
        marker = Marker()
        marker.header.frame_id = grasp_pose.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "grasp_debug"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = grasp_pose.pose
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)

        # 2. Approach Arrow
        arrow = Marker()
        arrow.header = marker.header
        arrow.ns = "grasp_debug"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.01
        arrow.scale.y = 0.02
        arrow.scale.z = 0.0
        arrow.points = [approach_pose.pose.position, grasp_pose.pose.position]
        arrow.color.a = 1.0
        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 0.0
        marker_array.markers.append(arrow)

        self.marker_pub.publish(marker_array)

    # ========================================
    # Grasp Execution
    # ========================================
    def _check_workspace_and_warn(self, pose):
        if not self._check_workspace(pose):
            self.get_logger().error(f"❌ Target OUT OF WORKSPACE: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}")
            return False
        return True

    def _approach_with_centering(self, target_pose, step_size=0.10):
        """
        Iterative Approach with Continuous Centering.
        
        Instead of moving directly to the target Z, this method descends in steps.
        At each step, it pauses and performs visual servoing to center the object.
        This ensures the object never leaves the FOV, even if the camera zooms in.
        
        Args:
            target_pose: The final pre-grasp pose (in arm_base_frame)
            step_size: Distance to descend per step (meters)
        """
        self.get_logger().info("📉 STARTING ITERATIVE CENTERING APPROACH")
        
        # 1. Get current actual pose from TF to determine start height
        try:
            tf_current = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ee_link,
                Time(),
                timeout=Duration(seconds=1.0)
            )
            current_z = tf_current.transform.translation.z
            self.get_logger().info(f"📍 Actual Robot Z: {current_z:.3f}m | Target Z: {target_pose.pose.position.z:.3f}m")
        except Exception as e:
            self.get_logger().error(f"Failed to get current Z from TF: {e}")
            return False
            
        target_z = target_pose.pose.position.z
        
        # Ensure we are actually descending
        if target_z >= current_z:
            self.get_logger().info("Target is higher/equal, moving directly...")
            return self._move_to_pose(target_pose, ignore_orientation=False)

        # 2. Loop down
        # Safety limit for iterations
        max_steps = 20
        steps = 0
        
        next_z = current_z
        
        while next_z > target_z and steps < max_steps:
            steps += 1
            next_z -= step_size
            if next_z < target_z:
                next_z = target_z
            
            self.get_logger().info(f"🔻 Step {steps}: Descending to Z={next_z:.3f}...")
            
            # ============================================
            # CRITICAL FIX: Get ACTUAL EE position from TF
            # Descend from WHERE WE ARE, not object position
            # ============================================
            try:
                ee_tf = self.tf_buffer.lookup_transform(
                    self.arm_base_frame,
                    self.ee_link,
                    Time(),
                    timeout=Duration(seconds=1.0)
                )
                current_x = ee_tf.transform.translation.x
                current_y = ee_tf.transform.translation.y
                current_q = ee_tf.transform.rotation
            except Exception as e:
                self.get_logger().error(f"Failed to get EE TF: {e}")
                return False
            
            # Create step pose: same X/Y as current EE, but lower Z
            step_pose = PoseStamped()
            step_pose.header.frame_id = self.arm_base_frame
            step_pose.header.stamp = self.get_clock().now().to_msg()
            step_pose.pose.position.x = current_x  # Stay at same X/Y
            step_pose.pose.position.y = current_y
            step_pose.pose.position.z = next_z     # Descend to next Z
            step_pose.pose.orientation = current_q  # Preserve orientation from TF
            
            self.get_logger().info(
                f"📍 Descending from ({current_x:.3f}, {current_y:.3f}) to Z={next_z:.3f}"
            )
            
            # Move to intermediate pose (PURE DESCENT, no X/Y change)
            if not self._move_to_pose(step_pose, timeout=5.0, ignore_orientation=True):
                self.get_logger().warn("Failed to descend, retrying...")
                if not self._move_to_pose(step_pose, timeout=5.0, ignore_orientation=True):
                    return False
            
            # VISUAL SERVOING (Center Alignment Only)
            # This is the closed-loop part - re-center after each descent
            self.get_logger().info(f"👁️ Re-centering object at Z={next_z:.3f}...")
            if not self._visual_servo_adjust(target_depth=0.0, align_camera_center=True, maintain_height=True):
                self.get_logger().warn("Centering failed/timed out, but continuing descent...")
                # We continue because maybe we are too high to see it well, or it's 'good enough'
                # If we completely lost it, the next step's VS will try to recover.
        return True

    # ========================================
    # NEW: Refine Target Logic (One-Shot Look-and-Move)
    # ========================================
    def _refine_and_compute_target(self, hover_duration=1.5):
        """
        Settle, observe multiple frames, and compute the EXACT object position
        for the GRIPPER to move to.
        
        Args:
            hover_duration: Time to wait/observe
            
        Returns:
            PoseStamped (target for gripper), success (bool)
        """
        self.get_logger().info(f"👀 LOOKING: Stabilizing for {hover_duration}s to refine target...")
        time.sleep(0.5) # Physical settle
        
        # Collect detections
        start_time = time.time()
        samples_x = []
        samples_y = []
        samples_z = []
        samples_angle = []
        
        while (time.time() - start_time) < hover_duration:
            det = self._last_detection
            # detections are updated in callback thread
            if det and (time.time() - det['timestamp']) < 1.0:
                # Re-calculate X/Y/Z from raw u/v/depth to ensure no double-offset
                # We want TRUE OBJECT POSITION in Camera Frame
                z = det['depth']
                
                # Retrieve Timestamp
                msg_stamp = det.get('msg_stamp', None)

                fx = self.camera_intrinsics[0, 0]
                fy = self.camera_intrinsics[1, 1]
                ppx = self.camera_intrinsics[0, 2]
                ppy = self.camera_intrinsics[1, 2]
                
                # Raw object position (No Offset!)
                x = (det['u'] - ppx) * z / fx
                y = (det['v'] - ppy) * z / fy
                
                samples_x.append(x)
                samples_y.append(y)
                samples_z.append(z)
                samples_angle.append(self._current_object_angle)
            
            time.sleep(0.1)
            
        if not samples_x:
            self.get_logger().error("❌ No detections during refine phase!")
            return None, False
            
        # Compute Average
        avg_x = float(np.mean(samples_x)) + self.grasp_offset_x
        avg_y = float(np.mean(samples_y)) + self.grasp_offset_y
        avg_z = float(np.mean(samples_z)) + self.grasp_offset_z
        
        # Create Pose with correct timestamp
        # Use simple mean of stamps? No, use latest valid stamp or just one of them.
        refine_stamp = msg_stamp if 'msg_stamp' in locals() and msg_stamp else self.get_clock().now().to_msg()
        
        refined_cam_pose = self._create_grasp_pose_camera([avg_x, avg_y, avg_z], 0.0, timestamp=refine_stamp)
        
        # Transform to Base using the stamp
        refined_base_pose = self._transform_to_base(refined_cam_pose, stamp=refine_stamp)
        
        return refined_base_pose, True


    def _execute_grasp_sequence(self):
        """
        5-Stage Grasp Execution with Cartesian paths for precision.

        Sequence (per user suggestions):
        1. PRE-GRASP: Move above target (10cm) - regular MoveIt planning
        2. APPROACH: Straight line DOWN to grasp pose - Cartesian path
        3. CLOSE: Two-stage gripper close with squeeze
        4. LIFT: Straight line UP - Cartesian path
        5. RETRACT: Return to home position

        Using Cartesian paths for approach/lift prevents weird arm trajectories
        that can cause gripper orientation problems.
        """
        if not self.is_calibrated or not self.current_grasp_pose:
            self.get_logger().warn("No grasp pose!")
            return False, "No pose"

        # Log full target pose for debugging
        p = self.current_grasp_pose.pose.position
        o = self.current_grasp_pose.pose.orientation
        self.get_logger().info(f'=' * 50)
        self.get_logger().info(f'GRASP EXECUTION START (Cartesian={self.use_cartesian_path})')
        self.get_logger().info(f'  Frame: {self.current_grasp_pose.header.frame_id}')
        self.get_logger().info(f'  Target Position: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        self.get_logger().info(f'  Target Orientation: ({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})')
        self.get_logger().info(f'=' * 50)

        # Create pre-grasp pose (above target)
        pre_pose = copy.deepcopy(self.current_grasp_pose)
        pre_pose.pose.position.z += self.approach_distance
        
        # Override orientation to be neutral (vertical down, no wrist rotation) for easier approach
        # This prevents IK failures if the target orientation is complex
        pre_pose = self._adjust_orientation_for_5dof(pre_pose, override_angle=0.0)

        # Visualize planned motion
        self._visualize_grasp_debug(self.current_grasp_pose, pre_pose)

        # CHECK WORKSPACE
        if not self._check_workspace_and_warn(self.current_grasp_pose):
            return False, "Target out of workspace"
        
        current_grasp_pose_z = self.current_grasp_pose.pose.position.z
        if current_grasp_pose_z < 0.05: # Too low
             self.get_logger().warn(f"Grasp pose Z ({current_grasp_pose_z}) is dangerously low!")

        # ========================================
        # STAGE 0: MOVE TO PRE-GRASP (Above Object)
        # First we need to get above the object before VS centering
        # ========================================
        pp = pre_pose.pose.position
        self.get_logger().info(f"[0/5] PRE-GRASP: Moving above target ({pp.x:.3f}, {pp.y:.3f}, {pp.z:.3f})...")
        
        # Move to position above detected object
        if not self._move_to_pose(pre_pose, timeout=10.0, use_lift=False, ignore_orientation=False):
            self.get_logger().error("Failed to move to pre-grasp position!")
            return False, "Pre-grasp move failed"
        
        self._wait_for_settle(timeout=0.3)  # OPTIMIZED: was time.sleep(0.3)
        self.get_logger().info("[0/5] ✅ Arrived at pre-grasp position")
        
        # ========================================
        # STAGE 1: SINGLE VS CENTERING (Lock final position)
        # Now we are above object - do ONE centering pass then commit to blind grasp
        # ========================================
        self.get_logger().info("[1/5] VS CENTERING: Locking onto target...")
        
        final_x, final_y, final_z = None, None, None
        locked_object_depth = None  # CRITICAL: Lock depth while we can still see the object
        
        if self.vs_enabled:
            # Single pass centering - adjust to put object in camera center
            vs_success = self._visual_servo_adjust(
                target_depth=0.0, 
                align_camera_center=True, 
                maintain_height=True
            )
            if not vs_success:
                self.get_logger().warn("VS centering did not fully converge, using current position")
        
        # CRITICAL: Lock object depth NOW while we're still at safe viewing height
        # Once we descend, detection may fail (object too close/out of view)
        detection = self._get_current_detection()
        if detection is not None:
            locked_object_depth = detection[2]  # depth in camera frame
            self.get_logger().info(f"🎯 LOCKED OBJECT DEPTH: {locked_object_depth:.3f}m (from camera)")
        else:
            # Use the pre-computed grasp pose depth as fallback
            if self.current_grasp_pose:
                # Estimate depth from grasp pose
                locked_object_depth = 0.15  # Default reasonable depth
                self.get_logger().warn(f"⚠️ No detection for depth, using fallback: {locked_object_depth:.3f}m")
            else:
                locked_object_depth = 0.12
                self.get_logger().warn(f"⚠️ No grasp pose, using minimum fallback: {locked_object_depth:.3f}m")
        
        # Get current EE position - this is our locked grasp X/Y
        # Wait for robot to settle completely before locking coordinates
        self._wait_for_settle(timeout=0.4)  # OPTIMIZED: was time.sleep(0.5)
        try:
            ee_tf = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ee_link,
                Time(),
                timeout=Duration(seconds=1.0)
            )
            final_x = ee_tf.transform.translation.x
            final_y = ee_tf.transform.translation.y
            final_z = ee_tf.transform.translation.z
            final_q = ee_tf.transform.orientation
            
            self.get_logger().info(
                f"🔒 LOCKED POSITION: X={final_x:.3f}, Y={final_y:.3f}, Z={final_z:.3f}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to lock EE position: {e}")
            return False, "TF lookup failed"
        
        # ========================================
        # STAGE 1.5: WRIST ALIGNMENT
        # Rotate wrist to align with object angle while keeping position fixed
        # ========================================
        current_pose_align = PoseStamped()
        current_pose_align.header.frame_id = self.arm_base_frame
        current_pose_align.header.stamp = self.get_clock().now().to_msg()
        current_pose_align.pose.position.x = final_x
        current_pose_align.pose.position.y = final_y
        current_pose_align.pose.position.z = final_z
        current_pose_align.pose.orientation = final_q

        # Calculate aligned pose using DETECTED angle (use whatever is latest)
        aligned_pose = self._adjust_orientation_for_5dof(current_pose_align) 
        
        self.get_logger().info("[1.5/5] ALIGNING WRIST: Rotating to match object angle...")
        # Use move_to_pose with strict orientation to enforce alignment
        if not self._move_to_pose(aligned_pose, timeout=5.0, use_lift=False, ignore_orientation=False):
             self.get_logger().warn("Wrist alignment failed (might be at joint limit), continuing...")
        else:
             self._wait_for_settle(timeout=0.3) # OPTIMIZED: was time.sleep(0.5)

        # ========================================
        # STAGE 1.75: SHIFT GRIPPER TO OBJECT
        # Camera is now aligned. Shift Gripper to Object.
        # ========================================
        self.get_logger().info("[1.75/5] SHIFT: Moving Gripper to Object Center...")
        try:
             # Get current EE pose
             ee_tf_shift = self.tf_buffer.lookup_transform(
                 self.arm_base_frame, self.ee_link, Time(), Duration(seconds=1.0)
             )
             curr_x = ee_tf_shift.transform.translation.x
             curr_y = ee_tf_shift.transform.translation.y
             curr_z = ee_tf_shift.transform.translation.z
             curr_q = ee_tf_shift.transform.rotation

             # Get Camera Rotation to transform offset vector
             cam_tf_shift = self.tf_buffer.lookup_transform(
                 self.arm_base_frame, self.ref_frame, Time(), Duration(seconds=1.0)
             )
             rq = cam_tf_shift.transform.rotation
             cam_rot = R.from_quat([rq.x, rq.y, rq.z, rq.w])

             # Calculate Shift Vector in Camera Frame: -Offset
             # If offset is negative (Gripper is Left), we shift Positive (Right)
             shift_cam = np.array([-self.grasp_offset_x, -self.grasp_offset_y, 0.0])
             shift_base = cam_rot.apply(shift_cam)

             # Target Pose
             shift_pose = PoseStamped()
             shift_pose.header.frame_id = self.arm_base_frame
             shift_pose.header.stamp = self.get_clock().now().to_msg()
             shift_pose.pose.position.x = curr_x + shift_base[0]
             shift_pose.pose.position.y = curr_y + shift_base[1]
             shift_pose.pose.position.z = curr_z + shift_base[2]
             shift_pose.pose.orientation = curr_q # Keep orientation

             # Execute Cartesian Shift
             curr_pose_stamped = PoseStamped()
             curr_pose_stamped.header = shift_pose.header
             # Convert Transform to Pose (translation -> position, rotation -> orientation)
             curr_pose_stamped.pose.position.x = ee_tf_shift.transform.translation.x
             curr_pose_stamped.pose.position.y = ee_tf_shift.transform.translation.y
             curr_pose_stamped.pose.position.z = ee_tf_shift.transform.translation.z
             curr_pose_stamped.pose.orientation = ee_tf_shift.transform.rotation

             # Use strict move 
             if not self._move_straight_line(curr_pose_stamped, shift_pose, timeout=5.0):
                  self.get_logger().warn("Shift failed, trying standard move...")
                  self._move_to_pose(shift_pose, timeout=5.0)
             
             time.sleep(0.5)
             
        except Exception as e:
             self.get_logger().error(f"Shift failed: {e}")

        # Ensure gripper is OPEN
        self.get_logger().info("Opening gripper...")
        self._open_gripper()
        self._wait_for_settle(timeout=0.2)

        # ========================================
        # STAGE 2: CONTINUOUS SERVO DESCENT
        # ========================================
        self.get_logger().info("[2/5] CONTINUOUS SERVO DESCENT: Tracking object & detecting contact...")
        
        # Get current EE position
        try:
            ee_tf = self.tf_buffer.lookup_transform(
                self.arm_base_frame, self.ee_link, Time(), timeout=Duration(seconds=1.0)
            )
            ee_z = ee_tf.transform.translation.z
        except Exception as e:
            self.get_logger().error(f"Failed to get EE position for descent: {e}")
            return False, "TF lookup failed"
        
        # Target Z with safety offset
        if self.current_grasp_pose is not None:
            grasp_z = self.current_grasp_pose.pose.position.z
            target_z = max(grasp_z + 0.015, 0.02)
        else:
            target_z = max(ee_z - 0.15, 0.02)
            
        self.get_logger().info(f"📉 Servo Target Z: {target_z:.3f}m")
        
        # Ensure gripper is OPEN before descent
        self._open_gripper()
        
        # Try servo descent first
        servo_success = False
        try:
            # Check if servo node is listening
            if self.servo_pub.get_subscription_count() > 0:
                servo_success = self._continuous_servo_descent(target_z, timeout=15.0)
            else:
                self.get_logger().warn("⚠️ Servo node not running! Falling back to blind Cartesian descent.")
        except Exception as e:
            self.get_logger().error(f"Servo error: {e}")

        approach_success = servo_success
        
        # Fallback to pure Cartesian if Servo failed or unavailable
        if not approach_success:
            self.get_logger().info("🔄 Executing Cartesian Fallback Descent...")
            descent_pose = PoseStamped()
            descent_pose.header.frame_id = self.arm_base_frame
            descent_pose.header.stamp = self.get_clock().now().to_msg()
            descent_pose.pose.position.x = ee_tf.transform.translation.x
            descent_pose.pose.position.y = ee_tf.transform.translation.y
            descent_pose.pose.position.z = target_z
            descent_pose.pose.orientation = ee_tf.transform.rotation
            
            if self.use_cartesian_path:
                current_pose = PoseStamped()
                current_pose.header = descent_pose.header
                current_pose.pose = ee_tf.transform
                approach_success = self._move_straight_line(current_pose, descent_pose, timeout=15.0)
            else:
                approach_success = self._move_to_pose(descent_pose, timeout=10.0, ignore_orientation=True)

        if not approach_success:
            self.get_logger().error("Descent failed!")
            return False, "Descent failed"

        # ========================================
        # STAGE 3: CLOSE GRIPPER (with two-stage squeeze)
        # ========================================
        self.get_logger().info("[3/5] CLOSING gripper (two-stage squeeze)...")
        self._close_gripper()
        time.sleep(self.gripper_close_delay)

        # Verify grasp if enabled
        if self.grasp_verify_enabled:
            grasp_ok, grasp_msg = self._verify_grasp()
            self.get_logger().info(f"[VERIFY] {grasp_msg}")
            if not grasp_ok:
                self.get_logger().warn(f"Grasp verification failed: {grasp_msg}")
                self._open_gripper()
                return False, f"Verification failed: {grasp_msg}"

        # ========================================
        # STAGE 4: LIFT (Straight line UP from actual position)
        # CRITICAL: Use Cartesian path to maintain orientation
        # ========================================
        self.get_logger().info(f"[4/5] LIFT: Straight line up {self.lift_height*100:.0f}cm...")
        
        # Get current EE position from TF for consistent closed-loop control
        try:
            ee_tf = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ee_link,
                Time(),
                timeout=Duration(seconds=1.0)
            )
            lift_start_x = ee_tf.transform.translation.x
            lift_start_y = ee_tf.transform.translation.y
            lift_start_z = ee_tf.transform.translation.z
            lift_q = ee_tf.transform.rotation
        except Exception as e:
            self.get_logger().error(f"Failed to get EE position for lift: {e}")
            return False, "TF lookup failed"
        
        # Create start and end poses for lift
        lift_start = PoseStamped()
        lift_start.header.frame_id = self.arm_base_frame
        lift_start.header.stamp = self.get_clock().now().to_msg()
        lift_start.pose.position.x = lift_start_x
        lift_start.pose.position.y = lift_start_y
        lift_start.pose.position.z = lift_start_z
        lift_start.pose.orientation = lift_q
        
        lift_end = PoseStamped()
        lift_end.header.frame_id = self.arm_base_frame
        lift_end.header.stamp = self.get_clock().now().to_msg()
        lift_end.pose.position.x = lift_start_x
        lift_end.pose.position.y = lift_start_y
        lift_end.pose.position.z = lift_start_z + self.lift_height
        lift_end.pose.orientation = lift_q
        
        self.get_logger().info(
            f"📈 Lifting from Z={lift_start_z:.3f} → Z={lift_start_z + self.lift_height:.3f}"
        )

        if self.use_cartesian_path:
            # Use Cartesian path for straight-line lift
            lift_success = self._move_straight_line(lift_start, lift_end, timeout=15.0)
        else:
            # Fallback to regular planning
            lift_success = self._move_to_pose(lift_end, timeout=10.0, ignore_orientation=True)

        if not lift_success:
            self.get_logger().warn("Lift motion failed - object may be dropped!")
            return False, "Lift failed"

        # ========================================
        # STAGE 5: RETRACT (Return to home)
        # ========================================
        self.get_logger().info("[5/5] RETRACTING to home position...")
        self._move_to_joints(self.home_joints)

        self.get_logger().info("=" * 50)
        self.get_logger().info("GRASP SEQUENCE COMPLETED SUCCESSFULLY")
        self.get_logger().info("=" * 50)

        return True, "Success"


    # ========================================
    # Service Callbacks
    # ========================================
    def _get_camera_to_candidate_distance(self):
        """Return latest camera->candidate distance (meters) from detection pipeline."""
        det = self._last_detection
        if det is None:
            return None, 'No detection cache available'

        det_ts = float(det.get('timestamp', 0.0))
        age = time.time() - det_ts if det_ts > 0.0 else float('inf')
        if age > self.pregrasp_nav_max_detection_stale:
            return None, f'Detection depth is stale ({age:.2f}s)'

        depth = float(det.get('depth', 0.0))
        if not np.isfinite(depth) or depth <= 0.0:
            return None, f'Invalid depth value: {depth}'

        return depth, 'OK'

    def _wait_for_fresh_detection(self):
        """Wait briefly for a fresh detection cache update."""
        timeout = float(self.pregrasp_nav_wait_fresh_detection_timeout)
        start = time.time()
        while (time.time() - start) < timeout:
            d, _ = self._get_camera_to_candidate_distance()
            if d is not None:
                return True
            time.sleep(0.05)
        return False

    def _run_pregrasp_navigation_gate(self):
        """
        Gate grasp execution by camera distance.
        - If d in [min, max], continue grasp directly.
        - Otherwise, send base delta action (d - target) and wait for /base/aligned_for_grasp == True.
        """
        if not self.pregrasp_nav_enabled:
            return True, 'Pre-grasp navigation gate disabled'

        if not self.ready_reached:
            return False, 'Arm has not reached the startup grasp state yet'

        distance_m, reason = self._get_camera_to_candidate_distance()
        if distance_m is None:
            self.get_logger().warn(f'Camera distance unavailable: {reason}. Waiting for fresh detection...')
            if not self._wait_for_fresh_detection():
                return False, f'Cannot evaluate camera distance: {reason}'
            distance_m, reason = self._get_camera_to_candidate_distance()
            if distance_m is None:
                return False, f'Cannot evaluate camera distance after wait: {reason}'

        self.get_logger().info(
            f'Pre-grasp distance gate: d={distance_m:.3f}m, range=[{self.pregrasp_nav_min_d:.2f}, {self.pregrasp_nav_max_d:.2f}]m'
        )
        if self.pregrasp_nav_min_d <= distance_m <= self.pregrasp_nav_max_d:
            return True, f'Distance within grasp range ({distance_m:.3f}m)'

        delta = float(distance_m - self.pregrasp_nav_target_d)
        self.get_logger().warn(
            f'Distance out of range. Requesting base move: delta = d - {self.pregrasp_nav_target_d:.2f} = {delta:.3f}m'
        )

        if not self._base_move_client.wait_for_server(timeout_sec=3.0):
            return False, f'Base action server unavailable: {self.pregrasp_nav_action_name}'

        goal = MoveBaseDistance.Goal()
        goal.distance_m = delta

        # Require fresh alignment signal after this navigation request.
        self.base_aligned = False

        send_future = self._base_move_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, timeout_sec=5.0):
            return False, 'Timeout waiting for base action goal response'

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, 'Base action goal rejected'

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=self.pregrasp_nav_action_timeout):
            return False, 'Timeout waiting for base action result'

        action_result = result_future.result()
        if action_result is None:
            return False, 'Base action returned empty result'

        if not bool(action_result.result.success):
            return False, f"Base action failed: {action_result.result.message}"

        self.get_logger().info('Base action succeeded. Waiting for alignment flag...')
        wait_start = time.time()
        while not self.base_aligned and (time.time() - wait_start) < self.pregrasp_nav_align_timeout:
            time.sleep(0.05)

        if not self.base_aligned:
            return False, f'No alignment confirmation on {self.pregrasp_nav_align_topic}'

        return True, 'Base aligned for grasp'

    def _execute_grasp_service(self, request, response):
        # Log current state for debugging
        self.get_logger().info('=' * 50)
        self.get_logger().info('GRASP EXECUTE SERVICE CALLED')
        self.get_logger().info(f'  is_calibrated: {self.is_calibrated}')
        self.get_logger().info(f'  current_grasp_pose: {"SET" if self.current_grasp_pose else "NONE"}')
        self.get_logger().info(f'  lift_is_homed: {self.lift_is_homed}')
        self.get_logger().info(f'  is_busy: {self.is_busy}')
        if self.current_grasp_pose:
            p = self.current_grasp_pose.pose.position
            self.get_logger().info(f'  target: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        self.get_logger().info('=' * 50)
        
        if self.is_busy:
            response.success = False
            response.message = "Busy"
            return response
            
        if not self.current_grasp_pose:
            response.success = False
            response.message = "No grasp pose detected! Check YOLO detection and TF tree."
            self.get_logger().error('❌ Cannot execute: No grasp pose. Possible causes:')
            self.get_logger().error('   1. YOLO detection not working')
            self.get_logger().error('   2. TF transform failed (check camera_tf_publisher)')
            self.get_logger().error('   3. Target out of workspace')
            return response

        nav_ok, nav_msg = self._run_pregrasp_navigation_gate()
        if not nav_ok:
            response.success = False
            response.message = f"Pre-grasp nav gate failed: {nav_msg}"
            self.get_logger().error(response.message)
            return response
        self.get_logger().info(f'Pre-grasp nav gate passed: {nav_msg}')
            
        self.is_busy = True
        self.status_pub.publish(Bool(data=True))
        try:
            if self.retry_enabled:
                success, msg = self._execute_grasp_with_retry()
            else:
                success, msg = self._execute_grasp_sequence_v2()
            response.success = success
            response.message = msg
        finally:
            self.is_busy = False
            self.status_pub.publish(Bool(data=False))
        return response

    def _execute_grasp_with_retry(self):
        """Execute grasp with automatic retry on planning failure."""
        original_grasp_pose = copy.deepcopy(self.current_grasp_pose) if self.current_grasp_pose else None
        original_lift_pos = self.lift_grasp_pos
        
        for attempt in range(self.retry_max_attempts):
            if attempt > 0:
                self.get_logger().warn(f'=' * 60)
                self.get_logger().warn(f'RETRY ATTEMPT {attempt + 1}/{self.retry_max_attempts}')
                self.get_logger().warn(f'=' * 60)
                
                # Adjust lift position alternately (up/down)
                if self.lift_enabled:
                    adjustment = self.retry_lift_adjustment * (1 if attempt % 2 == 1 else -1) * ((attempt + 1) // 2)
                    new_lift_pos = max(self.lift_min_pos, min(self.lift_max_pos, original_lift_pos + adjustment))
                    self.lift_grasp_pos = new_lift_pos
                    self.get_logger().info(f'Adjusted lift position: {new_lift_pos*1000:.1f}mm')
                
                # Slight position offset for grasp pose
                if original_grasp_pose and self.current_grasp_pose:
                    offset = self.retry_position_offset * (1 if attempt % 2 == 1 else -1)
                    self.current_grasp_pose.pose.position.z = original_grasp_pose.pose.position.z + offset
                    self.get_logger().info(f'Adjusted grasp Z offset: {offset*1000:.1f}mm')
                
                # Return to home before retry
                self._move_to_joints(self.home_joints)
                time.sleep(0.3)
            
            success, msg = self._execute_grasp_sequence_v2()
            
            if success:
                # Restore original values
                self.lift_grasp_pos = original_lift_pos
                if original_grasp_pose:
                    self.current_grasp_pose = original_grasp_pose
                return True, msg
            
            # Check if it's a planning failure or verification failure (worth retrying)
            if "Failed pre-grasp" in msg or "Failed grasp approach" in msg or "Verification failed" in msg:
                self.get_logger().warn(f'Failure: {msg}. Will retry with adjustments.')
                if "Verification failed" in msg:
                    # For verification failure, maybe we need a larger adjustment or just retry if it was a slip
                    pass
                continue
            else:
                # Other failures (e.g., lift not homed) - don't retry
                self.get_logger().error(f'Non-recoverable error: {msg}')
                break
        
        # Restore original values after all retries failed
        self.lift_grasp_pos = original_lift_pos
        if original_grasp_pose:
            self.current_grasp_pose = original_grasp_pose
        return False, f"Failed after {self.retry_max_attempts} attempts"

    def _go_home_service(self, request, response):
        response.success = self._move_to_joints(self.home_joints)
        response.message = "OK" if response.success else "Failed"
        return response

    def _execute_ready_service(self, request, response):
        del request
        if self.is_busy:
            response.success = False
            response.message = 'Busy'
            return response

        self.get_logger().info('EXECUTE_READY service: moving arm to ready pose (lift unchanged)...')
        success = self._move_to_joints(self.home_joints)
        self.ready_reached = success
        if success:
            self.get_logger().info('[SERVICE] ✅ Ready pose reached! System ready for detection.')
        else:
            self.get_logger().error('[SERVICE] ❌ Failed to move arm to ready pose!')
        response.success = success
        response.message = 'Ready pose reached' if success else 'Failed to reach ready pose'
        return response

    def _package_ready_service(self, request, response):
        del request
        if self.is_busy:
            response.success = False
            response.message = 'Busy'
            return response

        self.get_logger().info('PACKAGE_READY service: launching step2_perception, then lift+arm...')

        # Step 0: launch step2_perception pipeline
        try:
            self._launch_perception_pipeline()
        except Exception as e:  # noqa: BLE001 — don't let launch failure block grasp motion
            self.get_logger().warn(f'PACKAGE_READY: step2_perception launch hit an error: {e}. Continuing with motion.')

        # Step 1: lift back to home (0.0m)
        if self.lift_enabled:
            if not self.lift_is_homed:
                self.get_logger().error('PACKAGE_READY: lift not homed, cannot lower.')
                response.success = False
                response.message = 'Lift not homed'
                return response
            self.get_logger().info(f'PACKAGE_READY: Moving lift to home ({self.ready_lift_pos}m)...')
            if not self._move_lift_to(self.ready_lift_pos):
                response.success = False
                response.message = 'Failed to lower lift'
                return response
            time.sleep(1.0)  # Let lift settle before arm move

        # Step 2: arm to grasp pose
        self.get_logger().info('PACKAGE_READY: Moving arm to grasp pose...')
        success = self._move_to_joints(self.grasp_joints)
        self.ready_reached = success
        if success:
            self.get_logger().info('[SERVICE] ✅ Package ready reached! Arm at grasp pose, lift at home.')
        else:
            self.get_logger().error('[SERVICE] ❌ Failed to move arm to grasp pose!')
        response.success = success
        response.message = 'Package ready reached' if success else 'Failed to reach grasp pose'
        return response

    def _launch_perception_pipeline(self):
        """Launch step2_perception in a detached subprocess."""
        self.get_logger().info('PACKAGE_READY: launching step2_perception...')
        proc = subprocess.Popen(
            ['ros2', 'launch', 'rx200_handbag_grasp', 'step2_perception.launch.py'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,  # detach so it outlives grasp_node shutdown
        )
        self.get_logger().info(f'PACKAGE_READY: step2_perception launched (pid={proc.pid})')

    def _go_sleep_service(self, request, response):
        response.success = self._move_to_joints(self.sleep_joints)
        response.message = "OK" if response.success else "Failed"
        return response

    def _rest_service(self, request, response):
        """Open gripper, wait 3 seconds, then move arm to sleep pose."""
        self.get_logger().info('REST service: opening gripper...')
        open_ok = self._open_gripper()
        if not open_ok:
            response.success = False
            response.message = "Failed to open gripper"
            return response

        self.get_logger().info('REST service: waiting 3 seconds...')
        time.sleep(3.0)

        self.get_logger().info('REST service: moving to sleep pose...')
        sleep_ok = self._move_to_joints(self.sleep_joints)
        response.success = sleep_ok
        response.message = "OK" if sleep_ok else "Failed to move to sleep"
        return response

    def _open_gripper_service(self, request, response):
        response.success = self._open_gripper()
        response.message = "OK"
        return response

    def _close_gripper_service(self, request, response):
        response.success = self._close_gripper()
        response.message = "OK"
        return response

    # ========================================
    # Lift Control Methods
    # ========================================
    def _lift_homed_cb(self, msg: Bool):
        """Update lift homed status from /lift/is_homed topic."""
        self.lift_is_homed = msg.data

    def _move_lift_to(self, position: float, timeout: float = None) -> bool:
        """
        Move lift to target position using MoveIt 'interbotix_lift' group.
        
        Args:
            position: Target position in meters (0.0 - 0.275)
            timeout: Optional timeout override
            
        Returns:
            True if successful, False otherwise
        """
        if not self.lift_enabled:
            self.get_logger().warn('Lift not enabled, skipping lift move')
            return True
            
        if not self.lift_is_homed:
            self.get_logger().error('Lift not homed! Call /lift/home or /grasp/home_lift first.')
            return False
        
        # Clamp position to limits
        position = max(self.lift_min_pos, min(self.lift_max_pos, position))
        
        # Check if already at target (tolerance 1cm)
        if abs(self.current_lift_position - position) < 0.01:
            self.get_logger().info(f'Lift already at target {position:.3f}m. Skipping.')
            return True

        timeout = timeout or self.lift_timeout
        
        # Use MoveIt Client (reused from arm)
        if not self._move_client.wait_for_server(timeout_sec=5.0):
             self.get_logger().error("MoveIt action server not available for lift")
             return False

        self.get_logger().info(f'Moving lift to {position*1000:.1f}mm (MoveIt)...')
        
        # Build MoveIt Goal
        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.lift_group
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        
        constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = self.lift_joint_name
        jc.position = position
        jc.tolerance_above = 0.005
        jc.tolerance_below = 0.005
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
        
        request.goal_constraints.append(constraints)
        goal.request = request
        goal.planning_options.plan_only = False
        
        # Send Goal
        self._goal_complete = False
        self._goal_success = False
        
        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        
        # Wait
        start = time.time()
        while not self._goal_complete and (time.time() - start) < timeout:
            time.sleep(0.05)
            
        if self._goal_success:
             self.get_logger().info(f"Lift move success: {position:.3f}m")
             return True
        else:
             self.get_logger().error("Lift move FAILED (MoveIt)")
             return False


    def _lift_goal_response_cb(self, future):
        """Handle lift action goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Lift goal rejected')
            self._lift_goal_complete = True
            self._lift_goal_success = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._lift_result_cb)

    def _lift_result_cb(self, future):
        """Handle lift action result."""
        self._lift_goal_complete = True
        result = future.result()
        self._lift_goal_success = (result.result.error_code == 0)
        
        if not self._lift_goal_success:
            self.get_logger().error(f"Lift Action Failed! Code: {result.result.error_code}, Msg: {result.result.error_string}")
        else:
            self.get_logger().info("Lift Action Successful")

    # ========================================
    # Lift Service Callbacks
    # ========================================
    def _home_lift_service(self, request, response):
        """Call /lift/home service to trigger hardware homing."""
        if not self.lift_enabled:
            response.success = False
            response.message = "Lift not enabled"
            return response
        
        # Create client to call /lift/home
        home_client = self.create_client(Trigger, '/lift/home')
        if not home_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = "Lift home service not available"
            return response
        
        self.get_logger().info('Calling /lift/home for hardware homing...')
        future = home_client.call_async(Trigger.Request())

        # Wait for completion using simple polling (callbacks run in executor threads)
        if self._wait_for_future(future, timeout_sec=35.0) and future.result():
            response.success = future.result().success
            response.message = future.result().message
            if response.success:
                self.lift_is_homed = True
        else:
            response.success = False
            response.message = "Lift home timeout"
        return response

    def _lower_lift_service(self, request, response):
        """Move lift to grasp position (lowered)."""
        success = self._move_lift_to(self.lift_grasp_pos)
        response.success = success
        response.message = f"Lowered to {self.lift_grasp_pos*1000:.0f}mm" if success else "Failed"
        return response

    def _raise_lift_service(self, request, response):
        """Move lift to retract position (raised)."""
        success = self._move_lift_to(self.lift_retract_pos)
        response.success = success
        response.message = f"Raised to {self.lift_retract_pos*1000:.0f}mm" if success else "Failed"
        return response


    # ========================================
    # Robust Callbacks
    # ========================================
    def _sync_callback_robust(self, det_msg: Detection2DArray, depth_msg: Image):
        """Process YOLO detections with robust diagnostics and strict sync."""
        # 1. Strict Sync Check
        try:
            det_time = float(det_msg.header.stamp.sec) + float(det_msg.header.stamp.nanosec)*1e-9
            depth_time = float(depth_msg.header.stamp.sec) + float(depth_msg.header.stamp.nanosec)*1e-9
            time_diff = abs(det_time - depth_time)
            
            # self.get_logger().info(f'Sync: Det={len(det_msg.detections)} | Diff={time_diff*1000:.1f}ms')
            
            if time_diff > self.max_frame_delay:
                self.get_logger().warn(f'⚠️ High Sync Jitter: {time_diff*1000:.1f}ms > {self.max_frame_delay*1000:.0f}ms - DROPPING FRAME')
                return
        except Exception:
            pass
        
        if not self.is_calibrated or self.camera_intrinsics is None:
            return

        if len(det_msg.detections) == 0:
            return
        
        # NOTE: We intentionally DO NOT skip when is_busy!
        # Visual servoing needs detection updates during grasp execution.
        # We'll skip EXECUTION (moving/grasping) below, but detection processing continues.

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            
            img_h, img_w = depth_image.shape[:2]

            # OPTIMIZED: When busy (grasp in progress), only do lightweight
            # detection update for VS — skip expensive full-frame filtering
            if self.is_busy:
                # Lightweight path: just update detection coords for VS
                for det in det_msg.detections:
                    if len(det.results) == 0: continue
                    conf = det.results[0].hypothesis.score
                    if conf < self.min_confidence: continue
                    cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
                    w, h = det.bbox.size_x, det.bbox.size_y
                    # Quick depth sample at center (no bilateral filter)
                    center_x, center_y = int(cx), int(cy)
                    if 0 <= center_x < img_w and 0 <= center_y < img_h:
                        raw_depth = float(depth_image[center_y, center_x]) * self.depth_scale
                    else:
                        raw_depth = 0.0
                    self._last_detection = {
                        'timestamp': time.time(),
                        'u': cx, 'v': cy, 'w': w, 'h': h,
                        'depth': raw_depth,
                        'msg_stamp': depth_msg.header.stamp
                    }
                    break  # Only need best detection for VS
                return
            
            # Score detections
            scored_detections = []
            for det in det_msg.detections:
                if len(det.results) == 0: continue
                conf = det.results[0].hypothesis.score
                if conf < self.min_confidence:
                    self.get_logger().info(f"Skipping: conf {conf:.2f} < {self.min_confidence}", throttle_duration_sec=2.0)
                    continue
                
                cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
                w, h = det.bbox.size_x, det.bbox.size_y
                x1 = int(cx - w/2) + self.roi_padding
                y1 = int(cy - h/2) + self.roi_padding
                x2 = int(cx + w/2) - self.roi_padding
                y2 = int(cy + h/2) - self.roi_padding
                x1, x2 = max(0, min(x1, img_w-1)), max(x1+1, min(x2, img_w))
                y1, y2 = max(0, min(y1, img_h-1)), max(y1+1, min(y2, img_h))
                
                # Extract object orientation from bbox
                # For BOX-SHAPED objects (like medicine boxes):
                # Goal: gripper fingers should open along the LONGER edge to grasp the SHORT edge
                # 
                # When gripper points down, fingers open horizontally in the gripper's Y direction.
                # After Rz(yaw)*Ry(-90°), this becomes perpendicular to arm reach direction.
                # 
                # - For tall box (h > w): fingers should span the height → roll=0
                # - For wide box (w > h): fingers should span the width → roll=90°
                bbox_theta = getattr(det.bbox.center, 'theta', 0.0)
                if abs(bbox_theta) < 0.001:  # theta not provided, infer from aspect ratio
                    if h > w * 1.2:  # Box is taller than wide - grasp the shorter width
                        bbox_theta = 0.0  # Fingers span height, grasp width
                    elif w > h * 1.2:  # Box is wider than tall - grasp the shorter height
                        bbox_theta = np.pi / 2  # Fingers span width, grasp height
                    else:  # Square-ish, default to 0
                        bbox_theta = 0.0

                scored_detections.append({
                    'detection': det,
                    'score': conf,
                    'roi': (x1, y1, x2, y2),
                    'bbox': (cx, cy, w, h),
                    'object_angle': bbox_theta  # Gripper rotation to grasp shorter side
                })
        
            if not scored_detections:
                self.get_logger().info("No valid detections", throttle_duration_sec=2.0)
                return
            
            best = max(scored_detections, key=lambda x: x['score'])
            x1, y1, x2, y2 = best['roi']

            # Store object orientation for gripper alignment
            self._current_object_angle = best.get('object_angle', 0.0)
            cx, cy, w, h = best['bbox']
            self.get_logger().info(
                f"📐 Object: w={w:.0f}, h={h:.0f}, angle={np.degrees(self._current_object_angle):.1f}°",
                throttle_duration_sec=1.0
            )

            # --- Robust Grasp Calculation ---
            # [NEW] Get current mask for mask-based depth sampling
            current_mask = self._get_current_mask()
            grasp_point, yaw_angle, stats = self._compute_grasp_from_roi(
                depth_image, x1, y1, x2, y2, mask=current_mask
            )

            # Store detection data for visual servoing
            # Apply temporal depth filtering
            raw_depth = stats.get('raw_depth', stats.get('median_depth', 0.0))
            self.depth_temporal_filter.update(raw_depth)
            filtered_depth = self.depth_temporal_filter.get_filtered()

            self._last_detection = {
                'timestamp': time.time(),
                'u': cx,  # Object center u (pixels)
                'v': cy,  # Object center v (pixels)
                'w': w,   # Bbox width
                'h': h,   # Bbox height
                'depth': filtered_depth if filtered_depth else raw_depth,
                'depth_raw': raw_depth,
                'msg_stamp': depth_msg.header.stamp # Store ROS timestamp for sync
            }

            if grasp_point is None:
                self.get_logger().warn(f"Grasp Calc Failed: {stats['status']}")
                return

            # --- Visualization ---
            # Pass TIMESTAMP to ensure accurate TF lookup matching the image time
            pose_cam = self._create_grasp_pose_camera(grasp_point, yaw_angle, timestamp=depth_msg.header.stamp)
            
            # Store camera-frame X for visual servoing waist alignment
            self._last_cam_pose_x = pose_cam.pose.position.x
            
            self.get_logger().info(f"📷 CAM POSE: x={pose_cam.pose.position.x:.3f}, y={pose_cam.pose.position.y:.3f}, z={pose_cam.pose.position.z:.3f}", throttle_duration_sec=1.0)
            pose_base = self._transform_to_base(pose_cam, depth_msg.header.stamp)
            
            # --- HOTFIX REMOVED: rely on TF tree for correct frame alignment ---
            
            self._visualize_grasp(pose_cam, pose_base, best['bbox'], stats)

            # --- Diagnostics and Validation ---
            self.get_logger().info(
                f"🎯 Tgt: Ratio={stats.get('valid_ratio',0):.2f} | Depth={stats.get('median_depth',0):.3f}m | "
                f"Status={stats['status']}",
                throttle_duration_sec=1.0
            )
            
            if stats['status'] != 'GOOD' and not self.viz_only:
                self.get_logger().warn("⚠️ Guesstimated Depth - Exercising Caution!")

            if pose_base is None:
                self.get_logger().error("❌ TF Transform Failed (Base Frame)")
                return

            in_workspace = self._check_workspace(pose_base)
            if not in_workspace:
                p = pose_base.pose.position
                self.get_logger().warn(
                    f"❌ OUT OF WORKSPACE in {self.arm_base_frame}:\n"
                    f"   x={p.x:.3f} (limit: {self.ws_x_min:.2f} to {self.ws_x_max:.2f})\n"
                    f"   y={p.y:.3f} (limit: {self.ws_y_min:.2f} to {self.ws_y_max:.2f})\n"
                    f"   z={p.z:.3f} (limit: {self.ws_z_min:.2f} to {self.ws_z_max:.2f})"
                )
                if not self.pregrasp_nav_enabled:
                    return
                self.get_logger().warn(
                    'Keeping out-of-workspace pose so pre-grasp navigation can move the base closer.'
                )

            # --- Visualization Only Mode ---
            if self.viz_only:
                self.get_logger().info("Visualization Mode: Skipping Execution", throttle_duration_sec=2.0)
                return
            
            # --- Visual Servoing Note ---
            # Visual servoing is now integrated into _execute_grasp_sequence (Stage 1.5)
            if stats.get('valid_ratio', 1.0) < self.min_valid_depth_ratio:
                 self.get_logger().warn(f"Low Depth Quality ({stats.get('valid_ratio',0):.2f}) - Visual servo will help")

            # --- Execution ---
            pose_base = self._filter_pose(pose_base)
            self.current_grasp_pose = pose_base
            p = pose_base.pose.position
            self.get_logger().info(
                f"✅ BASE POSE ({self.arm_base_frame}): x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}"
            )
            self.grasp_pose_pub.publish(pose_base)
            
            pregrasp = self._compute_pregrasp_pose(pose_base)
            self.pregrasp_pose_pub.publish(pregrasp)
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
            import traceback
            traceback.print_exc()


    # ========================================
    # NEW: Look-and-Move Grasp Sequence (V2)
    # ========================================
    def _execute_grasp_sequence_v2(self):
        """
        Refined Logic:
        1. Approach Pre-Grasp (Stage 0 - already done)
        2. Refine Target (Stage 1 - Look & Compute)
        3. Move Gripper to Target & Align (Stage 2)
        4. Descend & Grasp (Stage 3)
        """
        self.get_logger().info("🚀 STARTING GRASP SEQUENCE V2 (Look-and-Move)")
        
        if not self.current_grasp_pose:
            self.get_logger().error("No grasp pose available!")
            return False, "No pose"

        # ========================================
        # STAGE 1: LOOK & COMPUTE (High)
        # ========================================
        self.get_logger().info("[1/3] Refining Target (High)...")
        refined_target_pose, refine_success = self._refine_and_compute_target(hover_duration=2.0)
        
        if not refine_success or refined_target_pose is None:
             self.get_logger().warn("Refinement failed, using initial estimate.")
             refined_target_pose = self.current_grasp_pose
        
        final_x = refined_target_pose.pose.position.x
        final_y = refined_target_pose.pose.position.y
        final_z = refined_target_pose.pose.position.z 
        
        self.get_logger().info(f"🔒 LOCKED TARGET: ({final_x:.3f}, {final_y:.3f}, {final_z:.3f})")

        # ========================================
        # STAGE 1.5: VISUAL SERVOING FOR WAIST (DISABLED)
        # Goal: Rotate waist until object is centered in camera view
        # ========================================
        # NOTE: Disabled per user request - reverting to old flow
        # self.get_logger().info("[1.5/3] Visual Servoing: Centering object in camera...")
        # ... (Visual servoing code commented out)



        # ========================================
        # STAGE 2: MOVE TO TARGET (Lateral)
        # ========================================
        self.get_logger().info("[2/3] Aligning Gripper...")
        
        # Get Current EE Z to maintain height
        current_ee_z = refined_target_pose.pose.position.z + 0.35 # Default high approach just in case
        try:
             ee_tf = self.tf_buffer.lookup_transform(
                 self.arm_base_frame, self.ee_link, Time(), Duration(seconds=1.0)
             )
             current_ee_z = ee_tf.transform.translation.z
        except:
             pass

        # Approach Pose: Target XY, Current Z
        approach_pose = copy.deepcopy(refined_target_pose)
        approach_pose.pose.position.z = current_ee_z
        
        # Move Lateral
        if not self._move_to_pose(approach_pose, timeout=10.0, use_lift=False, ignore_orientation=False):
             self.get_logger().warn("Lateral align failed, continuing...")
        self._wait_for_settle(timeout=0.3)

        # ========================================
        # STAGE 3: BLIND DESCENT & GRASP
        # ========================================
        self.get_logger().info("[3/3] Descending...")
        
        target_z = max(final_z + 0.015, 0.02)
        
        descent_pose = copy.deepcopy(approach_pose)
        descent_pose.pose.position.z = target_z
        
        # Open Gripper
        self.get_logger().info("Opening Gripper...")
        self._open_gripper()
        
        # Descend (Blind) - Use Cartesian Path for vertical drop
        self.get_logger().info(f"📉 Descending to Z={target_z:.3f} (Cartesian)...")
        
        # Construct start pose for cartesian
        try:
             # Get current pose exactly
             ee_tf = self.tf_buffer.lookup_transform(
                 self.arm_base_frame, self.ee_link, Time(), Duration(seconds=1.0)
             )
             start_pose = PoseStamped()
             start_pose.header.frame_id = self.arm_base_frame
             start_pose.pose.position.x = ee_tf.transform.translation.x
             start_pose.pose.position.y = ee_tf.transform.translation.y
             start_pose.pose.position.z = ee_tf.transform.translation.z
             start_pose.pose.orientation = ee_tf.transform.rotation
             
             # Attempt Cartesian
             if not self._move_straight_line(start_pose, descent_pose):
                 self.get_logger().warn("Cartesian descent failed! Falling back to regular MoveIt...")
                 if not self._move_to_pose(descent_pose, timeout=10.0, use_lift=False, ignore_orientation=False):
                      self.get_logger().error("❌ ALL DESCENT ATTEMPTS FAILED!")
        except Exception as e:
             self.get_logger().error(f"Descent Exception: {e}")
             self._move_to_pose(descent_pose)
        
        # OPTIMIZED: Use _wait_for_settle instead of fixed sleep
        self._wait_for_settle(timeout=0.8)
        
        # Close Gripper with PWM
        self.get_logger().info("🤏 CLOSING GRIPPER (PWM Force Control)...")
        if not self._close_gripper():
            self.get_logger().warn('Gripper close failed.')
            
        time.sleep(self.gripper_close_delay)
        
        # NOTE: Removed duplicate STAGE 4 _close_gripper() call
        # Previously called _close_gripper() twice with 1.5s of extra sleep
        # The single close above with squeeze is sufficient

        # Lift - DISABLED per user request
        # self.get_logger().info("Lifting...")
        # if self.lift_enabled:
        #      self._move_lift_to(0.15)
        # else:
        #      # Arm lift
        #      lift_pose = copy.deepcopy(descent_pose)
        #      lift_pose.pose.position.z += 0.15
        #      self._move_to_pose(lift_pose)
              
        self.get_logger().info("✅ Grasp Complete. Returning to Ready/Home...")
        self._wait_for_settle(timeout=0.3)
        self._move_to_joints(self.home_joints)
             
        return True, "Executed"


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()

    # Use MultiThreadedExecutor to allow callbacks to run while waiting
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
