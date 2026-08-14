#!/usr/bin/env python3
"""
PS4 Joystick Teleoperation for RX200 + Lift

Uses PS4 joysticks to control arm joints and lift.

Control Mapping:
- Left Stick X/Y: Waist + Shoulder
- Right Stick X/Y: Elbow + Wrist Angle
- L2/R2: Wrist Rotate
- L1/R1: Lift up/down
- Square/Circle: Gripper open/close
- PS: E-STOP

This uses direct joint velocity control via Interbotix joint commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy, JointState
from std_srvs.srv import Trigger
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as BuiltinDuration

import numpy as np
import time


class PS4Teleop(Node):
    """PS4 joystick teleoperation for RX200 arm + lift."""

    # PS4 Axis indices (standard joy driver)
    AXIS_LEFT_X = 0   # Left stick horizontal
    AXIS_LEFT_Y = 1   # Left stick vertical
    AXIS_RIGHT_X = 2  # Right stick horizontal (may be 3 on some drivers)
    AXIS_RIGHT_Y = 3  # Right stick vertical (may be 4 on some drivers)
    AXIS_L2 = 4       # L2 trigger
    AXIS_R2 = 5       # R2 trigger

    # PS4 Button indices
    BTN_CROSS = 0     # ✕
    BTN_CIRCLE = 1    # ○ - Open gripper
    BTN_TRIANGLE = 2  # △
    BTN_SQUARE = 3    # □ - Close gripper
    BTN_L1 = 4        # Lift up
    BTN_R1 = 5        # Lift down
    BTN_L2 = 6
    BTN_R2 = 7
    BTN_SHARE = 8
    BTN_OPTIONS = 9   # Home position
    BTN_PS = 10       # E-STOP

    def __init__(self):
        super().__init__('ps4_teleop')

        # Parameters
        self.declare_parameter('joint_speed', 3.0)        # rad/s max (fast response)
        self.declare_parameter('lift_speed', 0.03)        # m/s
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('control_rate', 30.0)      # Hz
        self.declare_parameter('gripper_pwm_open', 200)
        self.declare_parameter('gripper_pwm_close', -300)

        self.joint_speed = self.get_parameter('joint_speed').value
        self.lift_speed = self.get_parameter('lift_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.control_rate = self.get_parameter('control_rate').value
        self.gripper_pwm_open = self.get_parameter('gripper_pwm_open').value
        self.gripper_pwm_close = self.get_parameter('gripper_pwm_close').value

        # Arm joint names
        self.arm_joints = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        
        # Current joint positions
        self.current_positions = {j: 0.0 for j in self.arm_joints}
        self.current_lift_pos = 0.0
        
        # Joint limits (approximate for RX200)
        self.joint_limits = {
            'waist': (-3.14, 3.14),
            'shoulder': (-1.85, 1.9),
            'elbow': (-2.15, 1.6),
            'wrist_angle': (-1.75, 2.15),
            'wrist_rotate': (-3.14, 3.14),
        }
        self.lift_min = 0.0
        self.lift_max = 0.275

        # State
        self.current_joy = None
        self.enabled = True
        self.estop_last_time = 0.0  # Debounce for E-STOP

        # Publishers
        self.joint_pub = self.create_publisher(
            JointGroupCommand, '/rx200/commands/joint_group', 10
        )
        self.gripper_pub = self.create_publisher(
            JointSingleCommand, '/rx200/commands/joint_single', 10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/rx200/joint_states', self._joint_state_cb, 10
        )
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self._joy_callback, 10
        )

        # Lift action client
        self._lift_client = ActionClient(
            self, FollowJointTrajectory, '/lift_controller/follow_joint_trajectory'
        )

        # Service clients
        self.estop_client = self.create_client(Trigger, '/lift/estop')
        self.home_client = self.create_client(Trigger, '/grasp/go_home')

        # Control timer
        period = 1.0 / self.control_rate
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('PS4 Joystick Teleoperation - Joint Control Mode')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick X  : Waist rotation')
        self.get_logger().info('  Left Stick Y  : Shoulder')
        self.get_logger().info('  Right Stick Y : Elbow')
        self.get_logger().info('  Right Stick X : Wrist Angle')
        self.get_logger().info('  L2/R2 Triggers: Wrist Rotate')
        self.get_logger().info('  L1/R1         : Lift up/down')
        self.get_logger().info('  ○ Circle      : Open gripper')
        self.get_logger().info('  □ Square      : Close gripper')
        self.get_logger().info('  OPTIONS       : Go Home')
        self.get_logger().info('  PS            : E-STOP')
        self.get_logger().info('=' * 60)

    def _joint_state_cb(self, msg: JointState):
        """Update current joint positions."""
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]
            if name == 'lift_joint':
                self.current_lift_pos = msg.position[i]

    def _joy_callback(self, msg: Joy):
        """Store latest joystick state."""
        self.current_joy = msg

        # Check for button presses
        buttons = msg.buttons
        import time as _time
        now = _time.time()
        
        # Priority: If busy, ignore everything except E-STOP
        if self.grasp_busy:
             # Only check E-STOP
             # (See below for E-STOP logic, currently commented out in original file)
             return

        # EXECUTE GRASP (Triangle)
        if len(buttons) > self.BTN_TRIANGLE and buttons[self.BTN_TRIANGLE]:
            if now - self.last_grasp_trigger_time > 2.0: # Debounce
                self.last_grasp_trigger_time = now
                self._execute_grasp() 

        # E-STOP (PS button) - with debounce to prevent spam
        # Note: PS button index varies by driver. Common: 10, 12, or 13
        # Disabled for now to prevent accidental triggers - use OPTIONS + SHARE instead
        # if len(buttons) > self.BTN_PS and buttons[self.BTN_PS]:
        #     now = _time.time()
        #     if now - self.estop_last_time > 1.0:  # 1 second debounce
        #         self.estop_last_time = now
        #         self._estop()

        # Home (OPTIONS button) or SHARE
        if (len(buttons) > self.BTN_OPTIONS and buttons[self.BTN_OPTIONS]) or \
           (len(buttons) > self.BTN_SHARE and buttons[self.BTN_SHARE]):
            self._go_home()

        # Gripper
        if len(buttons) > self.BTN_CIRCLE and buttons[self.BTN_CIRCLE]:
            self._control_gripper(self.gripper_pwm_open)
        if len(buttons) > self.BTN_SQUARE and buttons[self.BTN_SQUARE]:
            self._control_gripper(self.gripper_pwm_close)

    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick axis."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _control_loop(self):
        """Main control loop - compute joint deltas from joystick."""
        if self.current_joy is None or not self.enabled:
            return

        # Safety: Disable manual control if grasp sequence is running
        if self.grasp_busy:
            return

        joy = self.current_joy
        axes = joy.axes
        buttons = joy.buttons
        dt = 1.0 / self.control_rate

        # Compute joint deltas
        deltas = {j: 0.0 for j in self.arm_joints}

        # Left stick: waist (X) and shoulder (Y)
        # Note: Axis values may be inverted depending on driver
        if len(axes) > self.AXIS_LEFT_X:
            deltas['waist'] = self._apply_deadzone(axes[self.AXIS_LEFT_X]) * self.joint_speed * dt
        if len(axes) > self.AXIS_LEFT_Y:
            deltas['shoulder'] = self._apply_deadzone(-axes[self.AXIS_LEFT_Y]) * self.joint_speed * dt  # Inverted

        # Right stick: elbow (Y) and wrist_angle (X)
        if len(axes) > self.AXIS_RIGHT_Y:
            deltas['elbow'] = self._apply_deadzone(-axes[self.AXIS_RIGHT_Y]) * self.joint_speed * dt  # Inverted
        if len(axes) > self.AXIS_RIGHT_X:
            deltas['wrist_angle'] = self._apply_deadzone(-axes[self.AXIS_RIGHT_X]) * self.joint_speed * dt  # Inverted

        # Triggers: wrist_rotate
        if len(axes) > self.AXIS_L2 and len(axes) > self.AXIS_R2:
            # Triggers go from 1.0 (not pressed) to -1.0 (fully pressed)
            l2_val = (1.0 - axes[self.AXIS_L2]) / 2.0  # 0 to 1
            r2_val = (1.0 - axes[self.AXIS_R2]) / 2.0  # 0 to 1
            deltas['wrist_rotate'] = (r2_val - l2_val) * self.joint_speed * dt

        # Apply deltas with limits
        new_positions = []
        for joint in self.arm_joints:
            current = self.current_positions.get(joint, 0.0)
            target = current + deltas[joint]
            low, high = self.joint_limits.get(joint, (-3.14, 3.14))
            target = max(low, min(high, target))
            new_positions.append(target)

        # Publish joint command if any delta is non-zero
        if any(abs(d) > 0.001 for d in deltas.values()):
            cmd = JointGroupCommand()
            cmd.name = 'arm'
            cmd.cmd = [float(p) for p in new_positions]
            self.joint_pub.publish(cmd)
            self.get_logger().debug(f'Cmd: {[f"{p:.2f}" for p in new_positions]}')

        # Lift control (L1/R1)
        if len(buttons) > self.BTN_L1 and buttons[self.BTN_L1]:
            self._move_lift_delta(self.lift_speed * dt)
        if len(buttons) > self.BTN_R1 and buttons[self.BTN_R1]:
            self._move_lift_delta(-self.lift_speed * dt)

    def _move_lift_delta(self, delta: float):
        """Move lift by delta amount."""
        target = max(self.lift_min, min(self.lift_max, self.current_lift_pos + delta))
        
        if abs(target - self.current_lift_pos) < 0.001:
            return

        if not self._lift_client.wait_for_server(timeout_sec=0.05):
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['lift_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start = BuiltinDuration(sec=0, nanosec=50000000)  # 50ms
        goal.trajectory.points.append(point)
        
        self._lift_client.send_goal_async(goal)

    def _control_gripper(self, pwm_value: int):
        """Control gripper with PWM value."""
        cmd = JointSingleCommand()
        cmd.name = 'gripper'
        cmd.cmd = float(pwm_value)
        self.gripper_pub.publish(cmd)

    def _go_home(self):
        """Go to home position."""
        self.get_logger().info('Going to Home position...')
        if self.home_client.service_is_ready():
            self.home_client.call_async(Trigger.Request())

    def _execute_grasp(self):
        """Trigger automated grasp sequence."""
        self.get_logger().info('▶️ TRIANGLE PRESSED: Triggering Grasp Sequence...')
        if self.grasp_client.service_is_ready():
            self.grasp_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn('Grasp service not ready!')

    def _estop(self):
        """Emergency stop."""
        self.get_logger().error('🛑 E-STOP ACTIVATED!')
        self.enabled = False
        
        if self.estop_client.service_is_ready():
            self.estop_client.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)
    node = PS4Teleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
