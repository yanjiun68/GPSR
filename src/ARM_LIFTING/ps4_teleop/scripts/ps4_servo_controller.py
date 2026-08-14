#!/usr/bin/env python3
"""
PS4 Servo Teleop for RX200 + Lift (MoveIt Servo)

This node converts PS4 joystick inputs into Cartesian twist commands for MoveIt Servo,
allowing intuitive X/Y/Z control of the end-effector.

Control Mapping:
- Left Stick X/Y:  Move End-Effector in World X/Y (Planar movement)
- Right Stick Y:   Move End-Effector in World Z (Up/Down)
- Right Stick X:   Rotate End-Effector (Yaw/Wrist Rotate)
- Square/Circle:   Close/Open Gripper
- OPTIONS:         Go Home (via Servo? or Service if available?) - Maybe skip for now to keep it simple.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped, Twist
from interbotix_xs_msgs.msg import JointSingleCommand
import time

class PS4ServoTeleop(Node):
    """PS4 joystick teleoperation using MoveIt Servo."""

    # PS4 Axis indices (standard joy driver)
    AXIS_LEFT_X = 0   # Left stick horizontal -> World X
    AXIS_LEFT_Y = 1   # Left stick vertical   -> World Y
    AXIS_RIGHT_X = 2  # Right stick horizontal -> Yaw
    AXIS_RIGHT_Y = 3  # Right stick vertical   -> World Z
    AXIS_L2 = 4       # L2 trigger
    AXIS_R2 = 5       # R2 trigger

    # PS4 Button indices
    BTN_CROSS = 0     # ✕
    BTN_CIRCLE = 1    # ○ - Open gripper
    BTN_TRIANGLE = 2  # △
    BTN_SQUARE = 3    # □ - Close gripper
    BTN_L1 = 4        
    BTN_R1 = 5        
    BTN_L2 = 6
    BTN_R2 = 7
    BTN_SHARE = 8
    BTN_OPTIONS = 9 
    BTN_PS = 10       

    def __init__(self):
        super().__init__('ps4_servo_teleop')

        # Parameters
        self.declare_parameter('linear_speed', 0.2)       # m/s
        self.declare_parameter('angular_speed', 1.0)      # rad/s
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('control_rate', 30.0)      # Hz
        self.declare_parameter('frame_id', 'world')       # Frame for twist commands (world or base_link)
        self.declare_parameter('gripper_pwm_open', 200)
        self.declare_parameter('gripper_pwm_close', -300)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.control_rate = self.get_parameter('control_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.gripper_pwm_open = self.get_parameter('gripper_pwm_open').value
        self.gripper_pwm_close = self.get_parameter('gripper_pwm_close').value

        # Publishers
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )
        self.gripper_pub = self.create_publisher(
            JointSingleCommand, '/rx200/commands/joint_single', 10
        )

        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self._joy_callback, 10
        )
        
        # State
        self.current_joy = None
        self.enabled = True

        # Control timer
        period = 1.0 / self.control_rate
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'PS4 Servo Teleop - Cartesian Control Mode (Frame: {self.frame_id})')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick X/Y : Move Forward/Back/Left/Right (World X/Y)')
        self.get_logger().info('  Right Stick Y  : Move Up/Down (World Z)')
        self.get_logger().info('  Right Stick X  : Rotate Yaw')
        self.get_logger().info('  ○ Circle       : Open gripper')
        self.get_logger().info('  □ Square       : Close gripper')
        self.get_logger().info('=' * 60)

    def _joy_callback(self, msg: Joy):
        """Store latest joystick state."""
        self.current_joy = msg

        # Check for button presses
        buttons = msg.buttons
        
        # Gripper (Direct control, independent of Servo)
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
        """Main control loop - compute twist commands from joystick."""
        if self.current_joy is None or not self.enabled:
            return

        joy = self.current_joy
        axes = joy.axes
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Linear Control
        # Left Stick Y -> X (Forward/Back)
        if len(axes) > self.AXIS_LEFT_Y:
            val = self._apply_deadzone(axes[self.AXIS_LEFT_Y])
            if val != 0.0:
                self.get_logger().info(f"Commanding Linear X: {val} * {self.linear_speed}")
            msg.twist.linear.x = val * self.linear_speed
            
        # Left Stick X -> Y (Left/Right)
        if len(axes) > self.AXIS_LEFT_X:
            val = self._apply_deadzone(axes[self.AXIS_LEFT_X])
            if val != 0.0:
                self.get_logger().info(f"Commanding Linear Y: {val} * {self.linear_speed}")
            msg.twist.linear.y = val * self.linear_speed
            
        # Right Stick Y -> Z (Up/Down)
        if len(axes) > self.AXIS_RIGHT_Y:
            val = self._apply_deadzone(axes[self.AXIS_RIGHT_Y])
            if val != 0.0:
                self.get_logger().info(f"Commanding Linear Z: {val} * {self.linear_speed}")
            msg.twist.linear.z = val * self.linear_speed

        # Angular Control
        # Right Stick X -> Yaw (Rotate around Z)
        if len(axes) > self.AXIS_RIGHT_X:
            msg.twist.angular.z = self._apply_deadzone(axes[self.AXIS_RIGHT_X]) * self.angular_speed

        # Only publish if non-zero to avoid spamming Servo (though Servo expects stream)
        # Actually Servo expects typical stream at >10Hz. Even zeros are good to stop motion.
        self.twist_pub.publish(msg)

    def _control_gripper(self, pwm_value: int):
        """Control gripper with PWM value."""
        cmd = JointSingleCommand()
        cmd.name = 'gripper'
        cmd.cmd = float(pwm_value)
        self.gripper_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PS4ServoTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
