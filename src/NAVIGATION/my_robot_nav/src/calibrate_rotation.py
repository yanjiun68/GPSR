#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys
import math
import termios
import tty

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

class RotationCalibrator(Node):
    def __init__(self):
        super().__init__('rotation_calibrator')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.total_yaw = 0.0
        self.last_raw_yaw = None
        self.get_logger().info("\n\n=== Rotation Calibrator Ready ===")
        print("Please rotate the robot. I will calculate total degrees turned.\n")

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, current_raw_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        if self.last_raw_yaw is not None:
            # Calculate delta
            delta = current_raw_yaw - self.last_raw_yaw
            
            # Handle wrapping (-pi to pi)
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi

            self.total_yaw += delta
            
            degrees = math.degrees(self.total_yaw)
            
            # Print with carriage return to update same line
            sys.stdout.write(f"\rOdom Total Angle: {degrees: .2f} degrees")
            sys.stdout.flush()

        self.last_raw_yaw = current_raw_yaw

def main():
    rclpy.init()
    node = RotationCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nDone.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
