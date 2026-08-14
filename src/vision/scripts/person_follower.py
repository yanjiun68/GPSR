#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision.msg import CamDetections

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        # ---------------- Parameters ----------------
        self.declare_parameter('target_distance', 1.0)  # Stop 1.0m away from person
        self.declare_parameter('max_linear_x', 0.3)     # Max speed m/s
        self.declare_parameter('max_angular_z', 0.5)    # Max rotation rad/s
        self.declare_parameter('kp_linear', 0.5)        # Proportional gain for linear
        self.declare_parameter('kp_angular', 1.0)       # Proportional gain for angular

        self.target_dist = self.get_parameter('target_distance').value
        self.max_lin = self.get_parameter('max_linear_x').value
        self.max_ang = self.get_parameter('max_angular_z').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value

        # ---------------- Pub/Sub ----------------
        self.sub_dets = self.create_subscription(
            CamDetections,
            '/cam_detections',
            self.dets_cb,
            10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_detection_time = self.get_clock().now()
        # Timer to stop robot if no detections for a while
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Current target
        self.target_x = None # Forward distance
        self.target_y = None # Lateral distance

        self.get_logger().info("Person Follower Node Started")

    def dets_cb(self, msg: CamDetections):
        if not msg.detections:
            return

        # Simple logic: follow the closest person (smallest 'z' usually, 
        # but here x,y,z are in global frame (base_footprint).
        # In base_footprint (standard ROS): 
        # x = forward, y = left, z = up.
        
        # Let's find detection with smallest Euclidean distance in XY plane
        closest_dist = 999.0
        target = None

        for det in msg.detections:
            # We assume detections are already TF-transformed to base_footprint
            # by the vision_node (if vision_node global_frame is base_footprint).
            
            d = (det.x**2 + det.y**2)**0.5
            if d < closest_dist:
                closest_dist = d
                target = det

        if target:
            self.last_detection_time = self.get_clock().now()
            self.target_x = target.x
            self.target_y = target.y

    def control_loop(self):
        # 1. Check timeout
        time_since_last = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if time_since_last > 0.5:
            # No detection for 0.5s -> Stop
            self.stop_robot()
            return

        if self.target_x is None:
            return

        # 2. Control Logic
        twist = Twist()

        # Linear control: Maintains distance
        # Error = Current X - Target Distance
        # If person is at 2.0m, target is 1.0m -> error = 1.0 -> move forward
        # If person is at 0.5m, target is 1.0m -> error = -0.5 -> move backward
        
        # Note: target_x from vision_node is coordinate. 
        # If robot frame, x is distance forward.
        error_dist = self.target_x - self.target_dist
        
        # Deadband to prevent jitter
        if abs(error_dist) < 0.1:
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.kp_lin * error_dist

        # Angular control: Turn to face person (y = 0)
        # Error = target_y - 0
        # If y is positive (left), we need to turn left (+)
        # atan2(y, x) is the angle to the target
        # For small angles, y ~ angle * x
        
        # Using y directly works if we just want P controller
        # better to use angle
        import math
        angle_to_person = math.atan2(self.target_y, self.target_x)
        
        twist.angular.z = self.kp_ang * angle_to_person

        # 3. Clamp limits
        twist.linear.x = max(min(twist.linear.x, self.max_lin), -self.max_lin)
        twist.angular.z = max(min(twist.angular.z, self.max_ang), -self.max_ang)

        self.pub_cmd.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
