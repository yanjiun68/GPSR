#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class OmniOdometry(Node):
    """
    REP-105 compliant omni wheel odometry (X-drive).

    - Subscribes:
        /wheel_ticks   (Int32MultiArray [FL, FR, RR, RL], cumulative ticks)

    - Publishes:
        /odom          (nav_msgs/Odometry)

    - TF (optional, controlled by parameter):
        odom -> base_footprint
    """

    def __init__(self):
        super().__init__("wheel_odometry")

        # ======================================================
        # Parameters
        # ======================================================

        # Frames
        self.frame_odom = self.declare_parameter(
            "odom_frame", "odom"
        ).value
        self.frame_basefp = self.declare_parameter(
            "base_footprint_frame", "base_footprint"
        ).value

        # TF control (IMPORTANT)
        self.publish_tf = self.declare_parameter(
            "publish_tf", False
        ).value

        # Geometry
        self.R  = float(self.declare_parameter("wheel_radius", 0.05).value)
        self.lx = float(self.declare_parameter("lx", 0.2051).value)
        self.ly = float(self.declare_parameter("ly", 0.2051).value)
        self.a  = self.lx + self.ly

        # Encoder ticks per revolution
        self.tpr = [
            float(self.declare_parameter("tpr_m1", 3332.0).value),  # FL
            float(self.declare_parameter("tpr_m2", 3332.0).value),  # FR
            float(self.declare_parameter("tpr_m3", 3332.0).value),  # RR
            float(self.declare_parameter("tpr_m4", 3332.0).value),  # RL
        ]

        # Encoder polarity
        self.sign = [
            int(self.declare_parameter("enc_sign_m1", 1).value),
            int(self.declare_parameter("enc_sign_m2", 1).value),
            int(self.declare_parameter("enc_sign_m3", 1).value),
            int(self.declare_parameter("enc_sign_m4", 1).value),
        ]

        # Covariances
        self.pose_cov_xy = float(self.declare_parameter("pose_cov_xy", 1e-3).value)
        self.pose_cov_th = float(self.declare_parameter("pose_cov_th", 1e-2).value)
        self.twist_cov   = float(self.declare_parameter("twist_cov",   1e-2).value)

        # ======================================================
        # ROS Interfaces
        # ======================================================

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.sub = self.create_subscription(
            Int32MultiArray,
            "wheel_ticks",
            self.ticks_cb,
            10
        )

        # TF broadcaster (used only if publish_tf = True)
        self.tf_br = TransformBroadcaster(self)

        # ======================================================
        # State
        # ======================================================

        self.last_ticks = None
        self.last_time  = self.get_clock().now()

        self.x  = 0.0
        self.y  = 0.0
        self.th = 0.0

        self.get_logger().info(
            f"[wheel_odometry] started | publish_tf={self.publish_tf}"
        )

    # ==========================================================
    # Encoder callback
    # ==========================================================
    def ticks_cb(self, msg: Int32MultiArray):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        ticks = list(msg.data)
        if len(ticks) != 4:
            self.get_logger().warn("wheel_ticks must have 4 values")
            return

        if self.last_ticks is None:
            self.last_ticks = ticks
            return

        # ------------------------------------------------------
        # Delta ticks
        # ------------------------------------------------------
        d = [ticks[i] - self.last_ticks[i] for i in range(4)]
        self.last_ticks = ticks

        # ------------------------------------------------------
        # ticks → wheel angular velocity (rad/s)
        # ------------------------------------------------------
        w = [
            (d[i] / self.tpr[i]) * 2.0 * math.pi / dt * self.sign[i]
            for i in range(4)
        ]

        # ------------------------------------------------------
        # X-drive forward kinematics (body frame)
        # ------------------------------------------------------
        vx  = (self.R / 4.0)            * ( w[0] + w[1] + w[2] + w[3] )
        vy  = (self.R / 4.0)            * (-w[0] + w[1] - w[2] + w[3] )
        vth = (self.R / (4.0 * self.a)) * (-w[0] + w[1] + w[2] - w[3] )

        # ------------------------------------------------------
        # Integrate to odom frame
        # ------------------------------------------------------
        dx  = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        dy  = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        dth = vth * dt

        self.x  += dx
        self.y  += dy
        self.th += dth
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # ------------------------------------------------------
        # Quaternion
        # ------------------------------------------------------
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.th)
        quat = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # ======================================================
        # Odometry message
        # ======================================================
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_basefp

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat

        odom.pose.covariance = [
            float(self.pose_cov_xy), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, float(self.pose_cov_xy), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, float(self.pose_cov_th),
        ]


        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = vth

        odom.twist.covariance = [
            float(self.twist_cov), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, float(self.twist_cov), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, float(self.twist_cov),
        ]


        self.odom_pub.publish(odom)

        # ======================================================
        # TF (optional)
        # ======================================================
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.frame_odom
            t.child_frame_id  = self.frame_basefp
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = quat
            self.tf_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OmniOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
