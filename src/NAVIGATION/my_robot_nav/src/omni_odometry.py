#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

class OmniOdometry(Node):
    """
    REP-105 compliant:
      Publishes a configurable odometry topic and optional TF odom -> base_footprint
    Consumes cumulative encoder ticks Int32MultiArray [FL, FR, RR, RL]
    for a 4-omni X-drive.
    """

    def __init__(self):
        super().__init__('wheel_odometry')

        # ---------- Frames ----------
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_footprint_frame", "base_footprint")
        self.declare_parameter("odom_topic", "/odom")

        self.frame_odom   = self.get_parameter("odom_frame").value
        self.frame_basefp = self.get_parameter("base_footprint_frame").value
        self.odom_topic   = self.get_parameter("odom_topic").value

        # ---------- Geometry ----------
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("lx", 0.2051)
        self.declare_parameter("ly", 0.2051)

        self.R  = self.get_parameter("wheel_radius").value   # [m]
        self.lx = self.get_parameter("lx").value           # [m]
        self.ly = self.get_parameter("ly").value           # [m]
        self.a  = self.lx + self.ly

        # ---------- Encoders (ticks per *wheel* revolution) ----------
        self.declare_parameter("tpr_m1", 3332.0)
        self.declare_parameter("tpr_m2", 3332.0)
        self.declare_parameter("tpr_m3", 3332.0)
        self.declare_parameter("tpr_m4", 3332.0)

        self.tpr_m1 = self.get_parameter("tpr_m1").value   # FL
        self.tpr_m2 = self.get_parameter("tpr_m2").value   # FR
        self.tpr_m3 = self.get_parameter("tpr_m3").value   # RR
        self.tpr_m4 = self.get_parameter("tpr_m4").value   # RL

        # ---------- Encoder polarity ----------
        self.declare_parameter("enc_sign_m1", 1)
        self.declare_parameter("enc_sign_m2", 1)
        self.declare_parameter("enc_sign_m3", 1)
        self.declare_parameter("enc_sign_m4", 1)

        self.s1 = self.get_parameter("enc_sign_m1").value   # FL
        self.s2 = self.get_parameter("enc_sign_m2").value   # FR
        self.s3 = self.get_parameter("enc_sign_m3").value   # RR
        self.s4 = self.get_parameter("enc_sign_m4").value   # RL

        # ---------- Correction Factors ----------
        self.declare_parameter("angle_scale_factor", 1.0)
        self.declare_parameter("invert_z", False)

        self.angle_scale_factor = self.get_parameter("angle_scale_factor").value
        self.invert_z = self.get_parameter("invert_z").value

        # ✅✅✅ 新增：控制是否发布 TF 的参数
        self.declare_parameter("publish_tf", False)
        self.publish_tf = self.get_parameter("publish_tf").value

        # ---------- Publishers / Subscribers ----------
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_br    = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Int32MultiArray, "wheel_ticks", self.ticks_cb, 10)

        # ---------- Timer ----------
        # Publish at 20Hz regardless of serial data rate
        self.create_timer(0.05, self.timer_cb)

        # ---------- State ----------
        self.last_ticks = None
        self.last_time  = self.get_clock().now()
        self.x  = 0.0
        self.y  = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Optional covariances
        self.declare_parameter("pose_cov_xy", 5e-2)
        self.declare_parameter("pose_cov_th", 5e-1)
        self.declare_parameter("twist_cov", 5e-2)

        self.pose_cov_xy = self.get_parameter("pose_cov_xy").value
        self.pose_cov_th = self.get_parameter("pose_cov_th").value
        self.twist_cov   = self.get_parameter("twist_cov").value

    def ticks_cb(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        ticks = list(msg.data)
        if self.last_ticks is None:
            self.last_ticks = ticks
            return

        # delta ticks
        d = [ticks[i] - self.last_ticks[i] for i in range(4)]
        self.last_ticks = ticks

        # ticks -> wheel angular rates (rad/s)
        w1 = (d[0] / self.tpr_m1) * 2.0 * math.pi / dt * self.s1
        w2 = (d[1] / self.tpr_m2) * 2.0 * math.pi / dt * self.s2
        w3 = (d[2] / self.tpr_m3) * 2.0 * math.pi / dt * self.s3
        w4 = (d[3] / self.tpr_m4) * 2.0 * math.pi / dt * self.s4

        # Forward kinematics using the same wheel order as commands.h:
        #   M1=FL, M2=FR, M3=RR, M4=RL
        self.vx  = (self.R / 4.0) * ( w1 + w2 + w3 + w4 )
        self.vy  = (self.R / 4.0) * (-w1 + w2 - w3 + w4 )
        self.vth = (self.R / (4.0 * self.a)) * (-w1 + w2 + w3 - w4 )
        
        # Apply Corrections
        self.vth *= self.angle_scale_factor
        if self.invert_z:
            self.vth = -self.vth

        # Integrate
        self.x += (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        self.y += (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        self.th += self.vth * dt
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))  # wrap

        # DEBUG LOGGING (throttle 5s to reduce spam)
        # self.get_logger().info(
        #     f"Odom: x={self.x:.2f} y={self.y:.2f} th={self.th:.2f} | vth={self.vth:.3f} | Ticks={ticks}",
        #     throttle_duration_sec=5.0
        # )

        # Note: We do NOT publish here anymore. We just update state.

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def timer_cb(self):
        # Always publish TF and Odom, even if robot is stationary or serial is lagging
        now = self.get_clock().now()

        # q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.th)
        q = self.euler_to_quaternion(0.0, 0.0, self.th)
        
        quat = Quaternion()
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]

        # Odometry msg
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id  = self.frame_basefp

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        # Covariances
        odom.pose.covariance = [
            self.pose_cov_xy, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.pose_cov_xy, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.pose_cov_th
        ]
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.linear.y  = self.vy
        odom.twist.twist.angular.z = self.vth
        odom.twist.covariance = [
            self.twist_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.twist_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.twist_cov
        ]

        self.odom_pub.publish(odom)

        # TF: odom -> base_footprint
        t = TransformStamped()
        
        # FIX: Publish TF slightly in the future to ensure valid lookups for laser scans
        # This prevents "Extrapolation into future" errors if laser scan is slightly ahead
        # or allows lookups for slightly old scans if they fall within the window.
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_odom
        t.child_frame_id = self.frame_basefp
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat

        # ✅✅✅ 修改：只有当开关打开时，才发布 TF
        if self.publish_tf:
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
