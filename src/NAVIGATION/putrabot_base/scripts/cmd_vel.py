#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class CmdVelToWheelTargets(Node):
    """
    Convert /cmd_vel (Twist) → /wheel_targets (Int32MultiArray)
    Order: [FL, FR, RR, RL]  in ticks/sec

    - Holonomic X-drive
    - Accel limited
    - Watchdog timeout
    """

    def __init__(self):
        super().__init__("cmdvel_to_wheel_targets")

        # ---------------- Robot geometry ----------------
        self.R  = self.declare_parameter("wheel_radius", 0.05).value
        self.lx = self.declare_parameter("lx", 0.2051).value
        self.ly = self.declare_parameter("ly", 0.2051).value
        self.a  = self.lx + self.ly

        # ---------------- Encoder ticks / rev ----------------
        self.tpr = [
            self.declare_parameter("tpr_fl", 3332.0).value,
            self.declare_parameter("tpr_fr", 3332.0).value,
            self.declare_parameter("tpr_rr", 3332.0).value,
            self.declare_parameter("tpr_rl", 3332.0).value,
        ]

        # ---------------- Motor sign ----------------
        self.sign = [
            -1 if self.declare_parameter("invert_fl", False).value else 1,
            -1 if self.declare_parameter("invert_fr", False).value else 1,
            -1 if self.declare_parameter("invert_rr", False).value else 1,
            -1 if self.declare_parameter("invert_rl", False).value else 1,
        ]

        # ---------------- Limits ----------------
        self.vx_max = self.declare_parameter("vx_max", 0.8).value
        self.vy_max = self.declare_parameter("vy_max", 0.8).value
        self.wz_max = self.declare_parameter("wz_max", 2.5).value

        # accel limits
        self.ax = self.declare_parameter("ax_max", 1.5).value
        self.ay = self.declare_parameter("ay_max", 1.5).value
        self.aw = self.declare_parameter("aw_max", 4.0).value

        # ---------------- Timing ----------------
        self.rate = self.declare_parameter("publish_rate", 50.0).value
        self.timeout = self.declare_parameter("timeout", 1.0).value

        # ---------------- ROS I/O ----------------
        self.pub = self.create_publisher(Int32MultiArray, "wheel_targets", 10)
        self.sub = self.create_subscription(Twist, "cmd_vel", self.cmd_cb, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.update)

        # ---------------- State ----------------
        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.get_logger().info("Holonomic cmd_vel → wheel_targets node started")

    # --------------------------------------------------
    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def slew(curr, target, rate, dt):
        if rate <= 0.0:
            return target
        step = rate * dt
        return max(curr - step, min(curr + step, target))

    # --------------------------------------------------
    def cmd_cb(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    # --------------------------------------------------
    def update(self):
        now = self.get_clock().now()
        dt = 1.0 / self.rate

        # Watchdog
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout:
            vx_t = vy_t = wz_t = 0.0
        else:
            vx_t = self.clamp(self.cmd.linear.x,  -self.vx_max, self.vx_max)
            vy_t = self.clamp(self.cmd.linear.y,  -self.vy_max, self.vy_max)
            wz_t = self.clamp(self.cmd.angular.z, -self.wz_max, self.wz_max)

        # Accel limiting
        self.vx = self.slew(self.vx, vx_t, self.ax, dt)
        self.vy = self.slew(self.vy, vy_t, self.ay, dt)
        self.wz = self.slew(self.wz, wz_t, self.aw, dt)

        # ---------------- Holonomic X-drive kinematics ----------------
        # FL, FR, RR, RL
        w = [
            ( self.vx - self.vy - self.a * self.wz ) / self.R,  # FL
            ( self.vx + self.vy + self.a * self.wz ) / self.R,  # FR
            ( self.vx + self.vy - self.a * self.wz ) / self.R,  # RR  ✅ FIXED
            ( self.vx - self.vy + self.a * self.wz ) / self.R,  # RL  ✅ FIXED
        ]

        # rad/s → ticks/s
        targets = []
        for i in range(4):
            tps = w[i] * self.tpr[i] / (2.0 * math.pi)
            targets.append(int(round(self.sign[i] * tps)))

        self.pub.publish(Int32MultiArray(data=targets))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheelTargets()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
