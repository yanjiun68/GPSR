#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class CmdVelToTargets(Node):
    """
    Twist (/cmd_vel) -> Int32MultiArray (/wheel_targets: [FL, FR, RR, RL] in ticks/sec)
    Also logs per-wheel RPM to terminal for easy tuning.
    Kinematics: 4-omni X-drive
        w_FL = (vx - vy - a*wz) / R
        w_FR = (vx + vy + a*wz) / R
        w_RR = (vx - vy + a*wz) / R
        w_RL = (vx + vy - a*wz) / R
      where a = lx + ly, R = wheel_radius
    """

    def __init__(self):
        super().__init__('cmdvel_to_targets')

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

        # ---------- Optional per-wheel sign (software invert for motors) ----------
        self.declare_parameter("invert_m1", False)
        self.declare_parameter("invert_m2", False)
        self.declare_parameter("invert_m3", False)
        self.declare_parameter("invert_m4", False)

        self.s1 = -1 if self.get_parameter("invert_m1").value else 1
        self.s2 = -1 if self.get_parameter("invert_m2").value else 1
        self.s3 = -1 if self.get_parameter("invert_m3").value else 1
        self.s4 = -1 if self.get_parameter("invert_m4").value else 1

        # ---------- Motion limits & filters ----------
        self.declare_parameter("vx_max", 0.50)
        self.declare_parameter("vy_max", 0.20)
        self.declare_parameter("wz_max", 0.50)
        self.declare_parameter("deadband", 0.0)
        self.declare_parameter("min_effective_wz", 0.0)

        self.vx_max   = self.get_parameter("vx_max").value    # [m/s]
        self.vy_max   = self.get_parameter("vy_max").value    # [m/s]
        self.wz_max   = self.get_parameter("wz_max").value    # [rad/s]
        self.deadband = self.get_parameter("deadband").value
        self.min_effective_wz = self.get_parameter("min_effective_wz").value

        # Per-wheel max ticks/sec (set from measurement; keep huge to disable)
        self.declare_parameter("max_tps_m1", 1e9)
        self.declare_parameter("max_tps_m2", 1e9)
        self.declare_parameter("max_tps_m3", 1e9)
        self.declare_parameter("max_tps_m4", 1e9)

        self.max_tps = [
            self.get_parameter("max_tps_m1").value,  # FL
            self.get_parameter("max_tps_m2").value,  # FR
            self.get_parameter("max_tps_m3").value,  # RR
            self.get_parameter("max_tps_m4").value,  # RL
        ]

        # ---------- Timing ----------
        self.declare_parameter("timeout", 0.35)
        self.declare_parameter("pub_rate", 20.0)

        self.timeout_s = self.get_parameter("timeout").value  # watchdog for /cmd_vel
        self.pub_rate  = self.get_parameter("pub_rate").value
        self.last_cmd_time = self.get_clock().now()

        # ---------- Topics ----------
        # 默认订阅最终底盘控制话题。
        # 在 Nav2 场景下推荐通过 launch 把它指向 /cmd_vel_safe，
        # 让链路变成 controller -> /cmd_vel_nav -> smoother -> /cmd_vel -> collision_monitor -> /cmd_vel_safe
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("targets_topic", "/wheel_targets")

        cmd_topic     = self.get_parameter("cmd_vel_topic").value
        targets_topic = self.get_parameter("targets_topic").value

        self.pub = self.create_publisher(Int32MultiArray, targets_topic, 10)
        self.sub = self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / self.pub_rate, self.on_timer)
        
        self.last_targets = [0, 0, 0, 0]

        self.get_logger().info(
            f"[cmdvel] R={self.R:.3f}  lx={self.lx:.3f}  ly={self.ly:.3f}  a={self.a:.3f}  TPR=[{self.tpr_m1:.0f} {self.tpr_m2:.0f} {self.tpr_m3:.0f} {self.tpr_m4:.0f}]"
        )

    # ---------- helpers ----------
    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def _dead(self, x):
        return 0.0 if abs(x) < self.deadband else x

    def _compute_targets(self, vx, vy, wz):
        # deadband + clamp
        vx = self._dead(self._clamp(vx, -self.vx_max, self.vx_max))
        vy = self._dead(self._clamp(vy, -self.vy_max, self.vy_max))
        wz = self._dead(self._clamp(wz, -self.wz_max, self.wz_max))
        if 0.0 < abs(wz) < self.min_effective_wz:
            wz = math.copysign(self.min_effective_wz, wz)

        # Standard 4-wheel X-drive / mecanum-style inverse kinematics.
        # Wheel order matches firmware commands.h:
        #   M1=FL, M2=FR, M3=RR, M4=RL
        w1 = (vx - vy - self.a * wz) / self.R
        w2 = (vx + vy + self.a * wz) / self.R
        w3 = (vx - vy + self.a * wz) / self.R
        w4 = (vx + vy - self.a * wz) / self.R

        # apply optional motor polarity
        w1 *= self.s1; w2 *= self.s2; w3 *= self.s3; w4 *= self.s4

        # ---------- NEW: compute RPM for logging ----------
        # rpm = rad/s * 60 / (2π)
        k_rpm = 60.0 / (2.0 * math.pi)
        rpm = [
            w1 * k_rpm,   # FL
            w2 * k_rpm,   # FR
            w3 * k_rpm,   # RR
            w4 * k_rpm,   # RL
        ]

        # rad/s -> ticks/s (keep as float for scaling)
        k_tps = 1.0 / (2.0 * math.pi)
        t = [
            w1 * self.tpr_m1 * k_tps,
            w2 * self.tpr_m2 * k_tps,
            w3 * self.tpr_m3 * k_tps,
            w4 * self.tpr_m4 * k_tps,
        ]

        # uniform scale-down if any wheel exceeds its cap
        scales = []
        for i in range(4):
            mt = self.max_tps[i]
            if abs(t[i]) > mt:
                scales.append(mt / float(abs(t[i])))
        if scales:
            s = min(scales)
            t  = [s * ti for ti in t]
            rpm = [s * r  for r  in rpm]  # keep RPM log consistent with scaled targets

        # round to ints for Arduino
        ti = [int(round(x)) for x in t]

        # ---------- Logs (throttled) ----------
        # self.get_logger().info(
        #     f"CMD vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} | RPM[FL,FR,RR,RL]=[{rpm[0]:.1f} {rpm[1]:.1f} {rpm[2]:.1f} {rpm[3]:.1f}] | TPS=[{ti[0]} {ti[1]} {ti[2]} {ti[3]}]",
        #     throttle_duration_sec=1.0
        # )

        return ti

    # ---------- callbacks ----------
    def on_cmd(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.last_targets = self._compute_targets(msg.linear.x, msg.linear.y, msg.angular.z)
        self.pub.publish(Int32MultiArray(data=self.last_targets))

    def publish_stop(self):
        stop_msg = Int32MultiArray(data=[0, 0, 0, 0])
        self.last_targets = [0, 0, 0, 0]
        self.pub.publish(stop_msg)

    def on_timer(self):
        # Watchdog
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9 > self.timeout_s:
            if self.last_targets != [0, 0, 0, 0]:
                self.get_logger().warn("Watchdog: publishing STOP [0,0,0,0]")
                self.last_targets = [0, 0, 0, 0]
            self.pub.publish(Int32MultiArray(data=[0, 0, 0, 0]))
        else:
            # Republish last targets (optional but good for robustness to packet loss)
            self.pub.publish(Int32MultiArray(data=self.last_targets))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToTargets()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
