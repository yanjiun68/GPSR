#!/usr/bin/env python3
"""
自动运动执行程序 (side_motion_executor) — X-drive 全向底盘版

功能:
    订阅手指指向方向, 一旦稳定识别到 "left" 或 "right" 就自动向 /cmd_vel
    发布 Twist, 执行以下运动序列 (共 total_duration_sec 默认 6.5s 后停止):

        left  (手指指向左边的包): 向左横移 strafe_duration_sec (1.5s)
                                   -> 前进 (total - strafe) (5s) -> 停止
        right (手指指向右边的包): 向右横移 strafe_duration_sec (1.5s)
                                   -> 前进 (total - strafe) (5s) -> 停止

    因为底盘是 X-drive (全向), 横移使用 Twist.linear.y:
        ROS 约定: +linear.y = 向左,  -linear.y = 向右
        +linear.x = 向前
    请确认底盘驱动监听 linear.y, 否则横移无效.

触发源 (默认 /pointing_direction_raw, 不需要启动 session):
  1) /pointing_direction_raw  [std_msgs/String, 值 "left"/"right"/"none"]
     - pose_pointing_node 每帧发布, 手指指向已识别的包就会发出.
     - 内置防抖: 同一方向稳定 confirm_sec 秒后才触发.
  2) /selected_side  [std_msgs/String]
     - 需要先调用 /bag_selection/start 启动完整 session 流程.
     - 触发时建议把 confirm_sec 设为 0.

输出:
  1) /bag_ready_for_pickup [std_msgs/Bool]
     - False: 尚未到达 / 已重置 / 新一轮动作开始
     - True : 运动序列执行完成, 机器人已到位并停止

运行:
    # 默认参数 (横移 1.5s + 前进 5s = 总 6.5s)
    python3 side_motion_executor.py

    # 调参数
    python3 side_motion_executor.py --ros-args \
        -p strafe_speed:=0.15 \
        -p forward_speed:=0.12 \
        -p strafe_duration_sec:=1.5 \
        -p total_duration_sec:=6.5 \
        -p confirm_sec:=0.7 \
        -p cooldown_sec:=2.0

    # 改用 /selected_side 触发
    python3 side_motion_executor.py --ros-args \
        -p trigger_topic:=/selected_side \
        -p confirm_sec:=0.0
"""

from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


Step = Tuple[float, float, float, float, str]  # (linear_x, linear_y, angular_z, duration_sec, label)


class SideMotionExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("side_motion_executor_node")

        self.declare_parameter("trigger_topic", "/pointing_direction_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("reached_topic", "/bag_ready_for_pickup")
        self.declare_parameter("reset_topic", "/bag_selection/reset")
        self.declare_parameter("strafe_speed", 0.15)
        self.declare_parameter("forward_speed", 0.12)
        self.declare_parameter("strafe_duration_sec", 1.5)
        self.declare_parameter("total_duration_sec", 6.5)
        self.declare_parameter("confirm_sec", 0.7)
        self.declare_parameter("cooldown_sec", 2.0)
        self.declare_parameter("control_rate_hz", 20.0)

        trigger_topic = str(self.get_parameter("trigger_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        reached_topic = str(self.get_parameter("reached_topic").value)
        reset_topic = str(self.get_parameter("reset_topic").value)

        self.strafe_speed = float(self.get_parameter("strafe_speed").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.strafe_duration = float(self.get_parameter("strafe_duration_sec").value)
        self.total_duration = float(self.get_parameter("total_duration_sec").value)
        self.confirm_sec = float(self.get_parameter("confirm_sec").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.forward_duration = max(0.0, self.total_duration - self.strafe_duration)

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.reached_pub = self.create_publisher(Bool, reached_topic, 10)
        self.create_subscription(String, trigger_topic, self.trigger_callback, 10)
        self.create_subscription(Bool, reset_topic, self.reset_callback, 10)

        self.pending_side: Optional[str] = None
        self.pending_started_sec: Optional[float] = None

        self.sequence: List[Step] = []
        self.step_idx: int = 0
        self.step_end_sec: Optional[float] = None
        self.executing: bool = False
        self.reached: bool = False
        self.cooldown_until_sec: float = 0.0

        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self.control_loop)
        self._publish_reached(False)

        self.get_logger().info(
            f"side_motion_executor 启动 [X-drive]: 订阅 {trigger_topic}, 发布 {cmd_topic}, "
            f"到位布尔 {reached_topic} | "
            f"横移 {self.strafe_speed} m/s x {self.strafe_duration}s, "
            f"前进 {self.forward_speed} m/s x {self.forward_duration}s "
            f"(总 {self.total_duration}s), 防抖 {self.confirm_sec}s, 冷却 {self.cooldown_sec}s"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def trigger_callback(self, msg: String) -> None:
        side = msg.data.strip().lower()
        now = self.now_sec()

        if self.executing or now < self.cooldown_until_sec:
            self.pending_side = None
            self.pending_started_sec = None
            return

        if side not in ("left", "right"):
            self.pending_side = None
            self.pending_started_sec = None
            return

        if self.pending_side != side:
            self.pending_side = side
            self.pending_started_sec = now
            return

        if self.pending_started_sec is None:
            self.pending_started_sec = now
            return

        if now - self.pending_started_sec >= self.confirm_sec:
            self._start_sequence(side)
            self.pending_side = None
            self.pending_started_sec = None

    def reset_callback(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        self.pending_side = None
        self.pending_started_sec = None
        self.sequence = []
        self.step_idx = 0
        self.step_end_sec = None
        self.executing = False
        self.cooldown_until_sec = 0.0
        self._publish_stop()
        self._publish_reached(False)

    def _build_sequence(self, side: str) -> List[Step]:
        strafe_y = +self.strafe_speed if side == "left" else -self.strafe_speed
        strafe_label = "向左横移" if side == "left" else "向右横移"
        steps: List[Step] = []
        if self.strafe_duration > 0.0:
            steps.append((0.0, strafe_y, 0.0, self.strafe_duration, strafe_label))
        if self.forward_duration > 0.0:
            steps.append((self.forward_speed, 0.0, 0.0, self.forward_duration, "前进"))
        return steps

    def _start_sequence(self, side: str) -> None:
        self.sequence = self._build_sequence(side)
        if not self.sequence:
            self.get_logger().warn("运动序列为空 (strafe_duration 和 forward_duration 都为 0), 不执行")
            return

        desc = " -> ".join(f"{label}{dur}s" for _, _, _, dur, label in self.sequence)
        self.get_logger().info(
            f"识别到 {'左' if side == 'left' else '右'}边 -> {desc} -> 停止"
        )

        self.step_idx = 0
        self._publish_reached(False)
        _, _, _, dur, _ = self.sequence[0]
        self.step_end_sec = self.now_sec() + dur
        self.executing = True
        self._publish_current_step()

    def _publish_current_step(self) -> None:
        if not (0 <= self.step_idx < len(self.sequence)):
            return
        lx, ly, az, _, _ = self.sequence[self.step_idx]
        self._publish_twist(lx, ly, az)

    def _publish_twist(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)

    def _publish_stop(self) -> None:
        self._publish_twist(0.0, 0.0, 0.0)

    def _publish_reached(self, reached: bool) -> None:
        self.reached = bool(reached)
        self.reached_pub.publish(Bool(data=self.reached))

    def control_loop(self) -> None:
        if not self.executing:
            return

        now = self.now_sec()
        if self.step_end_sec is not None and now >= self.step_end_sec:
            self.step_idx += 1
            if self.step_idx >= len(self.sequence):
                self._publish_stop()
                self.executing = False
                self.sequence = []
                self.step_end_sec = None
                self.cooldown_until_sec = now + self.cooldown_sec
                self._publish_reached(True)
                self.get_logger().info(
                    f"运动序列完成, 机器人已停止 (冷却 {self.cooldown_sec}s)"
                )
                return
            _, _, _, dur, name = self.sequence[self.step_idx]
            self.step_end_sec = now + dur
            self.get_logger().info(f"进入第 {self.step_idx + 1} 步: {name} ({dur}s)")

        self._publish_current_step()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SideMotionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
