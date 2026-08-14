#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Person Follower with Nav2 Integration
跟人 + Nav2 避障版本

使用 Nav2 navigate_to_pose 代替直接控制 /cmd_vel，
实现跟人的同时具备避障能力。

使用方法:
    1. 先启动导航系统: ros2 launch my_robot_nav real_robot.launch.py
    2. 再启动跟随系统: ros2 launch vision follow_bringup.launch.py
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from vision.msg import CamDetections
from action_msgs.msg import GoalStatus


class PersonFollowerNav2(Node):
    def __init__(self):
        super().__init__('person_follower_nav2')

        # ---------------- Parameters ----------------
        self.declare_parameter('target_distance', 1.0)  # 与人保持的距离 (米)
        self.declare_parameter('goal_update_interval', 1.0)  # 目标更新间隔 (秒)
        self.declare_parameter('min_move_distance', 0.3)  # 最小移动距离，避免频繁发目标

        self.target_dist = self.get_parameter('target_distance').value
        self.goal_interval = self.get_parameter('goal_update_interval').value
        self.min_move_dist = self.get_parameter('min_move_distance').value

        # ---------------- Nav2 Action Client ----------------
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # ---------------- Subscription ----------------
        self.sub_dets = self.create_subscription(
            CamDetections,
            '/cam_detections',
            self.dets_cb,
            10
        )

        # ---------------- State ----------------
        self.last_detection_time = self.get_clock().now()
        self.last_goal_time = self.get_clock().now()
        self.current_goal_handle = None
        self.target_pose = None  # PoseStamped in map frame
        self.last_sent_pose = None  # 上次发送的目标

        # Timer for timeout check
        self.timer = self.create_timer(0.5, self.check_timeout)

        self.get_logger().info("Person Follower Nav2 Node Started")
        self.get_logger().info(f"Target distance: {self.target_dist}m, Update interval: {self.goal_interval}s")

    def dets_cb(self, msg: CamDetections):
        """接收人检测结果，计算目标位置"""
        if not msg.detections:
            return

        # 找到最近的人
        closest_dist = 999.0
        target = None

        for det in msg.detections:
            d = math.sqrt(det.x**2 + det.y**2)
            if d < closest_dist:
                closest_dist = d
                target = det

        if target is None:
            return

        self.last_detection_time = self.get_clock().now()

        # 计算目标点 (在人的位置前 target_dist 米处)
        # 人的位置已经在 map 坐标系 (由 vision_node 转换)
        person_x = target.x
        person_y = target.y

        # 计算机器人应该停留的位置 (人的位置减去指向机器人方向的 target_dist)
        # 假设机器人在原点，方向向量是 (person_x, person_y)
        dist_to_person = math.sqrt(person_x**2 + person_y**2)
        
        if dist_to_person < 0.1:  # 太近了，不处理
            return

        # 单位向量
        ux = person_x / dist_to_person
        uy = person_y / dist_to_person

        # 目标点：人的位置后退 target_dist
        goal_x = person_x - ux * self.target_dist
        goal_y = person_y - uy * self.target_dist

        # 朝向：面向人
        goal_yaw = math.atan2(person_y - goal_y, person_x - goal_x)

        # 创建目标 Pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0

        # 四元数 (yaw only)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

        self.target_pose = goal_pose

        # 检查是否应该发送新目标
        self.maybe_send_goal()

    def maybe_send_goal(self):
        """根据时间间隔和距离变化决定是否发送新目标"""
        if self.target_pose is None:
            return

        now = self.get_clock().now()
        time_since_last = (now - self.last_goal_time).nanoseconds / 1e9

        # 检查时间间隔
        if time_since_last < self.goal_interval:
            return

        # 检查移动距离 (避免人不动时频繁发目标)
        if self.last_sent_pose is not None:
            dx = self.target_pose.pose.position.x - self.last_sent_pose.pose.position.x
            dy = self.target_pose.pose.position.y - self.last_sent_pose.pose.position.y
            move_dist = math.sqrt(dx**2 + dy**2)
            if move_dist < self.min_move_dist:
                return

        # 发送目标
        self.send_goal(self.target_pose)

    def send_goal(self, pose: PoseStamped):
        """发送导航目标到 Nav2"""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        # 取消之前的目标
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling previous goal')
            self.current_goal_handle.cancel_goal_async()

        # 发送新目标
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.last_goal_time = self.get_clock().now()
        self.last_sent_pose = pose

    def goal_response_callback(self, future):
        """目标发送响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """导航结果回调"""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal canceled (new target sent)')
        else:
            self.get_logger().info(f'Goal finished with status: {status}')

        self.current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        """导航反馈回调"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        # 减少日志输出
        # self.get_logger().info(f'Distance remaining: {distance:.2f}m')

    def check_timeout(self):
        """检查检测超时，停止跟随"""
        time_since_last = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        
        if time_since_last > 2.0:  # 2秒无检测
            if self.current_goal_handle is not None:
                self.get_logger().info('No person detected, canceling navigation')
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNav2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # 清理：取消当前目标
    if node.current_goal_handle is not None:
        node.current_goal_handle.cancel_goal_async()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
