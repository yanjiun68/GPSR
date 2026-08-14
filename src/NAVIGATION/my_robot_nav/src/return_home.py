#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
返回原点脚本 - 使用 Nav2 Action Client
机器人到达目标后，运行此脚本让机器人返回起始点 (0, 0)

使用方法:
    ros2 run my_robot_nav return_home.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import sys
import time


class ReturnHomeNode(Node):
    def __init__(self):
        super().__init__('return_home_node')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None
        self.result_received = False
        self.goal_accepted = False
        self.exit_code = 1
        
    def send_goal(self):
        # 等待 action server
        self.get_logger().info('⏳ 等待 navigate_to_pose action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Nav2 action server 未启动')
            return False
            
        self.get_logger().info('✅ Nav2 已连接!')
        
        # 创建目标 Pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info('🚀 发送目标: 返回原点 (0, 0)...')
        
        # 发送目标 (异步)
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 目标被拒绝！Nav2 无法规划路径到 (0,0)')
            self.exit_code = 2
            self.result_received = True
            return
            
        self.get_logger().info('📍 目标已接受，正在导航...')
        self.goal_handle = goal_handle
        self.goal_accepted = True
        
        # 获取结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        
    def result_callback(self, future):
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ 成功返回原点!')
            self.exit_code = 0
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('⚠️ 导航被取消')
            self.exit_code = 3
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('❌ 返回原点失败 (路径被阻挡或规划失败)')
            self.exit_code = 4
        else:
            self.get_logger().warn(f'⚠️ 导航结束，状态码: {status}')
            self.exit_code = 5
            
        self.result_received = True
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().info(f'📍 距离原点: {distance:.2f} 米')


def main():
    rclpy.init()
    
    node = ReturnHomeNode()
    
    if not node.send_goal():
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # 等待结果，带超时
    start_time = time.time()
    timeout = 120.0  # 2分钟超时
    
    try:
        while not node.result_received:
            rclpy.spin_once(node, timeout_sec=0.5)
            
            elapsed = time.time() - start_time
            if elapsed > timeout:
                node.get_logger().error(f'❌ 超时 ({timeout}秒)，取消导航')
                node.exit_code = 6
                if node.goal_handle:
                    node.goal_handle.cancel_goal_async()
                break
                
            # 如果10秒还没接受目标，可能有问题
            if elapsed > 10.0 and not node.goal_accepted:
                node.get_logger().warn('⚠️ 等待目标接受中...(如果卡住请检查 Nav2 终端)')
                
    except KeyboardInterrupt:
        node.get_logger().info('⚠️ 用户取消')
        node.exit_code = 130
        if node.goal_handle:
            node.goal_handle.cancel_goal_async()
    
    node.destroy_node()
    rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    main()
