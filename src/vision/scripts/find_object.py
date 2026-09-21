#!/usr/bin/env -S bash -c 'exec "${VISION_PYTHON:-${HOME}/ai/qwen3v1/.venv/bin/python}" "$0" "$@"'
# -*- coding: utf-8 -*-
"""
独立搜索目标物品命令

功能：
    - 机器人旋转扫描 + 路点巡逻
    - 找到目标物体后导航过去
    - 停在目标前面，摄像头正对目标
    - 自动退出程序

使用方法：
    1. 先启动机器人: ros2 launch my_robot_nav real_robot.launch.py
    2. 运行搜索:     ros2 run vision find_object.py
    3. 搜索完成后程序自动退出
"""

import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy
from enum import Enum

# YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed, detection disabled")


class SearchState(Enum):
    SEARCHING = 1      # 搜索中
    APPROACHING = 2    # 正在接近目标
    ARRIVED = 3        # 已到达目标


class FindObject(Node):
    def __init__(self):
        super().__init__('find_object')

        # ---------------- Parameters ----------------
        self.declare_parameter('target_distance', 0.8)        # 停在目标前的距离
        self.declare_parameter('rotation_speed', 0.3)         # 旋转速度 (rad/s)
        self.declare_parameter('rotation_step', 0.5236)       # 每次旋转30度
        self.declare_parameter('scan_pause', 1.0)             # 旋转后暂停检测时间
        self.declare_parameter('detection_timeout', 3.0)      # 目标丢失超时
        self.declare_parameter('yolo_model', 'yolo11n.pt')    # YOLO模型路径
        self.declare_parameter('yolo_conf', 0.3)              # 检测置信度
        self.declare_parameter('target_class', 0)             # 目标类别ID (0=person)
        
        # 搜索路点
        self.declare_parameter('search_waypoints', [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0])

        self.target_dist = self.get_parameter('target_distance').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.rotation_step = self.get_parameter('rotation_step').value
        self.scan_pause = self.get_parameter('scan_pause').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.target_class = self.get_parameter('target_class').value
        
        # 解析路点
        wp = list(self.get_parameter('search_waypoints').value)
        self.waypoints = [(wp[i], wp[i+1]) for i in range(0, len(wp)-1, 2)]
        
        # ---------------- YOLO ----------------
        self.model = None
        if YOLO_AVAILABLE:
            model_path = self.get_parameter('yolo_model').value
            self.yolo_conf = self.get_parameter('yolo_conf').value
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f'Loaded YOLO model: {model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO: {e}')
        
        # ---------------- State ----------------
        self.state = SearchState.SEARCHING
        self.waypoint_index = 0
        self.rotation_accumulated = 0.0
        self.is_rotating = False
        self.is_navigating = False
        self.is_paused = False
        
        self.target_position = None
        self.last_detection_time = None
        self.task_complete = False
        
        # ---------------- Camera ----------------
        self.bridge = CvBridge()
        self.cam_info = None
        self.last_depth = None
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.sub_rgb = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.rgb_callback, qos)
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos)
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_callback, qos)
        
        # ---------------- Nav2 ----------------
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal = None
        
        # ---------------- Control ----------------
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 主循环
        self.timer = self.create_timer(0.1, self.main_loop)
        
        self.get_logger().info('='*50)
        self.get_logger().info('🔍 Find Object - 搜索目标物品')
        self.get_logger().info(f'   Target class: {self.target_class}')
        self.get_logger().info(f'   Target distance: {self.target_dist}m')
        self.get_logger().info('='*50)

    # ==========================================================
    # Camera & Detection
    # ==========================================================
    def info_callback(self, msg):
        self.cam_info = msg
    
    def depth_callback(self, msg):
        self.last_depth = msg
    
    def rgb_callback(self, msg):
        """处理RGB图像，进行目标检测"""
        if self.model is None or self.cam_info is None or self.last_depth is None:
            return
        
        if self.task_complete:
            return
        
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(self.last_depth, 'passthrough')
        except Exception as e:
            return
        
        # YOLO检测
        results = self.model.predict(rgb, imgsz=640, conf=self.yolo_conf, verbose=False)
        if not results or results[0].boxes is None:
            return
        
        # 找目标类别
        fx, fy = self.cam_info.k[0], self.cam_info.k[4]
        cx, cy = self.cam_info.k[2], self.cam_info.k[5]
        h, w = depth.shape[:2]
        
        best_target = None
        best_dist = float('inf')
        
        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())
            if cls_id != self.target_class:
                continue
            
            xyxy = box.xyxy[0].cpu().numpy()
            u = int((xyxy[0] + xyxy[2]) / 2)
            v = int((xyxy[1] + xyxy[3]) / 2)
            
            if 0 <= u < w and 0 <= v < h:
                Z = float(depth[v, u])
                if self.last_depth.encoding in ['16UC1', 'mono16']:
                    Z /= 1000.0
                
                if Z > 0.1 and Z < 10.0:
                    Xc = (u - cx) * Z / fx
                    Yc = (v - cy) * Z / fy
                    # 在base_footprint坐标系: X向前, Y向左
                    # 相机坐标系: Z向前, X向右, Y向下
                    dist = math.sqrt(Z**2 + Xc**2)
                    
                    if dist < best_dist:
                        best_dist = dist
                        # 简化: 假设相机朝前，Z就是前方距离
                        best_target = (Z, -Xc)  # (forward, left)
        
        if best_target:
            self.target_position = best_target
            self.last_detection_time = self.get_clock().now()
            
            self.get_logger().info(
                f'🎯 Target detected at ({best_target[0]:.2f}, {best_target[1]:.2f}), dist={best_dist:.2f}m',
                throttle_duration_sec=1.0
            )
            
            # 状态转换
            if self.state == SearchState.SEARCHING:
                self.get_logger().info('Target found! Switching to APPROACHING')
                self.state = SearchState.APPROACHING
                self.stop_robot()
                self.cancel_navigation()
                self.is_rotating = False
                self.is_paused = False
            
            # 检查是否到达
            if self.state == SearchState.APPROACHING and best_dist <= self.target_dist + 0.2:
                self.get_logger().info('✅ Arrived at target!')
                self.state = SearchState.ARRIVED
                self.cancel_navigation()

    # ==========================================================
    # Main Loop
    # ==========================================================
    def main_loop(self):
        if self.task_complete:
            return
        
        # 检查目标丢失
        if self.state == SearchState.APPROACHING and self.last_detection_time:
            elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
            if elapsed > self.detection_timeout:
                self.get_logger().warn('Target lost, returning to search')
                self.state = SearchState.SEARCHING
                self.cancel_navigation()
                self.target_position = None
        
        # 状态处理
        if self.state == SearchState.SEARCHING:
            self.do_search()
        elif self.state == SearchState.APPROACHING:
            self.do_approach()
        elif self.state == SearchState.ARRIVED:
            self.do_arrived()

    # ==========================================================
    # Search Behavior
    # ==========================================================
    def do_search(self):
        if self.is_paused or self.is_navigating:
            return
        
        if self.rotation_accumulated < 2 * math.pi:
            # 原地旋转
            if not self.is_rotating:
                self.is_rotating = True
                twist = Twist()
                twist.angular.z = self.rotation_speed
                self.pub_cmd.publish(twist)
                
                duration = self.rotation_step / abs(self.rotation_speed)
                self.create_timer(duration, self.on_rotation_done, oneshot=True)
        else:
            # 去下一个路点
            self.rotation_accumulated = 0.0
            self.go_to_next_waypoint()
    
    def on_rotation_done(self):
        self.stop_robot()
        self.rotation_accumulated += self.rotation_step
        self.is_rotating = False
        
        # 暂停让相机检测
        self.is_paused = True
        self.create_timer(self.scan_pause, self.on_pause_done, oneshot=True)
        
        deg = math.degrees(self.rotation_accumulated)
        self.get_logger().info(f'Scanning: {deg:.0f}°/360°')
    
    def on_pause_done(self):
        self.is_paused = False
    
    def go_to_next_waypoint(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints!')
            return
        
        wp = self.waypoints[self.waypoint_index % len(self.waypoints)]
        self.waypoint_index += 1
        
        self.get_logger().info(f'📍 Going to waypoint ({wp[0]:.1f}, {wp[1]:.1f})')
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        pose.pose.orientation.w = 1.0
        
        self.send_nav_goal(pose)

    # ==========================================================
    # Approach Behavior
    # ==========================================================
    def do_approach(self):
        """接近目标：简单的前进+转向控制"""
        if self.target_position is None:
            return
        
        forward, left = self.target_position
        dist = math.sqrt(forward**2 + left**2)
        
        if dist <= self.target_dist:
            self.stop_robot()
            return
        
        twist = Twist()
        
        # 转向目标
        angle_to_target = math.atan2(left, forward)
        twist.angular.z = 1.0 * angle_to_target
        twist.angular.z = max(-0.5, min(0.5, twist.angular.z))
        
        # 如果角度小，则前进
        if abs(angle_to_target) < 0.3:
            error = dist - self.target_dist
            twist.linear.x = 0.3 * error
            twist.linear.x = max(0.0, min(0.15, twist.linear.x))
        
        self.pub_cmd.publish(twist)

    # ==========================================================
    # Arrived Behavior
    # ==========================================================
    def do_arrived(self):
        """到达目标，微调朝向后退出"""
        self.stop_robot()
        
        # 微调：确保正对目标
        if self.target_position:
            _, left = self.target_position
            if abs(left) > 0.05:
                twist = Twist()
                twist.angular.z = 0.3 if left > 0 else -0.3
                self.pub_cmd.publish(twist)
                return
        
        # 完成！
        self.get_logger().info('='*50)
        self.get_logger().info('🎉 搜索完成！机器人已停在目标物体前')
        self.get_logger().info('='*50)
        
        self.task_complete = True
        self.stop_robot()
        
        # 延迟退出
        self.create_timer(1.0, self.shutdown, oneshot=True)
    
    def shutdown(self):
        self.get_logger().info('程序退出')
        raise SystemExit(0)

    # ==========================================================
    # Navigation
    # ==========================================================
    def send_nav_goal(self, pose):
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 not available')
            return
        
        goal = NavigateToPose.Goal()
        goal.pose = pose
        
        self.is_navigating = True
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
    
    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.is_navigating = False
            return
        self.current_goal = handle
        handle.get_result_async().add_done_callback(self.on_nav_result)
    
    def on_nav_result(self, future):
        self.is_navigating = False
        self.current_goal = None
        self.get_logger().info('Arrived at waypoint')
    
    def cancel_navigation(self):
        if self.current_goal:
            self.current_goal.cancel_goal_async()
            self.current_goal = None
        self.is_navigating = False

    # ==========================================================
    # Control
    # ==========================================================
    def stop_robot(self):
        self.pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = FindObject()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    
    node.stop_robot()
    node.cancel_navigation()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
