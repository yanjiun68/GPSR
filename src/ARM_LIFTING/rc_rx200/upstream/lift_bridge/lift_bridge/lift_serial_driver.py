#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter

from std_msgs.msg import Float64, Bool, String
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger

import serial
import threading
import time
from typing import Optional

from rclpy.qos import qos_profile_sensor_data

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

class LiftSerialDriver(Node):
    def __init__(self):
        super().__init__('lift_serial_driver')
        self.cb_group = ReentrantCallbackGroup()

        # -------- Params --------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('serial_timeout_s', 0.1)
        self.declare_parameter('serial_exclusive', True)

        self.declare_parameter('joint_name', 'lift_joint')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('command_topic', '/lift/command')

        self.declare_parameter('pos_publish_hz', 50.0)
        self.declare_parameter('startup_grace_s', 2.0)
        self.declare_parameter('stale_joint_state_warn_s', 0.5)
        self.declare_parameter('warn_throttle_s', 2.0)
        
        # ✅ 关键：容差（你原代码用到了但没 declare，会埋雷）
        self.declare_parameter('tolerance_m', 0.01)  # 先 1cm 联调，后面再收紧

        # calibration
        self.declare_parameter('invert', False)
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('offset_m', 0.0)
        self.declare_parameter('min_m', 0.0)
        self.declare_parameter('max_m', 0.275)  # 275mm 行程

        # behavior
        self.declare_parameter('manual_command_preempts_action', True)
        self.declare_parameter('goto_prefix', 'GOTO')
        self.declare_parameter('stop_cmd', 'STOP')
        self.declare_parameter('send_stop_on_reached', True)   # ✅ 建议默认开，防止到位后继续爬行
        self.declare_parameter('follow_joint_trajectory_action', '/lift_controller/follow_joint_trajectory')

        self.declare_parameter('trajectory_mode', 'final')
        self.declare_parameter('cmd_rate_hz', 10.0)  # ✅ 提高重发频率至 10Hz，避免“电报式催动”导致卡顿
        self.declare_parameter('max_vel_mps', 0.15)
        self.declare_parameter('timeout_min_s', 3.0)
        self.declare_parameter('timeout_scale', 3.0)

        self.declare_parameter('reconnect_backoff_s', 1.0)

        # ✅ 工程零点
        self.declare_parameter('auto_zero_on_start', False)  # 你想要也可以开 true
        self.declare_parameter('auto_zero_wait_pos_s', 2.0)  # 等到收到 POS 后再自动置零

        # ✅ 硬件归零 (HOME)
        self.declare_parameter('home_cmd', 'HOME')
        self.declare_parameter('home_timeout_s', 30.0)  # 归零超时
        self.declare_parameter('step_size_m', 0.02)  # W/S 步进 20mm
        self.declare_parameter('auto_home_on_start', False)  # 启动时自动归零
        self.declare_parameter('auto_move_after_home_on_start', False)  # 启动归零后自动移动到目标位
        self.declare_parameter('auto_move_after_home_target_m', 0.0)

        # -------- Load params --------
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)
        self.serial_exclusive = bool(self.get_parameter('serial_exclusive').value)

        self.joint_name = str(self.get_parameter('joint_name').value)
        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)

        self.pos_publish_hz = float(self.get_parameter('pos_publish_hz').value)
        self.startup_grace_s = float(self.get_parameter('startup_grace_s').value)
        self.stale_warn_s = float(self.get_parameter('stale_joint_state_warn_s').value)
        self.warn_throttle_s = float(self.get_parameter('warn_throttle_s').value)
        self.tol = float(self.get_parameter('tolerance_m').value)

        self.invert = bool(self.get_parameter('invert').value)
        self.scale = float(self.get_parameter('scale').value)
        self.offset_m = float(self.get_parameter('offset_m').value)
        self.min_m = float(self.get_parameter('min_m').value)
        self.max_m = float(self.get_parameter('max_m').value)

        self.manual_preempt = bool(self.get_parameter('manual_command_preempts_action').value)
        self.goto_prefix = str(self.get_parameter('goto_prefix').value)
        self.stop_cmd = str(self.get_parameter('stop_cmd').value)
        self.send_stop_on_reached = bool(self.get_parameter('send_stop_on_reached').value)

        self.action_name = str(self.get_parameter('follow_joint_trajectory_action').value)
        self.trajectory_mode = str(self.get_parameter('trajectory_mode').value).lower()
        self.cmd_rate_hz = float(self.get_parameter('cmd_rate_hz').value)
        self.max_vel_mps = float(self.get_parameter('max_vel_mps').value)
        self.timeout_min_s = float(self.get_parameter('timeout_min_s').value)
        self.timeout_scale = float(self.get_parameter('timeout_scale').value)

        self.reconnect_backoff_s = float(self.get_parameter('reconnect_backoff_s').value)

        self.auto_zero_on_start = bool(self.get_parameter('auto_zero_on_start').value)
        self.auto_zero_wait_pos_s = float(self.get_parameter('auto_zero_wait_pos_s').value)

        self.home_cmd = str(self.get_parameter('home_cmd').value)
        self.home_timeout_s = float(self.get_parameter('home_timeout_s').value)
        self.step_size_m = float(self.get_parameter('step_size_m').value)
        self.auto_home_on_start = bool(self.get_parameter('auto_home_on_start').value)
        self.auto_move_after_home_on_start = bool(self.get_parameter('auto_move_after_home_on_start').value)
        self.auto_move_after_home_target_m = float(self.get_parameter('auto_move_after_home_target_m').value)

        # -------- State --------
        self.pos_m = 0.0
        self.pos_lock = threading.Lock()

        self.startup_mono = time.monotonic()
        self.seen_pos_once = False
        self.last_feedback_mono = time.monotonic()

        self.last_stale_warn_mono = 0.0
        self.last_serial_warn_mono = 0.0
        self.last_parse_warn_mono = 0.0

        self.active_goal_lock = threading.Lock()
        self.preempt_flag = False
        self.active_goal_handle = None

        self.ser: Optional[serial.Serial] = None
        self.ser_write_lock = threading.Lock()

        self._stop = False
        self._did_auto_zero = False
        self.is_homed = False  # 是否已完成硬件归零
        self.homing_in_progress = False  # 正在归零中
        self.is_estopped = False  # 急停状态
        self.is_moving = False  # 是否在运动中
        self.last_pos_m = 0.0  # 上一次位置（用于计算速度）
        self.last_pos_time = time.monotonic()  # 上一次位置时间
        self.velocity_mps = 0.0  # 当前速度 (m/s)

        # -------- Serial open --------
        self._open_serial_with_retry()

        # ✅ 使用标准 SensorDataQoS，确保与 robot_state_publisher 兼容
        self.pub = self.create_publisher(JointState, self.joint_states_topic, qos_profile_sensor_data)
        self.homed_pub = self.create_publisher(Bool, '/lift/is_homed', 10)  # ✅ 归零状态话题
        self.sub = self.create_subscription(Float64, self.command_topic, self.cmd_cb, 10, callback_group=self.cb_group)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group
        )

        # ✅ /lift/zero 服务：把"当前读到的位置"强制设为 0（软件置零）
        self.zero_srv = self.create_service(Trigger, '/lift/zero', self.zero_cb, callback_group=self.cb_group)

        # ✅ /lift/home 服务：发送 HOME 命令触发硬件归零（光电传感器）
        self.home_srv = self.create_service(Trigger, '/lift/home', self.home_cb, callback_group=self.cb_group)

        # ✅ /lift/step_up 服务：上升一步（默认 20mm）
        self.step_up_srv = self.create_service(Trigger, '/lift/step_up', self.step_up_cb, callback_group=self.cb_group)

        # ✅ /lift/step_down 服务：下降一步（默认 20mm）
        self.step_down_srv = self.create_service(Trigger, '/lift/step_down', self.step_down_cb, callback_group=self.cb_group)

        # ✅ /lift/estop 服务：急停
        self.estop_srv = self.create_service(Trigger, '/lift/estop', self.estop_cb, callback_group=self.cb_group)

        # ✅ /lift/reset_estop 服务：解除急停
        self.reset_estop_srv = self.create_service(Trigger, '/lift/reset_estop', self.reset_estop_cb, callback_group=self.cb_group)

        # ✅ /lift/status 话题：状态信息
        self.status_pub = self.create_publisher(String, '/lift/status', 10)

        # ✅ /diagnostics 话题：诊断信息
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # reader thread
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        # timer publish
        period = 1.0 / max(1.0, self.pos_publish_hz)
        self.timer = self.create_timer(period, self._publish_state, callback_group=self.cb_group)

        self.get_logger().info(
            f'LiftSerialDriver READY | joint={self.joint_name} '
            f'limits=[{self.min_m*1000:.0f}mm, {self.max_m*1000:.0f}mm] tol={self.tol*1000:.1f}mm '
            f'invert={self.invert}, scale={self.scale}, offset_m={self.offset_m} | '
            f'mode={self.trajectory_mode}, cmd_rate_hz={self.cmd_rate_hz} | '
            f'step_size={self.step_size_m*1000:.0f}mm | '
            f'serial={self.port}@{self.baud}'
        )
        self.get_logger().info(
            'Services: /lift/home, /lift/zero, /lift/step_up, /lift/step_down, '
            '/lift/estop, /lift/reset_estop'
        )
        self.get_logger().info(
            'Topics: /lift/is_homed, /lift/status, /diagnostics'
        )

        # ✅ 启动时自动归零
        self._auto_home_timer = None
        self._auto_home_done = False
        if self.auto_home_on_start:
            self.get_logger().info('🏠 Auto-homing enabled, will start in 2 seconds...')
            self._auto_home_timer = self.create_timer(2.0, self._auto_home_once, callback_group=self.cb_group)
        else:
            self.get_logger().warn('⚠️  Please call /lift/home before any motion!')

    # -------- Auto home on start --------
    def _maybe_run_startup_post_home_move(self, reason: str) -> None:
        """Optionally move to a configured target after startup homing completes."""
        if not self.auto_move_after_home_on_start:
            return

        target = clamp(self.auto_move_after_home_target_m, self.min_m, self.max_m)
        if abs(target - self.auto_move_after_home_target_m) > 1e-9:
            self.get_logger().warn(
                f'[AUTO HOME] Requested post-home target {self.auto_move_after_home_target_m:.4f}m '
                f'clamped to {target:.4f}m'
            )

        # Give the serial reader a brief moment to settle after homing completion.
        time.sleep(0.5)
        if self._send_goto_m(target):
            self.get_logger().info(
                f'[AUTO HOME] {reason} -> moving lift to startup target {target:.4f}m'
            )
        else:
            self.get_logger().error(
                f'[AUTO HOME] {reason} -> failed to send startup target {target:.4f}m'
            )

    def _auto_home_once(self) -> None:
        """启动时自动归零（只执行一次）"""
        # ✅ 防止重复执行
        if self._auto_home_done:
            return
        self._auto_home_done = True
        
        # 销毁定时器，确保只执行一次
        if self._auto_home_timer is not None:
            self._auto_home_timer.cancel()
            self._auto_home_timer = None

        if self.is_homed:
            self.get_logger().info('[AUTO HOME] Already homed, skipping.')
            self._maybe_run_startup_post_home_move('Already homed')
            return

        self.get_logger().info('[AUTO HOME] Starting hardware homing...')
        self.homing_in_progress = True
        self.is_homed = False

        # 发送 HOME 命令
        try:
            with self.ser_write_lock:
                if self.ser is not None:
                    self.ser.write((self.home_cmd + '\n').encode())
        except Exception as e:
            self.homing_in_progress = False
            self.get_logger().error(f'[AUTO HOME] Failed to send HOME: {e}')
            return

        # 在后台等待归零完成
        def wait_for_homing():
            start = time.monotonic()
            while (time.monotonic() - start) < self.home_timeout_s:
                if not self.homing_in_progress:
                    break
                time.sleep(0.1)

            if self.is_homed:
                self.offset_m = 0.0
                try:
                    self.set_parameters([Parameter('offset_m', Parameter.Type.DOUBLE, 0.0)])
                except Exception:
                    pass
                self.get_logger().info('[AUTO HOME] ✅ Homing complete! Ready to use.')
                self._maybe_run_startup_post_home_move('Homing complete')
            else:
                self.homing_in_progress = False
                self.get_logger().error(f'[AUTO HOME] ❌ Homing timeout after {self.home_timeout_s}s')

        threading.Thread(target=wait_for_homing, daemon=True).start()

    # -------- Zero service --------
    def zero_cb(self, request, response):
        # 让“当前 pos_m”变成 0：通过调整 offset_m 实现
        with self.pos_lock:
            cur = self.pos_m
        self.offset_m = self.offset_m - cur

        # 同步更新参数（方便你 ros2 param get 看到）
        try:
            self.set_parameters([Parameter('offset_m', Parameter.Type.DOUBLE, float(self.offset_m))])
        except Exception:
            pass

        self.get_logger().info(f'[ZERO] current={cur:.6f} -> new offset_m={self.offset_m:.6f} (reported pos becomes 0)')
        response.success = True
        response.message = f'Zeroed. new offset_m={self.offset_m:.6f}'
        return response

    # -------- HOME service (硬件归零) --------
    def home_cb(self, request, response):
        """发送 HOME 命令给 Arduino，触发光电传感器归零"""
        if self.homing_in_progress:
            response.success = False
            response.message = 'Homing already in progress'
            return response

        self.homing_in_progress = True
        self.is_homed = False
        self.get_logger().info('[HOME] Sending HOME command...')

        # 发送 HOME 命令
        try:
            with self.ser_write_lock:
                if self.ser is not None:
                    self.ser.write((self.home_cmd + '\n').encode())
        except Exception as e:
            self.homing_in_progress = False
            response.success = False
            response.message = f'Failed to send HOME: {e}'
            return response

        # 等待 Arduino 完成归零（监听 "HOMING COMPLETE" 消息）
        start = time.monotonic()
        while (time.monotonic() - start) < self.home_timeout_s:
            if not self.homing_in_progress:
                # 归零完成（由 reader_loop 设置）
                break
            time.sleep(0.1)

        if self.is_homed:
            # 归零成功后，重置 offset_m
            self.offset_m = 0.0
            try:
                self.set_parameters([Parameter('offset_m', Parameter.Type.DOUBLE, 0.0)])
            except Exception:
                pass

            response.success = True
            response.message = 'Homing complete. Position set to 0.'
            self.get_logger().info('[HOME] Homing complete!')
        else:
            self.homing_in_progress = False
            response.success = False
            response.message = f'Homing timeout after {self.home_timeout_s}s'
            self.get_logger().warn('[HOME] Homing timeout!')

        return response

    # -------- Step Up/Down services --------
    def step_up_cb(self, request, response):
        """上升一步（默认 20mm）"""
        if not self.is_homed:
            response.success = False
            response.message = 'Please HOME first!'
            return response

        with self.pos_lock:
            cur = self.pos_m
        target = clamp(cur + self.step_size_m, self.min_m, self.max_m)
        self._send_goto_m(target)

        response.success = True
        response.message = f'Step up: {cur*1000:.1f}mm -> {target*1000:.1f}mm'
        self.get_logger().info(f'[STEP UP] {cur*1000:.1f}mm -> {target*1000:.1f}mm')
        return response

    def step_down_cb(self, request, response):
        """下降一步（默认 20mm）"""
        if not self.is_homed:
            response.success = False
            response.message = 'Please HOME first!'
            return response

        with self.pos_lock:
            cur = self.pos_m
        target = clamp(cur - self.step_size_m, self.min_m, self.max_m)
        self._send_goto_m(target)

        response.success = True
        response.message = f'Step down: {cur*1000:.1f}mm -> {target*1000:.1f}mm'
        self.get_logger().info(f'[STEP DOWN] {cur*1000:.1f}mm -> {target*1000:.1f}mm')
        return response

    # -------- E-STOP service --------
    def estop_cb(self, request, response):
        """急停 - 立即停止并锁定"""
        self.is_estopped = True
        self._send_stop()

        # 取消当前的 action
        with self.active_goal_lock:
            self.preempt_flag = True

        self.get_logger().error('🛑 [E-STOP] Emergency stop activated!')
        response.success = True
        response.message = 'E-STOP activated. Call /lift/reset_estop to resume.'
        return response

    def reset_estop_cb(self, request, response):
        """解除急停"""
        if not self.is_estopped:
            response.success = True
            response.message = 'E-STOP was not active.'
            return response

        self.is_estopped = False
        self.get_logger().info('✅ [E-STOP] Emergency stop released.')
        response.success = True
        response.message = 'E-STOP released. System ready.'
        return response

    # -------- Serial open --------
    def _open_serial_with_retry(self) -> None:
        while rclpy.ok() and not self._stop:
            try:
                self._open_serial_once()
                return
            except Exception as e:
                now = time.monotonic()
                if (now - self.last_serial_warn_mono) >= self.warn_throttle_s:
                    self.get_logger().warn(f'Failed to open serial ({self.port}): {e}')
                    self.last_serial_warn_mono = now
                time.sleep(self.reconnect_backoff_s)

    def _open_serial_once(self) -> None:
        try:
            if self.ser is not None:
                with self.ser_write_lock:
                    self.ser.close()
        except Exception:
            pass

        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=self.serial_timeout_s,
            exclusive=self.serial_exclusive
        )

        time.sleep(1.5)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        self.get_logger().info(f'Opened serial: {self.port} @ {self.baud}')
        self.last_feedback_mono = time.monotonic()
        self.seen_pos_once = False

    # -------- Action callbacks --------
    def goal_cb(self, goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle) -> CancelResponse:
        self._send_stop()
        return CancelResponse.ACCEPT

    # -------- Reader loop --------
    def _reader_loop(self) -> None:
        while not self._stop:
            try:
                if self.ser is None:
                    self._open_serial_with_retry()
                    continue

                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode(errors='ignore').strip()
                if not line:
                    continue

                # ✅ 检测归零完成消息
                if 'HOMING COMPLETE' in line.upper():
                    self.get_logger().info(f'[READER] Detected homing complete: "{line}"')
                    with self.pos_lock:
                        self.pos_m = 0.0
                    self.is_homed = True
                    self.homing_in_progress = False
                    continue

                # ✅ 检测到位消息（调试用）
                if 'TARGET REACHED' in line.upper():
                    self.get_logger().debug(f'[READER] Target reached: "{line}"')
                    continue

                # ✅ 归零进行中消息
                if line.upper().startswith('HOMING'):
                    self.get_logger().debug(f'[READER] {line}')
                    continue

                if line.startswith('POS'):
                    mm = self._parse_pos_mm(line)
                    if mm is None:
                        continue

                    m = mm / 1000.0
                    if self.invert:
                        m = -m
                    m = m * self.scale + self.offset_m

                    with self.pos_lock:
                        self.pos_m = m

                    self.last_feedback_mono = time.monotonic()
                    self.seen_pos_once = True

                    # ✅ 可选：自动置零（等收到 POS 再置零）
                    if self.auto_zero_on_start and (not self._did_auto_zero):
                        if (time.monotonic() - self.startup_mono) >= self.auto_zero_wait_pos_s:
                            # 直接调用 zero 的逻辑
                            with self.pos_lock:
                                cur = self.pos_m
                            self.offset_m = self.offset_m - cur
                            try:
                                self.set_parameters([Parameter('offset_m', Parameter.Type.DOUBLE, float(self.offset_m))])
                            except Exception:
                                pass
                            self._did_auto_zero = True
                            self.get_logger().info(f'[AUTO ZERO] done. new offset_m={self.offset_m:.6f}')

            except serial.SerialException as e:
                now = time.monotonic()
                if (now - self.last_serial_warn_mono) >= self.warn_throttle_s:
                    self.get_logger().warn(f'serial exception: {e} -> reopening...')
                    self.last_serial_warn_mono = now
                self._open_serial_with_retry()
            except Exception as e:
                now = time.monotonic()
                if (now - self.last_serial_warn_mono) >= self.warn_throttle_s:
                    self.get_logger().warn(f'serial read error: {e}')
                    self.last_serial_warn_mono = now
                time.sleep(0.02)

    def _parse_pos_mm(self, line: str) -> Optional[float]:
        try:
            parts = line.replace(':', ' ').replace(',', ' ').split()
            for token in parts[1:]:
                try:
                    return float(token)
                except ValueError:
                    continue
            now = time.monotonic()
            if (now - self.last_parse_warn_mono) >= self.warn_throttle_s:
                self.get_logger().warn(f'Bad POS line (no number): "{line}"')
                self.last_parse_warn_mono = now
            return None
        except Exception as e:
            now = time.monotonic()
            if (now - self.last_parse_warn_mono) >= self.warn_throttle_s:
                self.get_logger().warn(f'POS parse error: {e} line="{line}"')
                self.last_parse_warn_mono = now
            return None

    # -------- Publish joint_states --------
    def _publish_state(self) -> None:
        now = time.monotonic()
        in_grace = (now - self.startup_mono) < self.startup_grace_s

        if (not in_grace) and self.seen_pos_once:
            age = now - self.last_feedback_mono
            if age > self.stale_warn_s:
                if (now - self.last_stale_warn_mono) >= self.warn_throttle_s:
                    self.get_logger().warn(f'POS feedback stale: {age:.2f}s')
                    self.last_stale_warn_mono = now

        # ✅ 计算速度
        with self.pos_lock:
            cur_pos = self.pos_m
        dt = now - self.last_pos_time
        if dt > 0.001:  # 防止除零
            self.velocity_mps = (cur_pos - self.last_pos_m) / dt
        self.last_pos_m = cur_pos
        self.last_pos_time = now

        # ✅ 判断是否停止（速度很小）
        if abs(self.velocity_mps) < 0.0005:  # < 0.5mm/s 认为停止
            self.is_moving = False

        # ✅ 发布 JointState（包含速度）
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [cur_pos]
        js.velocity = [self.velocity_mps]
        js.effort = []
        self.pub.publish(js)

        # ✅ 发布归零状态
        homed_msg = Bool()
        homed_msg.data = self.is_homed
        self.homed_pub.publish(homed_msg)

        # ✅ 发布状态信息
        status_str = f'pos:{cur_pos*1000:.1f}mm vel:{self.velocity_mps*1000:.1f}mm/s homed:{self.is_homed} moving:{self.is_moving} estop:{self.is_estopped}'
        status_msg = String()
        status_msg.data = status_str
        self.status_pub.publish(status_msg)

        # ✅ 发布诊断信息（每秒一次）
        if not hasattr(self, '_last_diag_time'):
            self._last_diag_time = 0.0
        if (now - self._last_diag_time) >= 1.0:
            self._last_diag_time = now
            self._publish_diagnostics(cur_pos)

    def _publish_diagnostics(self, cur_pos: float) -> None:
        """发布诊断信息到 /diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'Lift Axis'
        status.hardware_id = f'{self.port}'

        # 确定状态级别
        if self.is_estopped:
            status.level = DiagnosticStatus.ERROR
            status.message = 'E-STOP ACTIVE'
        elif not self.is_homed:
            status.level = DiagnosticStatus.WARN
            status.message = 'Not homed'
        elif not self.seen_pos_once:
            status.level = DiagnosticStatus.STALE
            status.message = 'No position feedback'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'

        # 添加键值对
        status.values = [
            KeyValue(key='Position (mm)', value=f'{cur_pos*1000:.2f}'),
            KeyValue(key='Velocity (mm/s)', value=f'{self.velocity_mps*1000:.2f}'),
            KeyValue(key='Homed', value=str(self.is_homed)),
            KeyValue(key='Moving', value=str(self.is_moving)),
            KeyValue(key='E-Stop', value=str(self.is_estopped)),
            KeyValue(key='Serial Port', value=self.port),
            KeyValue(key='Limits (mm)', value=f'{self.min_m*1000:.0f} - {self.max_m*1000:.0f}'),
        ]

        diag_array.status = [status]
        self.diag_pub.publish(diag_array)

    # -------- Manual command --------
    def cmd_cb(self, msg: Float64) -> None:
        if not self.is_homed:
            self.get_logger().warn('[CMD] Ignored: Please HOME first!')
            return
        self._send_goto_m(float(msg.data))
        if self.manual_preempt:
            with self.active_goal_lock:
                self.preempt_flag = True

    def _send_goto_m(self, target_m: float) -> bool:
        """发送 GOTO 命令，返回是否成功"""
        # 急停检查
        if self.is_estopped:
            self.get_logger().warn('[GOTO] Blocked: E-STOP active!')
            return False

        target_m = clamp(target_m, self.min_m, self.max_m)
        self.is_moving = True

        # inverse mapping (software -> raw)
        m = target_m
        if self.invert:
            m = -m
        if abs(self.scale) > 1e-9:
            m = (m - self.offset_m) / self.scale
        else:
            m = (m - self.offset_m)

        mm = m * 1000.0
        cmd = f'{self.goto_prefix} {mm:.3f}\n'

        try:
            with self.ser_write_lock:
                if self.ser is not None:
                    self.ser.write(cmd.encode())
        except serial.SerialException as e:
            now = time.monotonic()
            if (now - self.last_serial_warn_mono) >= self.warn_throttle_s:
                self.get_logger().warn(f'serial write exception: {e} -> reopening...')
                self.last_serial_warn_mono = now
            self._open_serial_with_retry()
        except Exception as e:
            now = time.monotonic()
            if (now - self.last_serial_warn_mono) >= self.warn_throttle_s:
                self.get_logger().warn(f'serial write error: {e}')
                self.last_serial_warn_mono = now
            return False

        return True

    def _send_stop(self) -> None:
        self.is_moving = False
        if not self.stop_cmd:
            return
        try:
            with self.ser_write_lock:
                if self.ser is not None:
                    self.ser.write((self.stop_cmd + '\n').encode())
        except Exception:
            pass

    # -------- Action execute --------
    def execute_cb(self, goal_handle) -> FollowJointTrajectory.Result:
        traj = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        # ✅ 安全检查：必须先归零
        if not self.is_homed:
            self.get_logger().error('[ACTION] Rejected: Please HOME first!')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Lift not homed. Call /lift/home service first.'
            return result

        # ✅ 安全检查：急停状态
        if self.is_estopped:
            self.get_logger().error('[ACTION] Rejected: E-STOP active!')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'E-STOP active. Call /lift/reset_estop first.'
            return result

        with self.active_goal_lock:
            self.active_goal_handle = goal_handle
            self.preempt_flag = False

        if (not traj.joint_names) or (self.joint_name not in traj.joint_names):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        if not traj.points:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        idx = traj.joint_names.index(self.joint_name)

        # final mode recommended
        return self._execute_final_target(goal_handle, result, traj.points[-1], idx)

    def _execute_final_target(self, goal_handle, result, last_pt, idx: int) -> FollowJointTrajectory.Result:
        if (not last_pt.positions) or (idx >= len(last_pt.positions)):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        target = clamp(float(last_pt.positions[idx]), self.min_m, self.max_m)

        with self.pos_lock:
            cur0 = self.pos_m
        dist = abs(target - cur0)
        
        self.get_logger().info(f'[ACTION] Executing: target={target:.4f}, cur={cur0:.4f}, dist={dist:.4f}, tol={self.tol:.4f}')

        vel = max(1e-6, abs(self.max_vel_mps))
        timeout_s = max(self.timeout_min_s, (dist / vel) * self.timeout_scale + 1.0)

        # ✅ Revert to 1Hz keepalive to prevent resetting firmware motion ramp
        resend_period = 1.0 
        start = time.monotonic()
        last_send = 0.0

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = [self.joint_name]
        
        # Send initial command immediately
        self._send_goto_m(target)
        last_send = time.monotonic()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._send_stop()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            with self.active_goal_lock:
                if self.preempt_flag:
                    self._send_stop()
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    return result

            now = time.monotonic()
            # Only resend if enough time passed (keepalive), don't spam
            if (now - last_send) >= resend_period:
                self._send_goto_m(target)
                last_send = now

            with self.pos_lock:
                cur = self.pos_m
            err = target - cur

            feedback.actual.positions = [cur]
            feedback.desired.positions = [target]
            feedback.error.positions = [err]
            goal_handle.publish_feedback(feedback)

            if abs(err) <= self.tol:
                if self.send_stop_on_reached:
                    self._send_stop()
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            if (now - start) > timeout_s:
                self.get_logger().warn(
                    f'Final target timeout: target={target:.4f} cur={cur:.4f} err={err:.4f} timeout={timeout_s:.1f}s'
                )
                self._send_stop()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                return result

            time.sleep(0.02)

        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        return result

    # -------- Cleanup --------
    def destroy_node(self) -> None:
        self._stop = True
        try:
            if self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            with self.ser_write_lock:
                if self.ser is not None:
                    self.ser.close()
        except Exception:
            pass

        super().destroy_node()

def main() -> None:
    rclpy.init()
    node = LiftSerialDriver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.remove_node(node)
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
