#!/usr/bin/env python3

import math
import select
import sys
import termios
import tty
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String


@dataclass
class MotionStep:
    key: str
    label: str
    vx: float
    vy: float
    wz: float
    expected_signs: list[int]


class KinematicsTester(Node):
    def __init__(self):
        super().__init__('kinematics_tester')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('targets_topic', '/wheel_targets')
        self.declare_parameter('ticks_topic', '/wheel_ticks')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('trigger_topic', '/kinematics_test/trigger')

        self.declare_parameter('mode', 'auto')
        self.declare_parameter('sequence', 'all')
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('move_duration_sec', 2.0)
        self.declare_parameter('pause_duration_sec', 1.5)
        self.declare_parameter('pub_rate', 10.0)
        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('strafe_speed', 0.08)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('auto_shutdown', True)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.targets_topic = self.get_parameter('targets_topic').value
        self.ticks_topic = self.get_parameter('ticks_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.mode = str(self.get_parameter('mode').value).strip().lower()

        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.move_duration_sec = float(self.get_parameter('move_duration_sec').value)
        self.pause_duration_sec = float(self.get_parameter('pause_duration_sec').value)
        self.pub_rate = float(self.get_parameter('pub_rate').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.strafe_speed = float(self.get_parameter('strafe_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.auto_shutdown = bool(self.get_parameter('auto_shutdown').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Int32MultiArray, self.targets_topic, self.targets_cb, 10)
        self.create_subscription(Int32MultiArray, self.ticks_topic, self.ticks_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(String, self.trigger_topic, self.trigger_cb, 10)

        self.latest_targets = None
        self.latest_ticks = None
        self.latest_pose = None

        self.step_lookup = {}
        self.steps = self.build_steps(self.get_parameter('sequence').value)
        self.current_step_index = 0
        self.current_step = None
        self.phase = 'start_delay' if self.mode == 'auto' else 'idle'
        self.phase_started_at = self.now_sec()
        self.active_metrics = None
        self.finished = False
        self.stop_requested = False

        self.manual_keymap = {
            'w': 'forward',
            's': 'backward',
            'a': 'left',
            'd': 'right',
            'q': 'ccw',
            'e': 'cw',
            ' ': 'stop',
            'h': 'help',
            'x': 'quit',
        }
        self.stdin_fd = None
        self.stdin_termios = None
        self.stdin_ready = False

        if self.mode not in ('auto', 'manual'):
            raise ValueError(f'Unsupported mode: {self.mode}')

        if self.mode == 'auto':
            self.get_logger().info(
                'Kinematics tester ready. Sequence: '
                + ', '.join(step.label for step in self.steps)
            )
            self.get_logger().info(
                f'Will start in {self.start_delay_sec:.1f}s. '
                f'Forward={self.forward_speed:.3f} m/s, '
                f'Strafe={self.strafe_speed:.3f} m/s, '
                f'Rotate={self.angular_speed:.3f} rad/s'
            )
        else:
            self.setup_manual_input()
            self.get_logger().info(
                'Kinematics tester ready in MANUAL mode. '
                f'Each key runs one motion for {self.move_duration_sec:.1f}s.'
            )
            self.get_logger().info(
                f'Trigger topic: {self.trigger_topic} '
                '(String: forward/backward/left/right/ccw/cw/stop/help/quit)'
            )
            self.log_manual_help()

        self.timer = self.create_timer(1.0 / self.pub_rate, self.timer_cb)

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def build_steps(self, sequence_name):
        all_steps = {
            'forward': MotionStep('forward', 'Forward', self.forward_speed, 0.0, 0.0, [1, 1, 1, 1]),
            'backward': MotionStep('backward', 'Backward', -self.forward_speed, 0.0, 0.0, [-1, -1, -1, -1]),
            'left': MotionStep('left', 'Strafe Left', 0.0, self.strafe_speed, 0.0, [-1, 1, -1, 1]),
            'right': MotionStep('right', 'Strafe Right', 0.0, -self.strafe_speed, 0.0, [1, -1, 1, -1]),
            'ccw': MotionStep('ccw', 'Rotate CCW', 0.0, 0.0, self.angular_speed, [-1, 1, 1, -1]),
            'cw': MotionStep('cw', 'Rotate CW', 0.0, 0.0, -self.angular_speed, [1, -1, -1, 1]),
        }
        self.step_lookup = dict(all_steps)

        aliases = {
            'strafe_left': 'left',
            'strafe_right': 'right',
            'rotate_left': 'ccw',
            'rotate_right': 'cw',
        }

        if sequence_name == 'all':
            order = ['forward', 'backward', 'left', 'right', 'ccw', 'cw']
            return [all_steps[name] for name in order]

        requested = []
        for token in sequence_name.split(','):
            name = token.strip().lower()
            if not name:
                continue
            name = aliases.get(name, name)
            if name not in all_steps:
                raise ValueError(f'Unsupported sequence step: {token}')
            requested.append(all_steps[name])

        if not requested:
            raise ValueError('Sequence is empty')
        return requested

    def targets_cb(self, msg):
        self.latest_targets = list(msg.data)
        if self.phase == 'run' and self.active_metrics is not None and len(msg.data) == 4:
            for i, value in enumerate(msg.data):
                self.active_metrics['target_sum'][i] += float(value)
            self.active_metrics['target_count'] += 1

    def ticks_cb(self, msg):
        self.latest_ticks = list(msg.data)

    def odom_cb(self, msg):
        pose = msg.pose.pose
        self.latest_pose = (
            float(pose.position.x),
            float(pose.position.y),
            self.yaw_from_quaternion(pose.orientation),
        )

    def trigger_cb(self, msg):
        self.handle_manual_command(str(msg.data).strip().lower(), source='topic')

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def sign(value, eps):
        if value > eps:
            return 1
        if value < -eps:
            return -1
        return 0

    @staticmethod
    def sign_list_string(values):
        symbols = {1: '+', -1: '-', 0: '0'}
        return '[' + ' '.join(symbols[v] for v in values) + ']'

    @staticmethod
    def vector_string(values, digits=1):
        return '[' + ' '.join(f'{value:.{digits}f}' for value in values) + ']'

    def publish_cmd(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0, 0.0)

    def setup_manual_input(self):
        if not sys.stdin.isatty():
            self.get_logger().warn(
                'stdin is not a TTY. Keyboard input is unavailable in this environment. '
                f'Use topic triggers on {self.trigger_topic} instead.'
            )
            return

        try:
            self.stdin_fd = sys.stdin.fileno()
            self.stdin_termios = termios.tcgetattr(self.stdin_fd)
            tty.setcbreak(self.stdin_fd)
            self.stdin_ready = True
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to enable keyboard input: {exc}. '
                f'Use topic triggers on {self.trigger_topic} instead.'
            )
            self.stdin_fd = None
            self.stdin_termios = None
            self.stdin_ready = False

    def restore_manual_input(self):
        if self.stdin_fd is None or self.stdin_termios is None:
            return
        try:
            termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_termios)
        except Exception:
            pass
        finally:
            self.stdin_fd = None
            self.stdin_termios = None
            self.stdin_ready = False

    def log_manual_help(self):
        self.get_logger().info(
            'Keys: w=forward s=backward a=left d=right q=ccw e=cw '
            'space=stop h=help x=quit'
        )

    def poll_keyboard(self):
        if not self.stdin_ready:
            return

        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                return

            key = sys.stdin.read(1)
            if key == '\x03':
                self.request_quit('keyboard')
                return

            command = self.manual_keymap.get(key.lower())
            if command is None:
                continue

            self.handle_manual_command(command, source=f'key {repr(key)}')

    def start_step(self, step, source='auto'):
        self.phase = 'run'
        self.phase_started_at = self.now_sec()
        self.stop_requested = False
        self.current_step = step
        self.active_metrics = {
            'start_ticks': list(self.latest_ticks) if self.latest_ticks is not None else None,
            'start_pose': tuple(self.latest_pose) if self.latest_pose is not None else None,
            'target_sum': [0.0, 0.0, 0.0, 0.0],
            'target_count': 0,
        }

        self.get_logger().info(
            f'[{step.label}] command vx={step.vx:.3f} vy={step.vy:.3f} wz={step.wz:.3f} '
            f'| expected target signs {self.sign_list_string(step.expected_signs)} '
            f'| source={source}'
        )

    def finish_step(self, step):
        metrics = self.active_metrics
        end_ticks = list(self.latest_ticks) if self.latest_ticks is not None else None
        end_pose = tuple(self.latest_pose) if self.latest_pose is not None else None

        if metrics['target_count'] > 0:
            avg_targets = [value / metrics['target_count'] for value in metrics['target_sum']]
            avg_target_signs = [self.sign(value, 1.0) for value in avg_targets]
        else:
            avg_targets = None
            avg_target_signs = None

        tick_delta = None
        tick_signs = None
        if metrics['start_ticks'] is not None and end_ticks is not None:
            tick_delta = [end_ticks[i] - metrics['start_ticks'][i] for i in range(4)]
            tick_signs = [self.sign(value, 2.0) for value in tick_delta]

        pose_delta = None
        if metrics['start_pose'] is not None and end_pose is not None:
            pose_delta = (
                end_pose[0] - metrics['start_pose'][0],
                end_pose[1] - metrics['start_pose'][1],
                self.wrap_angle(end_pose[2] - metrics['start_pose'][2]),
            )

        self.log_step_summary(step, avg_targets, avg_target_signs, tick_delta, tick_signs, pose_delta)
        self.active_metrics = None
        self.current_step = None

    def log_step_summary(self, step, avg_targets, avg_target_signs, tick_delta, tick_signs, pose_delta):
        self.get_logger().info(f'[{step.label}] summary')

        if avg_targets is None:
            self.get_logger().warn('  No /wheel_targets feedback seen during this step.')
        else:
            self.get_logger().info(
                f'  avg wheel_targets = {self.vector_string(avg_targets)} '
                f'| signs {self.sign_list_string(avg_target_signs)}'
            )
            if avg_target_signs != step.expected_signs:
                self.get_logger().warn(
                    '  wheel_targets sign pattern does not match the expected X-drive formula. '
                    'Check cmd_vel_to_targets or invert_m1..m4.'
                )

        if tick_delta is None:
            self.get_logger().warn('  No usable /wheel_ticks delta captured.')
        else:
            self.get_logger().info(
                f'  wheel_ticks delta = {tick_delta} '
                f'| signs {self.sign_list_string(tick_signs)}'
            )
            if avg_target_signs is not None:
                mismatched = [
                    index + 1
                    for index in range(4)
                    if avg_target_signs[index] != 0 and tick_signs[index] != avg_target_signs[index]
                ]
                if mismatched:
                    self.get_logger().warn(
                        '  Encoder direction disagrees with target on wheel(s): '
                        + ', '.join(f'M{index}' for index in mismatched)
                        + '. Check motor polarity, encoder polarity, or wheel wiring.'
                    )

        if pose_delta is None:
            self.get_logger().warn(f'  No usable {self.odom_topic} delta captured.')
            return

        dx, dy, dyaw = pose_delta
        self.get_logger().info(
            f'  odom delta: dx={dx:.3f} m dy={dy:.3f} m dyaw={dyaw:.3f} rad'
        )
        self.get_logger().info('  interpretation: ' + self.interpret_pose_delta(step, dx, dy, dyaw))

    def interpret_pose_delta(self, step, dx, dy, dyaw):
        min_translation = max(0.02, 0.25 * self.move_duration_sec * max(abs(step.vx), abs(step.vy), 0.0))
        min_rotation = max(0.10, 0.25 * self.move_duration_sec * abs(step.wz))

        if step.key in ('forward', 'backward'):
            expected_sign = 1 if step.key == 'forward' else -1
            if abs(dx) < min_translation:
                return 'forward/backward response is too small.'
            if self.sign(dx, 0.01) != expected_sign:
                return 'robot moved in the opposite x direction.'
            if abs(dy) > abs(dx) * 0.6 and abs(dy) > 0.02:
                return 'forward command produced strong sideways drift.'
            if abs(dyaw) > 0.25:
                return 'forward command produced too much unwanted rotation.'
            return 'x-axis response looks roughly correct.'

        if step.key in ('left', 'right'):
            expected_sign = 1 if step.key == 'left' else -1
            if abs(dy) < min_translation:
                return 'left/right response is too small.'
            if self.sign(dy, 0.01) != expected_sign:
                return 'robot moved in the opposite y direction.'
            if abs(dx) > abs(dy) * 0.6 and abs(dx) > 0.02:
                return 'strafe command produced strong x drift.'
            if abs(dyaw) > 0.25:
                return 'strafe command produced too much unwanted rotation.'
            return 'y-axis response looks roughly correct.'

        expected_sign = 1 if step.key == 'ccw' else -1
        if abs(dyaw) < min_rotation:
            return 'rotation response is too small.'
        if self.sign(dyaw, 0.05) != expected_sign:
            return 'robot rotated in the opposite direction.'
        if math.hypot(dx, dy) > 0.08:
            return 'rotation command produced too much translation drift.'
        return 'yaw response looks roughly correct.'

    def request_stop(self, source):
        if self.phase == 'run' and self.current_step is not None:
            self.stop_requested = True
            self.get_logger().info(f'Stop requested from {source}.')
        else:
            self.publish_stop()

    def request_quit(self, source):
        self.get_logger().info(f'Quit requested from {source}.')
        if self.phase == 'run' and self.current_step is not None:
            self.stop_requested = True
        self.finish_test()

    def handle_manual_command(self, command, source):
        if self.mode != 'manual':
            return

        if command == 'help':
            self.log_manual_help()
            return

        if command in ('quit', 'exit'):
            self.request_quit(source)
            return

        if command == 'stop':
            self.request_stop(source)
            return

        step = self.step_lookup.get(command)
        if step is None:
            self.get_logger().warn(f'Unknown manual command: {command}')
            return

        if self.phase == 'run' and self.current_step is not None:
            self.get_logger().warn(
                f'Already running {self.current_step.label}. '
                'Press space to stop first, or wait for it to finish.'
            )
            return

        self.start_step(step, source=source)

    def finish_test(self):
        if self.finished:
            return
        self.finished = True
        self.publish_stop()
        self.get_logger().info('Kinematics test sequence finished.')
        if self.auto_shutdown:
            self.get_logger().info('Auto shutdown enabled, stopping node.')
            rclpy.shutdown()

    def timer_cb_manual(self):
        self.poll_keyboard()

        if self.finished:
            self.publish_stop()
            return

        if self.phase == 'run' and self.current_step is not None:
            elapsed = self.now_sec() - self.phase_started_at
            self.publish_cmd(self.current_step.vx, self.current_step.vy, self.current_step.wz)
            if self.stop_requested or elapsed >= self.move_duration_sec:
                self.finish_step(self.current_step)
                self.phase = 'idle'
                self.phase_started_at = self.now_sec()
                self.stop_requested = False
                self.publish_stop()
            return

        self.publish_stop()

    def timer_cb(self):
        if self.mode == 'manual':
            self.timer_cb_manual()
            return

        if self.finished:
            self.publish_stop()
            return

        if self.current_step_index >= len(self.steps):
            self.finish_test()
            return

        step = self.steps[self.current_step_index]
        elapsed = self.now_sec() - self.phase_started_at

        if self.phase == 'start_delay':
            self.publish_stop()
            if elapsed >= self.start_delay_sec:
                self.start_step(step)
            return

        if self.phase == 'run':
            self.publish_cmd(step.vx, step.vy, step.wz)
            if elapsed >= self.move_duration_sec:
                self.finish_step(step)
                self.phase = 'pause'
                self.phase_started_at = self.now_sec()
                self.publish_stop()
            return

        if self.phase == 'pause':
            self.publish_stop()
            if elapsed >= self.pause_duration_sec:
                self.current_step_index += 1
                if self.current_step_index >= len(self.steps):
                    self.finish_test()
                else:
                    self.start_step(self.steps[self.current_step_index])

    def destroy_node(self):
        self.restore_manual_input()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KinematicsTester()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
