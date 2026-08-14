#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import re
import serial
import threading
import time

class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')
        # Params
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('reconnect_interval_sec', 1.0)
        self.declare_parameter('targets_timeout_sec', 0.35)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.reconnect_interval_sec = float(self.get_parameter('reconnect_interval_sec').value)
        self.targets_timeout_sec = float(self.get_parameter('targets_timeout_sec').value)

        self.ser = None
        self.serial_lock = threading.Lock()
        self.latest_targets = [0, 0, 0, 0]
        self.last_sent_targets = None
        self.last_targets_sent_at = 0.0
        self.last_connect_attempt_at = 0.0
        self.last_target_update_at = time.monotonic()
        self.serial_error_count = 0
        self.connect_serial()

        self.sub = self.create_subscription(Int32MultiArray, 'wheel_targets', self.targets_cb, 10)
        self.pub = self.create_publisher(Int32MultiArray, 'wheel_ticks', 10)
        
        self.timer = self.create_timer(0.05, self.update) # 20Hz

    def connect_serial(self):
        now_mono = time.monotonic()
        if (now_mono - self.last_connect_attempt_at) < self.reconnect_interval_sec:
            return

        self.last_connect_attempt_at = now_mono
        self.get_logger().info(f'Attempting to connect to {self.port} at {self.baud}...')
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=0.2)
            time.sleep(2.0) # Wait for Arduino reset
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.last_sent_targets = None
            self.last_targets_sent_at = 0.0
            self.serial_error_count = 0
            self.get_logger().info(f'Successfully connected to {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to {self.port}: {e}')
            self.ser = None

    def parse_tick_line(self, line):
        parts = re.findall(r'-?\d+', line)
        if len(parts) < 4:
            return None

        if len(parts) > 4:
            # Recover from concatenated/partial serial frames by taking the most recent 4 integers.
            parts = parts[-4:]

        ticks = [int(p) for p in parts]
        if any(abs(tick) > 2_147_483_647 for tick in ticks):
            return None
        return ticks

    @staticmethod
    def is_noise_line(line):
        if not line:
            return True
        if line.upper() == 'OK':
            return True
        if line.startswith('t') or line.startswith('m'):
            return True
        return False

    def read_encoder_reply(self, deadline_s):
        while time.monotonic() < deadline_s:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if self.is_noise_line(line):
                continue

            ticks = self.parse_tick_line(line)
            if ticks is not None:
                return ticks, line

            self.get_logger().warn(
                f'Unexpected format: "{line}"',
                throttle_duration_sec=1.0,
            )

        return None, None

    def mark_serial_error(self, context, error):
        self.serial_error_count += 1
        self.get_logger().error(f'{context}: {error}')
        if self.serial_error_count >= 3:
            self.get_logger().warn('Too many serial errors, forcing reconnect')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.serial_error_count = 0

    def targets_cb(self, msg):
        if len(msg.data) != 4:
            return

        # Only store the latest command here. The timer thread owns the serial
        # port so motor commands and encoder queries cannot interleave.
        self.latest_targets = list(msg.data)
        self.last_target_update_at = time.monotonic()

    def get_active_targets(self):
        if (time.monotonic() - self.last_target_update_at) > self.targets_timeout_sec:
            if self.latest_targets != [0, 0, 0, 0]:
                self.get_logger().warn(
                    'Wheel target timeout, forcing STOP [0,0,0,0]',
                    throttle_duration_sec=1.0,
                )
                self.latest_targets = [0, 0, 0, 0]
            return [0, 0, 0, 0]
        return list(self.latest_targets)

    def send_targets_locked(self, targets, now_mono, force=False):
        if len(targets) != 4:
            return
        if not force and self.last_sent_targets == targets and (now_mono - self.last_targets_sent_at) < 0.2:
            return

        cmd = f'm {targets[0]}:{targets[1]}:{targets[2]}:{targets[3]}\r'
        self.ser.write(cmd.encode())
        self.last_sent_targets = list(targets)
        self.last_targets_sent_at = now_mono

    def send_stop_locked(self, repeats=3):
        if not self.ser:
            return
        stop_targets = [0, 0, 0, 0]
        for _ in range(repeats):
            self.send_targets_locked(stop_targets, time.monotonic(), force=True)
            time.sleep(0.02)

    def update(self):
        with self.serial_lock:
            if not self.ser:
                self.get_logger().warn('Serial not connected, retrying...', throttle_duration_sec=5.0)
                self.connect_serial()
                return

            try:
                # Drop stale ACK chatter so the next response belongs to this cycle.
                self.ser.reset_input_buffer()

                targets = self.get_active_targets()
                now_mono = time.monotonic()
                self.send_targets_locked(targets, now_mono)

                self.ser.write(b'e\r')
                ticks, _ = self.read_encoder_reply(time.monotonic() + 0.08)
                if ticks is not None:
                    self.serial_error_count = 0
                    self.pub.publish(Int32MultiArray(data=ticks))
                else:
                    self.get_logger().warn('No encoder reply received this cycle', throttle_duration_sec=2.0)

            except serial.SerialException as e:
                self.mark_serial_error('Serial read error', e)
            except Exception as e:
                self.mark_serial_error('Serial update failure', e)

    def destroy_node(self):
        with self.serial_lock:
            try:
                self.send_stop_locked()
            except Exception as e:
                self.get_logger().warn(f'Failed to send stop during shutdown: {e}')
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
