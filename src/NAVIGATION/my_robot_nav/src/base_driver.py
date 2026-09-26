#!/usr/bin/env python3
# Section: Standard-library imports provide a port-specific exclusive driver lease.
import fcntl
import hashlib
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import re
import serial
import threading
import time
from pathlib import Path
from typing import TextIO


def base_driver_lock_path(port: str) -> Path:
    """Special function: Derive one stable lock file for a physical Arduino serial port."""
    # Section: Resolve symlinks so equivalent /dev paths cannot start separate drivers.
    canonical_port = os.path.realpath(str(port))
    # Section: Hash only the canonical device path to keep the temporary filename filesystem-safe.
    port_digest = hashlib.sha256(canonical_port.encode('utf-8')).hexdigest()[:16]
    return Path('/tmp') / f'putrabot_base_driver_{port_digest}.lock'


def acquire_base_driver_lock(port: str) -> tuple[TextIO | None, str]:
    """Special function: Claim the exclusive serial-port ownership used by one base-driver process."""
    # Section: Open the port-specific lock whose advisory kernel lock clears if this process exits.
    lock_handle = base_driver_lock_path(port).open('a+', encoding='utf-8')
    try:
        # Section: Reject a second driver immediately before either process opens the Arduino serial port.
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        # Section: Read the current owner diagnostic before closing the unsuccessful lock handle.
        lock_handle.seek(0)
        owner_pid = lock_handle.read().strip() or 'unknown'
        lock_handle.close()
        return None, owner_pid
    # Section: Record the current owner PID only after exclusive serial ownership is secured.
    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(str(os.getpid()))
    lock_handle.flush()
    return lock_handle, str(os.getpid())


def release_base_driver_lock(lock_handle: TextIO | None) -> None:
    """Special function: Release the exclusive serial-port lease during orderly driver shutdown."""
    # Section: A missing lock means this process never became the base-driver owner.
    if lock_handle is None:
        return
    try:
        # Section: Unlock before close so a replacement launch can immediately claim the Arduino port.
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
    finally:
        # Section: Closing guarantees cleanup if explicit unlock cannot report success.
        lock_handle.close()


class BaseDriver(Node):
    """Special function: Own one Arduino base serial port and exchange wheel commands and ticks."""

    def __init__(self):
        """Special function: Initialize one guarded base driver before creating ROS I/O."""
        super().__init__('base_driver')
        # Section: Declare and read all serial and command freshness parameters.
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('reconnect_interval_sec', 1.0)
        self.declare_parameter('targets_timeout_sec', 0.35)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.reconnect_interval_sec = float(self.get_parameter('reconnect_interval_sec').value)
        self.targets_timeout_sec = float(self.get_parameter('targets_timeout_sec').value)

        # Section: Reject a competing driver before either process opens the same physical serial device.
        self._base_driver_lock, owner_pid = acquire_base_driver_lock(self.port)
        if self._base_driver_lock is None:
            self.get_logger().error(
                f'Base serial port {self.port} is already owned by PID {owner_pid}; '
                'stop the duplicate driver launch.'
            )
            raise RuntimeError('duplicate base driver owner')

        # Section: Initialize serial state only after this process owns the physical Arduino port.
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
        """Special function: Open the exclusively owned Arduino serial port with retry throttling."""
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
        """Special function: Parse one four-wheel encoder reply while rejecting malformed values."""
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
        """Special function: Recognize serial acknowledgements that are not encoder replies."""
        if not line:
            return True
        if line.upper() == 'OK':
            return True
        if line.startswith('t') or line.startswith('m'):
            return True
        return False

    def read_encoder_reply(self, deadline_s):
        """Special function: Read the next valid encoder reply before the bounded deadline."""
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
        """Special function: Count serial faults and reset the connection after repeated failures."""
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
        """Special function: Cache the newest valid motor target for the serial timer."""
        if len(msg.data) != 4:
            return

        # Only store the latest command here. The timer thread owns the serial
        # port so motor commands and encoder queries cannot interleave.
        self.latest_targets = list(msg.data)
        self.last_target_update_at = time.monotonic()

    def get_active_targets(self):
        """Special function: Convert stale wheel commands into a safe zero-velocity target."""
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
        """Special function: Send one rate-limited wheel target while the serial mutex is held."""
        if len(targets) != 4:
            return
        if not force and self.last_sent_targets == targets and (now_mono - self.last_targets_sent_at) < 0.2:
            return

        cmd = f'm {targets[0]}:{targets[1]}:{targets[2]}:{targets[3]}\r'
        self.ser.write(cmd.encode())
        self.last_sent_targets = list(targets)
        self.last_targets_sent_at = now_mono

    def send_stop_locked(self, repeats=3):
        """Special function: Send repeated zero targets while the serial mutex is held."""
        if not self.ser:
            return
        stop_targets = [0, 0, 0, 0]
        for _ in range(repeats):
            self.send_targets_locked(stop_targets, time.monotonic(), force=True)
            time.sleep(0.02)

    def update(self):
        """Special function: Run one exclusive serial command-and-encoder transaction."""
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
        """Special function: Stop the base, close serial, and release the port ownership lease."""
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
        # Section: Release the physical-port lock only after this node has stopped serial traffic.
        release_base_driver_lock(getattr(self, '_base_driver_lock', None))
        self._base_driver_lock = None
        return super().destroy_node()

def main(args=None):
    """Special function: Run one guarded base driver and exit cleanly on duplicate ownership."""
    # Section: Initialize ROS before constructing the port-owning driver node.
    rclpy.init(args=args)
    node = None
    try:
        # Section: Construct the driver only after its port lease is available.
        node = BaseDriver()
        rclpy.spin(node)
    except RuntimeError as exc:
        # Section: A rejected duplicate exits without opening or commanding the Arduino port.
        rclpy.logging.get_logger('base_driver').error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        # Section: Destroy resources only when this process successfully became the driver owner.
        if node is not None:
            node.destroy_node()
        # Section: Close the ROS context after normal or rejected startup.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
