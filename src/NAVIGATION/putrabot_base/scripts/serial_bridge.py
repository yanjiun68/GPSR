#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import re
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class BaseSerialBridge(Node):
    """
    ROS2 Serial Bridge for Arduino streaming encoder ticks.

    - Subscribes:
        /wheel_targets  →  sends "m t1:t2:t3:t4\n"

    - Reads:
        Arduino streaming lines: "t e1:e2:e3:e4"

    - Publishes:
        /wheel_ticks (Int32MultiArray[4])
    """

    def __init__(self):
        super().__init__('base_serial_bridge')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # ---------------- Serial ----------------
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.01)
            time.sleep(2.0)  # allow Arduino auto-reset
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Opened serial port {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            raise

        # ---------------- ROS Interfaces ----------------
        self.targets_sub = self.create_subscription(
            Int32MultiArray,
            'wheel_targets',
            self.targets_cb,
            10
        )

        self.ticks_pub = self.create_publisher(
            Int32MultiArray,
            'wheel_ticks',
            10
        )

        # High-rate serial reader (1000 Hz)
        self.read_timer = self.create_timer(0.001, self.read_serial)

        self.get_logger().info("Listening for encoder stream from Arduino...")

    # --------------------------------------------------
    # SEND MOTOR TARGETS → Arduino
    # --------------------------------------------------
    def targets_cb(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn("wheel_targets wrong length")
            return

        t1, t2, t3, t4 = msg.data
        cmd = f"m {t1}:{t2}:{t3}:{t4}\n"

        try:
            self.ser.write(cmd.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    # --------------------------------------------------
    # READ STREAMING TICKS → publish /wheel_ticks
    # --------------------------------------------------
    def read_serial(self):
        try:
            raw = self.ser.readline()
            if not raw:
                return

            line = raw.decode('ascii', errors='ignore').strip()

            # Expect lines starting with "t "
            if not line.startswith("t"):
                return

            # Extract 4 integers
            nums = re.findall(r"-?\d+", line)
            if len(nums) != 4:
                return

            ticks = [int(n) for n in nums]

            msg = Int32MultiArray(data=ticks)
            self.ticks_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")


# ======================================================
# MAIN
# ======================================================
def main(args=None):
    rclpy.init(args=args)
    node = BaseSerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
