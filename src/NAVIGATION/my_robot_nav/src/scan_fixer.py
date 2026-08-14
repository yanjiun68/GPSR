#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanFixer(Node):
    def __init__(self):
        super().__init__('scan_fixer')
        
        # Best Effort QoS for Lidar data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_cb, qos)
        self.pub = self.create_publisher(LaserScan, '/scan', qos)
        
        self.get_logger().info("Scan Fixer Running: /scan_raw -> /scan (with new timestamp)")

    def scan_cb(self, msg):
        # Overwrite timestamp with current system time minus a small buffer
        # to ensure TF data (published at 20Hz) is available for this time.
        # Otherwise, Scan Time > Last TF Time -> Future Extrapolation Error.
        now = self.get_clock().now()
        # Create a Duration of 0.1 seconds
        from rclpy.duration import Duration
        past_time = now - Duration(seconds=0.1)
        
        msg.header.stamp = past_time.to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
