#!/usr/bin/env python3
from copy import deepcopy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy


class ImuDecimator(Node):
    def __init__(self):
        super().__init__('imu_decimator')

        # Keep the original frame_id so robot_localization can transform it via TF.
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.pub = self.create_publisher(Imu, '/camera/camera/imu_decimated', pub_qos)

        # Subscribe to BEST_EFFORT RealSense IMU
        self.sub = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_cb,
            qos_profile_sensor_data
        )

        self.declare_parameter('decimation', 10)  # 200 Hz / 10 = 20 Hz
        self.declare_parameter('use_host_timestamp', False)
        self.declare_parameter('angular_velocity_covariance_diag', [0.05, 0.05, 0.08])
        self.declare_parameter('linear_acceleration_covariance_diag', [0.2, 0.2, 0.2])

        self.decimation = int(self.get_parameter('decimation').value)
        self.use_host_timestamp = bool(self.get_parameter('use_host_timestamp').value)
        self.angular_velocity_covariance_diag = list(
            self.get_parameter('angular_velocity_covariance_diag').value
        )
        self.linear_acceleration_covariance_diag = list(
            self.get_parameter('linear_acceleration_covariance_diag').value
        )
        self.count = 0

        self.get_logger().info(
            f'IMU Decimator started. Decimating 1 in {self.decimation} messages. '
            f'use_host_timestamp={self.use_host_timestamp}'
        )

    def imu_cb(self, msg):
        self.count += 1
        if self.count >= self.decimation:
            out_msg = deepcopy(msg)
            if self.use_host_timestamp:
                out_msg.header.stamp = self.get_clock().now().to_msg()

            # Use conservative covariances so wheel odom remains the primary source.
            out_msg.angular_velocity_covariance = [
                self.angular_velocity_covariance_diag[0], 0.0, 0.0,
                0.0, self.angular_velocity_covariance_diag[1], 0.0,
                0.0, 0.0, self.angular_velocity_covariance_diag[2],
            ]
            out_msg.linear_acceleration_covariance = [
                self.linear_acceleration_covariance_diag[0], 0.0, 0.0,
                0.0, self.linear_acceleration_covariance_diag[1], 0.0,
                0.0, 0.0, self.linear_acceleration_covariance_diag[2],
            ]

            self.pub.publish(out_msg)
            self.count = 0


def main():
    rclpy.init()
    node = ImuDecimator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
