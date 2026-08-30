#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class RX200JointAngleReader(Node):

    def __init__(self):
        super().__init__('rx200_joint_angle_reader')

        self.topic = '/rx200/sdk_joint_states'

        # Joint order from your rx200_xs_sdk.yaml
        self.arm_joints = [
            'waist',
            'shoulder',
            'elbow',
            'wrist_angle',
            'wrist_rotate',
            'gripper'
        ]

        self.received = False

        self.subscription = self.create_subscription(
            JointState,
            self.topic,
            self.joint_state_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Waiting for joint states on {self.topic} ...'
        )

    def joint_state_callback(self, msg):

        # Create dictionary:
        # joint name -> position in radians
        joint_positions = {
            name: position
            for name, position in zip(msg.name, msg.position)
        }

        print()
        print('=' * 67)
        print('              INTERBOTIX RX200 CURRENT JOINT ANGLES')
        print('=' * 67)
        print(
            f'{"Joint":<18}'
            f'{"Radians":>14}'
            f'{"Degrees":>14}'
        )
        print('-' * 67)

        for joint in self.arm_joints:

            if joint in joint_positions:

                radians = joint_positions[joint]
                degrees = math.degrees(radians)

                print(
                    f'{joint:<18}'
                    f'{radians:>14.6f}'
                    f'{degrees:>14.2f}'
                )

            else:
                print(
                    f'{joint:<18}'
                    f'{"NOT FOUND":>28}'
                )

        print('=' * 67)

        self.received = True


def main(args=None):

    rclpy.init(args=args)

    node = RX200JointAngleReader()

    timeout_seconds = 10.0
    start_time = time.time()

    try:

        while rclpy.ok() and not node.received:

            rclpy.spin_once(
                node,
                timeout_sec=0.2
            )

            if time.time() - start_time > timeout_seconds:

                print()
                print('[ERROR] No joint-state message received.')
                print()
                print('Check whether xs_sdk is running:')
                print('  ros2 node list | grep xs_sdk')
                print()
                print('Check whether the topic exists:')
                print(
                    '  ros2 topic list | grep sdk_joint_states'
                )
                print()

                break

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
