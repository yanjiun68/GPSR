#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from rcl_interfaces.msg import Log
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class GraspReadyMonitor(Node):
    def __init__(self):
        super().__init__('grasp_ready_monitor')

        # Latched ready state.
        self.ready_ = False

        # Readiness signature from grasp_node logs.
        self.target_logger_names_ = {'grasp_node', '/grasp_node'}
        self.target_message_core_ = 'Ready pose reached! System ready for detection.'
        self.ready_topic_name_ = '/grasp/ready'

        self.rosout_sub_ = self.create_subscription(
            Log,
            '/rosout',
            self.on_rosout,
            100,
        )
        ready_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_topic_sub_ = self.create_subscription(
            Bool,
            self.ready_topic_name_,
            self.on_ready_topic,
            ready_qos,
        )

        self.ready_srv_ = self.create_service(
            Trigger,
            '/grasp/is_ready',
            self.on_is_ready,
        )

        self.get_logger().info(
            'grasp_ready_monitor started. Waiting for /grasp/ready topic or grasp_node ready log.'
        )

    def on_ready_topic(self, msg: Bool):
        if self.ready_:
            return
        if not msg.data:
            return
        self.ready_ = True
        self.get_logger().info(
            'Detected /grasp/ready=True. /grasp/is_ready is now True.'
        )

    def on_rosout(self, msg: Log):
        if self.ready_:
            return

        if 'grasp_node' not in msg.name:
            return

        if msg.level != Log.INFO:
            return

        message = msg.msg.strip()

        # Be tolerant to log decoration (emoji/prefix) while still strict on the semantic text.
        if self.target_message_core_ not in message:
            return

        self.ready_ = True
        self.get_logger().info(
            'Detected target ready message from grasp_node. /grasp/is_ready is now True.'
        )

    def on_is_ready(self, request, response):
        del request

        response.success = self.ready_
        if self.ready_:
            response.message = self.target_message_core_
        else:
            response.message = 'Ready message not detected yet.'

        return response


def main():
    rclpy.init()
    node = GraspReadyMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
