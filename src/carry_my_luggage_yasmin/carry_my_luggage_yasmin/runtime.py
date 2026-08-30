"""Section: Shared ROS process lifecycle for both YASMIN executables."""

from __future__ import annotations

import threading
from collections.abc import Callable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


def run_state_machine(
    node_name: str,
    viewer_name: str,
    builder: Callable[[Node], StateMachine],
) -> str:
    """Spin ROS callbacks while executing a blocking YASMIN graph."""
    # Section: A multithreaded executor lets service/action callbacks wake blocking states.
    rclpy.init()
    node = Node(node_name)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Section: Publish the complete graph before task execution begins.
    machine = builder(node)
    viewer = YasminViewerPub(machine, viewer_name, node=node)
    try:
        outcome = machine()
        node.get_logger().info("State machine finished with outcome: %s", outcome)
        return outcome
    finally:
        # Section: Stop visualization and ROS resources deterministically on every outcome.
        viewer.shutdown()
        executor.remove_node(node)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
