"""Section: Lifecycle capability marker for conditionally launched GPSR skills."""

# Section: Lifecycle server resources, callbacks, and executable entrypoint.

from __future__ import annotations

import rclpy
from lifecycle_msgs.msg import State
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class SkillAvailabilityResource(LifecycleNode):
    """Special function: Make a launch-time skill switch visible to lifecycle preparation."""

    def __init__(self) -> None:
        """Special function: Create an unconfigured capability marker with no hardware ownership."""
        super().__init__("skill_availability_resource")
        self.get_logger().info("GPSR skill availability resource is UNCONFIGURED.")

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Confirm that the conditionally launched capability exists."""
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Mark the selected capability as available for execution."""
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Special function: Release active capability selection after execution."""
        return super().on_deactivate(state)

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """Special function: Restore the capability marker to its launch-time baseline."""
        return TransitionCallbackReturn.SUCCESS


def main(args: list[str] | None = None) -> None:
    """Special function: Run one lifecycle capability marker process."""
    rclpy.init(args=args)
    node = SkillAvailabilityResource()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
