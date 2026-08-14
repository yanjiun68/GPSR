#!/usr/bin/env python3

# Import time so the pointing sequence can pause between commands.
import time

# Import ROS 2 Python APIs.
import rclpy

# Import the ROS 2 node base class.
from rclpy.node import Node

# Import the service exposed by this node.
from receptionist_interfaces.srv import PointToSeat

# Import the Interbotix single-joint command only when the package is available.
try:
    from interbotix_xs_msgs.msg import JointSingleCommand
except Exception:
    JointSingleCommand = None


# Provide a safe service wrapper for the RX200 waist pointing gesture.
class SeatPointingNode(Node):
    # Construct publisher and service resources.
    def __init__(self):
        # Give the node the executable name expected by launch files.
        super().__init__("seat_pointing_node")

        # Declare whether hardware commands should be skipped.
        self.declare_parameter("dry_run", True)

        # Declare the joint command topic for real robot mode.
        self.declare_parameter("joint_single_topic", "/rx200/commands/joint_single")

        # Declare the waist joint name used by Interbotix.
        self.declare_parameter("waist_joint_name", "waist")

        # Declare a pause between point and home commands.
        self.declare_parameter("command_pause_sec", 1.0)

        # Cache runtime parameters.
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.joint_single_topic = str(self.get_parameter("joint_single_topic").value)
        self.waist_joint_name = str(self.get_parameter("waist_joint_name").value)
        self.command_pause_sec = max(0.0, float(self.get_parameter("command_pause_sec").value))

        # Create the publisher only when the Interbotix message type is available.
        self.command_pub = (
            self.create_publisher(JointSingleCommand, self.joint_single_topic, 10)
            if JointSingleCommand is not None
            else None
        )

        # Expose the safe seat-pointing service expected by the state machine.
        self.create_service(PointToSeat, "/chair/point_to_seat", self._point_to_seat_cb)

        # Log readiness with dry-run status.
        self.get_logger().info(
            f"Seat pointing ready. dry_run={self.dry_run}, topic={self.joint_single_topic}"
        )

    # Publish one waist command when real hardware mode is enabled.
    def _publish_waist(self, position_rad: float) -> bool:
        # Succeed without hardware when dry-run is active.
        if self.dry_run:
            return True

        # Fail closed when the Interbotix message type is unavailable.
        if self.command_pub is None or JointSingleCommand is None:
            return False

        # Build the Interbotix single-joint command.
        command = JointSingleCommand()
        command.name = self.waist_joint_name
        command.cmd = float(position_rad)

        # Publish the command to the configured topic.
        self.command_pub.publish(command)

        # Report success after publish.
        return True

    # Handle one seat-pointing request.
    def _point_to_seat_cb(self, request, response):
        # Move only home when the caller requested a home-only sequence.
        if request.home_only:
            ok = self._publish_waist(0.0)
            response.success = ok
            response.message = "Waist returned home." if ok else "Failed to return waist home."
            return response

        # Point toward the selected chair.
        ok = self._publish_waist(float(request.waist_yaw_rad))

        # Return early on pointing failure.
        if not ok:
            response.success = False
            response.message = f"Failed to point to chair {request.chair_id}."
            return response

        # Pause before the optional home return.
        if request.return_home and self.command_pause_sec > 0.0:
            time.sleep(self.command_pause_sec)

        # Return home when requested.
        if request.return_home:
            ok = self._publish_waist(0.0)

        # Fill the response based on final success.
        response.success = ok
        response.message = (
            f"Pointed to chair {request.chair_id}."
            if ok
            else f"Failed during pointing sequence for chair {request.chair_id}."
        )
        return response


# Provide the standard ROS 2 executable entry point.
def main(args=None):
    # Initialize ROS 2.
    rclpy.init(args=args)

    # Construct the pointing service node.
    node = SeatPointingNode()

    # Serve requests until shutdown.
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# Run the entry point when executed directly.
if __name__ == "__main__":
    main()
