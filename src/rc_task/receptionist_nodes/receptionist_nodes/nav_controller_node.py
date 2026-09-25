#!/usr/bin/env python3

# Import math so yaw can be computed from pose quaternions when needed.
import math

# Import ROS 2 Python APIs.
import rclpy

# Import the node base class.
from rclpy.node import Node

# Import PoseStamped for cached startup pose data.
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# Import the navigation services exposed by this support node.
from robot_ai_interfaces.srv import FacePose, PrepareNavigationSession


# Provide safe navigation helper services for the receptionist state machine.
class NavControllerNode(Node):
    # Create subscribers and service endpoints.
    def __init__(self):
        # Give the node the executable name expected by launch files.
        super().__init__("nav_controller_node")

        # Declare whether the node should avoid real robot motion.
        self.declare_parameter("dry_run", True)

        # Declare the localization topic used to capture the startup pose.
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")

        # Cache runtime configuration.
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)

        # Keep the latest localization pose for prepare-session reporting.
        self.latest_pose = None

        # Subscribe to AMCL pose when it exists.
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self._amcl_pose_cb,
            10,
        )

        # Expose the prepare-session service expected by RobotClients.
        self.create_service(
            PrepareNavigationSession,
            "/nav/prepare_session",
            self._prepare_session_cb,
        )

        # Expose the face-pose service expected by presentation states.
        self.create_service(FacePose, "/nav/face_pose", self._face_pose_cb)

        # Log readiness with dry-run status.
        self.get_logger().info(
            f"Nav controller ready. dry_run={self.dry_run}, amcl_pose_topic={self.amcl_pose_topic}"
        )

    # Cache the latest AMCL pose as a simple PoseStamped.
    def _amcl_pose_cb(self, msg):
        # Create a stamped pose from the covariance-stamped AMCL message.
        pose = PoseStamped()

        # Preserve the header from AMCL.
        pose.header = msg.header

        # Preserve only the pose field required by downstream code.
        pose.pose = msg.pose.pose

        # Store the latest pose for session preparation.
        self.latest_pose = pose

    # Prepare navigation for one receptionist session.
    def _prepare_session_cb(self, request, response):
        # Silence the empty request field.
        del request

        # Report success in dry-run and real mode because this node only validates readiness.
        response.success = True

        # Report whether localization has produced a pose.
        response.start_pose_captured = self.latest_pose is not None or self.dry_run

        # Return a readable message for logs.
        response.message = (
            "Navigation session prepared."
            if response.start_pose_captured
            else "Navigation session prepared without an AMCL pose yet."
        )

        # Return the populated response object.
        return response

    # Face a target pose without exposing raw velocity control to the LLM.
    def _face_pose_cb(self, request, response):
        # Compute the target yaw only for logging and traceability.
        yaw = self._yaw_from_pose(request.target_pose)

        # Keep this service as a safe no-op in the dry-run-first implementation.
        response.success = True

        # Return a clear message that this service did not publish direct motor commands.
        response.message = (
            f"Face-pose request accepted as safe no-op at yaw {yaw:.3f} rad."
            if self.dry_run
            else f"Face-pose request acknowledged at yaw {yaw:.3f} rad."
        )

        # Return the populated response object.
        return response

    # Extract yaw from a target PoseStamped quaternion.
    def _yaw_from_pose(self, pose_stamped):
        # Read z and w because the receptionist poses are planar.
        z = float(pose_stamped.pose.orientation.z)
        w = float(pose_stamped.pose.orientation.w)

        # Convert the planar quaternion into yaw.
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


# Provide the standard ROS 2 executable entry point.
def main(args=None):
    # Initialize ROS 2.
    rclpy.init(args=args)

    # Construct the support node.
    node = NavControllerNode()

    # Serve requests until shutdown.
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# Run the entry point when executed directly.
if __name__ == "__main__":
    main()
