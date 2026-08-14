#!/usr/bin/env python3

# Import math so chair occupancy can use planar distances.
import math

# Import Path for robust default layout-file discovery.
from pathlib import Path

# Import YAML parsing for the receptionist layout file.
import yaml

# Import ROS 2 Python APIs.
import rclpy

# Import package-share lookup for default layout configuration.
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

# Import the ROS 2 node base class.
from rclpy.node import Node

# Import PoseStamped for selected chair pose output.
from geometry_msgs.msg import PoseStamped

# Import camera detection messages used for chair occupancy.
from putrabot_interfaces.msg import CamDetections

# Import the empty-chair service exposed by this node.
from receptionist_interfaces.srv import FindEmptyChair


# Select an empty chair from the configured receptionist layout.
class ChairManagerNode(Node):
    # Construct subscriptions, services, and layout state.
    def __init__(self):
        # Give the node the executable name expected by launch files.
        super().__init__("chair_manager_node")

        # Declare the layout file path with package-share fallback.
        self.declare_parameter("layout_file", self._default_layout_file())

        # Declare the detections topic used to estimate chair occupancy.
        self.declare_parameter("detections_topic", "/cam_detections")

        # Declare how close a person detection must be to occupy a chair.
        self.declare_parameter("occupancy_radius_m", 0.6)

        # Cache parameters.
        self.layout_file = str(self.get_parameter("layout_file").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.occupancy_radius_m = float(self.get_parameter("occupancy_radius_m").value)

        # Load configured chair slots.
        self.layout_data = self._load_layout_file(self.layout_file)
        self.chair_slots = self.layout_data.get("chairs", {}).get("slots", [])

        # Keep the latest camera detections.
        self.latest_detections = []

        # Subscribe to semantic camera detections.
        self.create_subscription(CamDetections, self.detections_topic, self._detections_cb, 10)

        # Expose the empty-chair service expected by the state machine.
        self.create_service(FindEmptyChair, "/chair/find_empty_chair", self._find_empty_chair_cb)

        # Log readiness with the number of configured chair slots.
        self.get_logger().info(
            f"Chair manager ready with {len(self.chair_slots)} slots from {self.layout_file}."
        )

    # Return the package default layout file path.
    def _default_layout_file(self):
        # Resolve the installed receptionist_nodes share directory when available.
        try:
            share_dir = Path(get_package_share_directory("receptionist_nodes"))
            return str(share_dir / "config" / "receptionist_layout.yaml")
        except PackageNotFoundError:
            return ""

    # Load one YAML layout file.
    def _load_layout_file(self, layout_file):
        # Use a built-in fallback when the file is missing.
        if not layout_file or not Path(layout_file).exists():
            return {
                "chairs": {
                    "slots": [
                        {
                            "id": "default_chair",
                            "frame_id": "map",
                            "x": 0.0,
                            "y": 0.0,
                            "yaw": 0.0,
                            "waist_yaw_rad": 0.0,
                        }
                    ]
                }
            }

        # Parse the YAML file into a dictionary.
        with open(layout_file, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {}

    # Cache the latest detection list.
    def _detections_cb(self, msg):
        # Store a plain list so service callbacks can iterate safely.
        self.latest_detections = list(msg.detections)

    # Return true when a chair slot is occupied by a nearby detection.
    def _chair_is_occupied(self, chair_slot):
        # Compare each latest detection against the chair center.
        for detection in self.latest_detections:
            # Compute planar distance in the map-like semantic detection frame.
            distance = math.hypot(
                float(detection.x) - float(chair_slot.get("x", 0.0)),
                float(detection.y) - float(chair_slot.get("y", 0.0)),
            )

            # Treat the chair as occupied when the distance is under the radius.
            if distance <= self.occupancy_radius_m:
                return True

        # Treat the chair as free when no detection matches.
        return False

    # Convert a chair slot dictionary into a PoseStamped response field.
    def _chair_pose(self, chair_slot):
        # Create the pose message.
        pose = PoseStamped()

        # Fill the configured frame and current timestamp.
        pose.header.frame_id = str(chair_slot.get("frame_id", "map"))
        pose.header.stamp = self.get_clock().now().to_msg()

        # Fill the planar chair position.
        pose.pose.position.x = float(chair_slot.get("x", 0.0))
        pose.pose.position.y = float(chair_slot.get("y", 0.0))
        pose.pose.position.z = 0.0

        # Convert configured yaw to a planar quaternion.
        yaw = float(chair_slot.get("yaw", 0.0))
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)

        # Return the populated pose.
        return pose

    # Handle one empty-chair query.
    def _find_empty_chair_cb(self, request, response):
        # Silence the empty request object.
        del request

        # Search slots in layout order for the first currently empty chair.
        for chair_slot in self.chair_slots:
            if not self._chair_is_occupied(chair_slot):
                response.success = True
                response.message = "Empty chair selected."
                response.chair_id = str(chair_slot.get("id", "chair"))
                response.chair_pose = self._chair_pose(chair_slot)
                response.waist_yaw_rad = float(chair_slot.get("waist_yaw_rad", 0.0))
                return response

        # Report failure when every configured chair appears occupied.
        response.success = False
        response.message = "No empty chair is currently available."
        response.chair_id = ""
        response.chair_pose = PoseStamped()
        response.waist_yaw_rad = 0.0
        return response


# Provide the standard ROS 2 executable entry point.
def main(args=None):
    # Initialize ROS 2.
    rclpy.init(args=args)

    # Construct the chair manager node.
    node = ChairManagerNode()

    # Serve requests until shutdown.
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# Run the entry point when executed directly.
if __name__ == "__main__":
    main()
