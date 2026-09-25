#!/usr/bin/env python3

# Import math so the node can measure chair-to-person distances.
import math
# Import yaml so the node can load the receptionist layout file.
import yaml

# Import the ROS 2 Python client library.
import rclpy
# Import the Node base class used to build ROS 2 nodes.
from rclpy.node import Node

# Import the seated-target service exposed by this node.
from robot_ai_interfaces.srv import ResolvePresentationTargets
# Import the detections topic used to estimate occupied chairs.
from putrabot_interfaces.msg import CamDetections
# Import the stamped pose message used in the service response.
from geometry_msgs.msg import PoseStamped


# Define one node that resolves who should be introduced using chair occupancy.
class SeatedPeopleNode(Node):
    def __init__(self):
        # Give the ROS 2 node a stable name so clients can find its service.
        super().__init__("seated_people_node")

        # Declare the path to the receptionist layout file.
        self.declare_parameter("layout_file", "")
        # Declare the camera detections topic used to estimate occupied chairs.
        self.declare_parameter("detections_topic", "/cam_detections")

        # Cache the configured layout file path.
        self.layout_file = self.get_parameter("layout_file").value
        # Cache the configured detections topic name.
        self.detections_topic = self.get_parameter("detections_topic").value

        # Load the receptionist layout file once during startup.
        self.layout_data = self._load_layout_file(self.layout_file)
        # Cache the chair-slot definitions from the layout file.
        self.chair_slots = self.layout_data.get("chairs", {}).get("slots", [])
        # Cache the host chair id used by the simple target-resolution rule.
        self.host_chair_id = self.layout_data.get("chairs", {}).get("host_chair_id", "host_chair")
        # Cache the guest-1 chair id used by the simple target-resolution rule.
        self.guest1_chair_id = self.layout_data.get("chairs", {}).get("guest_1_chair_id", "guest_1_chair")
        # Cache the occupancy radius used for chair-slot matching.
        self.occupancy_radius_m = float(self.layout_data.get("chairs", {}).get("occupancy_radius_m", 0.6))

        # Keep the latest camera detections so target resolution can use them.
        self.latest_detections = []

        # Subscribe to the detections topic so the node can estimate chair occupancy.
        self.create_subscription(CamDetections, self.detections_topic, self.detections_cb, 10)
        # Expose the presentation-target resolution service.
        self.create_service(
            ResolvePresentationTargets,
            "/seated_people/resolve_presentation_targets",
            self.resolve_targets_cb,
        )

    # Load the raw receptionist layout YAML file.
    def _load_layout_file(self, layout_file):
        # Return an empty layout when no path was configured.
        if not layout_file:
            return {}
        # Open the layout file in text mode.
        with open(layout_file, "r", encoding="utf-8") as stream:
            # Parse the YAML file into a plain Python dictionary.
            return yaml.safe_load(stream) or {}

    # Cache the most recent detection array.
    def detections_cb(self, msg):
        # Store the raw detection list so chair occupancy can be recomputed on demand.
        self.latest_detections = list(msg.detections)

    # Look up one chair slot dictionary by its configured id.
    def chair_by_id(self, chair_id):
        # Loop over every configured chair slot.
        for chair_slot in self.chair_slots:
            # Return the slot when the ids match.
            if chair_slot.get("id") == chair_id:
                return chair_slot
        # Return None when no configured chair uses the requested id.
        return None

    # Decide whether one configured chair currently appears occupied.
    def chair_is_occupied(self, chair_slot):
        # Return False immediately when the caller provided no chair slot.
        if chair_slot is None:
            return False
        # Loop over every latest camera detection.
        for detection in self.latest_detections:
            # Measure the planar distance between the detection and the chair center.
            distance = math.hypot(
                float(detection.x) - float(chair_slot.get("x", 0.0)),
                float(detection.y) - float(chair_slot.get("y", 0.0)),
            )
            # Treat the chair as occupied when a detection falls within the configured radius.
            if distance <= self.occupancy_radius_m:
                return True
        # Treat the chair as empty when no detection matched it.
        return False

    # Convert one chair slot dictionary into a PoseStamped response field.
    def chair_pose(self, chair_slot):
        # Create a new stamped pose message.
        pose = PoseStamped()
        # Use the configured frame id or default to "map".
        pose.header.frame_id = chair_slot.get("frame_id", "map")
        # Stamp the pose with the current ROS time.
        pose.header.stamp = self.get_clock().now().to_msg()
        # Copy the configured chair x coordinate.
        pose.pose.position.x = float(chair_slot.get("x", 0.0))
        # Copy the configured chair y coordinate.
        pose.pose.position.y = float(chair_slot.get("y", 0.0))
        # Keep z fixed at zero because this prototype is planar.
        pose.pose.position.z = 0.0
        # Convert the configured chair yaw into a quaternion.
        yaw = float(chair_slot.get("yaw", 0.0))
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        # Return the stamped pose.
        return pose

    # Resolve who the current guest should be introduced to.
    def resolve_targets_cb(self, request, response):
        # Look up the configured host chair from the layout file.
        host_chair = self.chair_by_id(self.host_chair_id)
        # Look up the configured guest-1 chair from the layout file.
        guest1_chair = self.chair_by_id(self.guest1_chair_id)

        # Fail when the host chair is missing because the simple rule depends on it.
        if host_chair is None:
            response.success = False
            response.message = "Host chair is missing from the receptionist layout."
            response.target_profile_ids = []
            response.face_target_profile_id = ""
            response.face_target_pose = PoseStamped()
            return response

        # Start with the host because the host is always a valid introduction target.
        target_ids = ["host"]
        face_target_profile_id = "host"
        face_target_pose = self.chair_pose(host_chair)

        # For guest 2 and later, add guest_1 when the configured chair appears occupied.
        if request.guest_index >= 2 and self.chair_is_occupied(guest1_chair):
            target_ids.append("guest_1")
            # Face guest_1 directly when that seated guest is present so the spoken
            # introduction can address the dedicated human instead of always the host.
            if guest1_chair is not None:
                face_target_profile_id = "guest_1"
                face_target_pose = self.chair_pose(guest1_chair)

        # Report success because the deterministic target-resolution rule completed.
        response.success = True
        # Return a short explanation for logs and debugging.
        response.message = "Presentation targets resolved from configured chair occupancy."
        # Return the ordered list of target profile ids.
        response.target_profile_ids = target_ids
        # Return the main profile id the robot should face before speaking.
        response.face_target_profile_id = face_target_profile_id
        # Return the pose for that main face target.
        response.face_target_pose = face_target_pose
        # Return the populated response object to ROS 2.
        return response


# Provide the standard ROS 2 Python entry point.
def main(args=None):
    # Initialize the ROS 2 client library.
    rclpy.init(args=args)
    # Construct the seated-people node.
    node = SeatedPeopleNode()
    # Spin the node so it can answer service calls until shutdown.
    rclpy.spin(node)
    # Destroy the node explicitly during shutdown.
    node.destroy_node()
    # Shut down the ROS 2 client library.
    rclpy.shutdown()


# Run the entry point when the file is executed as a script.
if __name__ == "__main__":
    main()
