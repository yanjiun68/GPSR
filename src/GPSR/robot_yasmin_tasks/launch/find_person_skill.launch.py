"""Section: Start only the managed FIND_PERSON identity-perception resource."""

# Section: ROS 2 launch primitives for the focused lifecycle graph.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


# Special function: Build the standalone FIND_PERSON resource launch description.
def generate_launch_description() -> LaunchDescription:
    """Start the FIND_PERSON identity resource without GPSR orchestration or navigation."""
    # Section: Expose identity perception inputs and model controls without starting a camera driver.
    use_sim_time = LaunchConfiguration("use_sim_time")
    rgb_topic = LaunchConfiguration("rgb_topic")
    output_topic = LaunchConfiguration("output_topic")
    person_model = LaunchConfiguration("person_model")
    attribute_model = LaunchConfiguration("attribute_model")
    pose_model = LaunchConfiguration("pose_model")
    device = LaunchConfiguration("device")
    identity_threshold = LaunchConfiguration("identity_threshold")
    ambiguity_margin = LaunchConfiguration("ambiguity_margin")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    autostart = LaunchConfiguration("autostart")

    # Section: The lifecycle node is the sole resource registered for FIND_PERSON.
    human_identity_node = LifecycleNode(
        package="vision",
        executable="gpsr_human_identity_lifecycle_node.py",
        name="gpsr_human_identity_node",
        namespace="",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "image_topic": rgb_topic,
            "output_topic": output_topic,
            "person_model": person_model,
            "attribute_model": attribute_model,
            "pose_model": pose_model,
            "device": device,
            "identity_threshold": ParameterValue(identity_threshold, value_type=float),
            "ambiguity_margin": ParameterValue(ambiguity_margin, value_type=float),
            "confidence_threshold": ParameterValue(confidence_threshold, value_type=float),
        }],
    )

    # Section: One lifecycle manager configures and activates the sole FIND_PERSON resource.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_find_person",
        namespace="",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "autostart": ParameterValue(autostart, value_type=bool),
            # Section: Generic lifecycle resources do not implement Nav2 bond protocol.
            "bond_timeout": 0.0,
            "node_names": ["gpsr_human_identity_node"],
        }],
    )

    # Section: Return only the managed identity resource and its lifecycle manager.
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time."),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw", description="RGB image topic used for person tracking."),
        DeclareLaunchArgument("output_topic", default_value="/human_identity/recognized_people", description="Tracked-person identity output topic."),
        DeclareLaunchArgument("person_model", default_value="yolo11n.pt", description="YOLO person-detection model installed in the vision package."),
        DeclareLaunchArgument("attribute_model", default_value="human_attributes.pt", description="Optional person-attribute model installed in the vision package."),
        DeclareLaunchArgument("pose_model", default_value="pose_landmarker_full.task", description="Pose model installed in the vision package."),
        DeclareLaunchArgument("device", default_value="0", description="Inference device accepted by the YOLO runtime."),
        DeclareLaunchArgument("identity_threshold", default_value="0.72", description="Minimum identity-match confidence."),
        DeclareLaunchArgument("ambiguity_margin", default_value="0.08", description="Minimum separation between two identity matches."),
        DeclareLaunchArgument("confidence_threshold", default_value="0.40", description="Minimum person-detection confidence."),
        DeclareLaunchArgument("autostart", default_value="true", description="Configure and activate the FIND_PERSON identity resource."),
        human_identity_node,
        lifecycle_manager,
    ])
