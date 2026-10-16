"""Section: Start only the managed FIND_OBJECT skill and its object detector."""

# Section: ROS 2 launch primitives used by the focused lifecycle graph.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Special function: Build the standalone FIND_OBJECT lifecycle launch description.
def generate_launch_description() -> LaunchDescription:
    """Start no GPSR scheduler or executor; only perception and FIND_OBJECT."""
    # Section: Expose camera and perception wiring without assuming a camera driver.
    use_sim_time = LaunchConfiguration("use_sim_time")
    rgb_topic = LaunchConfiguration("rgb_topic")
    enable_depth = LaunchConfiguration("enable_depth")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    show_gui = LaunchConfiguration("show_gui")
    inference_backend = LaunchConfiguration("inference_backend")
    yolo_model_path = LaunchConfiguration("yolo_model_path")

    # Section: Allow an already-running detector to be reused rather than duplicated.
    start_object_detector = LaunchConfiguration("start_object_detector")
    autostart = LaunchConfiguration("autostart")

    # Section: Keep all FIND_OBJECT action and status interfaces independently configurable.
    action_name = LaunchConfiguration("action_name")
    detections_topic = LaunchConfiguration("detections_topic")
    detector_ready_topic = LaunchConfiguration("detector_ready_topic")
    detector_query_topic = LaunchConfiguration("detector_query_topic")
    target_status_topic = LaunchConfiguration("target_status_topic")
    search_config_file = LaunchConfiguration("search_config_file")
    object_knowledge_file = LaunchConfiguration("object_knowledge_file")
    furniture_map_file = LaunchConfiguration("furniture_map_file")
    default_timeout_sec = LaunchConfiguration("default_timeout_sec")
    # Section: Reuse the existing navigation stack without launching duplicate Nav2 nodes.
    enable_navigation = LaunchConfiguration("enable_navigation")
    named_navigation_action = LaunchConfiguration("named_navigation_action")
    spin_action = LaunchConfiguration("spin_action")
    camera_horizontal_fov_deg = LaunchConfiguration("camera_horizontal_fov_deg")

    # Section: Resolve installed package resources so the launch stays workspace-portable.
    default_search_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_yasmin_tasks"), "config", "find_object_search.yaml"]
    )
    default_object_knowledge_file = PathJoinSubstitution(
        [FindPackageShare("robot_yasmin_tasks"), "config", "object_knowledge.yaml"]
    )
    default_yolo_model_path = PathJoinSubstitution(
        [FindPackageShare("vision"), "models", "yolo11n.pt"]
    )

    # Section: The shared detector publishes only FIND_OBJECT perception interfaces.
    object_detector = LifecycleNode(
        package="vision",
        executable="gpsr_object_detection_node.py",
        name="gpsr_object_detection_node",
        namespace="",
        output="screen",
        condition=IfCondition(start_object_detector),
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "rgb_topic": rgb_topic,
            "detections_topic": detections_topic,
            "annotated_image_topic": "/perception/object_detections_image",
            "inference_backend": inference_backend,
            "qwen_query_topic": detector_query_topic,
            "yolo_model_path": yolo_model_path,
            "object_knowledge_file": object_knowledge_file,
            "furniture_map_file": furniture_map_file,
            "confidence_threshold": 0.35,
            "exclude_classes": ["person"],
            "publish_annotated_image": True,
            "show_gui": ParameterValue(show_gui, value_type=bool),
            "ready_topic": detector_ready_topic,
            "enable_model_warmup": True,
        }],
    )

    # Section: The lifecycle server owns the complete FIND_OBJECT state machine.
    find_object_skill_server = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="find_object_skill_server",
        name="find_object_skill_server",
        namespace="",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "action_name": action_name,
            "detections_topic": detections_topic,
            "enable_depth": ParameterValue(enable_depth, value_type=bool),
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "target_status_topic": target_status_topic,
            "detector_ready_topic": detector_ready_topic,
            "detector_query_topic": detector_query_topic,
            "search_config_file": search_config_file,
            "object_knowledge_file": object_knowledge_file,
            "default_timeout_sec": ParameterValue(default_timeout_sec, value_type=float),
            "enable_navigation": ParameterValue(enable_navigation, value_type=bool),
            "named_navigation_action": named_navigation_action,
            "spin_action": spin_action,
            "camera_horizontal_fov_deg": ParameterValue(camera_horizontal_fov_deg, value_type=float),
        }],
    )

    # Section: Configure and activate both nodes when this launch owns the detector.
    managed_detector_and_skill = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_find_object",
        namespace="",
        output="screen",
        condition=IfCondition(start_object_detector),
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "autostart": ParameterValue(autostart, value_type=bool),
            # Section: These generic lifecycle nodes do not implement Nav2 bonds;
            # disable incompatible bond monitoring while retaining lifecycle transitions.
            "bond_timeout": 0.0,
            "node_names": ["gpsr_object_detection_node", "find_object_skill_server"],
        }],
    )

    # Section: Activate only the skill when an external detector is already managed elsewhere.
    managed_skill_only = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_find_object",
        namespace="",
        output="screen",
        condition=UnlessCondition(start_object_detector),
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "autostart": ParameterValue(autostart, value_type=bool),
            # Section: Apply the same no-bond policy when reusing an external detector.
            "bond_timeout": 0.0,
            "node_names": ["find_object_skill_server"],
        }],
    )

    # Section: Return only the detector, state-machine server, and one lifecycle manager.
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time."),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw", description="RGB image topic used by the object detector."),
        DeclareLaunchArgument("enable_depth", default_value="true", description="Use aligned depth when locating a found object."),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw", description="Aligned depth image topic for FIND_OBJECT."),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info", description="Color CameraInfo topic for depth projection."),
        DeclareLaunchArgument("show_gui", default_value="false", description="Show object-detector debug windows."),
        DeclareLaunchArgument("inference_backend", default_value="yolo", description="Detector backend: qwen or yolo."),
        DeclareLaunchArgument("yolo_model_path", default_value=default_yolo_model_path, description="YOLO model path when inference_backend:=yolo."),
        DeclareLaunchArgument("start_object_detector", default_value="true", description="Start and manage gpsr_object_detection_node; false reuses an external active detector."),
        DeclareLaunchArgument("autostart", default_value="true", description="Configure and activate the launched lifecycle nodes."),
        DeclareLaunchArgument("action_name", default_value="/gpsr/find_object", description="FindObject action endpoint."),
        DeclareLaunchArgument("detections_topic", default_value="/perception/raw_object_detections", description="Object-detection input topic."),
        DeclareLaunchArgument("detector_ready_topic", default_value="/perception/object_detector_ready", description="Detector readiness topic."),
        DeclareLaunchArgument("detector_query_topic", default_value="/perception/object_detector_query", description="Goal constraint topic sent to Qwen detection."),
        DeclareLaunchArgument("target_status_topic", default_value="/gpsr/find_object/target_status", description="Latched FIND_OBJECT mission-status topic."),
        DeclareLaunchArgument("search_config_file", default_value=default_search_config_file, description="Installed FIND_OBJECT search policy."),
        DeclareLaunchArgument("object_knowledge_file", default_value=default_object_knowledge_file, description="Installed object aliases and category policy."),
        DeclareLaunchArgument("furniture_map_file", default_value="", description="Optional mapped furniture priors for metric RGB-D grounding."),
        DeclareLaunchArgument("default_timeout_sec", default_value="300.0", description="Default FIND_OBJECT action timeout in seconds."),
        DeclareLaunchArgument("enable_navigation", default_value="true", description="Navigate to configured waypoints and use Nav2 Spin for object search."),
        DeclareLaunchArgument("named_navigation_action", default_value="/nav/navigate_named_pose", description="Existing named-waypoint navigation action."),
        DeclareLaunchArgument("spin_action", default_value="/spin", description="Existing Nav2 relative-spin action."),
        DeclareLaunchArgument("camera_horizontal_fov_deg", default_value="69.4", description="Camera horizontal field of view used for final target-facing rotation."),
        object_detector,
        find_object_skill_server,
        managed_detector_and_skill,
        managed_skill_only,
    ])
