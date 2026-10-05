"""Section: Hardware-free end-to-end test harness for the FIND_OBJECT launch."""

# Section: Standard-library helpers create safe ROS CLI payloads for the test streams.
import json

# Section: ROS 2 launch primitives compose the production launch with test processes.
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Special function: Read one launch argument after ROS 2 has created the launch context.
def _argument(context, name: str) -> str:
    """Resolve one test-harness launch argument to a plain string."""
    # Section: Centralize launch substitution evaluation for dynamically built CLI commands.
    return LaunchConfiguration(name).perform(context)


# Special function: Create the optional fake detector streams and action goal process.
def _create_test_processes(context, *args, **kwargs):
    """Build hardware-free test processes from the current launch arguments."""
    # Section: Resolve the shared topic and action names used by the included skill launch.
    detections_topic = _argument(context, "detections_topic")
    detector_ready_topic = _argument(context, "detector_ready_topic")
    action_name = _argument(context, "action_name")

    # Section: Keep the synthetic message aligned with the configured test target.
    object_name = _argument(context, "test_object_name")
    category = _argument(context, "test_category")
    location = _argument(context, "test_location")
    search_scope = _argument(context, "test_search_scope")
    query_json = _argument(context, "test_query_json")
    timeout_sec = float(_argument(context, "test_timeout_sec"))
    goal_delay_sec = float(_argument(context, "goal_delay_sec"))

    # Section: Build one high-confidence detection that satisfies three-frame confirmation.
    detection_message = json.dumps(
        {
            "detections": [
                {
                    "name": object_name,
                    "class_id": 0,
                    "confidence": 0.95,
                    "x1": 100,
                    "y1": 80,
                    "x2": 300,
                    "y2": 280,
                    "center_u": 200.0,
                    "center_v": 180.0,
                    "image_width": 640,
                    "image_height": 480,
                    "attributes_json": "{}",
                }
            ]
        }
    )

    # Section: Build the action goal from the same target values as the synthetic frame.
    goal_message = json.dumps(
        {
            "object_name": object_name,
            "category": category,
            "location": location,
            "search_scope": search_scope,
            "approach_target": False,
            "stand_off_distance": 0.0,
            "timeout_sec": timeout_sec,
            "preferred_viewpoint": "",
            "query_json": query_json,
        }
    )

    # Section: Start the fake detector only when the harness owns synthetic perception.
    processes = []
    if _argument(context, "publish_test_detections").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }:
        processes.extend(
            [
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "topic",
                        "pub",
                        "-r",
                        "2",
                        detector_ready_topic,
                        "std_msgs/msg/Bool",
                        "{data: true}",
                    ],
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "topic",
                        "pub",
                        "-r",
                        "5",
                        detections_topic,
                        "putrabot_interfaces/msg/ObjectDetections",
                        detection_message,
                    ],
                    output="screen",
                ),
            ]
        )

    # Section: Send one action goal after lifecycle activation and fake streams are ready.
    if _argument(context, "send_test_goal").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }:
        # Section: The CLI action client reports feedback and the final result on screen.
        goal_process = ExecuteProcess(
            cmd=[
                "ros2",
                "action",
                "send_goal",
                action_name,
                "robot_ai_interfaces/action/FindObject",
                goal_message,
                "--feedback",
            ],
            output="screen",
        )
        processes.append(
            TimerAction(
                period=goal_delay_sec,
                actions=[goal_process],
            )
        )

    # Section: Return dynamically created test processes to the launch service.
    return processes


# Special function: Compose the production FIND_OBJECT launch with its isolated test harness.
def generate_launch_description() -> LaunchDescription:
    """Launch FIND_OBJECT with optional synthetic perception and one test action goal."""
    # Section: Resolve the installed production launch so the test follows its real wiring.
    production_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_yasmin_tasks"),
                    "launch",
                    "find_object_skill.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "enable_depth": LaunchConfiguration("enable_depth"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "show_gui": LaunchConfiguration("show_gui"),
            "inference_backend": LaunchConfiguration("inference_backend"),
            "yolo_model_path": LaunchConfiguration("yolo_model_path"),
            "start_object_detector": LaunchConfiguration("start_object_detector"),
            "autostart": LaunchConfiguration("autostart"),
            "action_name": LaunchConfiguration("action_name"),
            "detections_topic": LaunchConfiguration("detections_topic"),
            "detector_ready_topic": LaunchConfiguration("detector_ready_topic"),
            "detector_query_topic": LaunchConfiguration("detector_query_topic"),
            "target_status_topic": LaunchConfiguration("target_status_topic"),
            "search_config_file": LaunchConfiguration("search_config_file"),
            "object_knowledge_file": LaunchConfiguration("object_knowledge_file"),
            "default_timeout_sec": LaunchConfiguration("default_timeout_sec"),
            # Section: The harness disables real navigation unless fake action servers exist.
            "enable_navigation": LaunchConfiguration("enable_navigation"),
            "named_navigation_action": LaunchConfiguration("named_navigation_action"),
            "spin_action": LaunchConfiguration("spin_action"),
        }.items(),
    )

    # Section: Return production nodes, test controls, and dynamic test processes together.
    return LaunchDescription(
        [
            # Section: Production launch controls are forwarded without changing its defaults.
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "rgb_topic",
                default_value="/camera/camera/color/image_raw",
                description="RGB image topic for a real detector.",
            ),
            DeclareLaunchArgument(
                "enable_depth",
                default_value="false",
                description="Enable aligned depth in the included skill launch.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
                description="Aligned depth image topic.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/color/camera_info",
                description="Color camera info topic.",
            ),
            DeclareLaunchArgument(
                "show_gui",
                default_value="false",
                description="Show detector debug windows.",
            ),
            DeclareLaunchArgument(
                "inference_backend",
                default_value="qwen",
                description="Real detector backend: qwen or yolo.",
            ),
            DeclareLaunchArgument(
                "yolo_model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("vision"), "models", "yolo11n.pt"]
                ),
                description="YOLO model path for a real detector.",
            ),
            DeclareLaunchArgument(
                "start_object_detector",
                default_value="false",
                description="Start the real object detector instead of synthetic detections.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically configure and activate lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "action_name",
                default_value="/gpsr/find_object",
                description="FindObject action endpoint.",
            ),
            DeclareLaunchArgument(
                "detections_topic",
                default_value="/perception/raw_object_detections",
                description="Object detection input topic.",
            ),
            DeclareLaunchArgument(
                "detector_ready_topic",
                default_value="/perception/object_detector_ready",
                description="Object detector readiness topic.",
            ),
            DeclareLaunchArgument(
                "detector_query_topic",
                default_value="/perception/object_detector_query",
                description="Object detector query topic.",
            ),
            DeclareLaunchArgument(
                "target_status_topic",
                default_value="/gpsr/find_object/target_status",
                description="FIND_OBJECT status topic.",
            ),
            DeclareLaunchArgument(
                "search_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robot_yasmin_tasks"),
                        "config",
                        "find_object_search.yaml",
                    ]
                ),
                description="FIND_OBJECT search policy.",
            ),
            DeclareLaunchArgument(
                "object_knowledge_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robot_yasmin_tasks"),
                        "config",
                        "object_knowledge.yaml",
                    ]
                ),
                description="FIND_OBJECT object knowledge.",
            ),
            DeclareLaunchArgument(
                "default_timeout_sec",
                default_value="30.0",
                description="Default skill timeout.",
            ),
            DeclareLaunchArgument(
                "enable_navigation",
                default_value="false",
                description="Enable waypoint navigation and Nav2 spins in this harness.",
            ),
            DeclareLaunchArgument(
                "named_navigation_action",
                default_value="/nav/navigate_named_pose",
                description="Named navigation action forwarded to the skill launch.",
            ),
            DeclareLaunchArgument(
                "spin_action",
                default_value="/spin",
                description="Nav2 spin action forwarded to the skill launch.",
            ),
            # Section: Hardware-free test controls can be disabled for real-camera testing.
            DeclareLaunchArgument(
                "publish_test_detections",
                default_value="true",
                description="Publish synthetic readiness and detection messages.",
            ),
            DeclareLaunchArgument(
                "send_test_goal",
                default_value="true",
                description="Send one synthetic FindObject action goal.",
            ),
            DeclareLaunchArgument(
                "goal_delay_sec",
                default_value="4.0",
                description="Seconds to wait before sending the test goal.",
            ),
            DeclareLaunchArgument(
                "test_object_name",
                default_value="bottle",
                description="Object name used by the synthetic test goal.",
            ),
            DeclareLaunchArgument(
                "test_category",
                default_value="",
                description="Optional category used by the synthetic test goal.",
            ),
            DeclareLaunchArgument(
                "test_location",
                default_value="",
                description="Optional strict location used by the synthetic test goal.",
            ),
            DeclareLaunchArgument(
                "test_search_scope",
                default_value="GLOBAL",
                description="Search scope used by the synthetic test goal.",
            ),
            DeclareLaunchArgument(
                "test_query_json",
                default_value="{}",
                description="JSON query filters used by the synthetic test goal.",
            ),
            DeclareLaunchArgument(
                "test_timeout_sec",
                default_value="15.0",
                description="Action timeout used by the synthetic test goal.",
            ),
            production_launch,
            OpaqueFunction(function=_create_test_processes),
        ]
    )
