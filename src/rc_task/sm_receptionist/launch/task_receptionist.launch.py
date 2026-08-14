"""Launch the exact-flow SMACC2 Task Receptionist state machine."""

# Section: import Python path helpers for package-share defaults.
import os

# Section: import package-share lookup for installed config and waypoint paths.
from ament_index_python.packages import get_package_share_directory

# Section: import launch description primitives.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler

# Section: import launch conditions and process-exit event handling.
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Section: import launch substitutions.
from launch.substitutions import LaunchConfiguration

# Section: import ROS node actions and typed parameter values.
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


# Special function: build the launch description used by ros2 launch.
def generate_launch_description():
    # Section: resolve package shares for default support-node configuration.
    receptionist_share = get_package_share_directory("receptionist_nodes")
    my_robot_nav_share = get_package_share_directory("my_robot_nav")

    #=============================================================
    #    NAVIGATION IMPORTANT 
    #=============================================================
    # Section: resolve installed file defaults used by support nodes. 
    task2_config = os.path.join(receptionist_share, "config", "task2.yaml")
    waypoints_file = os.path.join(my_robot_nav_share, "maps", "waypoints.yaml")
    default_nav_map = os.path.join(my_robot_nav_share, "maps", "20260624_map.yaml") # set the map here. 

    # Section: create launch substitutions for task control.
    dry_run = LaunchConfiguration("dry_run")
    max_cycles = LaunchConfiguration("max_cycles")
    wait_before_guest_sec = LaunchConfiguration("wait_before_guest_sec")
    post_feature_wait_sec = LaunchConfiguration("post_feature_wait_sec")
    start_support_nodes = LaunchConfiguration("start_support_nodes")
    start_nav2_stack = LaunchConfiguration("start_nav2_stack")
    start_named_pose_server = LaunchConfiguration("start_named_pose_server")
    start_lifecycle_manager = LaunchConfiguration("start_lifecycle_manager")
    startup_readiness_timeout_sec = LaunchConfiguration("startup_readiness_timeout_sec")

    # Section: create launch substitutions for navigation.
    map_yaml_file = LaunchConfiguration("map")
    navigation_action_name = LaunchConfiguration("navigation_action_name")
    navigate_to_pose_action = LaunchConfiguration("navigate_to_pose_action")
    navigate_to_pose_server_timeout_sec = LaunchConfiguration(
        "navigate_to_pose_server_timeout_sec"
    )
    waypoint_7 = LaunchConfiguration("waypoint_7")
    waypoint_6 = LaunchConfiguration("waypoint_6")
    waypoint_8 = LaunchConfiguration("waypoint_8")
    waypoint_99 = LaunchConfiguration("waypoint_99")

    # Section: create launch substitutions for speech and dialogue.
    tts_action_name = LaunchConfiguration("tts_action_name")
    dialogue_action_name = LaunchConfiguration("dialogue_action_name")
    dialogue_mic_device = LaunchConfiguration("dialogue_mic_device")
    dialogue_record_sec = LaunchConfiguration("dialogue_record_sec")
    dialogue_piper_model_path = LaunchConfiguration("dialogue_piper_model_path")
    dialogue_whisper_model = LaunchConfiguration("dialogue_whisper_model")
    dialogue_whisper_language = LaunchConfiguration("dialogue_whisper_language")
    allow_dialogue_fallback = LaunchConfiguration("allow_dialogue_fallback")
    store_profile_in_memory = LaunchConfiguration("store_profile_in_memory")
    piper_model_path = LaunchConfiguration("piper_model_path")
    piper_tts_executable = LaunchConfiguration("piper_tts_executable")
    piper_tts_model_path = LaunchConfiguration("piper_tts_model_path")
    piper_tts_config_path = LaunchConfiguration("piper_tts_config_path")

    # Section: create launch substitutions for InsightFace feature capture.
    personal_test_mode = LaunchConfiguration("personal_test_mode")
    camera_index = LaunchConfiguration("camera_index")
    camera_backend = LaunchConfiguration("camera_backend")
    image_topic = LaunchConfiguration("image_topic")
    enable_depth = LaunchConfiguration("enable_depth")
    face_topic = LaunchConfiguration("face_topic")
    appearance_topic = LaunchConfiguration("appearance_topic")
    face_wait_timeout_sec = LaunchConfiguration("face_wait_timeout_sec")
    appearance_wait_timeout_sec = LaunchConfiguration("appearance_wait_timeout_sec")
    collection_duration_sec = LaunchConfiguration("collection_duration_sec")
    min_stable_samples = LaunchConfiguration("min_stable_samples")
    show_appearance_gui = LaunchConfiguration("show_appearance_gui")
    stream_info_period_sec = LaunchConfiguration("stream_info_period_sec")
    force_process_exit_after_terminal_state = LaunchConfiguration(
        "force_process_exit_after_terminal_state"
    )

    # Section: declare launch arguments with safe dry-run defaults.
    declared_arguments = [
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("max_cycles", default_value="2"),
        DeclareLaunchArgument("wait_before_guest_sec", default_value="3.0"),
        DeclareLaunchArgument("post_feature_wait_sec", default_value="2.0"),
        DeclareLaunchArgument("start_support_nodes", default_value="true"),
        DeclareLaunchArgument("start_nav2_stack", default_value="false"),
        DeclareLaunchArgument("start_named_pose_server", default_value="false"),
        DeclareLaunchArgument("start_lifecycle_manager", default_value="true"),
        DeclareLaunchArgument("startup_readiness_timeout_sec", default_value="0.0"),
        DeclareLaunchArgument("map", default_value=default_nav_map),  # This is map argument for static_map launch
        DeclareLaunchArgument("navigation_action_name", default_value="/nav/navigate_named_pose"),
        DeclareLaunchArgument("navigate_to_pose_action", default_value="/navigate_to_pose"),
        DeclareLaunchArgument("navigate_to_pose_server_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("waypoint_7", default_value="7"),
        DeclareLaunchArgument("waypoint_6", default_value="6"),
        DeclareLaunchArgument("waypoint_8", default_value="8"),
        DeclareLaunchArgument("waypoint_99", default_value="99"),
        DeclareLaunchArgument("tts_action_name", default_value="/tts/speak"),
        DeclareLaunchArgument("dialogue_action_name", default_value="/dialogue/collect_guest_profile"),
        DeclareLaunchArgument("dialogue_mic_device", default_value="auto"),
        DeclareLaunchArgument("dialogue_record_sec", default_value="4.0"),
        DeclareLaunchArgument("dialogue_piper_model_path", default_value=""),
        DeclareLaunchArgument("dialogue_whisper_model", default_value="base"),
        DeclareLaunchArgument("dialogue_whisper_language", default_value="en"),
        DeclareLaunchArgument("allow_dialogue_fallback", default_value="false"),
        DeclareLaunchArgument("store_profile_in_memory", default_value="true"),
        DeclareLaunchArgument("piper_model_path", default_value=""),
        DeclareLaunchArgument(
            "piper_tts_executable",
            default_value=os.path.expanduser("~/piper-venv/bin/piper"),
        ),
        DeclareLaunchArgument(
            "piper_tts_model_path",
            default_value=os.path.expanduser("~/piper_models/en_US-lessac-medium.onnx"),
        ),
        DeclareLaunchArgument(
            "piper_tts_config_path",
            default_value=os.path.expanduser("~/piper_models/en_US-lessac-medium.onnx.json"),
        ),
        DeclareLaunchArgument("personal_test_mode", default_value="false"),
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument("camera_backend", default_value="realsense"),
        DeclareLaunchArgument("image_topic", default_value="/camera/rx200_camera/color/image_raw"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("face_topic", default_value="/perception/largest_face"),
        DeclareLaunchArgument("appearance_topic", default_value="/appearance/current_estimate"),
        DeclareLaunchArgument("face_wait_timeout_sec", default_value="10.0"),
        DeclareLaunchArgument("appearance_wait_timeout_sec", default_value="10.0"),
        DeclareLaunchArgument("collection_duration_sec", default_value="2.0"),
        DeclareLaunchArgument("min_stable_samples", default_value="3"),
        DeclareLaunchArgument("show_appearance_gui", default_value="true"),
        DeclareLaunchArgument("stream_info_period_sec", default_value="1.0"),
        DeclareLaunchArgument("force_process_exit_after_terminal_state", default_value="true"),
    ]

    # Section: operator reminder used when Nav2 is expected to be launched externally.
    external_nav2_notice = LogInfo(
        msg=(
            "[task_receptionist] my_robot_nav is external to the SMACC2 lifecycle launch. "
            "Start it separately, for example: ros2 launch my_robot_nav static_map_nav.launch.py"
        ),
        condition=UnlessCondition(start_nav2_stack),
    )

    # Section: optional legacy internal Nav2 stack for manual one-shot launches.
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_nav_share, "launch", "static_map_nav.launch.py")
        ),
        condition=IfCondition(start_nav2_stack),
        launch_arguments={
            "map": map_yaml_file,
            "use_external_drivers": "true",
            "use_rviz": "true",
            "nav2_autostart": "false",
            "use_startup_gate": "true",
            "set_initial_pose": "true",
            "navigate_to_pose_server_timeout_sec": navigate_to_pose_server_timeout_sec,
        }.items(),
    )

    # Section: optional named-pose action server used by every waypoint navigation step.
    named_pose_server = Node(
        package="my_robot_nav",
        executable="navigate_named_pose_server.py",
        name="navigate_named_pose_server",
        output="screen",
        condition=IfCondition(start_named_pose_server),
        parameters=[
            {
                "waypoints_file": waypoints_file,
                "action_name": ParameterValue(navigation_action_name, value_type=str),
                "navigate_to_pose_action": ParameterValue(navigate_to_pose_action, value_type=str),
                "navigate_to_pose_server_timeout_sec": ParameterValue(
                    navigate_to_pose_server_timeout_sec,
                    value_type=float,
                ),
            }
        ],
    )

    # Section: guest-memory node used to save and clear the current guest profile.
    guest_memory_node = Node(
        package="receptionist_nodes",
        executable="guest_memory_node",
        name="guest_memory_node",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[task2_config],
    )

    # Section: dialogue action server triggered through /dialogue/collect_guest_profile.
    dialogue_action_server = Node(
        package="receptionist_nodes",
        executable="dialogue_action_server",
        name="dialogue_action_server",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[
            task2_config,
            {
                "mic_device": ParameterValue(dialogue_mic_device, value_type=str),
                "record_sec": ParameterValue(dialogue_record_sec, value_type=float),
                "piper_model_path": ParameterValue(piper_model_path, value_type=str),
                "whisper_model": ParameterValue(dialogue_whisper_model, value_type=str),
                "whisper_language": ParameterValue(dialogue_whisper_language, value_type=str),
            },
        ],
    )

    # Section: my_tts_system action server used by all deterministic speech states.
    piper_tts_node = Node(
        package="my_tts_system",
        executable="piper_tts_node",
        name="piper_tts_node",
        output="screen",
        condition=IfCondition(start_support_nodes),
        parameters=[
            {
                "action_name": ParameterValue(tts_action_name, value_type=str),
                "piper_executable": ParameterValue(piper_tts_executable, value_type=str),
                "model_path": ParameterValue(piper_tts_model_path, value_type=str),
                "config_path": ParameterValue(piper_tts_config_path, value_type=str),
            },
        ],
    )

    # Section: lifecycle node that starts the SMACC2 child only while active.
    lifecycle_node = LifecycleNode(
        package="sm_receptionist",
        executable="task_receptionist_lifecycle_node",
        name="task_receptionist_lifecycle",
        namespace="",
        output="screen",
        parameters=[
            {
                "dry_run": ParameterValue(dry_run, value_type=bool),
                "max_cycles": ParameterValue(max_cycles, value_type=int),
                "wait_before_guest_sec": ParameterValue(wait_before_guest_sec, value_type=float),
                "post_feature_wait_sec": ParameterValue(post_feature_wait_sec, value_type=float),
                "startup_readiness_timeout_sec": ParameterValue(
                    startup_readiness_timeout_sec,
                    value_type=float,
                ),
                "navigation_action_name": ParameterValue(navigation_action_name, value_type=str),
                "navigate_to_pose_action": ParameterValue(navigate_to_pose_action, value_type=str),
                "waypoint_7": ParameterValue(waypoint_7, value_type=str),
                "waypoint_6": ParameterValue(waypoint_6, value_type=str),
                "waypoint_8": ParameterValue(waypoint_8, value_type=str),
                "waypoint_99": ParameterValue(waypoint_99, value_type=str),
                "tts_action_name": ParameterValue(tts_action_name, value_type=str),
                "dialogue_action_name": ParameterValue(dialogue_action_name, value_type=str),
                "dialogue_mic_device": ParameterValue(dialogue_mic_device, value_type=str),
                "dialogue_record_sec": ParameterValue(dialogue_record_sec, value_type=float),
                "dialogue_piper_model_path": ParameterValue(
                    dialogue_piper_model_path,
                    value_type=str,
                ),
                "dialogue_whisper_model": ParameterValue(dialogue_whisper_model, value_type=str),
                "dialogue_whisper_language": ParameterValue(
                    dialogue_whisper_language,
                    value_type=str,
                ),
                "allow_dialogue_fallback": ParameterValue(
                    allow_dialogue_fallback,
                    value_type=bool,
                ),
                "store_profile_in_memory": ParameterValue(store_profile_in_memory, value_type=bool),
                "personal_test_mode": ParameterValue(personal_test_mode, value_type=bool),
                "camera_index": ParameterValue(camera_index, value_type=str),
                "camera_backend": ParameterValue(camera_backend, value_type=str),
                "image_topic": ParameterValue(image_topic, value_type=str),
                "enable_depth": ParameterValue(enable_depth, value_type=bool),
                "face_topic": ParameterValue(face_topic, value_type=str),
                "appearance_topic": ParameterValue(appearance_topic, value_type=str),
                "face_wait_timeout_sec": ParameterValue(face_wait_timeout_sec, value_type=float),
                "appearance_wait_timeout_sec": ParameterValue(
                    appearance_wait_timeout_sec,
                    value_type=float,
                ),
                "collection_duration_sec": ParameterValue(
                    collection_duration_sec,
                    value_type=float,
                ),
                "min_stable_samples": ParameterValue(min_stable_samples, value_type=int),
                "show_appearance_gui": ParameterValue(show_appearance_gui, value_type=bool),
                "stream_info_period_sec": ParameterValue(stream_info_period_sec, value_type=float),
                "force_process_exit_after_terminal_state": ParameterValue(
                    force_process_exit_after_terminal_state,
                    value_type=bool,
                ),
            }
        ],
    )

    # Section: Nav2 lifecycle manager autostarts the task lifecycle node.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_task_receptionist",
        output="screen",
        condition=IfCondition(start_lifecycle_manager),
        parameters=[
            {
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["task_receptionist_lifecycle"],
            }
        ],
    )

    # Section: return the full launch graph in dependency-friendly order.
    return LaunchDescription(
        [
            *declared_arguments,
            external_nav2_notice,
            nav2_stack,
            named_pose_server,
            guest_memory_node,
            dialogue_action_server,
            piper_tts_node,
            lifecycle_node,
            lifecycle_manager,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=lifecycle_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="Task Receptionist lifecycle node exited."
                            )
                        )
                    ],
                )
            ),
        ]
    )
