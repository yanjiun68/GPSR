"""Section: Umbrella launch for deterministic GPSR task scheduler."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    NotEqualsSubstitution,
    OrSubstitution,
)
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Special function: Launch the GPSR scheduler and optional executor/CV support nodes."""
    # Section: Core scheduler launch-time controls.
    use_sim_time = LaunchConfiguration("use_sim_time")
    dry_run = LaunchConfiguration("dry_run")
    command_input_mode = LaunchConfiguration("command_input_mode")
    command_topic = LaunchConfiguration("command_topic")
    schedule_command_service = LaunchConfiguration("schedule_command_service")
    reset_state_service = LaunchConfiguration("reset_state_service")
    skill_registry_file = LaunchConfiguration("skill_registry_file")
    default_skill_registry_file = PathJoinSubstitution(
        [FindPackageShare("robot_yasmin_tasks"), "config", "skill_registry.yaml"]
    )
    # Section: Person find skills share one installed waypoint search policy.
    find_person_search_config_file = LaunchConfiguration("find_person_search_config_file")
    default_find_person_search_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_yasmin_tasks"), "config", "find_person_search.yaml"]
    )
    # Section: Object find uses its own installed room and scan waypoint policy.
    find_object_search_config_file = LaunchConfiguration("find_object_search_config_file")
    default_find_object_search_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_yasmin_tasks"), "config", "find_object_search.yaml"]
    )

    # Section: Optional integrated runtime nodes.
    start_executor = LaunchConfiguration("start_executor")
    start_count_people_pipeline = LaunchConfiguration("start_count_people_pipeline")
    start_count_objects_pipeline = LaunchConfiguration("start_count_objects_pipeline")
    start_find_object_pipeline = LaunchConfiguration("start_find_object_pipeline")
    start_find_person_pipeline = LaunchConfiguration("start_find_person_pipeline")
    start_find_person_attribute_pipeline = LaunchConfiguration("start_find_person_attribute_pipeline")
    start_follow_person_pipeline = LaunchConfiguration("start_follow_person_pipeline")
    start_voice_tts = LaunchConfiguration("start_voice_tts")
    start_voice_stt = LaunchConfiguration("start_voice_stt")
    start_voice_command = LaunchConfiguration("start_voice_command")
    command_input_timeout_sec = LaunchConfiguration("command_input_timeout_sec")
    # Section: Use one echo guard setting for the GPSR dialogue handoff and STT node.
    post_tts_guard_ms = LaunchConfiguration("post_tts_guard_ms")
    # Section: Choose the Piper ONNX voice used for every GPSR prompt.
    piper_model_path = LaunchConfiguration("piper_model_path")
    # Section: Keep a bounded voice reply window inside the scheduler dialogue deadline.
    max_clarification_attempts = LaunchConfiguration("max_clarification_attempts")
    clarification_timeout_sec = LaunchConfiguration("clarification_timeout_sec")
    clarification_listen_timeout_sec = LaunchConfiguration("clarification_listen_timeout_sec")
    allow_simulated_primitives = LaunchConfiguration("allow_simulated_primitives")

    # Section: Shared perception launch-time controls.
    rgb_topic = LaunchConfiguration("rgb_topic")
    enable_depth = LaunchConfiguration("enable_depth")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    show_gui = LaunchConfiguration("show_gui")
    enable_count_report_tts = LaunchConfiguration("enable_count_report_tts")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    human_yolo_model_path = LaunchConfiguration("human_yolo_model_path")
    # Section: Use the requested workspace YOLOE segmentation weights for shared object detection.
    default_yolo_model_path = "/home/yanjiun68/workspace/WS1/PutrabotUPM-project/yoloe-26s-seg.pt"
    # Section: Preserve the class-labelled YOLO11 model for human-only perception.
    default_human_yolo_model_path = PathJoinSubstitution([FindPackageShare("vision"), "models", "yolo11n.pt"])
    default_pose_model_path = PathJoinSubstitution([FindPackageShare("vision"), "models", "pose_landmarker_full.task"])

    # Section: Count stability parameters shared by object and people perception services.
    confirmation_window_sec = LaunchConfiguration("confirmation_window_sec")
    confirmation_required_ratio = LaunchConfiguration("confirmation_required_ratio")
    confirmation_confidence_threshold = LaunchConfiguration("confirmation_confidence_threshold")
    confirmation_last_seen_timeout_sec = LaunchConfiguration("confirmation_last_seen_timeout_sec")
    confirmation_min_frames = LaunchConfiguration("confirmation_min_frames")

    # Section: Scheduler mission policy and dependency switches.
    instruction_point = LaunchConfiguration("instruction_point")
    return_after_completion = LaunchConfiguration("return_after_completion")
    return_after_failure = LaunchConfiguration("return_after_failure")
    max_commands = LaunchConfiguration("max_commands")
    navigate_to_instruction_point_on_start = LaunchConfiguration("navigate_to_instruction_point_on_start")
    return_target_mode = LaunchConfiguration("return_target_mode")
    amcl_pose_topic = LaunchConfiguration("amcl_pose_topic")
    amcl_pose_max_age_sec = LaunchConfiguration("amcl_pose_max_age_sec")
    amcl_pose_capture_timeout_sec = LaunchConfiguration("amcl_pose_capture_timeout_sec")
    max_find_history_records = LaunchConfiguration("max_find_history_records")
    return_fallback_named_location = LaunchConfiguration("return_fallback_named_location")
    enable_error_recovery = LaunchConfiguration("enable_error_recovery")
    error_recovery_max_attempts = LaunchConfiguration("error_recovery_max_attempts")
    error_recovery_retry_delay_sec = LaunchConfiguration("error_recovery_retry_delay_sec")
    executor_goal_response_timeout_sec = LaunchConfiguration("executor_goal_response_timeout_sec")
    navigation_action = LaunchConfiguration("navigation_action")
    navigate_to_pose_action = LaunchConfiguration("navigate_to_pose_action")
    navigate_to_named_pose_action = LaunchConfiguration("navigate_to_named_pose_action")
    follow_person_action = LaunchConfiguration("follow_person_action")
    find_object_action = LaunchConfiguration("find_object_action")
    find_person_action = LaunchConfiguration("find_person_action")
    lifecycle_transition_timeout_sec = LaunchConfiguration("lifecycle_transition_timeout_sec")
    lifecycle_operation_timeout_margin_sec = LaunchConfiguration("lifecycle_operation_timeout_margin_sec")
    startup_dependency_poll_sec = LaunchConfiguration("startup_dependency_poll_sec")
    tts_action = LaunchConfiguration("tts_action")
    default_skill_timeout_sec = LaunchConfiguration("default_skill_timeout_sec")
    default_skill_retries = LaunchConfiguration("default_skill_retries")
    enable_navigation = LaunchConfiguration("enable_navigation")
    enable_perception = LaunchConfiguration("enable_perception")
    enable_manipulation = LaunchConfiguration("enable_manipulation")
    enable_tts = LaunchConfiguration("enable_tts")
    enable_scheduler_viewer = LaunchConfiguration("enable_scheduler_viewer")

    # Section: FIND_OBJECT and COUNT_OBJECTS share exactly one lifecycle detector process.
    start_shared_object_detector = OrSubstitution(
        start_count_objects_pipeline,
        start_find_object_pipeline,
    )

    # Section: COUNT_PEOPLE and attribute FIND_PERSON share human and pose inference nodes.
    start_shared_human_pose = OrSubstitution(
        start_count_people_pipeline,
        start_find_person_attribute_pipeline,
    )
    # Section: Name-only FIND_PERSON also needs the shared pose/appearance track stream.
    start_shared_human_appearance = OrSubstitution(
        start_count_people_pipeline,
        # Section: ROS Humble accepts exactly two operands per OrSubstitution.
        OrSubstitution(
            start_find_person_pipeline,
            start_find_person_attribute_pipeline,
        ),
    )
    # Section: Name and attribute searches reuse exactly one managed YOLO person tracker.
    start_shared_human_identity = OrSubstitution(
        start_find_person_pipeline,
        start_find_person_attribute_pipeline,
    )

    # Section: Main deterministic scheduler entrypoint.
    scheduler_node = Node(
        package="gpsr_task_scheduler",
        executable="gpsr_scheduler",
        name="gpsr_task_scheduler",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "dry_run": dry_run,
                "command_input_mode": command_input_mode,
                "command_topic": command_topic,
                "schedule_command_service": schedule_command_service,
                "reset_state_service": reset_state_service,
                "skill_registry_file": skill_registry_file,
                "allow_simulated_primitives": ParameterValue(allow_simulated_primitives, value_type=bool),
                "instruction_point": instruction_point,
                "return_after_completion": return_after_completion,
                "return_after_failure": return_after_failure,
                "max_commands": max_commands,
                "navigate_to_instruction_point_on_start": navigate_to_instruction_point_on_start,
                "return_target_mode": return_target_mode,
                "amcl_pose_topic": amcl_pose_topic,
                "amcl_pose_max_age_sec": ParameterValue(amcl_pose_max_age_sec, value_type=float),
                "amcl_pose_capture_timeout_sec": ParameterValue(amcl_pose_capture_timeout_sec, value_type=float),
                "return_fallback_named_location": return_fallback_named_location,
                "enable_error_recovery": ParameterValue(enable_error_recovery, value_type=bool),
                "error_recovery_max_attempts": ParameterValue(error_recovery_max_attempts, value_type=int),
                "error_recovery_retry_delay_sec": ParameterValue(error_recovery_retry_delay_sec, value_type=float),
                "executor_goal_response_timeout_sec": ParameterValue(executor_goal_response_timeout_sec, value_type=float),
                "navigation_action": navigation_action,
                "find_object_action": find_object_action,
                "find_person_action": find_person_action,
                "tts_action": tts_action,
                "default_skill_timeout_sec": default_skill_timeout_sec,
                "default_skill_retries": default_skill_retries,
                "max_clarification_attempts": ParameterValue(max_clarification_attempts, value_type=int),
                "clarification_timeout_sec": ParameterValue(clarification_timeout_sec, value_type=float),
                "enable_navigation": enable_navigation,
                "enable_perception": enable_perception,
                "enable_manipulation": enable_manipulation,
                "enable_tts": enable_tts,
                "enable_scheduler_viewer": ParameterValue(enable_scheduler_viewer, value_type=bool),
                # Section: A disabled executor permits planning-only startup without a false wait.
                "require_executor_ready": ParameterValue(start_executor, value_type=bool),
            }
        ],
    )

    # Section: Queue executor required for non-dry-run GPSR task execution.
    gpsr_executor_node = Node(
        package="robot_yasmin_tasks",
        executable="gpsr_executor",
        name="gpsr_executor",
        output="screen",
        condition=IfCondition(start_executor),
        parameters=[{
            "skill_registry_file": skill_registry_file,
            "allow_simulated_primitives": ParameterValue(allow_simulated_primitives, value_type=bool),
            "navigate_to_pose_action": navigate_to_pose_action,
            "navigate_to_named_pose_action": navigate_to_named_pose_action,
            "follow_person_action": follow_person_action,
            "find_person_search_config_file": find_person_search_config_file,
            "find_object_search_config_file": find_object_search_config_file,
            "lifecycle_transition_timeout_sec": ParameterValue(lifecycle_transition_timeout_sec, value_type=float),
            "lifecycle_operation_timeout_margin_sec": ParameterValue(lifecycle_operation_timeout_margin_sec, value_type=float),
            "startup_dependency_poll_sec": ParameterValue(startup_dependency_poll_sec, value_type=float),
            # Section: GPSR waits this interval after completed playback before issuing STT.
            "post_tts_guard_ms": ParameterValue(post_tts_guard_ms, value_type=int),
        }],
    )

    # Section: The FIXED_INFORMATION skill requires the deterministic local answer provider.
    fixed_information_node = Node(
        package="fixed_information_dialogue",
        executable="fixed_information_node",
        name="fixed_information_dialogue",
        output="screen",
        condition=IfCondition(start_executor),
    )

    # Section: Every executor ActionStep resolves resource policy through one manager.
    gpsr_lifecycle_manager = Node(
        package="robot_yasmin_tasks",
        executable="gpsr_lifecycle_manager",
        name="gpsr_lifecycle_manager",
        output="screen",
        condition=IfCondition(start_executor),
        parameters=[{
            "registry_file": skill_registry_file,
            "allow_simulated_primitives": ParameterValue(allow_simulated_primitives, value_type=bool),
            "transition_timeout_sec": ParameterValue(lifecycle_transition_timeout_sec, value_type=float),
            # Section: Only non-legacy providers use the managed lifecycle adapter.
            "manage_follow_person_adapter": ParameterValue(
                NotEqualsSubstitution(follow_person_action, "/person_follow/control"),
                value_type=bool,
            ),
        }],
    )

    # Section: Persistent GPSR-only guest memory remains available across all ActionSteps.
    gpsr_guest_memory_node = Node(
        package="robot_yasmin_tasks",
        executable="gpsr_guest_memory_node",
        name="gpsr_guest_memory_node",
        output="screen",
        condition=IfCondition(start_executor),
        parameters=[{
            "use_sim_time": use_sim_time,
            "amcl_pose_topic": amcl_pose_topic,
            "amcl_pose_max_age_sec": ParameterValue(amcl_pose_max_age_sec, value_type=float),
            "max_find_records": ParameterValue(max_find_history_records, value_type=int),
        }],
    )

    # Section: GPSR-facing action server that bridges COUNT_PEOPLE skills to CV service calls.
    count_people_skill_server = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="count_people_skill_server",
        name="count_people_skill_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_count_people_pipeline),
        parameters=[{
            "action_name": "/gpsr/count_people",
            "perception_service": "/perception/count_people",
            "default_timeout_sec": 5.0,
        }],
    )

    # Section: GPSR-facing action server that bridges COUNT_OBJECTS skills to CV service calls.
    count_objects_skill_server = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="count_objects_skill_server",
        name="count_objects_skill_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_count_objects_pipeline),
        parameters=[{
            "action_name": "/gpsr/count_objects",
            "perception_service": "/perception/count_objects",
            "default_timeout_sec": 5.0,
        }],
    )

    # Section: Human detector publishes raw person detections consumed by /perception/count_people.
    gpsr_human_detector = LifecycleNode(
        package="vision",
        executable="gpsr_object_detection_node.py",
        name="gpsr_human_detection_node",
        namespace="",
        output="screen",
        condition=IfCondition(start_shared_human_pose),
        parameters=[{
            "rgb_topic": rgb_topic,
            "detections_topic": "/perception/raw_human_detections",
            "annotated_image_topic": "/perception/human_detections_image",
            # Section: Human tracking keeps its dedicated YOLO backend and does not consume Qwen world sensing.
            "inference_backend": "yolo",
            "yolo_model_path": human_yolo_model_path,
            "confidence_threshold": 0.35,
            "include_classes": ["person"],
            "exclude_classes": ["__none__"],
            "publish_annotated_image": True,
            "show_gui": show_gui,
            "gui_window_name": "GPSR Human Detection",
        }],
    )

    # Section: Count-people CV service used by /gpsr/count_people action goals.
    count_people_perception_server = LifecycleNode(
        package="vision",
        executable="count_people_perception_server.py",
        name="count_people_perception_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_count_people_pipeline),
        parameters=[{
            "detections_topic": "/perception/raw_human_detections",
            "service_name": "/perception/count_people",
            "human_quantities_topic": "/perception/human_quantities",
            "raw_human_quantities_topic": "/perception/human_quantities_raw",
            "enable_pose_sorting": False,
            "skeletons_topic": "/perception/human_skeletons",
            "stale_timeout_sec": 5.0,
            "confirmation_window_sec": ParameterValue(confirmation_window_sec, value_type=float),
            "confirmation_required_ratio": ParameterValue(confirmation_required_ratio, value_type=float),
            "confirmation_confidence_threshold": ParameterValue(confirmation_confidence_threshold, value_type=float),
            "confirmation_last_seen_timeout_sec": ParameterValue(confirmation_last_seen_timeout_sec, value_type=float),
            "confirmation_min_frames": ParameterValue(confirmation_min_frames, value_type=int),
        }],
    )

    # Section: Object detector publishes raw object detections consumed by /perception/count_objects.
    gpsr_object_detector = LifecycleNode(
        package="vision",
        executable="gpsr_object_detection_node.py",
        name="gpsr_object_detection_node",
        namespace="",
        output="screen",
        condition=IfCondition(start_shared_object_detector),
        parameters=[{
            "rgb_topic": rgb_topic,
            "detections_topic": "/perception/raw_object_detections",
            "annotated_image_topic": "/perception/object_detections_image",
            # Section: GPSR object skills use the integrated test_camera-derived Qwen backend by default.
            "inference_backend": "qwen",
            # Section: The shared topic carries FIND_OBJECT constraints to the Qwen world-sensing backend.
            "qwen_query_topic": "/perception/object_detector_query",
            "yolo_model_path": yolo_model_path,
            "confidence_threshold": 0.35,
            "exclude_classes": ["person"],
            "publish_annotated_image": True,
            "show_gui": show_gui,
            # Section: Publish readiness only after the object model handles a live camera frame.
            "ready_topic": "/perception/object_detector_ready",
            "enable_model_warmup": True,
        }],
    )

    # Section: Count-object CV service used by /gpsr/count_objects action goals.
    count_objects_perception_server = LifecycleNode(
        package="vision",
        executable="count_objects_perception_server.py",
        name="count_objects_perception_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_count_objects_pipeline),
        parameters=[{
            "detections_topic": "/perception/raw_object_detections",
            "service_name": "/perception/count_objects",
            # Section: COUNT_OBJECTS sends its target/filter query to the shared Qwen object detector.
            "detector_query_topic": "/perception/object_detector_query",
            "qwen_query_wait_timeout_sec": 1.5,
            "object_quantities_topic": "/perception/object_quantities",
            "raw_object_quantities_topic": "/perception/object_quantities_raw",
            "stale_timeout_sec": 5.0,
            "confirmation_window_sec": ParameterValue(confirmation_window_sec, value_type=float),
            "confirmation_required_ratio": ParameterValue(confirmation_required_ratio, value_type=float),
            "confirmation_confidence_threshold": ParameterValue(confirmation_confidence_threshold, value_type=float),
            "confirmation_last_seen_timeout_sec": ParameterValue(confirmation_last_seen_timeout_sec, value_type=float),
            "confirmation_min_frames": ParameterValue(confirmation_min_frames, value_type=int),
        }],
    )

    # Section: COUNT_PEOPLE registry coverage includes pose inference before its service bridge.
    human_skeleton_detector = LifecycleNode(
        package="vision",
        executable="human_skeleton_detection_node.py",
        name="human_skeleton_detection_node",
        namespace="",
        output="screen",
        condition=IfCondition(start_shared_human_appearance),
        parameters=[{
            "image_topic": rgb_topic,
            "model_path": default_pose_model_path,
            "show_gui": show_gui,
            "enable_tts": False,
            "legacy_skeletons_topic": "/perception/human_skeletons",
        }],
    )

    # Section: Canonical tracked person IDs and registered profile hints are lifecycle-owned.
    gpsr_human_identity_node = LifecycleNode(
        package="vision",
        executable="gpsr_human_identity_lifecycle_node.py",
        name="gpsr_human_identity_node",
        namespace="",
        output="screen",
        condition=IfCondition(start_shared_human_identity),
        parameters=[{
            "image_topic": rgb_topic,
            "output_topic": "/human_identity/recognized_people",
            "person_model": "yolo11n.pt",
            "device": "0",
        }],
    )

    # Section: Reuse receptionist appearance estimators for every canonical visible track.
    gpsr_tracked_appearance_node = LifecycleNode(
        package="face_detection",
        executable="tracked_appearance_lifecycle",
        name="gpsr_tracked_appearance_node",
        namespace="",
        output="screen",
        condition=IfCondition(start_shared_human_appearance),
        parameters=[{
            "image_topic": rgb_topic,
            "identity_topic": "/human_identity/recognized_people",
            "skeleton_topic": "/perception/human_skeletons",
            "pose_summary_topic": "/perception/human_poses",
            "output_topic": "/appearance/tracked_people",
            "appearance_device": "0",
        }],
    )

    # Section: The lifecycle server owns the complete FIND_PERSON_BY_ATTRIBUTE YASMIN mission.
    find_person_by_attribute_skill_server = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="find_person_by_attribute_skill_server",
        name="find_person_by_attribute_skill_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_find_person_attribute_pipeline),
        parameters=[{
            "action_name": "/gpsr/find_person_by_attribute",
            "appearance_topic": "/appearance/tracked_people",
            "memory_save_service": "/gpsr/guest_memory/save",
            "confirmation_min_frames": ParameterValue(confirmation_min_frames, value_type=int),
            "enabled_wearable_predicates": ["glasses"],
            # Section: VLM is an opt-in semantic fallback after tracked deterministic matching.
            "enable_vlm_fallback": True,
            "vlm_query_topic": "/perception/person_vlm_query",
            "vlm_result_topic": "/perception/person_vlm_result",
            "vlm_minimum_confidence": 0.55,
            # Section: S_err reports unsupported negative visible attributes then
            # returns through the same configured instruction-point navigation path.
            "tts_service": tts_action,
            "navigation_action": navigate_to_named_pose_action,
            "instruction_point": instruction_point,
        }],
    )

    # Section: Name searches use the same persistent tracks as attribute searches and
    # verify identity through a bounded raised-hand interaction.
    find_person_skill_server = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="find_person_skill_server",
        name="find_person_skill_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_shared_human_identity),
        parameters=[{
            "action_name": "/gpsr/find_person",
            "appearance_topic": "/appearance/tracked_people",
            "tts_service": "/voice/tts/say",
            "gesture_confidence_threshold": 0.55,
            "gesture_min_stable_frames": 3,
        }],
    )

    # Section: The extraction server enriches and re-saves the same GPSR guest record.
    extract_person_info_skill_server = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="extract_person_info_skill_server",
        name="extract_person_info_skill_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_find_person_attribute_pipeline),
        parameters=[{
            "action_name": "/gpsr/extract_guest_info",
            "appearance_topic": "/appearance/tracked_people",
            "memory_get_service": "/gpsr/guest_memory/get",
            "memory_save_service": "/gpsr/guest_memory/save",
            "profile_get_service": "/gpsr/identity/get_profile",
        }],
    )

    # Section: FIND_OBJECT shares the managed object detector and owns its compound mission server.
    find_object_skill_server = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="find_object_skill_server",
        name="find_object_skill_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_find_object_pipeline),
        parameters=[{
            "detections_topic": "/perception/raw_object_detections",
            "enable_depth": ParameterValue(enable_depth, value_type=bool),
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "action_name": "/gpsr/find_object",
            "target_status_topic": "/gpsr/find_object/target_status",
            "detector_ready_topic": "/perception/object_detector_ready",
            # Section: Keep the C++ selector and Qwen detector on one explicit query transport.
            "detector_query_topic": "/perception/object_detector_query",
        }],
    )

    # Section: GUIDE_PERSON owns its sequential Nav2/TTS state machine and is
    # activated only by the GPSR lifecycle manager for a guide ActionStep.
    guide_person_skill_server = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="guide_person_skill_server",
        name="guide_person_skill_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_executor),
        parameters=[{
            "action_name": "/person_guide/control",
            "navigation_action": navigate_to_named_pose_action,
            "tts_service": "/voice/tts/say",
            "max_navigation_retries": 1,
        }],
    )

    # Section: Optional local TTS service used by SAY/report skills.
    voice_tts_node = Node(
        package="voice",
        executable="tts_node.py",
        name="voice_tts",
        output="screen",
        condition=IfCondition(start_voice_tts),
        # Section: Forward the selected model instead of relying on a node-local default.
        parameters=[{"piper_model_path": piper_model_path}],
    )

    # Section: Local STT supplies deterministic yes/no transcripts for verbal identity mode.
    voice_stt_node = Node(
        package="voice",
        executable="stt_node.py",
        name="voice_stt",
        output="screen",
        condition=IfCondition(start_voice_stt),
        parameters=[{
            # Section: Keep the scheduler-owned Whisper STT path in English.
            "whisper_language": "en",
            # Section: Retain STT-side protection for direct topic requests outside GPSR dialogue.
            "post_tts_guard_ms": ParameterValue(post_tts_guard_ms, value_type=int),
        }],
    )

    # Section: Initial spoken command reception remains at the robot's current position.
    gpsr_voice_command_node = Node(
        package="robot_yasmin_tasks",
        executable="gpsr_voice_command",
        name="gpsr_voice_command",
        output="screen",
        condition=IfCondition(start_voice_command),
        parameters=[{
            "command_topic": command_topic,
            "scheduler_state_topic": "/gpsr/state",
            "command_result_topic": "/gpsr/command_result",
            # Section: Apply the bounded logical listening window without touching subscriptions.
            "listen_timeout_sec": ParameterValue(
                command_input_timeout_sec,
                value_type=float,
            ),
            "clarification_listen_timeout_sec": ParameterValue(
                clarification_listen_timeout_sec,
                value_type=float,
            ),
            # Section: Keep command-intake STT closed until TTS playback and echo guard finish.
            "post_tts_guard_ms": ParameterValue(post_tts_guard_ms, value_type=int),
        }],
    )

    # Section: Reuse the existing YuNet/SFace pipeline instead of starting a GPSR-specific detector.
    find_person_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("face_detection"), "launch", "appearance_pipeline.launch.py"]
            )
        ),
        condition=IfCondition(start_find_person_pipeline),
        launch_arguments={
            "image_topic": rgb_topic,
            "enable_depth": enable_depth,
            "depth_topic": depth_topic,
            "enable_profile_capture": "true",
            "enable_identification": "true",
            "enable_appearance_node": "false",
            "face_selection_mode": "tracked_people",
            "show_appearance_gui": show_gui,
        }.items(),
    )

    # Section: The opt-in master follower uses external robot drivers and explicit action enrollment.
    follow_person_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("person_follower_master"), "launch", "person_follow.launch.py"]
            )
        ),
        condition=IfCondition(start_follow_person_pipeline),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_external_drivers": "true",
            "auto_enroll": "false",
            "cmd_vel_topic": "/cmd_vel_follow",
            "rviz": "false",
            "publish_map_tf": "false",
        }.items(),
    )

    # Section: Lifecycle gating exposes the selected master action only for a prepared FOLLOW_PERSON step.
    follow_person_skill_server = LifecycleNode(
        package="person_follower_master",
        executable="person_follow_action_adapter.py",
        name="follow_person_skill_server",
        namespace="",
        output="screen",
        condition=IfCondition(start_follow_person_pipeline),
        parameters=[{
            # Section: Keep the managed provider stable while its client is selectable.
            "action_name": "/person_follow/master",
            "master_topic": "/master_target",
            "enroll_service": "/enroll_master",
            "reset_service": "/reset_master",
            "controller_parameter_service": "/follow_controller_node/set_parameters",
            "follow_cmd_topic": "/cmd_vel_follow",
            "output_cmd_topic": "/cmd_vel",
        }],
    )

    # Section: Optional people count reporter that speaks stable human quantities.
    count_people_blackboard_tts_node = Node(
        package="robot_yasmin_tasks",
        executable="count_people_blackboard_tts_node",
        name="count_people_blackboard_tts_node",
        output="screen",
        condition=IfCondition(enable_count_report_tts),
        parameters=[{
            "human_quantities_topic": "/perception/human_quantities",
            "tts_service_name": "/voice/tts/say",
            "speak_confirmed_only": True,
            "speak_empty_scene": False,
            "speak_on_change_only": True,
            "min_report_interval_sec": 5.0,
        }],
    )

    # Section: Optional object count reporter that speaks stable object quantities.
    count_objects_blackboard_tts_node = Node(
        package="robot_yasmin_tasks",
        executable="count_objects_blackboard_tts_node",
        name="count_objects_blackboard_tts_node",
        output="screen",
        condition=IfCondition(enable_count_report_tts),
        parameters=[{
            "object_quantities_topic": "/perception/object_quantities",
            "tts_service_name": "/voice/tts/say",
            "speak_confirmed_only": True,
            "speak_empty_scene": False,
            "speak_on_change_only": True,
            "min_report_interval_sec": 5.0,
            "report_style": "combined",
        }],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time."),
            DeclareLaunchArgument("dry_run", default_value="false", description="Print/publish the ordered plan without invoking hardware skills."),
            DeclareLaunchArgument("command_input_mode", default_value="topic", description="Input mode for commands: topic or action."),
            DeclareLaunchArgument("command_topic", default_value="/gpsr/command", description="Plain text GPSR command input topic."),
            DeclareLaunchArgument("schedule_command_service", default_value="/gpsr/schedule_command", description="Service endpoint for one-shot GPSR command scheduling."),
            DeclareLaunchArgument("reset_state_service", default_value="/gpsr/reset_state", description="Guarded service that re-arms recovery from scheduler ERROR."),
            DeclareLaunchArgument("skill_registry_file", default_value=default_skill_registry_file, description="Authoritative executable primitive capability registry."),
            DeclareLaunchArgument("find_person_search_config_file", default_value=default_find_person_search_config_file, description="Shared YAML waypoint policy for name and attribute person searches."),
            DeclareLaunchArgument("find_object_search_config_file", default_value=default_find_object_search_config_file, description="YAML waypoint and scan policy for object searches."),
            DeclareLaunchArgument("lifecycle_transition_timeout_sec", default_value="60.0", description="Absolute manager deadline for one lifecycle group operation."),
            DeclareLaunchArgument("lifecycle_operation_timeout_margin_sec", default_value="10.0", description="Executor wait margin beyond the manager lifecycle deadline."),
            DeclareLaunchArgument("startup_dependency_poll_sec", default_value="5.0", description="Bounded retry interval while the executor waits for lifecycle-manager services."),
            DeclareLaunchArgument("start_executor", default_value="true", description="Start the GPSR action queue executor."),
            DeclareLaunchArgument("start_count_people_pipeline", default_value="true", description="Instantiate UNCONFIGURED people-count lifecycle resources for on-demand activation."),
            DeclareLaunchArgument("start_count_objects_pipeline", default_value="true", description="Instantiate UNCONFIGURED object-count lifecycle resources for on-demand activation."),
            DeclareLaunchArgument("start_find_object_pipeline", default_value="true", description="Instantiate UNCONFIGURED FIND_OBJECT lifecycle resources for on-demand activation."),
            DeclareLaunchArgument("start_find_person_pipeline", default_value="true", description="Start shared YuNet/SFace identity, lookup, and enrollment interfaces."),
            DeclareLaunchArgument("start_find_person_attribute_pipeline", default_value="true", description="Instantiate UNCONFIGURED person-attribute lifecycle resources for on-demand activation."),
            DeclareLaunchArgument(
                "start_follow_person_pipeline",
                default_value="false",
                description="Opt in to person_follower_master and its GPSR PersonFollow adapter.",
            ),
            DeclareLaunchArgument("start_voice_tts", default_value="true", description="Start the local /voice/tts/say service."),
            DeclareLaunchArgument("start_voice_stt", default_value="true", description="Start local STT used by verbal FIND_PERSON confirmation."),
            DeclareLaunchArgument("start_voice_command", default_value="true", description="Start bounded spoken GPSR command reception at the current robot position."),
            DeclareLaunchArgument("command_input_timeout_sec", default_value="12.0", description="Seconds in each spoken GPSR command listening window before re-arming."),
            DeclareLaunchArgument("post_tts_guard_ms", default_value="400", description="Milliseconds to wait after TTS playback before GPSR starts STT."),
            # Section: Default GPSR to the installed Piper medium ONNX model.
            DeclareLaunchArgument("piper_model_path", default_value="~/piper_models/en_US-lessac-medium.onnx", description="Path to the Piper ONNX model used by GPSR TTS."),
            DeclareLaunchArgument("max_clarification_attempts", default_value="2", description="Maximum spoken clarification replies per incomplete GPSR command."),
            DeclareLaunchArgument("clarification_timeout_sec", default_value="40.0", description="Seconds allowed for each clarification TTS and STT exchange."),
            DeclareLaunchArgument("clarification_listen_timeout_sec", default_value="20.0", description="Seconds allowed for one request-scoped STT clarification result."),
            DeclareLaunchArgument("allow_simulated_primitives", default_value="false", description="Test-only opt-in for registry entries explicitly marked simulated."),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw", description="RGB image topic for GPSR perception detectors."),
            DeclareLaunchArgument("enable_depth", default_value="true", description="Enable aligned-depth localization and target approach; false permits RGB-only object and face searches."),
            DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw", description="Aligned depth topic for managed FIND_OBJECT."),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info", description="Color CameraInfo topic for managed FIND_OBJECT."),
            DeclareLaunchArgument("show_gui", default_value="false", description="Open OpenCV debug windows for GPSR detectors."),
            DeclareLaunchArgument("enable_count_report_tts", default_value="false", description="Speak stable count summaries from perception blackboard reporters."),
            DeclareLaunchArgument("yolo_model_path", default_value=default_yolo_model_path, description="YOLO model path used by GPSR object detectors."),
            DeclareLaunchArgument("human_yolo_model_path", default_value=default_human_yolo_model_path, description="YOLO model path used by GPSR human detectors."),
            DeclareLaunchArgument("confirmation_window_sec", default_value="2.0", description="Seconds of recent frames used to confirm detections."),
            DeclareLaunchArgument("confirmation_required_ratio", default_value="0.60", description="Minimum fraction of frames where a target must appear."),
            DeclareLaunchArgument("confirmation_confidence_threshold", default_value="0.35", description="Minimum detection confidence required for a frame vote."),
            DeclareLaunchArgument("confirmation_last_seen_timeout_sec", default_value="1.0", description="Maximum age since the target was last seen."),
            DeclareLaunchArgument("confirmation_min_frames", default_value="3", description="Minimum frames required before confirmation can pass."),
            DeclareLaunchArgument("instruction_point", default_value="instruction_point", description="Named pose used for mission-policy return."),
            DeclareLaunchArgument("return_after_completion", default_value="false", description="Append return-to-instruction-point after success."),
            DeclareLaunchArgument("return_after_failure", default_value="false", description="Reserved policy flag for failure recovery return."),
            DeclareLaunchArgument("max_commands", default_value="3", description="Maximum competition commands; zero runs indefinitely."),
            DeclareLaunchArgument("navigate_to_instruction_point_on_start", default_value="false", description="Opt in to named Instruction Point navigation before command intake."),
            DeclareLaunchArgument("return_target_mode", default_value="command_start_amcl", description="Return policy: command_start_amcl or named_location."),
            DeclareLaunchArgument("amcl_pose_topic", default_value="/amcl_pose", description="Localization pose captured when each command begins."),
            DeclareLaunchArgument("amcl_pose_max_age_sec", default_value="1.0", description="Maximum age of an AMCL observation accepted for command-start capture."),
            DeclareLaunchArgument("amcl_pose_capture_timeout_sec", default_value="3.0", description="Maximum wait for a fresh command-start AMCL pose."),
            DeclareLaunchArgument("max_find_history_records", default_value="32", description="Maximum compact entity observations retained in RAM."),
            DeclareLaunchArgument("return_fallback_named_location", default_value="instruction_point", description="Named later-return target used by explicit named mode or unavailable AMCL capture."),
            DeclareLaunchArgument("enable_error_recovery", default_value="true", description="Retry the frozen command-start return target after scheduler ERROR."),
            DeclareLaunchArgument("error_recovery_max_attempts", default_value="3", description="Maximum automatic recovery returns; zero retries indefinitely."),
            DeclareLaunchArgument("error_recovery_retry_delay_sec", default_value="5.0", description="Delay between failed automatic recovery returns."),
            DeclareLaunchArgument("executor_goal_response_timeout_sec", default_value="10.0", description="Seconds to wait for executor goal acceptance or rejection."),
            DeclareLaunchArgument("navigation_action", default_value="/nav/navigate_named_pose", description="Existing navigation skill action endpoint."),
            DeclareLaunchArgument("navigate_to_pose_action", default_value="/navigate_to_pose", description="Nav2 action used for captured raw-pose returns."),
            DeclareLaunchArgument("navigate_to_named_pose_action", default_value="/nav/navigate_named_pose", description="Named-pose action used for semantic navigation and fallback return."),
            DeclareLaunchArgument(
                "follow_person_action",
                default_value="/person_follow/master",
                description=(
                    "PersonFollow provider selected by the executor; use "
                    "/person_follow/control for legacy."
                ),
            ),
            DeclareLaunchArgument("find_object_action", default_value="/perception/find_object", description="Existing object perception action endpoint."),
            DeclareLaunchArgument("find_person_action", default_value="/perception/find_person", description="Existing person perception action endpoint."),
            DeclareLaunchArgument("tts_action", default_value="/voice/tts/say", description="Existing TTS service/action endpoint."),
            DeclareLaunchArgument("default_skill_timeout_sec", default_value="30.0", description="Default timeout assigned to generated skill steps."),
            DeclareLaunchArgument("default_skill_retries", default_value="0", description="Default retry count assigned to generated skill steps."),
            DeclareLaunchArgument("enable_navigation", default_value="true", description="Expose navigation dependency policy to scheduler."),
            DeclareLaunchArgument("enable_perception", default_value="true", description="Expose perception dependency policy to scheduler."),
            DeclareLaunchArgument("enable_manipulation", default_value="true", description="Expose manipulation dependency policy to scheduler."),
            DeclareLaunchArgument("enable_tts", default_value="true", description="Expose TTS dependency policy to scheduler."),
            DeclareLaunchArgument("enable_scheduler_viewer", default_value="true", description="Publish the GPSR scheduler cycle, including ERROR, to YASMIN Viewer."),
            scheduler_node,
            gpsr_guest_memory_node,
            gpsr_lifecycle_manager,
            gpsr_executor_node,
            fixed_information_node,
            count_people_skill_server,
            count_objects_skill_server,
            gpsr_human_detector,
            count_people_perception_server,
            gpsr_object_detector,
            count_objects_perception_server,
            human_skeleton_detector,
            gpsr_human_identity_node,
            gpsr_tracked_appearance_node,
            find_person_by_attribute_skill_server,
            find_person_skill_server,
            extract_person_info_skill_server,
            find_object_skill_server,
            guide_person_skill_server,
            voice_tts_node,
            voice_stt_node,
            gpsr_voice_command_node,
            find_person_pipeline,
            follow_person_pipeline,
            follow_person_skill_server,
            count_people_blackboard_tts_node,
            count_objects_blackboard_tts_node,
        ]
    )
