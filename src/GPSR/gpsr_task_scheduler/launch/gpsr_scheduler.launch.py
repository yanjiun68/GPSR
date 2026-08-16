"""Section: Umbrella launch for deterministic GPSR task scheduler."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, OrSubstitution
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

    # Section: Optional integrated runtime nodes.
    start_executor = LaunchConfiguration("start_executor")
    start_count_people_pipeline = LaunchConfiguration("start_count_people_pipeline")
    start_count_objects_pipeline = LaunchConfiguration("start_count_objects_pipeline")
    start_find_object_pipeline = LaunchConfiguration("start_find_object_pipeline")
    start_find_person_pipeline = LaunchConfiguration("start_find_person_pipeline")
    start_find_person_attribute_pipeline = LaunchConfiguration("start_find_person_attribute_pipeline")
    start_voice_tts = LaunchConfiguration("start_voice_tts")
    start_voice_stt = LaunchConfiguration("start_voice_stt")
    allow_simulated_primitives = LaunchConfiguration("allow_simulated_primitives")

    # Section: Shared perception launch-time controls.
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    show_gui = LaunchConfiguration("show_gui")
    enable_count_report_tts = LaunchConfiguration("enable_count_report_tts")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    default_yolo_model_path = PathJoinSubstitution([FindPackageShare("vision"), "models", "yolo11n.pt"])
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
    enable_error_recovery = LaunchConfiguration("enable_error_recovery")
    error_recovery_max_attempts = LaunchConfiguration("error_recovery_max_attempts")
    error_recovery_retry_delay_sec = LaunchConfiguration("error_recovery_retry_delay_sec")
    executor_goal_response_timeout_sec = LaunchConfiguration("executor_goal_response_timeout_sec")
    navigation_action = LaunchConfiguration("navigation_action")
    find_object_action = LaunchConfiguration("find_object_action")
    find_person_action = LaunchConfiguration("find_person_action")
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
                "enable_navigation": enable_navigation,
                "enable_perception": enable_perception,
                "enable_manipulation": enable_manipulation,
                "enable_tts": enable_tts,
                "enable_scheduler_viewer": ParameterValue(enable_scheduler_viewer, value_type=bool),
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
        }],
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
        }],
    )

    # Section: Persistent GPSR-only guest memory remains available across all ActionSteps.
    gpsr_guest_memory_node = Node(
        package="robot_yasmin_tasks",
        executable="gpsr_guest_memory_node",
        name="gpsr_guest_memory_node",
        output="screen",
        condition=IfCondition(start_executor),
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
            "yolo_model_path": yolo_model_path,
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
            "yolo_model_path": yolo_model_path,
            "confidence_threshold": 0.35,
            "exclude_classes": ["person"],
            "publish_annotated_image": True,
            "show_gui": show_gui,
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
        condition=IfCondition(start_shared_human_pose),
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
        condition=IfCondition(start_find_person_attribute_pipeline),
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
        condition=IfCondition(start_find_person_attribute_pipeline),
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
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "action_name": "/gpsr/find_object",
            "target_status_topic": "/gpsr/find_object/target_status",
        }],
    )

    # Section: Optional local TTS service used by SAY/report skills.
    voice_tts_node = Node(
        package="voice",
        executable="tts_node.py",
        name="voice_tts",
        output="screen",
        condition=IfCondition(start_voice_tts),
    )

    # Section: Local STT supplies deterministic yes/no transcripts for verbal identity mode.
    voice_stt_node = Node(
        package="voice",
        executable="stt_node.py",
        name="voice_stt",
        output="screen",
        condition=IfCondition(start_voice_stt),
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
            "enable_depth": "false",
            "enable_profile_capture": "true",
            "enable_identification": "true",
            "enable_appearance_node": "false",
            "show_appearance_gui": show_gui,
        }.items(),
    )

    # Section: Identity availability marker makes the name pipeline switch lifecycle-visible.
    find_person_identity_resource = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="skill_availability_resource",
        name="find_person_identity_resource",
        namespace="",
        output="screen",
        condition=IfCondition(start_find_person_pipeline),
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
            DeclareLaunchArgument("dry_run", default_value="true", description="Print/publish the ordered plan without invoking hardware skills."),
            DeclareLaunchArgument("command_input_mode", default_value="topic", description="Input mode for commands: topic or action."),
            DeclareLaunchArgument("command_topic", default_value="/gpsr/command", description="Plain text GPSR command input topic."),
            DeclareLaunchArgument("schedule_command_service", default_value="/gpsr/schedule_command", description="Service endpoint for one-shot GPSR command scheduling."),
            DeclareLaunchArgument("reset_state_service", default_value="/gpsr/reset_state", description="Guarded service that re-arms recovery from scheduler ERROR."),
            DeclareLaunchArgument("skill_registry_file", default_value=default_skill_registry_file, description="Authoritative executable primitive capability registry."),
            DeclareLaunchArgument("start_executor", default_value="true", description="Start the GPSR action queue executor."),
            DeclareLaunchArgument("start_count_people_pipeline", default_value="true", description="Start people detection, people count service, and people count action bridge."),
            DeclareLaunchArgument("start_count_objects_pipeline", default_value="true", description="Start object detection, object count service, and object count action bridge."),
            DeclareLaunchArgument("start_find_object_pipeline", default_value="true", description="Start the compound FIND_OBJECT server and its shared object detector."),
            DeclareLaunchArgument("start_find_person_pipeline", default_value="true", description="Start shared YuNet/SFace identity, lookup, and enrollment interfaces."),
            DeclareLaunchArgument("start_find_person_attribute_pipeline", default_value="true", description="Start real human pose resources for FIND_PERSON_BY_ATTRIBUTE."),
            DeclareLaunchArgument("start_voice_tts", default_value="true", description="Start the local /voice/tts/say service."),
            DeclareLaunchArgument("start_voice_stt", default_value="true", description="Start local STT used by verbal FIND_PERSON confirmation."),
            DeclareLaunchArgument("allow_simulated_primitives", default_value="false", description="Test-only opt-in for registry entries explicitly marked simulated."),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw", description="RGB image topic for GPSR perception detectors."),
            DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw", description="Aligned depth topic for managed FIND_OBJECT."),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info", description="Color CameraInfo topic for managed FIND_OBJECT."),
            DeclareLaunchArgument("show_gui", default_value="false", description="Open OpenCV debug windows for GPSR detectors."),
            DeclareLaunchArgument("enable_count_report_tts", default_value="false", description="Speak stable count summaries from perception blackboard reporters."),
            DeclareLaunchArgument("yolo_model_path", default_value=default_yolo_model_path, description="YOLO model path used by GPSR object/person detectors."),
            DeclareLaunchArgument("confirmation_window_sec", default_value="2.0", description="Seconds of recent frames used to confirm detections."),
            DeclareLaunchArgument("confirmation_required_ratio", default_value="0.60", description="Minimum fraction of frames where a target must appear."),
            DeclareLaunchArgument("confirmation_confidence_threshold", default_value="0.35", description="Minimum detection confidence required for a frame vote."),
            DeclareLaunchArgument("confirmation_last_seen_timeout_sec", default_value="1.0", description="Maximum age since the target was last seen."),
            DeclareLaunchArgument("confirmation_min_frames", default_value="3", description="Minimum frames required before confirmation can pass."),
            DeclareLaunchArgument("instruction_point", default_value="instruction_point", description="Named pose used for mission-policy return."),
            DeclareLaunchArgument("return_after_completion", default_value="false", description="Append return-to-instruction-point after success."),
            DeclareLaunchArgument("return_after_failure", default_value="false", description="Reserved policy flag for failure recovery return."),
            DeclareLaunchArgument("max_commands", default_value="3", description="Maximum competition commands; zero runs indefinitely."),
            DeclareLaunchArgument("navigate_to_instruction_point_on_start", default_value="true", description="Navigate to the Instruction Point before command intake."),
            DeclareLaunchArgument("enable_error_recovery", default_value="true", description="Retry returning to the Instruction Point after scheduler ERROR."),
            DeclareLaunchArgument("error_recovery_max_attempts", default_value="3", description="Maximum automatic recovery returns; zero retries indefinitely."),
            DeclareLaunchArgument("error_recovery_retry_delay_sec", default_value="5.0", description="Delay between failed automatic recovery returns."),
            DeclareLaunchArgument("executor_goal_response_timeout_sec", default_value="10.0", description="Seconds to wait for executor goal acceptance or rejection."),
            DeclareLaunchArgument("navigation_action", default_value="/nav/navigate_named_pose", description="Existing navigation skill action endpoint."),
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
            extract_person_info_skill_server,
            find_object_skill_server,
            voice_tts_node,
            voice_stt_node,
            find_person_pipeline,
            find_person_identity_resource,
            count_people_blackboard_tts_node,
            count_objects_blackboard_tts_node,
        ]
    )
