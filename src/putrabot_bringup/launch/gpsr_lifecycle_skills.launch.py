"""Section: Instantiate all GPSR transient skill nodes as UNCONFIGURED lifecycle nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start lifecycle-capable skill processes without activating them."""
    # Section: Shared persistent-camera input and optional debug display policy.
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    show_gui = LaunchConfiguration("show_gui")
    # Section: Use the requested workspace YOLOE segmentation weights for GPSR detection.
    yolo_model_path = "/home/yanjiun68/workspace/WS1/PutrabotUPM-project/yoloe-26s-seg.pt"
    # Section: Preserve the class-labelled YOLO11 model for human-only perception.
    human_yolo_model_path = PathJoinSubstitution(
        [FindPackageShare("vision"), "models", "yolo11n.pt"]
    )
    pose_model_path = PathJoinSubstitution(
        [FindPackageShare("vision"), "models", "pose_landmarker_full.task"]
    )

    # Section: Shared object detector used exclusively by FIND_OBJECT or COUNT_OBJECTS.
    object_detector = LifecycleNode(
        package="vision",
        executable="gpsr_object_detection_node.py",
        name="gpsr_object_detection_node",
        namespace="",
        output="screen",
        parameters=[{
            "rgb_topic": rgb_topic,
            "detections_topic": "/perception/raw_object_detections",
            "annotated_image_topic": "/perception/object_detections_image",
            # Section: GPSR object skills use Qwen world sensing through the existing detector contract.
            "inference_backend": "qwen",
            # Section: Active FIND_OBJECT and COUNT_OBJECTS goals supply semantic constraints through this shared topic.
            "qwen_query_topic": "/perception/object_detector_query",
            "yolo_model_path": yolo_model_path,
            "confidence_threshold": 0.35,
            "exclude_classes": ["person"],
            "show_gui": ParameterValue(show_gui, value_type=bool),
            "ready_topic": "/perception/object_detector_ready",
            "enable_model_warmup": True,
        }],
    )

    # Section: Compound FIND_OBJECT mission owns search decisions and transient clients.
    find_object_skill = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="find_object_skill_server",
        name="find_object_skill_server",
        namespace="",
        output="screen",
        parameters=[{
            "detections_topic": "/perception/raw_object_detections",
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "action_name": "/gpsr/find_object",
            "target_status_topic": "/gpsr/find_object/target_status",
            "detector_ready_topic": "/perception/object_detector_ready",
            # Section: Keep the FIND_OBJECT selector correlated with the Qwen detector query topic.
            "detector_query_topic": "/perception/object_detector_query",
        }],
    )

    # Section: Human-only detector remains separate but shares the vision_gpu conflict group.
    human_detector = LifecycleNode(
        package="vision",
        executable="gpsr_object_detection_node.py",
        name="gpsr_human_detection_node",
        namespace="",
        output="screen",
        parameters=[{
            "rgb_topic": rgb_topic,
            "detections_topic": "/perception/raw_human_detections",
            "annotated_image_topic": "/perception/human_detections_image",
            # Section: Human-only perception remains on the class-trained YOLO backend.
            "inference_backend": "yolo",
            "yolo_model_path": human_yolo_model_path,
            "confidence_threshold": 0.35,
            "include_classes": ["person"],
            "exclude_classes": ["__none__"],
            "show_gui": ParameterValue(show_gui, value_type=bool),
            "gui_window_name": "GPSR Human Detection",
        }],
    )

    # Section: Object count cache/service and GPSR-facing action bridge.
    count_objects_perception = LifecycleNode(
        package="vision",
        executable="count_objects_perception_server.py",
        name="count_objects_perception_server",
        namespace="",
        output="screen",
        parameters=[{
            "detections_topic": "/perception/raw_object_detections",
            "service_name": "/perception/count_objects",
            # Section: COUNT_OBJECTS directs the shared Qwen detector before taking its confirmed ROI count.
            "detector_query_topic": "/perception/object_detector_query",
            "qwen_query_wait_timeout_sec": 1.5,
        }],
    )
    count_objects_skill = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="count_objects_skill_server",
        name="count_objects_skill_server",
        namespace="",
        output="screen",
        parameters=[{
            "action_name": "/gpsr/count_objects",
            "perception_service": "/perception/count_objects",
        }],
    )

    # Section: Optional pose inference is configured only for COUNT_PEOPLE.
    human_pose = LifecycleNode(
        package="vision",
        executable="human_skeleton_detection_node.py",
        name="human_skeleton_detection_node",
        namespace="",
        output="screen",
        parameters=[{
            "image_topic": rgb_topic,
            "model_path": pose_model_path,
            "show_gui": ParameterValue(show_gui, value_type=bool),
            "enable_tts": False,
            "legacy_skeletons_topic": "/perception/human_skeletons",
        }],
    )

    # Section: People count cache/service and GPSR-facing action bridge.
    count_people_perception = LifecycleNode(
        package="vision",
        executable="count_people_perception_server.py",
        name="count_people_perception_server",
        namespace="",
        output="screen",
        parameters=[{
            "detections_topic": "/perception/raw_human_detections",
            "skeletons_topic": "/perception/human_skeletons",
            "service_name": "/perception/count_people",
            "enable_pose_sorting": True,
        }],
    )
    count_people_skill = LifecycleNode(
        package="robot_yasmin_tasks",
        executable="count_people_skill_server",
        name="count_people_skill_server",
        namespace="",
        output="screen",
        parameters=[{
            "action_name": "/gpsr/count_people",
            "perception_service": "/perception/count_people",
        }],
    )

    # Section: Launch creates processes only; the central manager owns all transitions.
    return LaunchDescription([
        DeclareLaunchArgument(
            "rgb_topic",
            default_value="/camera/image_raw",
            description="Persistent camera topic consumed only while a skill is ACTIVE.",
        ),
        DeclareLaunchArgument(
            "show_gui",
            default_value="false",
            description="Show debug windows only while the corresponding detector is ACTIVE.",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
            description="Aligned depth image used only while FIND_OBJECT is configured.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info",
            description="Color camera intrinsics used for target localization.",
        ),
        object_detector,
        find_object_skill,
        human_detector,
        count_objects_perception,
        count_objects_skill,
        human_pose,
        count_people_perception,
        count_people_skill,
    ])
