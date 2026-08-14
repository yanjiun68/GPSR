#!/usr/bin/env python3

"""Section: GPSR test umbrella launch for vision package sub-launch files."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _vision_launch(launch_file: str) -> PythonLaunchDescriptionSource:
    """Special function: Resolve one launch file from the installed vision package."""
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("vision"), "launch", launch_file])
    )


def generate_launch_description() -> LaunchDescription:
    """Special function: Include GPSR-related vision launch files for integrated testing."""
    # Section: Shared camera and UI arguments.
    rgb_topic = LaunchConfiguration("rgb_topic")
    camera_index = LaunchConfiguration("camera_index")
    show_gui = LaunchConfiguration("show_gui")
    start_voice_tts = LaunchConfiguration("start_voice_tts")

    # Section: Per-sublaunch enable switches.
    start_external_camera = LaunchConfiguration("start_external_camera")
    start_find_object = LaunchConfiguration("start_find_object")
    start_follow_bringup = LaunchConfiguration("start_follow_bringup")
    start_count_objects = LaunchConfiguration("start_count_objects")
    start_count_people = LaunchConfiguration("start_count_people")
    start_human_skeleton = LaunchConfiguration("start_human_skeleton")

    # Section: External camera adapter that can provide /camera/image_raw.
    external_camera_launch = IncludeLaunchDescription(
        _vision_launch("external_camera.launch.py"),
        condition=IfCondition(start_external_camera),
        launch_arguments={
            "image_topic": rgb_topic,
            "camera_index": camera_index,
        }.items(),
    )

    # Section: Find-object perception action server for GPSR object search tests.
    find_object_launch = IncludeLaunchDescription(
        _vision_launch("find_object_perception_server.launch.py"),
        condition=IfCondition(start_find_object),
        launch_arguments={
            "rgb_topic": rgb_topic,
            "show_gui": show_gui,
            "start_voice_tts": start_voice_tts,
            "start_object_detector": "true",
        }.items(),
    )

    # Section: Follow-person bringup for vision-assisted following tests.
    follow_bringup_launch = IncludeLaunchDescription(
        _vision_launch("follow_bringup.launch.py"),
        condition=IfCondition(start_follow_bringup),
    )

    # Section: GPSR object-count perception pipeline and action bridge.
    gpsr_count_objects_launch = IncludeLaunchDescription(
        _vision_launch("gpsr_count_objects_test.launch.py"),
        condition=IfCondition(start_count_objects),
        launch_arguments={
            "rgb_topic": rgb_topic,
            "show_gui": show_gui,
            "start_voice_tts": "false",
            "enable_count_report_tts": "true",
        }.items(),
    )

    # Section: GPSR people-count perception pipeline and action bridge.
    gpsr_count_people_launch = IncludeLaunchDescription(
        _vision_launch("gpsr_count_people_test.launch.py"),
        condition=IfCondition(start_count_people),
        launch_arguments={
            "rgb_topic": rgb_topic,
            "show_gui": show_gui,
            "start_voice_tts": "false",
            "enable_count_report_tts": "true",
            "start_pose_detector": "false",
        }.items(),
    )

    # Section: MediaPipe human skeleton, gestures, and pose debug pipeline.
    human_skeleton_launch = IncludeLaunchDescription(
        _vision_launch("human_skeleton_detection.launch.py"),
        condition=IfCondition(start_human_skeleton),
        launch_arguments={
            "image_topic": rgb_topic,
            "show_gui": show_gui,
            "start_voice_tts": "false",
        }.items(),
    )

    # Section: Return all launch arguments first, then included vision sub-launches.
    return LaunchDescription(
        [
            DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw", description="Shared RGB image topic for GPSR vision tests."),
            DeclareLaunchArgument("camera_index", default_value="0", description="OpenCV camera index used by the external USB camera."),
            DeclareLaunchArgument("show_gui", default_value="true", description="Show OpenCV debug GUI windows for included vision nodes."),
            DeclareLaunchArgument("start_voice_tts", default_value="true", description="Allow the find-object sub-launch to start local TTS."),
            DeclareLaunchArgument("start_external_camera", default_value="true", description="Include vision/external_camera.launch.py."),
            DeclareLaunchArgument("start_find_object", default_value="false", description="Opt in to the standalone legacy find-object test launch."),
            DeclareLaunchArgument("start_follow_bringup", default_value="false", description="Opt in to the standalone follow test launch."),
            DeclareLaunchArgument("start_count_objects", default_value="false", description="Opt in to the standalone count-object test launch."),
            DeclareLaunchArgument("start_count_people", default_value="false", description="Opt in to the standalone count-people test launch."),
            DeclareLaunchArgument("start_human_skeleton", default_value="false", description="Opt in to the standalone human-pose test launch."),
            external_camera_launch,
            find_object_launch,
            follow_bringup_launch,
            gpsr_count_objects_launch,
            gpsr_count_people_launch,
            human_skeleton_launch,
        ]
    )

#Launch camera specifically,
