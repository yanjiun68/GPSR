from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_stable_item_memory = LaunchConfiguration("enable_stable_item_memory")

    common = [{"use_sim_time": use_sim_time}, params_file]

    nodes = [
        "bag_image_preprocessor_node",
        "pose_pointing_node",
        "bag_detector_node",
        "selection_manager_node",
        "selected_bag_cloud_cluster_node",
        "lidar_cluster_node",
        "target_fusion_node",
        "bag_approach_controller_node",
        "final_alignment_node",
        "bag_selection_action_server",
        "bag_selection_voice_node",
        "bag_selection_viz_node",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("bag_point_selection"), "config", "params.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "enable_stable_item_memory",
                default_value="false",
                description="Enable optional stable object-name memory output",
            ),
            *[
                Node(
                    package="bag_point_selection",
                    executable=name,
                    name=name,
                    output="screen",
                    parameters=common,
                )
                for name in nodes
            ],
            Node(
                package="vision",
                executable="perception_memory_node.py",
                name="perception_memory_node",
                output="screen",
                condition=IfCondition(enable_stable_item_memory),
                parameters=common,
            ),
        ]
    )
