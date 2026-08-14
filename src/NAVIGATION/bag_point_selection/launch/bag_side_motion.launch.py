"""
launch: 原识别/聚类/融合/动作/语音/可视化全部保留, 只把原运动控制器
bag_approach_controller_node 替换为 side_motion_executor_node
(按左/右自动执行 横移1.5s + 前进5s, 总6.5s).

额外:
    默认开启手势指向可视化, 方便观察 host 正在指向哪一个 bag.
    bag_selection_viz_node 会优先显示 pose_pointing_node 的调试图像
    (手部关键点 + 指向射线 + pointed bag 标注).

和原 launch 相比的变化:
    去掉: bag_approach_controller_node   (会往 /cmd_vel 发速度, 和本程序冲突)
    去掉: final_alignment_node           (用户决定不需要)
    新增: side_motion_executor_node      (本程序)

启动:
    ros2 launch bag_point_selection bag_side_motion.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    observe_pointing = LaunchConfiguration("observe_pointing")
    show_debug_window = LaunchConfiguration("show_debug_window")

    common = [{"use_sim_time": use_sim_time}, params_file]

    nodes = [
        "bag_image_preprocessor_node",
        "pose_pointing_node",
        "bag_detector_node",
        "selection_manager_node",
        "selected_bag_cloud_cluster_node",
        "lidar_cluster_node",
        "target_fusion_node",
        "bag_selection_action_server",
        "bag_selection_voice_node",
        "bag_selection_viz_node",
        "side_motion_executor_node",
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
            DeclareLaunchArgument("observe_pointing", default_value="true"),
            DeclareLaunchArgument("show_debug_window", default_value="true"),
            *[
                Node(
                    package="bag_point_selection",
                    executable=name,
                    name=name,
                    output="screen",
                    parameters=(
                        common
                        + (
                            [
                                {
                                    "pose_debug_enabled": observe_pointing,
                                    "pose_debug_publish_image": observe_pointing,
                                    "pose_debug_show_window": "false",
                                }
                            ]
                            if name == "pose_pointing_node"
                            else []
                        )
                        + (
                            [
                                {
                                    "bag_viz_enabled": observe_pointing,
                                    "bag_viz_use_pose_debug": observe_pointing,
                                    "bag_viz_show_window": show_debug_window,
                                }
                            ]
                            if name == "bag_selection_viz_node"
                            else []
                        )
                    ),
                )
                for name in nodes
            ],
        ]
    )
