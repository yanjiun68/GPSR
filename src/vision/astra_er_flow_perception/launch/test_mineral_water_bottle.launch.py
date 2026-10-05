from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="astra_er_flow_perception",
            executable="test_mineral_water_bottle",
            name="mineral_water_bottle_integration_test",
            output="screen",
            parameters=[{
                "target": "mineral water bottle",
                "prompt": (
                    "Locate and track the visible mineral water bottle. "
                    "Prefer a transparent or clear plastic drinking-water bottle "
                    "with a bottle cap. Ignore cups, cans, thermos flasks, "
                    "and other containers."
                ),
                "timeout_sec": 30.0,
                "min_tracking_frames": 10,
                "min_valid_depth_frames": 5,
                "min_depth_m": 0.20,
                "max_depth_m": 6.00,
                "min_roi_width": 10,
                "min_roi_height": 20,
            }],
        )
    ])
