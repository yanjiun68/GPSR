from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Resolve the shared receptionist package path once so multiple nodes can reuse it.
    package_share = FindPackageShare("receptionist_nodes")

    # Resolve the dialogue parameter file path used by speech-related nodes.
    dialogue_config = PathJoinSubstitution([package_share, "config", "dialogue.yaml"])
    # Resolve the raw receptionist layout file path used by layout-driven nodes.
    layout_file = PathJoinSubstitution([package_share, "config", "receptionist_layout.yaml"])

    return LaunchDescription(
        [
            Node(
                package="face_detection",
                executable="face_perception",
                name="face_perception_node",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="face_detection",
                executable="appearance_receptionist",
                name="appearance_node",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="face_detection",
                executable="profile_capture_receptionist",
                name="profile_capture_node",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="face_detection",
                executable="profile_identification_receptionist",
                name="profile_identification_node",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="receptionist_nodes",
                executable="appearance_action_server",
                name="appearance_action_server",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="receptionist_nodes",
                executable="guest_memory_node",
                name="guest_memory_node",
                output="screen",
            ),
            Node(
                package="voice",
                executable="receptionist_node.py",
                name="dialogue_action_server",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="receptionist_nodes",
                executable="guest_profile_memory_bridge",
                name="guest_profile_memory_bridge",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="receptionist_nodes",
                executable="host_profile_memory_bridge",
                name="host_profile_memory_bridge",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="receptionist_nodes",
                executable="introduction_node",
                name="introduction_node",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="receptionist_nodes",
                executable="tts_bridge_node",
                name="tts_bridge_node",
                output="screen",
                parameters=[dialogue_config],
            ),
            Node(
                package="receptionist_nodes",
                executable="nav_controller_node",
                name="nav_controller_node",
                output="screen",
                parameters=[
                    {
                        "layout_file": layout_file,
                    }
                ],
            ),
            Node(
                package="receptionist_nodes",
                executable="seated_people_node",
                name="seated_people_node",
                output="screen",
                parameters=[
                    {
                        "layout_file": layout_file,
                    }
                ],
            ),
            Node(
                package="receptionist_nodes",
                executable="chair_manager_node",
                name="chair_manager_node",
                output="screen",
                parameters=[
                    {
                        "layout_file": layout_file,
                    }
                ],
            ),
            Node(
                package="receptionist_nodes",
                executable="seat_pointing_node",
                name="seat_pointing_node",
                output="screen",
                parameters=[
                    {
                        "layout_file": layout_file,
                    }
                ],
            ),
        ]
    )
