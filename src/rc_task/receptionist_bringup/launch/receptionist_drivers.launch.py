#!/usr/bin/env python3

# Section: standard path helper used to locate the existing robot bringup launch.
import os

# Section: package-index helper used to resolve installed ROS package shares.
from ament_index_python.packages import get_package_share_directory

# Section: launch primitives used to declare operator-facing arguments.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Special function: generate the external driver launch graph for receptionist operation.
def generate_launch_description():
    # Section: resolve launch arguments that operators may override at runtime.
    serial_port = LaunchConfiguration("serial_port")
    enable_robot_vision = LaunchConfiguration("enable_robot_vision")
    enable_person_following = LaunchConfiguration("enable_person_following")
    enable_arm_lift_drivers = LaunchConfiguration("enable_arm_lift_drivers")

    # Section: locate the shared Putrabot driver bringup package.
    putrabot_bringup_share = get_package_share_directory("putrabot_bringup")

    # Section: include the existing hardware bringup so this package stays a thin task-specific wrapper.
    external_drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                putrabot_bringup_share,
                "launch",
                "drivers_bringup.launch.py",
            )
        ),
        launch_arguments={
            "serial_port": serial_port,
            "enable_robot_vision": enable_robot_vision,
            "enable_person_following": enable_person_following,
            "enable_arm_lift_drivers": enable_arm_lift_drivers,
        }.items(),
    )

    # Section: return all declarations and actions needed for external receptionist drivers.
    return LaunchDescription(
        [
            # Section: lidar serial argument forwarded to my_robot_nav/drivers_only.launch.py.
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
                description="Serial port for the lidar used by the receptionist navigation stack.",
            ),
            # Section: optional navigation/front RealSense camera toggle.
            DeclareLaunchArgument(
                "enable_robot_vision",
                default_value="false",
                description="Start the front/navigation RealSense camera in addition to the receptionist RX200 camera.",
            ),
            # Section: person-following mode stays disabled for receptionist operation by default.
            DeclareLaunchArgument(
                "enable_person_following",
                default_value="false",
                description="Keep false for receptionist; true switches the shared bringup into person-following mode.",
            ),
            # Section: arm and lift drivers stay enabled because the shared bringup owns RX200/lift startup.
            DeclareLaunchArgument(
                "enable_arm_lift_drivers",
                default_value="false",
                description="Start RX200/lift hardware drivers used by the receptionist camera and arm platform.",
            ),
            # Section: operator-visible reminder that this launch intentionally excludes task and Nav2 ownership.
            LogInfo(
                msg=(
                    "[receptionist_bringup] Starting external hardware drivers only. "
                    "Launch sm_receptionist/task_receptionist.launch.py separately for Nav2 and task flow."
                )
            ),
            external_drivers,
        ]
    )
