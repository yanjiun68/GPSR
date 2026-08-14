from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

state_machine_arg = LaunchConfiguration("sm_launch")


def launch_state_machine(context, *args, **kwargs):
    state_machine = state_machine_arg.perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare(state_machine),
                    'launch',
                    f'{state_machine}.launch.py',
                ])
            ),
            launch_arguments={
                'enable_person_following': 'true'
            }.items(),
        )
    ]


def generate_launch_description():
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('putrabot_bringup'),
                'launch',
                'drivers_bringup.launch.py',
            ])
        ),
        launch_arguments={
            'enable_person_following': 'true'
        }.items(),
    )

    button_trigger = Node(
        package="startup",
        executable="button_trigger.py",
        name="button_trigger_node"
    )

    sequential_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=button_trigger,
            on_exit=[OpaqueFunction(function=launch_state_machine)]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "sm_launch",
            default_value="carry_my_luggage_phase1",
        ),
        driver_launch,
        sequential_handler,
        button_trigger,
    ])
