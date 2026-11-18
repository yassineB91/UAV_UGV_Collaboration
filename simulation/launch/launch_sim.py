import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    package_name = 'my_package'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'launch.py'
            )
        ),
        launch_arguments={'robot_name': robot_name}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', f'/{robot_name}/robot_description',
            '-entity', f'{robot_name}',
            '-x', '0.0',
            '-y', '-4.0',
            '-z', '0.1'
        ],
        output='screen'
    )


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "-c", f"/{robot_name}/controller_manager"],
        output="screen",
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "-c", f"/{robot_name}/controller_manager"],
        output="screen",
    )

    # ✅ Les spawners ne démarrent qu'une fois le spawn_entity terminé
    start_spawners_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner, diff_drive_spawner],
        )
    )

    return [rsp, spawn_entity, start_spawners_after_spawn]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='default_robot'),
        OpaqueFunction(function=launch_setup)
    ])
