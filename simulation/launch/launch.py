import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    pkg_name = 'my_package'
    file_subpath = f'urdf/{robot_name}/robot.urdf.xacro'

    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,              # ✅ ICI
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'publish_frequency': 50.0,
            'use_sim_time': True,          # (recommandé en sim)
            'frame_prefix': f'{robot_name}/',
        }]
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='default_robot'),
        OpaqueFunction(function=launch_setup)
    ])
