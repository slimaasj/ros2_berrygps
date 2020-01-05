import os

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    this_prefix = get_package_share_directory('ros2_berrygps')

    gps_config_path = os.path.join(this_prefix, 'config', 'gps.yaml')
    imu_config_path = os.path.join(this_prefix, 'config', 'imu.yaml')


    return LaunchDescription([
        Node(
            package='ros2_berrygps',
            node_namespace='',
            node_name='imu_node',
            node_executable='imu_node',
            output='screen',
            parameters=[imu_config_path],
            remappings=None,
            arguments=[],
            name="",
            cwd=None,
            env=None
        ),
        Node(
            package='ros2_berrygps',
            node_namespace='',
            node_name='gps_node',
            node_executable='gps_node',
            output='screen',
            parameters=[gps_config_path],
            remappings=None,
            arguments=[],
            name="",
            cwd=None,
            env=None
        )
    ])

