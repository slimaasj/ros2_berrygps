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

    berrygps_config_path = os.path.join(this_prefix, 'config.yaml')


    return LaunchDescription([
        Node(
            package='ros2_berrygps',
            node_namespace='',
            node_name='berrygps_node',
            node_executable='berry_gps',
            output='screen',
            parameters=[berrygps_config_path],
            remappings=None,
            arguments=[],
            name="",
            cwd=None,
            env=None
        )
    ])
