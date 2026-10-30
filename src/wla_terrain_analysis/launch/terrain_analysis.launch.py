from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('wla_terrain_analysis')) / 'config' / 'terrain_analysis.yaml')
    return LaunchDescription([
        Node(package='wla_terrain_analysis', executable='terrain_odometry', name='terrain_odometry',
             output='screen', parameters=[config]),
        Node(package='wla_terrain_analysis', executable='terrain_analysis', name='terrain_analysis',
             output='screen', parameters=[config]),
    ])
