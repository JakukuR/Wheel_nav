from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='wla_behavior_bridge',
        executable='agent_person_api', output='screen')])
