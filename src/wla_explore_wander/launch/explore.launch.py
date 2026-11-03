"""Exploration loop only; the follow backend (r680_person_follow.sh) must already run."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = ('session_budget_s', 'boundary_radius_m', 'seed')
    defaults = dict(session_budget_s='600.0', boundary_radius_m='4.0', seed='-1')
    return LaunchDescription([DeclareLaunchArgument(a, default_value=defaults[a]) for a in args]+[
        Node(package='wla_explore_wander', executable='explore_wander', name='explore_wander', output='screen',
             parameters=[{a: LaunchConfiguration(a) for a in args}])])
