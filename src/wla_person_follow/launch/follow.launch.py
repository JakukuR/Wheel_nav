from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = str(Path(get_package_share_directory('wla_person_follow')) / 'config' / 'follow.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('python', default_value='/home/orin/Wheel_Legged_Agent/.venv-perception/bin/python'),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('publish_mount_tf', default_value='false'),
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'),
                 '/home/orin/Wheel_Legged_Agent/scripts/publish_r680_d455_floor.py'],
            condition=IfCondition(LaunchConfiguration('publish_mount_tf')),
            output='screen'),
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'), '-m', 'wla_person_follow.node',
                 '--ros-args', '--params-file', LaunchConfiguration('config')],
            output='screen'),
    ])
