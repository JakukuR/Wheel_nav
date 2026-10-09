#!/usr/bin/env python3
"""Gamepad Teleop Launch File"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('gamepad_control')
    params_file = os.path.join(pkg_dir, 'config', 'gamepad_params.yaml')

    gamepad_node = Node(
        package='gamepad_control',
        executable='gamepad_teleop',
        name='gamepad_teleop',
        output='screen',
        parameters=[params_file],
        # 若使用命名空间，可在此设置
        # namespace='robot',
    )

    return LaunchDescription([
        gamepad_node,
    ])
