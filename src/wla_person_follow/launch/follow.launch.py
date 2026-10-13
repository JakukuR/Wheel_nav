import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = str(Path(get_package_share_directory('wla_person_follow'))/'config'/'follow.yaml')
    process = ExecuteProcess(cmd=[LaunchConfiguration('python'), '-m', 'wla_person_follow.node',
        '--ros-args', '--params-file', LaunchConfiguration('config'),
        '-p', ['enable_motion:=', LaunchConfiguration('enable_motion')]], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('python', default_value=os.environ.get('WLA_FOLLOW_PYTHON', '/usr/bin/python3')),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        process,
        RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='relative person service exited'))])),
    ])
