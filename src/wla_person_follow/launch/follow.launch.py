"""Versioned backend selection; simulation port starts in preview mode."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def launch_backend(context):
    value=lambda name:LaunchConfiguration(name).perform(context)
    config=value('config') or str(Path(get_package_share_directory('wla_person_follow'))/'config'/'sim_follow.yaml')
    backend=value('transport')
    if backend not in ('cpp','python'):
        raise ValueError('transport must be cpp or python')
    if backend=='cpp':
        executable=str(Path(get_package_prefix('wla_follow_transport'))/'lib/wla_follow_transport/follow_transport')
        cmd=[executable]
    else:
        cmd=[value('python'),'-m','wla_person_follow.sim_node']
    cmd+=['--ros-args','--params-file',config,'-p','enable_motion:='+value('enable_motion')]
    if backend=='cpp':cmd+=['-p','worker_python:='+value('python')]
    cmd+=['-p','output_mode:='+value('output_mode'),'-p','direct_topic:='+value('direct_topic'),'-p','reid_enabled:='+value('enable_reid')]
    proc=ExecuteProcess(cmd=cmd,output='screen')
    actions=[proc,RegisterEventHandler(OnProcessExit(target_action=proc,on_exit=[EmitEvent(event=Shutdown(reason='person backend exited'))]))]
    actions.append(ExecuteProcess(cmd=[value('python'),'-m','wla_person_follow.status_recorder',
        '--directory',value('diagnostic_log_directory')],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('diagnostic_log_directory',default_value=str(Path.home()/'ros2_ws/.runtime/follow-diagnostics')),
        DeclareLaunchArgument('transport',default_value='cpp'),
        DeclareLaunchArgument('python',default_value=os.environ.get('WLA_FOLLOW_PYTHON','/usr/bin/python3')),
        DeclareLaunchArgument('config',default_value=''),
        DeclareLaunchArgument('enable_motion',default_value='false'),
        DeclareLaunchArgument('enable_reid',default_value='true'),
        DeclareLaunchArgument('output_mode',default_value='mux'),
        DeclareLaunchArgument('direct_topic',default_value='/r680_base/cmd_vel'),
        OpaqueFunction(function=launch_backend)])
