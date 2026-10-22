"""Versioned backend selection; simulation port starts in preview mode."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def launch_backend(context):
    value=lambda name:LaunchConfiguration(name).perform(context)
    backend=value('backend')
    if backend not in ('sim','legacy'):raise ValueError('backend must be sim or legacy')
    config=value('config') or str(Path(get_package_share_directory('wla_person_follow'))/'config'/('sim_follow.yaml' if backend=='sim' else 'follow.yaml'))
    module='wla_person_follow.sim_node' if backend=='sim' else 'wla_person_follow.node'
    cmd=[value('python'),'-m',module,'--ros-args','--params-file',config,'-p','enable_motion:='+value('enable_motion')]
    if backend=='sim':cmd+=['-p','output_mode:='+value('output_mode'),'-p','direct_topic:='+value('direct_topic'),'-p','reid_enabled:='+value('enable_reid')]
    proc=ExecuteProcess(cmd=cmd,output='screen')
    actions=[proc,RegisterEventHandler(OnProcessExit(target_action=proc,on_exit=[EmitEvent(event=Shutdown(reason='person backend exited'))]))]
    if backend=='sim':
        actions.append(ExecuteProcess(cmd=[value('python'),'-m','wla_person_follow.status_recorder',
            '--directory',value('diagnostic_log_directory')],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend',default_value='sim'),
        DeclareLaunchArgument('diagnostic_log_directory',default_value=str(Path.home()/'ros2_ws/.runtime/follow-diagnostics')),
        DeclareLaunchArgument('python',default_value=os.environ.get('WLA_FOLLOW_PYTHON','/usr/bin/python3')),
        DeclareLaunchArgument('config',default_value=''),
        DeclareLaunchArgument('enable_motion',default_value='false'),
        DeclareLaunchArgument('enable_reid',default_value='false'),
        DeclareLaunchArgument('output_mode',default_value='mux'),
        DeclareLaunchArgument('direct_topic',default_value='/r680_base/cmd_vel'),
        OpaqueFunction(function=launch_backend)])
