"""Versioned backend selection; simulation port starts in preview mode."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

TERRAIN_PREFIX='/r680/person_follow/terrain'


def terrain_nodes():
    """Follow's own slope/step analysis (wla_terrain_analysis) on follow-only topics,
    so it never collides with Nav2's terrain or depth_to_points outputs."""
    path=Path(get_package_share_directory('wla_terrain_analysis'))/'config'/'terrain_analysis.yaml'
    cfg=yaml.safe_load(path.read_text())
    odom=dict(cfg['terrain_odometry']['ros__parameters'],output_topic=TERRAIN_PREFIX+'/odom')
    analysis=dict(cfg['terrain_analysis']['ros__parameters'],odom_topic=TERRAIN_PREFIX+'/odom',
        output_topic=TERRAIN_PREFIX+'/points',safety_output_topic=TERRAIN_PREFIX+'/points_safety',
        clearing_output_topic=TERRAIN_PREFIX+'/clearing',terrain_map_topic=TERRAIN_PREFIX+'/map',
        blocked_cells_topic=TERRAIN_PREFIX+'/blocked_cells')
    return [Node(package='wla_terrain_analysis',executable='terrain_odometry',name='terrain_odometry_follow',
                 output='screen',parameters=[odom]),
            Node(package='wla_terrain_analysis',executable='terrain_analysis',name='terrain_analysis_follow',
                 output='screen',parameters=[analysis])]


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
    terrain=value('terrain')=='true'
    if terrain:cmd+=['-p','terrain_topic:='+TERRAIN_PREFIX+'/points']
    proc=ExecuteProcess(cmd=cmd,output='screen')
    actions=[proc,RegisterEventHandler(OnProcessExit(target_action=proc,on_exit=[EmitEvent(event=Shutdown(reason='person backend exited'))]))]
    if terrain:actions+=terrain_nodes()
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
        DeclareLaunchArgument('terrain',default_value='false',choices=['true','false']),
        OpaqueFunction(function=launch_backend)])
