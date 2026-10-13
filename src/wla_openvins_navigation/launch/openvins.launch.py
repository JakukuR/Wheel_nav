from pathlib import Path
import json
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(key): return LaunchConfiguration(key).perform(context)
    folder=Path(arg('config_dir')).expanduser()
    metadata=json.loads((folder/'calibration.json').read_text())
    commissioning=metadata.get('kind')=='commissioning'
    if commissioning and (arg('allow_commissioning')!='true' or metadata.get('calibration_validated') is not False):
        raise ValueError('Unvalidated commissioning config requires allow_commissioning:=true')
    if not commissioning and metadata.get('calibration_validated') is not True:
        raise ValueError('Calibration has not been validated')
    test=metadata.get('kind')=='synthetic'
    production=arg('production_topics')=='true'
    if test and (arg('allow_test_config')!='true' or production):
        raise ValueError('Synthetic config is permitted only on isolated trial topics')
    params=yaml.safe_load((folder/'frontend.yaml').read_text())['/**']['ros__parameters']
    params['config_path']=str((folder/'estimator_config.yaml').resolve())
    params.update(use_sim_time=arg('use_sim_time')=='true',publish_tf=production or arg('publish_tf')=='true',
                  odom_frame='d455_floor_odom' if production else 'openvins_trial_odom',
                  base_frame='r680_mapping_floor' if production else 'openvins_trial_base')
    ns='openvins' if production else 'openvins_trial'
    frontend=Node(package='wla_openvins_navigation',executable='openvins_frontend',namespace=ns,
                  output='screen',parameters=[params],
                  remappings=[('odom','/d455_slam/odom')] if production else [])
    gate=Node(package='wla_openvins_navigation',executable='quality_gate.py',namespace=ns,output='screen',
              parameters=[{'use_sim_time':params['use_sim_time'],'mode':arg('mode'),
                           'odom_topic':'/d455_slam/odom' if production else '/openvins_trial/odom',
                           'mapping_odom_topic':'/r680_nav/mapping_odom' if production else '/openvins_trial/mapping_odom',
                           'motion_output_enabled':not commissioning,
                           'contact_topic':arg('contact_topic')}])
    return [frontend,gate,RegisterEventHandler(OnProcessExit(target_action=frontend,
             on_exit=[EmitEvent(event=Shutdown(reason='OpenVINS frontend exited; explicit recovery required'))]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir'),
        DeclareLaunchArgument('production_topics',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('allow_test_config',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('allow_commissioning',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('publish_tf',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('use_sim_time',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('mode',default_value='mapping',choices=['mapping','localization']),
        DeclareLaunchArgument('contact_topic',default_value='/r680_nav/contact_state'),
        OpaqueFunction(function=setup)])
