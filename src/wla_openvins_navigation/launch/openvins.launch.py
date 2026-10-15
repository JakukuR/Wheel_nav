from pathlib import Path
import json
import os
from ament_index_python.packages import get_package_prefix
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
    # The estimator and sensor callbacks are upstream OpenVINS. The private
    # cv_bridge build uses the same OpenCV as OpenVINS on this Jetson; its
    # library path is scoped to this process, not the rest of the ROS graph.
    prefix = Path(get_package_prefix('ov_msckf'))
    dependency_lib = prefix.parents[1] / '.runtime/openvins_official/cv_bridge_install/cv_bridge/lib'
    environment = {}
    if dependency_lib.is_dir():
        environment['LD_LIBRARY_PATH'] = str(dependency_lib) + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    frontend=Node(package='ov_msckf',executable='run_subscribe_msckf',namespace=ns,
        output='screen',additional_env=environment,parameters=[{
            'config_path': params['config_path'], 'use_sim_time': params['use_sim_time'],
            'publish_global_to_imu_tf': False, 'publish_calibration_tf': False}])
    adapter=Node(package='wla_openvins_navigation',executable='odometry_adapter',namespace=ns,
        output='screen',parameters=[{key:params[key] for key in (
            'use_sim_time','publish_tf','odom_frame','base_frame',
            'p_base_in_imu','q_base_to_imu_xyzw')}],
        remappings=[('odom','/d455_slam/odom')] if production else [])
    return [frontend,adapter,RegisterEventHandler(OnProcessExit(target_action=frontend,
             on_exit=[EmitEvent(event=Shutdown(reason='Official OpenVINS process exited'))]))]



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
