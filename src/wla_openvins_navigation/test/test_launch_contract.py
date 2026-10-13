#!/usr/bin/env python3
"""Evaluate launch substitutions and calibration rejection without starting nodes."""
import importlib.util
import pathlib
import sys
import json
import tempfile
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters


root=pathlib.Path(sys.argv[1]).resolve()
spec=importlib.util.spec_from_file_location('bringup',root.parent/'wla_r680_navigation/launch/bringup.launch.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
ld=mod.generate_launch_description();c=LaunchContext()
for e in ld.entities:
    if isinstance(e,DeclareLaunchArgument) and e.default_value is not None:
        c.launch_configurations[e.name]=perform_substitutions(c,e.default_value)
c.launch_configurations.update(odom_source='openvins',openvins_config_dir='/measured/config',
                              start_nav2='false',start_web='false',mode='localization')
def enabled(e):return e.condition is None or e.condition.evaluate(c)
def node_name(e):return ' '.join(str(x) for x in getattr(e,'cmd',[]))
for e in ld.entities:
    text=node_name(e)
    if any(x in text for x in ('cuvslam_odometry','degraded_odometry','vo_watchdog','ekf_node','rgbd_odometry','mapping_odom_gate')):
        assert not enabled(e),text
monitor=next(e for e in ld.entities if 'interface_monitor' in node_name(e))
params=evaluate_parameters(c,monitor._Node__parameters)[0]
assert params['require_vio_health'] is True
assert params['vio_health_topic']=='/openvins/tracking_healthy'
assert params['require_motion_permission'] is True
assert params['motion_permission_topic']=='/openvins/motion_ready'
assert params['require_vo_watchdog'] is False
includes=[e for e in ld.entities if isinstance(e,IncludeLaunchDescription)]
# Shared camera launch is wrapped to scope the RealSense SDK environment.
# Inspect its argument without executing or starting the camera launch.
for e in ld.entities:
    includes.extend(x for x in getattr(e,'_OpaqueFunction__args',[]) if isinstance(x,IncludeLaunchDescription))
camera=next(e for e in includes if 'enable_infra1' in dict(e.launch_arguments))
camera_args=dict(camera.launch_arguments)
assert perform_substitutions(c,[camera_args['enable_infra1']])=='true'
assert perform_substitutions(c,[camera_args['enable_infra2']])=='true'
assert perform_substitutions(c,[camera_args['config_file']]).endswith('/d455_cuvslam.yaml')
rtab=next(e for e in includes if 'odom_topic' in dict(e.launch_arguments))
args=dict(rtab.launch_arguments)
assert perform_substitutions(c,[args['odom_topic']])=='/r680_nav/mapping_odom'
assert perform_substitutions(c,[args['imu_topic']])=='/r680_nav/disabled_rtabmap_imu'
mod.validate_navigation_map(c)
c.launch_configurations['enable_hardware_output']='true'
try:mod.validate_navigation_map(c)
except ValueError:pass
else:raise AssertionError('experimental hardware output must be rejected')
print('PASS OpenVINS exclusive odom, no plane fallback/restart watchdog, quality gate and hardware-output rejection')

for frontend in ('rgbd','cuvslam'):
    for mode in ('mapping','localization'):
        c.launch_configurations.update(odom_source=frontend,mode=mode,enable_hardware_output='false')
        # ParameterValue caches evaluation; a new launch invocation owns new
        # objects. Reusing the OpenVINS invocation would test cached values.
        case=mod.generate_launch_description()
        watchdog=next(e for e in case.entities if 'vo_watchdog' in node_name(e))
        monitor=next(e for e in case.entities if 'interface_monitor' in node_name(e))
        assert enabled(watchdog)==(mode=='localization')
        wp=evaluate_parameters(c,watchdog._Node__parameters)[1]
        mp=evaluate_parameters(c,monitor._Node__parameters)[0]
        assert wp['frontend']==frontend
        assert wp['frontend_health_topic']=='/r680_nav/vio_tracking_healthy'
        assert wp['allow_bounded_degraded']==(frontend=='cuvslam' and mode=='localization')
        assert mp['require_vio_health']==(frontend=='cuvslam')
        assert mp['require_motion_permission']==(frontend=='cuvslam')
        assert mp['vio_health_topic']=='/r680_nav/vio_tracking_healthy'
print('PASS RGBD/cuVSLAM frontend selection and explicit continuity wiring')

spec=importlib.util.spec_from_file_location('openvins_launch',root/'launch/openvins.launch.py')
frontend_launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(frontend_launch)
with tempfile.TemporaryDirectory() as d:
    folder=pathlib.Path(d)
    (folder/'frontend.yaml').write_bytes((root/'config/synthetic/frontend.yaml').read_bytes())
    (folder/'calibration.json').write_text(json.dumps({'kind':'commissioning','calibration_validated':False}))
    context=LaunchContext();context.launch_configurations.update(config_dir=d,production_topics='true',
        allow_test_config='false',allow_commissioning='false',publish_tf='false',use_sim_time='false',
        mode='mapping',contact_topic='/test/contact')
    try:frontend_launch.setup(context)
    except ValueError:pass
    else:raise AssertionError('unvalidated config needs explicit opt-in')
    context.launch_configurations['allow_commissioning']='true'
    nodes=frontend_launch.setup(context)
    assert evaluate_parameters(context,nodes[1]._Node__parameters)[0]['motion_output_enabled'] is False
print('PASS commissioning opt-in preserves unvalidated status and disables motion permission')
