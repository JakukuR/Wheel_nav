#!/usr/bin/env python3
"""Resolve actual launch routing without starting nodes or accessing sensors."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('bringup', root/'launch/bringup.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

def node(executable):
    return next(n for n in nodes if any(executable in str(x) for x in n.cmd))

def overrides(executable, context):
    return evaluate_parameters(context, [node(executable)._Node__parameters[-1]])[0]

for enabled in ('false', 'true'):
    # ParameterValue caches its evaluated value, as it does in one launch run.
    nodes = [n for n in module.generate_launch_description().entities
             if type(n).__name__ == 'Node']
    context = LaunchContext()
    context.launch_configurations.update(start_state_estimation='true', mode='localization',
        odom_source='cuvslam', enable_degraded_odom=enabled, chassis_odom_topic='/wheel/odom')
    active = enabled == 'true'
    assert node('degraded_odometry').condition.evaluate(context) == active
    params = overrides('cuvslam_odometry', context)
    assert params['output_topic'] == ('/r680_nav/vio_raw_odom' if active else '/d455_slam/odom')
    assert params['publish_tf'] == (not active)
    assert params['publish_visual_only'] == active
    assert overrides('vo_watchdog', context)['allow_bounded_degraded'] == active
    print('PASS actual launch routing enable_degraded_odom='+enabled)
