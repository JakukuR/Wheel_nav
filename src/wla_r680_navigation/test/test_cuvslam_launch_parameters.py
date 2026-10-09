#!/usr/bin/env python3
"""Evaluate the actual launch parameter rewrite without starting any node."""
import importlib.util
from pathlib import Path
import yaml
from launch import LaunchContext
from launch.utilities import perform_substitutions

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mapping_bringup', root/'launch/bringup.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
context = LaunchContext()
context.launch_configurations.update(cuvslam_params_file=str(root/'config/cuvslam.yaml'),
    chassis_imu_topic='/test/imu', chassis_odom_topic='/test/wheel',
    cuvslam_statistics_path='/tmp/namespaced-cuvslam-test.json')
node = next(n for n in module.generate_launch_description().entities
    if type(n).__name__ == 'Node' and any('cuvslam_odometry' in str(x) for x in n.cmd))
# Launch stores normalized parameter files as substitutions. Evaluate the actual
# file passed to this Node, rather than testing a separate hand-built rewrite.
path = Path(perform_substitutions(context, node._Node__parameters[0].param_file))
try:
    loaded = yaml.safe_load(path.read_text())['d455_vio']['cuvslam_odometry']['ros__parameters']
    assert loaded['imu_topic'] == '/test/imu'
    assert loaded['wheel_odom_topic'] == '/test/wheel'
    assert loaded['statistics_path'] == '/tmp/namespaced-cuvslam-test.json'
    assert loaded['initialization_diagnostics'] is True
    assert loaded['base_frame'] == 'r680_mapping_floor'
    assert loaded['initialization_stable_s'] == 2.0
finally:
    path.unlink(missing_ok=True)
print('actual launch namespaced parameter rewrite PASS')
