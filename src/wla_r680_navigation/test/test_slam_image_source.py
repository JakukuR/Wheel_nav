#!/usr/bin/env python3
"""RTAB-Map image inputs per slam_image / dark_infra, evaluated from the real launch file."""
import importlib.util
from pathlib import Path
import subprocess

from launch import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('slam_image_bringup', root/'launch/bringup.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
include = next(e for e in module.generate_launch_description().entities
               if type(e).__name__ == 'IncludeLaunchDescription'
               and 'rtabmap.launch.py' in str(e.launch_description_source.location))
D = '/r680/d455/'
cases = {
    ('color', 'false'): ('color/image_raw', 'aligned_depth_to_color/image_raw', 'color/camera_info', '0.025'),
    ('color', 'true'): ('color/image_raw', 'aligned_depth_to_color/image_raw', 'color/camera_info', '0.025'),
    ('infra', 'false'): ('infra1/image_rect_raw', 'depth/image_rect_raw', 'infra1/camera_info', '0.025'),
    ('infra', 'true'): ('infra1_dark/image_rect_raw', 'depth_lit/image_rect_raw', 'infra1/camera_info', '0.045'),
}
for (slam_image, dark_infra), (rgb, depth, info, interval) in cases.items():
    context = LaunchContext()
    context.launch_configurations.update(slam_image=slam_image, dark_infra=dark_infra)
    evaluate = lambda x: perform_substitutions(context, normalize_to_list_of_substitutions(x))
    args = {evaluate(name): value for name, value in include.launch_arguments}
    value = lambda key: evaluate(args[key])
    got = (value('rgb_topic'), value('depth_topic'), value('camera_info_topic'),
           value('approx_sync_max_interval'))
    assert got == (D+rgb, D+depth, D+info, interval), (slam_image, dark_infra, got)
print('PASS RTAB-Map image topics follow slam_image and dark_infra', flush=True)

# Argument validation runs before any ROS setup, so it is safe to exercise here.
script = root.parents[1]/'r680_mapping.sh'
if script.is_file():
    for argv in (['--slam-image', 'thermal'], ['--dark-infra']):
        result = subprocess.run(['bash', str(script), *argv], capture_output=True, text=True, timeout=10)
        assert result.returncode == 2, (argv, result.returncode, result.stderr)
    print('PASS r680_mapping.sh rejects invalid image options', flush=True)
