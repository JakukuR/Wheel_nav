"""Verify edited-map ownership using actual launch actions, without hardware."""
import argparse
import importlib.util
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np
from PIL import Image
import yaml
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('static_map_bringup', ROOT/'launch/bringup.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def description(mode, selected):
    context = LaunchContext()
    context.launch_configurations.update(mode=mode, navigation_map_yaml=str(selected),
        web_map_yaml='/unused-web-map.yaml', start_nav2='true', start_navigation_servers='true',
        start_state_estimation='true')
    ld = module.generate_launch_description()
    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context, ld


def named(ld, context, name):
    return next(action for action in ld.entities if isinstance(action, Node)
        and action._Node__node_name is not None
        and perform_substitutions(context, normalize_to_list_of_substitutions(action._Node__node_name)) == name)


def parameters(node, context):
    result = {}
    for value in evaluate_parameters(context, node._Node__parameters):
        if isinstance(value, dict):
            result.update(value)
    return result


def rtabmap_arguments(ld, context):
    for action in ld.entities:
        if hasattr(action, 'launch_arguments'):
            arguments = dict(action.launch_arguments)
            if 'map_topic' in arguments:
                return {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                        for name, value in arguments.items()}
    raise AssertionError('RTAB-Map launch arguments not found')


def test_navigation_and_mapping_routes(tmp_path):
    # Paths with apostrophes must remain literal launch values, not Python code.
    selected = tmp_path/"user's map.yaml"
    selected.write_text('image: map.pgm\n')
    context, ld = description('localization', selected)
    module.validate_navigation_map(context)
    server = named(ld, context, 'map_server')
    manager = named(ld, context, 'lifecycle_manager_static_map')
    assert server.condition.evaluate(context) and manager.condition.evaluate(context)
    assert parameters(server, context)['yaml_filename'] == str(selected)
    assert parameters(server, context)['topic_name'] == '/r680/d455/map'
    assert list(parameters(manager, context)['node_names']) == ['map_server']
    assert rtabmap_arguments(ld, context)['map_topic'] == '/d455_slam/localization_map'
    web = named(ld, context, 'r680_web_gateway')
    assert parameters(web, context)['map_yaml'] == str(selected)
    context, ld = description('mapping', '')
    module.validate_navigation_map(context)
    assert not named(ld, context, 'map_server').condition.evaluate(context)
    assert not named(ld, context, 'lifecycle_manager_static_map').condition.evaluate(context)
    assert rtabmap_arguments(ld, context)['map_topic'] == '/r680/d455/map'


def test_missing_navigation_map_is_rejected(tmp_path):
    context, _ = description('localization', '')
    try:
        module.validate_navigation_map(context)
    except ValueError as error:
        assert 'requires navigation_map_yaml' in str(error)
    else:
        raise AssertionError('Missing edited map silently fell back to DB')
    context, _ = description('localization', tmp_path/'missing.yaml')
    try:
        module.validate_navigation_map(context)
    except ValueError as error:
        assert 'does not exist' in str(error)
    else:
        raise AssertionError('Missing map file accepted')
    context, _ = description('localization', '')
    context.launch_configurations['start_state_estimation'] = 'false'
    module.validate_navigation_map(context)  # Synthetic graph tests supply their own map.


def serve(selected, with_rtabmap=False):
    context, ld = description('localization', selected)
    module.validate_navigation_map(context)
    actions = [
        *[SetLaunchConfiguration(key, value) for key, value in context.launch_configurations.items()],
        named(ld, context, 'map_server'), named(ld, context, 'lifecycle_manager_static_map')]
    with tempfile.TemporaryDirectory(prefix='edited-map-database-') as temporary:
        if with_rtabmap:
            database = Path(temporary)/'rtabmap.db'
            shutil.copyfile(Path(selected).parent/'rtabmap.db', database)
            actions.append(SetLaunchConfiguration('database_path', str(database)))
            actions.append(next(action for action in ld.entities
                if hasattr(action, 'launch_arguments') and 'map_topic' in dict(action.launch_arguments)))
        service = LaunchService()
        service.include_launch_description(LaunchDescription(actions))
        return service.run()


def live(selected):
    assert os.environ.get('ROS_DOMAIN_ID') == '174', 'Live map test requires isolated ROS_DOMAIN_ID=174'
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.srv import GetMap
    from lifecycle_msgs.srv import GetState
    selected = Path(selected)
    metadata = yaml.safe_load(selected.read_text())
    image = Image.open(selected.parent/metadata['image']).convert('RGB')
    intensity = np.asarray(image, dtype=float).mean(axis=2)/255.0
    occupancy = intensity if metadata.get('negate', 0) else 1.0-intensity
    assert metadata.get('mode', 'trinary') == 'trinary'
    expected = np.full(occupancy.shape, -1, dtype=np.int16)
    expected[occupancy > metadata['occupied_thresh']] = 100
    expected[occupancy < metadata['free_thresh']] = 0
    expected = np.flipud(expected).ravel()
    rclpy.init()
    node = rclpy.create_node('edited_map_ownership_probe')
    maps = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(OccupancyGrid, '/r680/d455/map', maps.append, qos)
    debug = node.create_publisher(OccupancyGrid, '/d455_slam/localization_map', qos)
    state = node.create_client(GetState, '/map_server/get_state')
    database_map = node.create_client(GetMap, '/d455_slam/rtabmap/get_map')
    with tempfile.TemporaryFile(mode='w+') as output:
        process = subprocess.Popen([sys.executable, __file__, '--serve', str(selected), '--with-rtabmap'],
                                   stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+20
            while not maps and time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert maps, 'No edited OccupancyGrid published'
            grid = maps[-1]
            assert grid.header.frame_id == 'map'
            assert (grid.info.width, grid.info.height) == image.size
            assert abs(grid.info.resolution-metadata['resolution']) < 1e-6
            assert abs(grid.info.origin.position.x-metadata['origin'][0]) < 1e-6
            assert abs(grid.info.origin.position.y-metadata['origin'][1]) < 1e-6
            assert np.array_equal(np.asarray(grid.data), expected), 'Published grid differs from edited PGM'
            assert state.wait_for_service(timeout_sec=3)
            future = state.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and future.result().current_state.id == 3
            deadline = time.monotonic()+10
            while time.monotonic() < deadline:
                publishers = node.get_publishers_info_by_topic('/d455_slam/localization_map')
                if any(owner.node_name == 'rtabmap' for owner in publishers):
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            assert any(owner.node_name == 'rtabmap' for owner in publishers), 'RTAB-Map debug map not isolated'
            # Wait for the loaded DB and working executor, not merely its early
            # publisher discovery. Otherwise shutdown can interrupt construction.
            assert database_map.wait_for_service(timeout_sec=25)
            future = database_map.call_async(GetMap.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.done() and future.result() is not None, 'RTAB-Map database not ready'
            db_grid = future.result().map
            print(f'RTAB-Map database map ready: {db_grid.info.width}x{db_grid.info.height}')
            if (db_grid.info.width == grid.info.width and db_grid.info.height == grid.info.height
                    and abs(db_grid.info.resolution-grid.info.resolution) < 1e-6
                    and abs(db_grid.info.origin.position.x-grid.info.origin.position.x) < 1e-6
                    and abs(db_grid.info.origin.position.y-grid.info.origin.position.y) < 1e-6):
                print('DB/edited-map differing cells:', np.count_nonzero(np.asarray(db_grid.data) != expected))
            # A conflicting DB/debug grid cannot replace the navigation map.
            fake = OccupancyGrid()
            fake.header.frame_id = 'map'
            fake.info.width = fake.info.height = 1
            fake.info.resolution = 0.05
            fake.data = [100]
            debug.publish(fake)
            deadline = time.monotonic()+1
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert np.array_equal(np.asarray(maps[-1].data), expected)
            owners = node.get_publishers_info_by_topic('/r680/d455/map')
            assert len(owners) == 1 and owners[0].node_name == 'map_server', owners
            print(f'PASS: {image.size[0]}x{image.size[1]} cells match edited PGM; '
                  'map_server active, sole navigation-map publisher; actual RTAB-Map DB/grid isolated.')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=15)
            output.seek(0)
            text = output.read()
            print(text[-1800:])
            node.destroy_node()
            rclpy.shutdown()
            assert 'process has died' not in text and "sending signal 'SIGKILL'" not in text, 'Unclean test shutdown'


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--serve')
    parser.add_argument('--live')
    parser.add_argument('--with-rtabmap', action='store_true')
    args = parser.parse_args()
    if args.serve:
        sys.exit(serve(args.serve, args.with_rtabmap))
    if args.live:
        live(args.live)
