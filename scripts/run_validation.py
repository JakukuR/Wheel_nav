#!/usr/bin/env python3
"""Bounded stationary validation. Publishes no motion; owns only its child processes."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions


def terminate(child):
    if child.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        try:
            os.killpg(child.pid, sig)
            child.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass
        except ProcessLookupError:
            return


def proc_sample(pid):
    root = Path('/proc') / str(pid)
    # stat fields start at field 3 after the parenthesized process name.
    fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
    status = {}
    for line in (root / 'status').read_text().splitlines():
        key, _, value = line.partition(':')
        status[key] = value.strip()
    return {'time_monotonic': time.monotonic(),
            'cpu_seconds': (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK'),
            'rss_mib': int(status['VmRSS'].split()[0]) / 1024,
            'threads': int(status['Threads'])}


class CameraParameters:
    def __init__(self):
        # Keep the client context alive through finally when the operator presses Ctrl+C.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node(f'cuvslam_camera_setup_{os.getpid()}')
        self.client = AsyncParameterClient(self.node, '/r680/d455')
        if not self.client.wait_for_services(timeout_sec=10):
            self.close()
            raise RuntimeError('/r680/d455 parameter services unavailable')

    def complete(self, future):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20)
        if not future.done():
            raise TimeoutError('D455 parameter transaction timed out')
        if future.exception():
            raise future.exception()
        return future.result()

    def get(self, names):
        response = self.complete(self.client.get_parameters(names))
        return dict(zip(names, (parameter_value_to_python(v) for v in response.values)))

    def set(self, values):
        # Update both IR flags together: changing one side alone can strand the driver's syncer.
        response = self.complete(self.client.set_parameters_atomically(
            [Parameter(name, value=value) for name, value in values.items()]))
        if not response.result.successful:
            raise RuntimeError('D455 parameters rejected: ' + response.result.reason)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def serial_in_use(port):
    resolved = str(Path(port).resolve())
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            for fd in (process / 'fd').iterdir():
                if os.readlink(fd) == resolved:
                    return int(process.name)
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--stereo-only', action='store_true')
    parser.add_argument('--image-decimation', type=int, choices=range(1, 7), default=2)
    parser.add_argument('--prepare-camera', action='store_true', help='Temporarily enable IR stereo, disable emitter; restore on exit')
    parser.add_argument('--start-chassis', action='store_true', help='Start a feedback-only driver with unused command inputs')
    parser.add_argument('--serial-port', default='/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00')
    parser.add_argument('--run-root', type=Path, default=Path.home() / 'nav_run' / 'cuvslam_validation')
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 600:
        parser.error('--seconds must be in [5, 600]')
    args.run_root.mkdir(parents=True, exist_ok=True)
    lock = (args.run_root / '.validation.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('another validation run is still active or restoring the camera')
    if args.start_chassis and serial_in_use(args.serial_port):
        parser.error('serial device already in use; stop the existing chassis/navigation stack first')
    share = Path(get_package_share_directory('wla_cuvslam_validation'))
    prefix = Path(get_package_prefix('wla_cuvslam_validation'))
    config = args.config or share / 'config' / 'validation.yaml'
    run = args.run_root / (datetime.datetime.now().strftime('run-%Y%m%d-%H%M%S-') + str(os.getpid()))
    run.mkdir(parents=True)
    (run / 'validation.yaml').write_bytes(config.read_bytes())
    children, handles, originals, samples = [], [], {}, []
    camera = None
    executable = prefix / 'lib' / 'wla_cuvslam_validation' / 'cuvslam_validation'
    command = [str(executable), '--ros-args', '--params-file', str(config),
               '-p', f'use_imu:={str(not args.stereo_only).lower()}',
               '-p', f'image_decimation:={args.image_decimation}',
               '-p', f'statistics_path:={run / "statistics.json"}']
    metadata = {'command': command, 'stationary_only': True, 'motion_commanded': False,
                'ros_domain_id': os.getenv('ROS_DOMAIN_ID'), 'rmw': os.getenv('RMW_IMPLEMENTATION'),
                'duration_seconds': args.seconds, 'run_dir': str(run)}
    def start(cmd, filename):
        handle = (run / filename).open('w')
        handles.append(handle)
        child = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        return child
    def on_termination(signum, frame):
        raise KeyboardInterrupt(f'signal {signum}')
    previous_term_handler = signal.signal(signal.SIGTERM, on_termination)
    try:
        if args.prepare_camera:
            camera = CameraParameters()
            changes = {'depth_module.infra_profile': '640x480x30',
                       'enable_infra1': True, 'enable_infra2': True,
                       'depth_module.emitter_enabled': 0}
            originals = camera.get(list(changes))
            (run / 'camera_original.json').write_text(json.dumps(originals, indent=2))
            camera.set(changes)
        if args.start_chassis:
            chassis = Path(get_package_prefix('turn_on_wheeltec_robot')) / 'lib/turn_on_wheeltec_robot/wheeltec_robot_node'
            cmd = [str(chassis), '--ros-args', '-r', '__node:=wheeltec_robot_vio_test']
            params = {'usart_port_name': args.serial_port, 'serial_baud_rate': '115200', 'serial_timeout_ms': '10',
                      'robot_frame_id': 'r680_vio_test_base', 'odom_frame_id': 'r680_vio_test_wheel_odom',
                      'gyro_frame_id': 'gyro_link', 'odom_x_scale': '1.0', 'odom_y_scale': '1.0',
                      'odom_z_scale_positive': '1.0', 'odom_z_scale_negative': '1.0'}
            remaps = {'cmd_vel': '/r680_vio_test/unused_cmd_vel', 'red_vel': '/r680_vio_test/unused_red_vel',
                      'robot_recharge_flag': '/r680_vio_test/unused_recharge_flag', '/set_charge': '/r680_vio_test/unused_set_charge',
                      'odom': '/r680_vio_test/wheel_odom', 'imu/data_raw': '/r680_vio_test/chassis/imu_raw'}
            for name, value in params.items():
                cmd += ['-p', f'{name}:={value}']
            for name, value in remaps.items():
                cmd += ['-r', f'{name}:={value}']
            start(cmd, 'chassis.log')
            time.sleep(2)
        child = start(command, 'tracker.log')
        metadata['tracker_pid'] = child.pid
        (run / 'run.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print(f'VALIDATION ONLY, no motion: {run}', flush=True)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            if child.poll() is not None:
                raise RuntimeError(f'tracker exited early: {child.returncode}; see tracker.log')
            try:
                samples.append(proc_sample(child.pid))
            except FileNotFoundError:
                pass
            time.sleep(.5)
    except KeyboardInterrupt:
        metadata['interrupted'] = True
        (run / 'run.json').write_text(json.dumps(metadata, indent=2) + '\n')
    finally:
        for child in reversed(children):
            terminate(child)
        for handle in handles:
            handle.close()
        restore_errors = []
        if camera is not None:
            try:
                if originals:
                    camera.set(originals)
            except Exception as exc:
                restore_errors.append(str(exc))
            finally:
                camera.close()
        cpu = [(b['cpu_seconds'] - a['cpu_seconds']) / (b['time_monotonic'] - a['time_monotonic']) * 100
               for a, b in zip(samples, samples[1:])]
        result = {'samples': samples, 'cpu_percent_one_core_mean': sum(cpu) / len(cpu) if cpu else None,
                  'rss_mib_max': max((x['rss_mib'] for x in samples), default=None),
                  'camera_restore_errors': restore_errors}
        (run / 'resources.json').write_text(json.dumps(result, indent=2) + '\n')
        signal.signal(signal.SIGTERM, previous_term_handler)
        print(f'Results: {run}', flush=True)
        if restore_errors:
            raise RuntimeError('camera restore failed: ' + '; '.join(restore_errors))


if __name__ == '__main__':
    main()
