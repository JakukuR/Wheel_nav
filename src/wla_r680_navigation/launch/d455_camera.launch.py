"""The single D455 owner shared by navigation and perception (r680_camera.sh).

Color and depth aligned to color are always on at 640x480x30, the profile
every consumer uses. The infrared stereo pair (cuVSLAM / OpenVINS) and the
IMU are optional. The camera also owns the measured mount edge
r680_mapping_floor -> d455_link (mount:=false when another process publishes
it); navigation, mapping and the debug entry then skip their own copy.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def camera(context):
    infra = LaunchConfiguration('infra').perform(context)
    imu = LaunchConfiguration('imu').perform(context)
    emitter = LaunchConfiguration('emitter').perform(context)
    config = Path(get_package_share_directory('wla_r680_navigation')) / 'config' / f'd455_emitter_{emitter}.yaml'
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_namespace': 'r680',
            'camera_name': 'd455',
            'serial_no': '_260922306083',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x30',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': infra,
            'enable_infra2': infra,
            'enable_gyro': imu,
            'enable_accel': imu,
            'gyro_fps': '200',
            'accel_fps': '100',
            'unite_imu_method': '2',
            'publish_tf': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'config_file': str(config),
        }.items())
    # Same SDK scoping as bringup.launch.py: the rsusb build only for the camera.
    runtime = Path.home() / 'robot_sensors/realsense/current/lib'
    libraries = [part for part in context.environment.get('LD_LIBRARY_PATH', '').split(':')
                 if part and part != str(runtime)]
    if context.environment.get('R680_REALSENSE_BACKEND', 'rsusb') != 'system':
        if (runtime / 'librealsense2.so.2.58').is_file():
            libraries.insert(0, str(runtime))
    # forwarding=False: rs_launch.py warns about every launch configuration it can see.
    actions = [GroupAction(scoped=True, forwarding=False, actions=[
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=':'.join(libraries)),
        realsense,
    ])]
    if emitter == 'alternate' and infra == 'true':
        actions.append(Node(package='wla_r680_navigation', executable='emitter_frame_splitter',
                            name='r680_emitter_frame_splitter', output='screen'))
    return actions


def generate_launch_description():
    # Same measured mounting transform as bringup.launch.py (publish_mount_tf).
    mount = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='r680_d455_floor_mount', condition=IfCondition(LaunchConfiguration('mount')),
        arguments=['--x', '0.184428484', '--y', '0.059803590', '--z', '0.501660007',
                   '--qx', '0.000096773', '--qy', '0.077125800',
                   '--qz', '-0.000560247', '--qw', '0.997021207',
                   '--frame-id', 'r680_mapping_floor', '--child-frame-id', 'd455_link'])
    return LaunchDescription([
        DeclareLaunchArgument('infra', default_value='false', choices=['true', 'false'],
                              description='Infrared stereo pair for cuVSLAM / OpenVINS'),
        DeclareLaunchArgument('imu', default_value='false', choices=['true', 'false'],
                              description='D455 gyro + accel, united on /r680/d455/imu'),
        DeclareLaunchArgument('emitter', default_value='on', choices=['on', 'off', 'alternate'],
                              description='Dot projector: on for depth obstacles, off for cleaner stereo VIO, '
                                          'alternate per frame (dark infrared pairs on infra*_dark)'),
        DeclareLaunchArgument('mount', default_value='true', choices=['true', 'false'],
                              description='Publish r680_mapping_floor -> d455_link; false when another process owns it'),
        OpaqueFunction(function=camera),
        mount,
    ])
