from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = Path(get_package_share_directory('wla_r680_navigation'))
    config = share / 'config'

    mode = LaunchConfiguration('mode')
    start_d455 = LaunchConfiguration('start_d455')
    start_chassis = LaunchConfiguration('start_chassis')
    start_nav2 = LaunchConfiguration('start_nav2')
    obstacle_scan = LaunchConfiguration('obstacle_scan')
    start_dynamic_obstacles = LaunchConfiguration('start_dynamic_obstacles')
    start_navigation_servers = LaunchConfiguration('start_navigation_servers')
    start_state_estimation = LaunchConfiguration('start_state_estimation')
    odom_source = LaunchConfiguration('odom_source')
    auto_vio_init = LaunchConfiguration('auto_vio_init')
    init_enabled = PythonExpression(["'", auto_vio_init, "' == 'true' and '", odom_source,
                                     "' == 'cuvslam' and '", start_nav2, "' == 'true'"])
    nav_command_topic = PythonExpression(["'/r680_nav/nav_command_input' if ", init_enabled,
                                          " else '/cmd_vel_nav'"])
    cuvslam_enabled = PythonExpression(["'", start_state_estimation, "' == 'true' and '", odom_source, "' == 'cuvslam'"])
    legacy_estimation = PythonExpression(["'", start_state_estimation, "' == 'true' and '", odom_source, "' == 'rgbd'"])
    legacy_chassis_imu = PythonExpression(["'", LaunchConfiguration('use_chassis_imu'), "' == 'true' and '", odom_source, "' == 'rgbd'"])
    chassis_imu_topic = LaunchConfiguration('chassis_imu_topic')
    chassis_odom_topic = LaunchConfiguration('chassis_odom_topic')
    start_web = LaunchConfiguration('start_web')
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    web_map_yaml = LaunchConfiguration('web_map_yaml')
    use_imu = LaunchConfiguration('use_d455_imu')
    use_chassis_imu = LaunchConfiguration('use_chassis_imu')
    enable_motion = LaunchConfiguration('enable_hardware_output')
    publish_mount_tf = LaunchConfiguration('publish_mount_tf')
    database = LaunchConfiguration('database_path')
    initial_pose = LaunchConfiguration('initial_pose')
    localization = PythonExpression(["'true' if '", mode, "' == 'localization' else 'false'"])
    full_navigation = PythonExpression(["'", start_nav2, "' == 'true' and '",
                                        start_navigation_servers, "' == 'true'"])
    safety_only = PythonExpression(["'", start_nav2, "' == 'true' and '",
                                    start_navigation_servers, "' == 'false'"])
    dynamic_obstacles_enabled = PythonExpression(["'", start_nav2, "' == 'true' and '",
                                                  start_dynamic_obstacles, "' == 'true'"])
    imu_topic = PythonExpression([
        "'/r680_nav/chassis/imu_filtered' if '", use_chassis_imu,
        "' == 'true' else '/r680_nav/d455/imu_filtered'"])

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        condition=IfCondition(start_d455),
        launch_arguments={
            'camera_namespace': 'r680',
            'camera_name': 'd455',
            'serial_no': '_260922306083',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': PythonExpression(["'true' if '", odom_source, "' == 'cuvslam' else 'false'"]),
            'enable_infra2': PythonExpression(["'true' if '", odom_source, "' == 'cuvslam' else 'false'"]),
            'depth_module.infra_profile': '640x480x30',
            'config_file': PythonExpression(["'", str(config / 'd455_cuvslam.yaml'),
                                              "' if '", odom_source, "' == 'cuvslam' else \"''\""]),
            'enable_gyro': use_imu,
            'enable_accel': use_imu,
            'gyro_fps': '200',
            'accel_fps': '100',
            'unite_imu_method': '2',
            'publish_tf': 'true',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
        }.items())

    # The RealSense driver owns all optical-frame TFs. This is the only external
    # mount edge and must not run beside publish_r680_d455_floor.py.
    mount_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='r680_d455_floor_mount', condition=IfCondition(publish_mount_tf),
        arguments=['--x', '0.184428484', '--y', '0.059803590', '--z', '0.501660007',
                   '--qx', '0.000096773', '--qy', '0.077125800',
                   '--qz', '-0.000560247', '--qw', '0.997021207',
                   '--frame-id', 'r680_mapping_floor', '--child-frame-id', 'd455_link'])

    # Provisional rigid-body extrinsic: chassis IMU at the vehicle center,
    # axis-aligned with r680_mapping_floor. The current EKF consumes all axes;
    # verify the physical mounting before treating this identity as calibrated.
    chassis_imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='r680_chassis_imu_mount', condition=IfCondition(use_chassis_imu),
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'r680_mapping_floor', '--child-frame-id', 'gyro_link'])

    vo = Node(
        package='rtabmap_odom', executable='rgbd_odometry',
        namespace='d455_vo', name='rgbd_odometry', output='screen',
        respawn=True, respawn_delay=2.0,
        condition=IfCondition(legacy_estimation),
        parameters=[str(config / 'rtabmap.yaml')],
        remappings=[
            ('rgb/image', '/r680/d455/color/image_raw'),
            ('depth/image', '/r680/d455/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/r680/d455/color/camera_info'),
            ('odom', '/r680_nav/vo_odom'),
            ('odom_info', '/d455_slam/odom_info')])

    vo_watchdog = Node(
        package='wla_r680_navigation', executable='vo_watchdog',
        name='r680_vo_watchdog', output='screen', condition=IfCondition(localization),
        parameters=[str(config / 'vo_watchdog.yaml'), {
            'frontend': odom_source,
            'vo_topic': PythonExpression(["'/d455_slam/odom' if '", odom_source, "' == 'cuvslam' else '/r680_nav/vo_odom'"]),
            'wheel_topic': chassis_odom_topic,
        }])

    cuvslam = Node(
        package='wla_cuvslam_navigation', executable='cuvslam_odometry',
        namespace='d455_vio', name='cuvslam_odometry', output='screen',
        respawn=True, respawn_delay=2.0, condition=IfCondition(cuvslam_enabled),
        parameters=[LaunchConfiguration('cuvslam_params_file'), {
            'imu_topic': chassis_imu_topic, 'wheel_odom_topic': chassis_odom_topic,
            'statistics_path': LaunchConfiguration('cuvslam_statistics_path'),
        }])

    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        namespace='r680_nav/d455', name='imu_filter_madgwick', output='screen',
        condition=IfCondition(use_imu), parameters=[str(config / 'rtabmap.yaml')],
        remappings=[('imu/data_raw', '/r680/d455/imu'),
                    ('imu/data', '/r680_nav/d455/imu_filtered')])

    chassis_imu_conditioner = Node(
        package='wla_r680_navigation', executable='imu_conditioner',
        name='r680_chassis_imu_conditioner', output='screen',
        condition=IfCondition(legacy_chassis_imu), parameters=[{
            'input_topic': chassis_imu_topic,
            'output_topic': '/r680_nav/chassis/imu_calibrated_raw',
            'calibration_samples': 100,
            'calibration_duration_s': 5.0,
        }])

    chassis_imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        namespace='r680_nav/chassis', name='imu_filter_madgwick', output='screen',
        condition=IfCondition(legacy_chassis_imu), parameters=[{
            'use_mag': False, 'publish_tf': False, 'world_frame': 'enu',
            'gain': 0.05, 'zeta': 0.0, 'orientation_stddev': 0.10,
        }], remappings=[
            ('imu/data_raw', '/r680_nav/chassis/imu_calibrated_raw'),
            ('imu/data', '/r680_nav/chassis/imu_filtered')])

    ekf = Node(
        package='robot_localization', executable='ekf_node',
        namespace='d455_slam', name='ekf_filter_node', output='screen',
        condition=IfCondition(legacy_estimation),
        parameters=[str(config / 'ekf_vo_imu.yaml'),
                    {'imu0': ParameterValue(imu_topic, value_type=str)}],
        remappings=[('odometry/filtered', 'odom')])

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])),
        condition=IfCondition(start_state_estimation), launch_arguments={
            'localization': localization,
            'namespace': 'd455_slam',
            'frame_id': 'r680_mapping_floor',
            'map_frame_id': 'map',
            'map_topic': '/r680/d455/map',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'odom_topic': '/d455_slam/odom',
            'rgb_topic': '/r680/d455/color/image_raw',
            'depth_topic': '/r680/d455/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/r680/d455/color/camera_info',
            'imu_topic': PythonExpression(["'/r680_nav/disabled_rtabmap_imu' if '", odom_source, "' == 'cuvslam' else '", imu_topic, "'"]),
            'subscribe_scan': 'false',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.025',
            'qos': '2',
            'rgb_image_transport': 'raw',
            'depth_image_transport': 'raw',
            'database_path': database,
            'initial_pose': initial_pose,
            'rtabmap_viz': 'false',
            'rviz': 'false',
            'args': (
                '--Grid/Sensor 1 --Grid/3D false --Grid/RayTracing true '
                '--Grid/RangeMin 0.25 --Grid/RangeMax 4.0 '
                '--Grid/NoiseFilteringRadius 0.10 '
                '--Grid/NoiseFilteringMinNeighbors 5 '
                '--GridGlobal/Eroded true '
                '--Grid/NormalsSegmentation false '
                '--Grid/MinGroundHeight -0.08 '
                '--Grid/MaxGroundHeight 0.04 '
                '--Rtabmap/DetectionRate 1.0'
            ),
        }.items())

    semantic_collector = ExecuteProcess(
        cmd=[LaunchConfiguration('semantic_python'),
             str(share.parents[1] / 'lib' / 'wla_r680_navigation' / 'furniture_semantic.py'),
             '--output', LaunchConfiguration('semantic_output'),
             '--map-id', LaunchConfiguration('semantic_map_id'),
             '--model', LaunchConfiguration('semantic_model'),
             '--hz', '1.0',
             '--mark-home', LaunchConfiguration('semantic_mark_home')],
        name='r680_furniture_semantic', output='screen',
        condition=IfCondition(LaunchConfiguration('start_semantics')))

    chassis = Node(
        package='turn_on_wheeltec_robot', executable='wheeltec_robot_node',
        name='wheeltec_robot', output='screen', condition=IfCondition(start_chassis),
        parameters=[str(config / 'chassis.yaml')],
        remappings=[
            ('cmd_vel', '/r680_nav/chassis_cmd_vel'),
            ('red_vel', '/r680_nav/disabled_red_vel'),
            ('robot_recharge_flag', '/r680_nav/disabled_recharge_flag'),
            ('/set_charge', '/r680_nav/disabled_set_charge'),
            ('odom', '/wheel/odom'),
            ('imu/data_raw', '/wheel/imu/data_raw')])

    depth_points = Node(
        package='wla_r680_navigation', executable='depth_to_points',
        name='r680_d455_depth_to_points', output='screen', condition=IfCondition(start_nav2),
        parameters=[str(config / 'd455_points.yaml')])

    scan_converter = Node(
        package='wla_r680_navigation', executable='point_to_scan',
        name='r680_d455_point_to_scan', output='screen',
        condition=IfCondition(obstacle_scan),
        parameters=[str(config / 'd455_scan.yaml')])

    dynamic_obstacles = Node(
        package='wla_r680_navigation', executable='dynamic_obstacle_memory',
        name='r680_dynamic_obstacle_memory', output='screen',
        condition=IfCondition(dynamic_obstacles_enabled),
        parameters=[str(config / 'dynamic_obstacles.yaml')])

    nav_params = LaunchConfiguration('nav_params_file')
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', condition=IfCondition(full_navigation),
             parameters=[nav_params], remappings=[('cmd_vel', '/r680_nav/cmd_vel_controller')]),
        Node(package='wla_r680_navigation', executable='goal_approach_limiter',
             name='r680_goal_approach_limiter', output='screen', condition=IfCondition(full_navigation),
             parameters=[RewrittenYaml(
                 source_file=str(config / 'goal_approach_limiter.yaml'),
                 param_rewrites={'output_topic': nav_command_topic}, convert_types=True)]),
        Node(package='wla_r680_navigation', executable='path_speed_profile',
             name='r680_path_speed_profile', output='screen', condition=IfCondition(full_navigation),
             parameters=[str(config / 'path_speed_profile.yaml')]),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen', condition=IfCondition(full_navigation),
             parameters=[nav_params]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', condition=IfCondition(full_navigation),
             parameters=[nav_params]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', condition=IfCondition(full_navigation),
             parameters=[nav_params], remappings=[('cmd_vel', nav_command_topic)]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', condition=IfCondition(full_navigation),
             parameters=[nav_params, {
                 'default_nav_to_pose_bt_xml': str(config / 'navigate_to_pose_no_spin.xml'),
                 'default_nav_through_poses_bt_xml': str(config / 'navigate_through_poses_no_spin.xml'),
             }]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', condition=IfCondition(full_navigation),
             parameters=[nav_params]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', condition=IfCondition(start_nav2),
             parameters=[nav_params], remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', condition=IfCondition(start_nav2),
             parameters=[nav_params]),
        TimerAction(period=3.0, actions=[
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 condition=IfCondition(full_navigation), parameters=[{
                     'autostart': True,
                     'node_names': ['controller_server', 'smoother_server', 'planner_server',
                                    'behavior_server', 'bt_navigator', 'waypoint_follower',
                                    'velocity_smoother', 'collision_monitor']}]),
        ]),
        TimerAction(period=3.0, actions=[
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_manual_safety', output='screen',
                 condition=IfCondition(safety_only), parameters=[{
                     'autostart': True,
                     'node_names': ['velocity_smoother', 'collision_monitor']}]),
        ]),
        Node(package='wla_r680_navigation', executable='navigation_goal_bridge',
             name='r680_navigation_goal_bridge', output='screen',
             condition=IfCondition(full_navigation)),
    ]

    monitor = Node(
        package='wla_r680_navigation', executable='interface_monitor', output='screen',
        parameters=[{
            'require_obstacle_points': True,
            'chassis_odom_topic': chassis_odom_topic,
            'require_chassis_odom': ParameterValue(enable_motion, value_type=bool),
            'require_vo_watchdog': ParameterValue(localization, value_type=bool),
        }])
    guard = Node(
        package='wla_r680_navigation', executable='command_guard', output='screen',
        parameters=[{
            'hardware_output_enabled': ParameterValue(enable_motion, value_type=bool),
            'initialization_mode_enabled': ParameterValue(init_enabled, value_type=bool),
            'forward_max': 1.20,
            'reverse_max': 0.70,
        }])
    initializer = Node(
        package='wla_r680_navigation', executable='vio_initializer', name='r680_vio_initializer',
        output='screen', condition=IfCondition(init_enabled),
        parameters=[RewrittenYaml(
            source_file=str(config / 'vio_initializer.yaml'),
            param_rewrites={
                'imu_topic': chassis_imu_topic, 'wheel_odom_topic': chassis_odom_topic,
                'result_path': LaunchConfiguration('vio_init_result_path'),
            }, convert_types=True)])
    web_gateway = Node(
        package='wla_r680_navigation', executable='web_gateway',
        name='r680_web_gateway', output='screen', condition=IfCondition(start_web),
        parameters=[str(config / 'web_gateway.yaml'), {
            'host': ParameterValue(web_host, value_type=str),
            'port': ParameterValue(web_port, value_type=int),
            'map_yaml': ParameterValue(web_map_yaml, value_type=str),
            'semantic_path': ParameterValue(LaunchConfiguration('semantic_output'), value_type=str),
        }])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localization']),
        DeclareLaunchArgument('start_d455', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_chassis', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_nav2', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('obstacle_scan', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_dynamic_obstacles', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_navigation_servers', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('nav_params_file', default_value=str(config / 'nav2.yaml'),
                              description='Complete Nav2 parameters file for this run.'),
        DeclareLaunchArgument('start_state_estimation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('auto_vio_init', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('vio_init_result_path', default_value='/tmp/r680_vio_init.json'),
        DeclareLaunchArgument('odom_source', default_value='rgbd', choices=['rgbd', 'cuvslam']),
        DeclareLaunchArgument('chassis_imu_topic', default_value='/wheel/imu/data_raw'),
        DeclareLaunchArgument('chassis_odom_topic', default_value='/wheel/odom'),
        DeclareLaunchArgument('cuvslam_params_file', default_value=str(config / 'cuvslam.yaml')),
        DeclareLaunchArgument('cuvslam_statistics_path', default_value='/tmp/cuvslam_navigation_statistics.json'),
        DeclareLaunchArgument('start_web', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_semantics', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('semantic_output', default_value='/tmp/wla-semantic.geojson'),
        DeclareLaunchArgument('semantic_map_id', default_value='unassigned'),
        DeclareLaunchArgument('semantic_mark_home', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('semantic_python',
                              default_value='/home/orin/Wheel_Legged_Agent/.venv-perception/bin/python'),
        DeclareLaunchArgument('semantic_model',
                              default_value='/home/orin/Wheel_Legged_Agent/models/yolo11s.engine'),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument(
            'web_map_yaml', default_value='',
            description='Optional map.yaml pinned in the Web view; empty uses live OccupancyGrid.'),
        DeclareLaunchArgument('use_d455_imu', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_chassis_imu', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_hardware_output', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('publish_mount_tf', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'database_path', default_value='/home/orin/.local/share/wla/r680-navigation/rtabmap.db'),
        DeclareLaunchArgument(
            'initial_pose', default_value='',
            description='Optional RTAB-Map initial pose: x y z roll pitch yaw.'),
        realsense, mount_tf, chassis_imu_tf, imu_filter, chassis_imu_conditioner,
        chassis_imu_filter, vo, cuvslam, vo_watchdog, ekf, rtabmap,
        chassis, depth_points, scan_converter, dynamic_obstacles, semantic_collector,
        *nav2_nodes, monitor, guard, initializer, web_gateway,
    ])
