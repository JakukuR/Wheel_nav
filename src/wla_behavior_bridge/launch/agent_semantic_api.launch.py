from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    names = ('map_directory', 'policy_file')
    whitelist = str(Path(get_package_share_directory('wla_behavior_bridge')) / 'config/yolo_realtime_whitelist.json')
    return LaunchDescription([
        *[DeclareLaunchArgument(name) for name in names],
        DeclareLaunchArgument('realtime_whitelist', default_value=whitelist),
        DeclareLaunchArgument('annotations', default_value=''),
        DeclareLaunchArgument('execution_enabled', default_value='false'),
        Node(package='wla_behavior_bridge', executable='agent_semantic_api', parameters=[{
            **{k: LaunchConfiguration(k) for k in (*names, 'annotations', 'realtime_whitelist')},
            'execution_enabled': ParameterValue(LaunchConfiguration('execution_enabled'), value_type=bool),
        }], output='screen')])
