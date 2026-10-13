from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tree = str(Path(get_package_share_directory('wla_behavior_bt')) / 'trees/navigate_semantic.xml')
    return LaunchDescription([
        DeclareLaunchArgument('map_directory'), DeclareLaunchArgument('policy_file'),
        DeclareLaunchArgument('selected_map_record'),
        DeclareLaunchArgument('annotations', default_value=''),
        DeclareLaunchArgument('output_topic', default_value='/r680_nav/task_safe_preview'),
        DeclareLaunchArgument('allow_guarded_initialization', default_value='false'),
        Node(package='wla_behavior_bt', executable='behavior_executor',
             parameters=[{'tree_path': tree}], output='screen'),
        Node(package='wla_behavior_bridge', executable='navigation_task_gateway', parameters=[{
            k: LaunchConfiguration(k) for k in ('map_directory', 'policy_file', 'annotations', 'selected_map_record')}], output='screen'),
        Node(package='wla_behavior_bridge', executable='task_velocity_gate', parameters=[{
            'output_topic': LaunchConfiguration('output_topic'),
            'allow_guarded_initialization': ParameterValue(LaunchConfiguration('allow_guarded_initialization'), value_type=bool)
        }], output='screen'),
    ])
