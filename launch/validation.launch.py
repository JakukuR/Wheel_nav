from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('wla_cuvslam_validation'))
    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('statistics_path', default_value='/tmp/cuvslam_validation_statistics.json'),
        Node(package='wla_cuvslam_validation', executable='cuvslam_validation',
             name='cuvslam_validation', output='screen', parameters=[
                 str(share / 'config' / 'validation.yaml'),
                 {'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool),
                  'statistics_path': LaunchConfiguration('statistics_path')}]),
    ])
