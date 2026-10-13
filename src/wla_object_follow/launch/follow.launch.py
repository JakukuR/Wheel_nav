from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    config=str(Path(get_package_share_directory('wla_object_follow'))/'config/follow.yaml')
    return LaunchDescription([DeclareLaunchArgument('enable_motion',default_value='false'),
        DeclareLaunchArgument('config',default_value=config),
        Node(package='wla_object_follow',executable='object_follow',output='screen',
             parameters=[LaunchConfiguration('config'),{'enable_motion':LaunchConfiguration('enable_motion')}])])
