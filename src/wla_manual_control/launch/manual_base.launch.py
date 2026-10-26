from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,RegisterEventHandler,EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution,LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    cfg=Path(get_package_share_directory('wla_manual_control'))/'config'
    driver=Node(package='turn_on_wheeltec_robot',executable='wheeltec_robot_node',name='wheeltec_robot',
        output='screen',parameters=[str(cfg/'chassis.yaml')],remappings=[
            ('cmd_vel','/r680_base/cmd_vel'),('red_vel','/r680_base/disabled_red_vel'),
            ('robot_recharge_flag','/r680_base/disabled_recharge'),('/set_charge','/r680_base/disabled_set_charge'),
            ('odom','/wheel/odom'),('imu/data_raw','/wheel/imu/data_raw')])
    gamepad=Node(package='gamepad_control',executable='gamepad_teleop',name='gamepad_teleop',
        output='screen',parameters=[str(cfg/'gamepad.yaml')])
    # C++ arbiter by default; mux_impl:=python is the rollback. Same velocity policy.
    impl=LaunchConfiguration('mux_impl')
    muxes=[Node(package='wla_manual_control',executable=exe,output='screen',
        condition=IfCondition(EqualsSubstitution(impl,name))) for name,exe in (('python','command_mux.py'),('cpp','command_mux'))]
    return LaunchDescription([DeclareLaunchArgument('mux_impl',default_value='cpp',choices=['python','cpp']),
        driver,gamepad,*muxes]+[RegisterEventHandler(OnProcessExit(target_action=n,
        on_exit=[EmitEvent(event=Shutdown(reason='manual base component exited'))])) for n in (driver,gamepad,*muxes)])
