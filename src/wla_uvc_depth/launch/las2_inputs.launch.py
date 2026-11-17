"""Shared camera inputs only: one RGB pipeline, one LAS2 engine, no motion nodes."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

def start(context):
 def value(k):return LaunchConfiguration(k).perform(context)
 root=Path(value('workspace')).resolve();calibration=root/'config/uvc_rig/calibrated_20261116/rig.yaml'
 if not calibration.is_file():raise RuntimeError('Missing calibrated rig.yaml')
 commands=[['bash',str(root/'robot_camera.sh'),'run','dicota','--capture-fps','30','--decode-fps','30','--publish-fps','30'],
           ['bash',str(root/'robot_camera.sh'),'pipeline','--calibration',str(calibration),'--rgb-mode','full','--rgb-only','--algorithm','none','--max-rate','30'],
           ['bash',str(root/'robot_stereo_depth.sh'),'--model',value('model'),'--profile',value('profile'),'--max-rate',value('depth_rate'),'--direct-camera','/dev/v4l/by-id/usb-SunplusIT_Inc_USB_Global_Camera-video-index0']]
 actions=[]
 for command in commands:
  process=ExecuteProcess(cmd=command,cwd=str(root),output='screen')
  actions.extend([RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason='shared camera input exited'))])),process])
 return actions

def generate_launch_description():
 return LaunchDescription([
  DeclareLaunchArgument('workspace',default_value=os.environ.get('WLA_ROS_WS',str(Path.home()/'ros2_ws'))),
  DeclareLaunchArgument('model',default_value='las2_s',choices=['las2_s','las2_m']),
  DeclareLaunchArgument('profile',default_value='640x384',choices=['640x384','384x288']),
  DeclareLaunchArgument('depth_rate',default_value='5'),OpaqueFunction(function=start)])
