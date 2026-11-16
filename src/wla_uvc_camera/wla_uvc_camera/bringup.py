"""Own the two camera process groups; shut down both if either exits."""
import argparse
from pathlib import Path
from launch import LaunchDescription,LaunchService
from launch.actions import ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',required=True)
    p.add_argument('--publish-fps',type=float,default=30.)
    args=p.parse_args()
    if not 0<args.publish_fps<=30:p.error('publish-fps must be within (0,30]')
    script=Path(args.workspace)/'robot_camera.sh'
    if not script.is_file():p.error('camera wrapper not found')
    processes=[ExecuteProcess(cmd=[str(script),'run',name,'--publish-fps',str(args.publish_fps)],output='screen',
        sigterm_timeout='3',sigkill_timeout='3') for name in ('sunplus','dicota')]
    failures=[]
    def exited(event,context):
        if event.returncode not in (0,-2,-15,130):failures.append(event.returncode)
        return [EmitEvent(event=Shutdown(reason='camera component exited'))]
    handlers=[RegisterEventHandler(OnProcessExit(target_action=process,on_exit=exited)) for process in processes]
    service=LaunchService()
    service.include_launch_description(LaunchDescription([*handlers,*processes]))
    result=service.run()
    return 1 if failures else result
