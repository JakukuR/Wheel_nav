#!/usr/bin/env python3
"""Foreground manual calibration recording; native rosbag2 writes the images.

Ctrl+C, monotonic wall-clock deadline and free-space guard all request SIGINT
and wait for metadata. No actuator interfaces, camera restarts or deletion.
"""
import argparse
from datetime import datetime
import fcntl
import json
import os
import re
from pathlib import Path
import shutil
import signal
import subprocess
import time
from zoneinfo import ZoneInfo
import yaml

TOPICS=[f'/camera/sunplus/{eye}/{kind}' for eye in ('left','right')
        for kind in ('image_raw','camera_info')]+[
    '/camera/dicota/color/image_raw','/camera/dicota/color/camera_info',
    '/camera/sunplus/status','/camera/dicota/status']


def camera_rates(workspace):
    rates={};other=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:args=(path/'cmdline').read_bytes().decode().split('\0')
        except (OSError,UnicodeError):continue
        if any(a.endswith('/ros2') for a in args[:2]) and 'record' in args and 'bag' in args:
            other.append(path.name)
        binary=str(workspace/'install/wla_uvc_camera/lib/wla_uvc_camera/uvc_camera')
        if binary not in args or '--name' not in args:continue
        name=args[args.index('--name')+1]
        values=[float(args[i+1]) for i,a in enumerate(args[:-1]) if a=='--publish-fps']
        if name in rates:raise RuntimeError('发现重复相机进程，请先处理：'+name)
        rates[name]=values[-1] if values else 30.
    if other:raise RuntimeError('已有 rosbag 录制进程，未启动另一份：'+','.join(other))
    if set(rates)!={'sunplus','dicota'}:raise RuntimeError('三路相机未运行。先启动 ./robot_camera.sh run both --publish-fps 4')
    if any(not 0<rate<=5 for rate in rates.values()):
        raise RuntimeError('当前仍是高帧率模式。静态标定请先使用 ./robot_camera.sh run both --publish-fps 4；不要重复启动占用相机。')
    return rates


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=180.,help='真正的墙钟时长上限，默认 180 秒')
    parser.add_argument('--output-root',type=Path,default=Path('/home/onana/calibration_data'))
    parser.add_argument('--dry-run',action='store_true',help='只检查条件，不录制、不创建会话')
    args=parser.parse_args()
    if not 1<=args.duration<=600:parser.error('duration 必须在 1–600 秒之间')
    ws=args.workspace.resolve();runtime=ws/'.runtime';runtime.mkdir(exist_ok=True)
    with (runtime/'manual-calibration-record.lock').open('w') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:parser.error('手动标定录制已经运行')
        try:rates=camera_rates(ws)
        except RuntimeError as exc:parser.error(str(exc))
        args.output_root.mkdir(parents=True,exist_ok=True)
        # Three BGR8 images, plus 15% allowance and 5 GiB reserved for the OS.
        bytes_per_second=2*1280*720*3*rates['sunplus']+1920*1080*3*rates['dicota']
        required=int(bytes_per_second*args.duration*1.15)+5*1024**3
        free=shutil.disk_usage(args.output_root).free
        if free<required:parser.error(f'空间不足：可用 {free/1e9:.1f} GB，本轮含预留需要 {required/1e9:.1f} GB')
        device='/dev/v4l/by-id/usb-DHZJ-260305-J_DICOTA_4K_01.00.00-video-index0'
        focus=subprocess.run(['v4l2-ctl','-d',device,'--get-ctrl=focus_automatic_continuous,focus_absolute'],capture_output=True,text=True)
        if focus.returncode or 'focus_automatic_continuous: 0' not in focus.stdout:
            parser.error('RGB 自动对焦尚未锁定，先固定对焦后再录制；本入口不自动改镜头设置。')
        print(f'相机发布频率：{rates}；预计图像写入 {bytes_per_second/1e6:.1f} MB/s',flush=True)
        print(f'墙钟时长上限 {args.duration:g} 秒；Ctrl+C 可提前停止。可用 {free/1e9:.1f} GB。',flush=True)
        if args.dry_run:
            print('检查通过；未开始录制。',flush=True);return
        root=args.output_root/datetime.now(ZoneInfo('Asia/Tokyo')).strftime('manual_%Y%m%d_%H%M%S_JST')
        root.mkdir(exist_ok=False)
        shutil.copyfile(ws/'config/uvc_rig/aprilgrid_measured_6x6_32p5mm.yaml',root/'target.yaml')
        command=['ros2','bag','record','--storage','mcap','--storage-preset-profile','fastwrite',
            '--disable-keyboard-controls','--max-bag-size','2000000000',
            '--max-cache-size','134217728','--node-name','manual_calibration_recorder',
            '-o',str(root/'bag'),'--topics',*TOPICS]
        stop={'reason':None}
        ready=False
        def request_stop(signum,_frame):
            stop['reason']=stop['reason'] or ('ctrl_c' if signum==signal.SIGINT else 'termination_signal')
        signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
        started=time.monotonic()
        with (root/'record.log').open('wb') as log:
            child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                session=dict(started_jst=datetime.now(ZoneInfo('Asia/Tokyo')).isoformat(),
                    recorder_pid=child.pid,topics=TOPICS,publish_fps=rates,focus_controls=focus.stdout,
                    wall_clock_limit_s=args.duration,command=command,solver_started=False)
                (root/'session.json').write_text(json.dumps(session,indent=2))
                print('录制进程已启动，等待三路订阅：'+str(root),flush=True)
                last_update=started
                while child.poll() is None:
                    now=time.monotonic()
                    if not ready and 'All requested topics are subscribed' in (root/'record.log').read_text(errors='replace'):
                        ready=True
                        print('三路已订阅，现在开始摆板；固定机器人，每个姿态停稳约 2 秒。',flush=True)
                    if now-started>=args.duration:stop['reason']=stop['reason'] or 'wall_clock_limit'
                    if shutil.disk_usage(root).free<5*1024**3:stop['reason']=stop['reason'] or 'free_space_guard'
                    if stop['reason']:
                        print('正在正常停止并保存：'+stop['reason'],flush=True)
                        break
                    if now-last_update>=10:
                        print(f'已录制 {now-started:.0f} 秒；Ctrl+C 结束。',flush=True);last_update=now
                    time.sleep(.2)
            finally:
                if child.poll() is None:
                    stop['reason']=stop['reason'] or 'cleanup_after_error'
                    try:os.killpg(child.pid,signal.SIGINT)
                    except ProcessLookupError:pass
                # Do not SIGKILL a recorder while it is finalizing metadata.
                while child.poll() is None:
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:print('仍在等待缓存和 metadata 保存…',flush=True)
        metadata=root/'bag/metadata.yaml'
        result=dict(stop_reason=stop['reason'] or 'recorder_exited',returncode=child.returncode,
            wall_elapsed_s=time.monotonic()-started,metadata_exists=metadata.exists(),
            all_topics_subscription_confirmed=ready)
        if metadata.exists():
            info=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
            result['bag_duration_s']=info['duration']['nanoseconds']/1e9
            result['message_counts']={t['topic_metadata']['name']:t['message_count'] for t in info['topics_with_message_count']}
            result['three_image_streams_nonempty']=all(result['message_counts'].get(topic,0)>0 for topic in TOPICS if topic.endswith('/image_raw'))
        log_text=(root/'record.log').read_text(errors='replace')
        transport=re.findall(r'Number of messages lost on the transport layer:\s*(\d+)',log_text)
        cache=re.findall(r'Total lost:\s*(\d+)',log_text)
        result['reported_transport_lost_messages']=int(transport[-1]) if transport else 0
        result['reported_cache_lost_messages']=int(cache[-1]) if cache else 0
        (root/'recording_result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
        if not metadata.exists():raise SystemExit('录制未正常收尾，请查看 '+str(root/'record.log'))
        if not result.get('three_image_streams_nonempty',False):raise SystemExit('文件已收尾，但三路图像未全部收到，请查看 record.log')
        print('录制已结束并保存。',flush=True)
        if result['reported_transport_lost_messages'] or result['reported_cache_lost_messages']:
            print('注意：日志报告了消息丢失，需检查有效图像和共视后再标定。详见 '+str(root/'record.log'),flush=True)


if __name__=='__main__':main()
