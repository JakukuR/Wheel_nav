"""Independent read-only status recorder. No publishers, services or commands."""
import argparse
from datetime import datetime
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import time


def make_handler(directory):
    directory=Path(directory).expanduser()
    directory.mkdir(parents=True,exist_ok=True)
    path=directory/('follow-status-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+str(os.getpid())+'.jsonl')
    handler=RotatingFileHandler(path,maxBytes=32*1024*1024,backupCount=8,encoding='utf-8')
    handler.setFormatter(logging.Formatter('%(message)s'))
    return path,handler


def main():
    import rclpy
    from std_msgs.msg import String
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',default=str(Path.home()/'ros2_ws/.runtime/follow-diagnostics'))
    args=p.parse_args()
    try:os.nice(5)
    except OSError:pass
    path,handler=make_handler(args.directory)
    logger=logging.getLogger('follow_status_record');logger.setLevel(logging.INFO)
    logger.addHandler(handler);logger.propagate=False
    rclpy.init();node=rclpy.create_node('person_follow_status_recorder')
    previous=None

    def receive(msg):
        nonlocal previous
        try:
            state=json.loads(msg.data)
            key=(state.get('follow_session'),state.get('diagnostic_sequence'))
            gap=None
            if previous is not None and previous[0]==key[0] and isinstance(key[1],int) and isinstance(previous[1],int):
                gap=max(0,key[1]-previous[1]-1)
            state['_recording']=dict(received_wall_ns=time.time_ns(),received_monotonic_s=time.monotonic(),
                sequence_gap=gap,source='/r680/person_follow/status',path=str(path))
            logger.info(json.dumps(state,ensure_ascii=False,allow_nan=False))
            previous=key
        except (ValueError,TypeError) as exc:
            node.get_logger().warning('status recording rejected payload: '+str(exc)[:100])

    # Bounded queue and best-effort delivery never backpressure the controller.
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(String,'/r680/person_follow/status',receive,qos)
    node.get_logger().info('read-only status log: '+str(path))
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        handler.flush();handler.close();logger.removeHandler(handler)
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
