#!/usr/bin/env python3
"""Compare ROS 2 / ROS 1 calibration payloads without ROS or clock restamping.

Requires rosbags 0.11.0 in the offline calibration environment. This verifies
format conversion only, not optical calibration, exposure timing or IMU quality.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader


def payloads(path):
    rows=defaultdict(list)
    with AnyReader([Path(path)]) as reader:
        connections=[c for c in reader.connections if c.msgtype in ('sensor_msgs/msg/Image','sensor_msgs/msg/Imu')]
        for connection,record_time,data in reader.messages(connections=connections):
            msg=reader.deserialize(data,connection.msgtype)
            header=msg.header
            row=dict(record_ns=record_time,stamp_ns=header.stamp.sec*1_000_000_000+header.stamp.nanosec,
                frame_id=header.frame_id,type=connection.msgtype)
            if connection.msgtype.endswith('/Image'):
                row.update(width=msg.width,height=msg.height,encoding=msg.encoding,
                    step=msg.step,is_bigendian=msg.is_bigendian,
                    sha256=hashlib.sha256(msg.data.tobytes()).hexdigest())
            else:
                for name in ('orientation','angular_velocity','linear_acceleration'):
                    value=getattr(msg,name)
                    row[name]=[getattr(value,x) for x in ('x','y','z','w') if hasattr(value,x)]
                for name in ('orientation_covariance','angular_velocity_covariance','linear_acceleration_covariance'):
                    row[name]=np.asarray(getattr(msg,name),dtype='<f8').tobytes().hex()
            rows[connection.topic].append(row)
    return {k:sorted(v,key=lambda r:(r['record_ns'],r['stamp_ns'])) for k,v in rows.items()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source');parser.add_argument('converted');parser.add_argument('--output')
    args=parser.parse_args()
    source=payloads(args.source);converted=payloads(args.converted)
    ok=bool(source) and source==converted
    result=dict(schema='wla-calibration-conversion-check/v1',matched=ok,
        source_counts={k:len(v) for k,v in source.items()},converted_counts={k:len(v) for k,v in converted.items()},
        differing_topics=[k for k in source.keys()|converted.keys() if source.get(k)!=converted.get(k)],
        scope='Image/Imu payloads, header and record timestamps; not calibration quality')
    text=json.dumps(result,ensure_ascii=False,indent=2);print(text)
    if args.output:Path(args.output).write_text(text+'\n')
    return 0 if ok else 1


if __name__=='__main__':raise SystemExit(main())
