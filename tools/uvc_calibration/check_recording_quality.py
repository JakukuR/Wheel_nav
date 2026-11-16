#!/usr/bin/env python3
"""Read-only quality audit of finalized, unchunked ROS2 MCAP camera bags.

The fastwrite recording has unchunked Message records. Seek over unsampled
payloads rather than transferring or decoding the whole 122 GB recording.
Chunked input is rejected explicitly. This is a sampling audit, not a solver.
"""
import argparse
from bisect import bisect_left
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import json
import multiprocessing
from pathlib import Path
import struct
import time

import cv2
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

MAGIC = b'\x89MCAP0\r\n'
TOPICS = {'left':'/camera/sunplus/left/image_raw',
          'right':'/camera/sunplus/right/image_raw',
          'rgb':'/camera/dicota/color/image_raw'}


def scan_bag(bag):
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    expected = {r['topic_metadata']['name']:r['message_count'] for r in metadata['topics_with_message_count']}
    indexed = {eye:[] for eye in TOPICS}
    by_topic = {topic:eye for eye,topic in TOPICS.items()}
    counts = Counter()
    for number,name in enumerate(metadata['relative_file_paths']):
        path = (bag/name).resolve()
        if not path.is_relative_to(bag.resolve()):
            raise ValueError('MCAP path escapes bag directory')
        channels = {}
        file_size = path.stat().st_size
        with path.open('rb') as stream:
            if stream.read(8) != MAGIC:
                raise ValueError(f'bad leading magic: {name}')
            stream.seek(-8,2)
            if stream.read(8) != MAGIC:
                raise ValueError(f'unfinalized/truncated MCAP: {name}')
            stream.seek(8)
            footer = False
            while stream.tell() < file_size-8:
                header = stream.read(9)
                if len(header) != 9:
                    raise ValueError(f'truncated record header: {name}')
                opcode,size = struct.unpack('<BQ',header)
                begin = stream.tell()
                if begin+size > file_size-8:
                    raise ValueError(f'record exceeds file: {name}')
                if opcode == 6:
                    raise ValueError('chunked MCAP requires native MCAP/rosbag reader; audit aborted')
                if opcode == 4:
                    if size < 8:
                        raise ValueError('short Channel record')
                    raw = stream.read(size)
                    channel_id,_,length = struct.unpack_from('<HHI',raw)
                    topic = raw[8:8+length].decode('utf8')
                    encoding_length = struct.unpack_from('<I',raw,8+length)[0]
                    encoding = raw[12+length:12+length+encoding_length].decode('utf8')
                    channels[channel_id] = (topic,encoding)
                elif opcode == 5:
                    if size < 22:
                        raise ValueError('short Message record')
                    channel_id,sequence,log_time,publish_time = struct.unpack('<HIQQ',stream.read(22))
                    topic,encoding = channels[channel_id]
                    counts[topic] += 1
                    if topic in by_topic:
                        payload = stream.tell()
                        raw = stream.read(12)
                        if encoding != 'cdr' or len(raw) != 12 or raw[:2] != b'\x00\x01':
                            raise ValueError('expected little-endian plain ROS2 CDR Image')
                        sec,ns = struct.unpack_from('<iI',raw,4)
                        if not 0 <= ns < 1_000_000_000:
                            raise ValueError('invalid Image header stamp')
                        indexed[by_topic[topic]].append(dict(file=str(path),offset=payload,
                            size=size-22,stamp=sec*1_000_000_000+ns,log_time=log_time))
                elif opcode == 2:
                    footer = True
                    break
                stream.seek(begin+size)
            if not footer:
                raise ValueError(f'missing footer: {name}')
        if (number+1)%10 == 0:
            print(f'Indexed {number+1}/{len(metadata["relative_file_paths"])} MCAP files',flush=True)
    if dict(counts) != expected:
        raise ValueError(f'MCAP counts disagree with metadata: {dict(counts)} vs {expected}')
    return metadata,indexed,dict(counts)


def load_image(entry):
    with open(entry['file'],'rb') as stream:
        stream.seek(entry['offset'])
        raw = stream.read(entry['size'])
    if len(raw) != entry['size']:
        raise ValueError('truncated Image payload')
    msg = deserialize_message(raw,Image)
    if msg.encoding != 'bgr8' or msg.step < msg.width*3 or len(msg.data) != msg.height*msg.step:
        raise ValueError('invalid BGR8 Image layout')
    actual = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
    if actual != entry['stamp']:
        raise ValueError('index/header stamp mismatch')
    pixels = np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)
    return pixels[:,:msg.width*3].reshape(msg.height,msg.width,3)


def detect(entry):
    cv2.setNumThreads(1)
    image = load_image(entry)
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters_create()
    params.markerBorderBits = 2
    params.adaptiveThreshWinSizeMax = 53
    params.minMarkerPerimeterRate = .015
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners,ids,_ = cv2.aruco.detectMarkers(gray,dictionary,parameters=params)
    keep = [] if ids is None else [(int(i),c.reshape(4,2)) for i,c in zip(ids.ravel(),corners) if 0 <= i < 36]
    result = dict(entry,width=image.shape[1],height=image.shape[0],
        ids=sorted({i for i,_ in keep}),corners={str(i):c.tolist() for i,c in keep})
    result['tag_count'] = len(result['ids'])
    if keep:
        points = np.concatenate([c for _,c in keep])
        x0,y0 = np.floor(points.min(axis=0)).astype(int)
        x1,y1 = np.ceil(points.max(axis=0)).astype(int)
        x0,y0=max(0,x0),max(0,y0)
        roi=gray[y0:min(gray.shape[0],y1+1),x0:min(gray.shape[1],x1+1)]
        result['bbox_fraction'] = float((x1-x0)*(y1-y0)/gray.size)
        result['board_laplacian_variance'] = float(cv2.Laplacian(roi,cv2.CV_64F).var()) if roi.size else 0.
        result['median_tag_edge_px'] = float(np.median([np.linalg.norm(c-np.roll(c,1,axis=0),axis=1).mean() for _,c in keep]))
    else:
        result.update(bbox_fraction=0.,board_laplacian_variance=0.,median_tag_edge_px=0.)
    return result


def time_statistics(entries):
    stamps = np.asarray([e['stamp'] for e in entries],dtype=np.int64)
    if len(stamps) < 2:
        raise ValueError('camera stream is empty/too short')
    dt = np.diff(stamps)/1e6
    return dict(count=len(stamps),duration_s=float((stamps[-1]-stamps[0])/1e9),
        average_hz=float((len(stamps)-1)*1e9/(stamps[-1]-stamps[0])),
        duplicate_or_reverse_stamps=int(np.count_nonzero(dt<=0)),
        median_dt_ms=float(np.median(dt)),p95_dt_ms=float(np.percentile(dt,95)),
        max_gap_ms=float(dt.max()),gaps_over_100ms=int(np.count_nonzero(dt>100)))


def nearest(entries,stamps,target):
    i=bisect_left(stamps,target)
    candidates=[j for j in (i-1,i) if 0 <= j < len(entries)]
    return min(candidates,key=lambda j:abs(stamps[j]-target))


def write_examples(results,out,start):
    strips=[]
    for eye in TOPICS:
        frames=[r for r in results if r['eye']==eye]
        chosen=[]
        for r in sorted(frames,key=lambda r:(r['tag_count'],r['bbox_fraction']),reverse=True):
            if all(abs(r['stamp']-other['stamp'])>10e9 for other in chosen):
                chosen.append(r)
            if len(chosen)==6:break
        tiles=[]
        for number,r in enumerate(chosen):
            image=load_image(r).copy()
            for tag,points in r['corners'].items():
                points=np.asarray(points,np.int32)
                cv2.polylines(image,[points],True,(0,220,0),2)
                cv2.putText(image,tag,tuple(points[0]),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),2)
            thumb=cv2.resize(image,(480,270),interpolation=cv2.INTER_AREA)
            tile=np.full((305,480,3),24,np.uint8);tile[35:]=thumb
            caption=f"{eye}  t={(r['stamp']-start)/1e9:.1f}s  tags={r['tag_count']}"
            cv2.putText(tile,caption,(8,24),cv2.FONT_HERSHEY_SIMPLEX,.58,(230,230,230),1)
            tiles.append(tile)
            if number==0:
                cv2.imwrite(str(out/f'best_{eye}.jpg'),image)
                # Lossless full-resolution grayscale for a subsequent official-detector check.
                cv2.imwrite(str(out/f'best_{eye}.png'),cv2.cvtColor(load_image(r),cv2.COLOR_BGR2GRAY))
        while len(tiles)<6:tiles.append(np.zeros((305,480,3),np.uint8))
        strips.append(np.concatenate(tiles,axis=0))
    cv2.imwrite(str(out/'samples.jpg'),np.concatenate(strips,axis=1))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--period',type=float,default=2.)
    parser.add_argument('--workers',type=int,default=2)
    args=parser.parse_args()
    if args.period<=0 or not 1<=args.workers<=4:parser.error('invalid period/workers')
    args.output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    metadata,indexed,counts=scan_bag(args.bag)
    stats={eye:time_statistics(entries) for eye,entries in indexed.items()}
    for eye in indexed:
        if stats[eye]['duplicate_or_reverse_stamps']:
            raise ValueError('nonmonotonic stamps require inspection before nearest-time pairing')
    (args.output/'image_index.json').write_text(json.dumps(indexed))
    stamps={eye:[e['stamp'] for e in entries] for eye,entries in indexed.items()}
    start=max(s[0] for s in stamps.values());end=min(s[-1] for s in stamps.values())
    jobs=[];groups=[];used={}
    for slot,target in enumerate(range(start+500_000_000,end,int(args.period*1e9))):
        group={}
        for eye in TOPICS:
            i=nearest(indexed[eye],stamps[eye],target)
            key=(eye,i)
            if key not in used:
                used[key]=len(jobs);jobs.append(dict(indexed[eye][i],eye=eye,index=i))
            group[eye]=used[key]
        groups.append(group)
    results=[]
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('spawn')) as executor:
        for number,result in enumerate(executor.map(detect,jobs,chunksize=3)):
            results.append(result)
            if (number+1)%30==0:
                print(f'Detected {number+1}/{len(jobs)} sampled images; elapsed {time.monotonic()-started:.0f}s',flush=True)
    per_camera={}
    for eye in TOPICS:
        frames=[r for r in results if r['eye']==eye]
        accepted=[r for r in frames if r['tag_count']>=7]
        histogram=Counter(r['tag_count'] for r in frames)
        cells=np.zeros((3,4),int)
        for r in accepted:
            points=np.concatenate([np.asarray(c) for c in r['corners'].values()])
            x,y=points.mean(axis=0)
            cells[min(2,int(y/r['height']*3)),min(3,int(x/r['width']*4))]+=1
        per_camera[eye]=dict(sampled_frames=len(frames),any_tag_frames=sum(r['tag_count']>0 for r in frames),
            frames_ge7_tags=len(accepted),tag_count_histogram=dict(sorted(histogram.items())),
            max_tag_count=max((r['tag_count'] for r in frames),default=0),
            board_centroid_grid_3x4_for_ge7=cells.tolist())
    pair_results={}
    for a,b in (('left','right'),('left','rgb'),('right','rgb')):
        rows=[]
        for group in groups:
            ra,rb=results[group[a]],results[group[b]]
            skew=abs(ra['stamp']-rb['stamp'])/1e6
            common=sorted(set(ra['ids'])&set(rb['ids']))
            rows.append(dict(t_s=(min(ra['stamp'],rb['stamp'])-start)/1e9,
                skew_ms=skew,tags_a=ra['tag_count'],tags_b=rb['tag_count'],common_ids=common,
                both_ge7=ra['tag_count']>=7 and rb['tag_count']>=7))
        close=[r for r in rows if r['skew_ms']<=25]
        usable=[r for r in close if r['both_ge7'] and r['common_ids']]
        pair_results[f'{a}--{b}']=dict(sampled_pairs=len(rows),within_25ms=len(close),
            both_ge7_and_common_tags_within_25ms=len(usable),
            common_ge4_and_both_ge7_within_25ms=sum(len(r['common_ids'])>=4 for r in usable),
            max_common_tags=max((len(r['common_ids']) for r in close),default=0),
            usable_times_s=[round(r['t_s'],2) for r in usable],details=rows)
    exact_lr=len(set(stamps['left'])&set(stamps['right']))
    report=dict(schema='camera-recording-quality/v1',bag=str(args.bag),
        audit_type='all_message_header_scan_and_sparse_image_detection',crc_verified=False,
        indexed_counts_match_metadata=True,all_topic_counts=counts,time_statistics=stats,
        exact_left_right_header_pairs=exact_lr,sampling_period_s=args.period,
        detector='OpenCV aruco DICT_APRILTAG_36h11; markerBorderBits=2',
        kalibr_default_min_tags_for_this_6x6_board=7,
        note='OpenCV sampling is a proxy, not official Kalibr acceptance or metric calibration validation. 25ms is host timestamp skew, not exposure synchronization.',
        camera_quality=per_camera,pair_quality=pair_results,elapsed_s=time.monotonic()-started)
    (args.output/'detections.json').write_text(json.dumps(results,indent=2))
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    write_examples(results,args.output,start)
    summary={k:v for k,v in report.items() if k not in ('pair_quality','all_topic_counts')}
    summary['pair_quality']={k:{a:b for a,b in v.items() if a not in ('details','usable_times_s')} for k,v in pair_results.items()}
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':
    main()
