"""Metadata exact join and asynchronous observation export; never owns inference."""
from collections import OrderedDict
from hashlib import sha256
import json
from pathlib import Path
import threading
import time
from uuid import uuid4


class ObservationExporter:
    def __init__(self, path, *, sensor_id, provenance, max_age_s=.5, metadata_items=64, clock_jump_s=.05, queue_items=2, queue_bytes=16_000_000, ipc_timeout_s=.15):
        from wla_sim.perception.observation_ipc import ObservationSender
        self.sensor_id=sensor_id;self.provenance=provenance;self.max_age=max_age_s;self.metadata_items=metadata_items
        self.metadata=OrderedDict();self.lock=threading.Lock();self.camera_session=uuid4().hex
        self.boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.clock_jump_s=clock_jump_s
        self.offset=None;self.last_number=None;self.last_stamp=None;self.last_meta_receive=None
        self.metrics=dict(metadata_rejected=0,join_missing=0,clock_reset=0)
        self.sender=ObservationSender(path,self.prepare,max_items=queue_items,max_bytes=queue_bytes,timeout_s=ipc_timeout_s)

    def on_metadata(self,msg):
        try:
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if len(msg.json_data)>32768:raise ValueError('oversized_metadata')
            data=json.loads(msg.json_data)
            if type(data.get('frame_number')) is not int or data['frame_number']<0 or data.get('clock_domain') not in ('global_time','system_time','hardware_clock'):
                raise ValueError('invalid_camera_metadata')
            with self.lock:
                now=time.monotonic()
                self.metadata[(stamp,msg.header.frame_id)]=(data,now)
                while self.metadata and (len(self.metadata)>self.metadata_items or now-next(iter(self.metadata.values()))[1]>self.max_age):self.metadata.popitem(last=False)
        except (ValueError,TypeError,KeyError):self.metrics['metadata_rejected']+=1

    def offer(self,message,batch,relative,received_mono,received_ros):
        # Called after Nav publication. No hash, image copy or blocking socket here.
        self.sender.offer((message,batch,relative,received_mono,received_ros),len(message.data))

    def prepare(self,item):
        message,batch,relative,received_mono,received_ros=item
        stamp=batch['stamp_ns']
        if time.monotonic()-received_mono>self.max_age:raise ValueError('export_expired')
        with self.lock:
            match=self.metadata.get((stamp,message.header.frame_id))
        if not match:
            self.metrics['join_missing']+=1;raise ValueError('exact_metadata_missing')
        metadata,arrived=match
        if time.monotonic()-arrived>self.max_age:raise ValueError('metadata_expired')
        if message.encoding not in ('rgb8','bgr8') or message.is_bigendian:raise ValueError('unsupported_image_encoding')
        offset=received_ros/1e9-received_mono
        # Preserve source domain; scheduling uses the ROS->same-host monotonic mapping.
        if (self.offset is not None and abs(offset-self.offset)>self.clock_jump_s or
                self.last_number is not None and metadata['frame_number']<=self.last_number or
                self.last_stamp is not None and stamp<=self.last_stamp or
                self.last_meta_receive is not None and received_mono-self.last_meta_receive>self.max_age):
            self.camera_session=uuid4().hex;self.metrics['clock_reset']+=1
        self.offset=offset;self.last_number=metadata['frame_number'];self.last_stamp=stamp;self.last_meta_receive=received_mono
        source_session=self.camera_session+':'+batch['session']+':'+str(batch['epoch'])
        ident=f"{self.sensor_id}:color:{source_session}:{metadata['frame_number']}"
        captured=float(metadata['frame_timestamp'])/1000.
        timeline=received_mono-(received_ros-stamp)/1e9
        rows=[dict(r,track_id=f"{batch['session']}:{batch['epoch']}:{r['track_id']}",
                   source_frame_id=ident,observed_at_s=captured) for r in batch['objects']]
        pixels=bytes(message.data)
        geometry=relative if relative and relative.get('stamp_ns')==stamp else None
        valid=bool(geometry and any(r.get('distance') is not None for r in geometry.get('objects',())))
        header=dict(source_frame_id=ident,sensor_id=self.sensor_id,captured_at_s=captured,
            clock_domain=metadata['clock_domain'],timeline_at_s=timeline,timeline_clock_id=self.boot_id,
            source_session=source_session,image_sha256=sha256(pixels).hexdigest(),image_format=message.encoding,
            width=message.width,height=message.height,stride=message.step,detections=rows,tracks=rows,action_events=[],
            localization_status=dict(status='available' if valid else 'unavailable',provider='nav_rgbd_relative',
                reason='matched_batch_geometry' if valid else 'depth_or_tf_unavailable',
                depth_stamp_ns=geometry.get('depth_stamp_ns') if geometry else None,
                timeline_at_s=received_mono-(received_ros-geometry['depth_stamp_ns'])/1e9 if geometry and geometry.get('depth_stamp_ns') else None,
                objects=geometry.get('objects',[]) if geometry else []),
            detector_provenance=dict(self.provenance,session=batch['session'],epoch=batch['epoch'],
                ros_stamp_ns=stamp,optical_frame_id=message.header.frame_id,
                clock_mapping='ros_system_to_host_monotonic',mapping_offset_s=offset,
                mapping_jump_limit_s=self.clock_jump_s,source_max_age_s=self.max_age,metadata_items=self.metadata_items,
                queue_items=self.sender.max_items,queue_bytes=self.sender.max_bytes,ipc_timeout_s=self.sender.timeout_s),raw_metadata=metadata)
        return header,pixels

    def close(self):self.sender.close()
