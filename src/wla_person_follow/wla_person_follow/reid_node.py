"""Read-only ReID sidecar: exact RGB/detection/geometry stamps, bounded CPU work."""
from collections import OrderedDict
import argparse
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from wla_r680_navigation_py.perception_geometry import color_array
from .reid_features import AppearanceEncoder,person_crops


def stamp_ns(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


class ReIDNode(Node):
    def __init__(self,encoder,hz=5.):
        super().__init__('person_reid_features')
        self.encoder=encoder;self.hz=hz;self.guard=threading.Lock()
        self.rgb=OrderedDict();self.raw=OrderedDict();self.relative=OrderedDict()
        self.done=threading.Event();self.wake=threading.Event();self.last=0;self.interest=None
        self.metrics=dict(state='waiting_for_rgb_detections',model_id=encoder.model_id)
        self.pub=self.create_publisher(String,'/r680/person_follow/reid_features',1)
        self.health=self.create_publisher(String,'/r680/person_follow/reid_status',1)
        self.create_subscription(Image,'/r680/d455/color/image_raw',self.on_rgb,qos_profile_sensor_data)
        self.create_subscription(String,'/r680/perception/detections',lambda m:self.on_json('raw',m),qos_profile_sensor_data)
        self.create_subscription(String,'/r680/perception/relative_objects',lambda m:self.on_json('relative',m),qos_profile_sensor_data)
        self.create_subscription(String,'/r680/person_follow/reid_request',self.on_interest,1)
        self.thread=threading.Thread(target=self.work,daemon=True);self.thread.start()
        self.create_timer(.5,self.status)

    def put(self,cache,key,value):
        with self.guard:
            cache[key]=value
            while len(cache)>16:cache.popitem(last=False)
        self.wake.set()

    def on_interest(self,msg):
        try:
            if len(msg.data)>4096:return
            d=json.loads(msg.data)
            if (d.get('schema')!='wla-reid-request/v1' or type(d['stamp_ns']) is not int
                    or type(d.get('urgent',False)) is not bool or type(d['enabled']) is not bool or not isinstance(d['allowed_ids'],list)
                    or len(d['allowed_ids'])>4 or not all(isinstance(x,str) for x in d['allowed_ids'])
                    or not isinstance(d['request_id'],str)
                    or not 0<=(self.get_clock().now().nanoseconds-d['stamp_ns'])/1e9<=.5):return
            with self.guard:self.interest=d
            self.wake.set()
        except (ValueError,KeyError,TypeError):return

    def on_rgb(self,msg):
        age=(self.get_clock().now().nanoseconds-stamp_ns(msg))/1e9
        if 0<=age<=.3 and msg.header.frame_id=='d455_color_optical_frame' and 0<msg.width*msg.height<=2_000_000:
            self.put(self.rgb,stamp_ns(msg),msg)

    def on_json(self,key,msg):
        try:
            if len(msg.data)>262144:return
            d=json.loads(msg.data);stamp=d['stamp_ns']
            if type(stamp) is not int or not 0<=(self.get_clock().now().nanoseconds-stamp)/1e9<=.3:return
            if not isinstance(d['objects'],list) or len(d['objects'])>64:return
            self.put(getattr(self,key),stamp,d)
        except (ValueError,KeyError,TypeError):return

    def status(self):
        now=self.get_clock().now().nanoseconds
        status=dict(self.metrics,stamp_ns=now)
        if status['state']=='ready' and (now-status['source_stamp_ns'])/1e9>.6:
            status['state']='waiting_for_fresh_features'
        self.health.publish(String(data=json.dumps(status)))

    def work(self):
        last_started=-math.inf;request_id=None
        while not self.done.is_set():
            self.wake.wait(.5)
            if self.done.is_set():return
            now=self.get_clock().now().nanoseconds
            with self.guard:
                self.wake.clear()
                interest=self.interest
                current=interest.get('request_id') if interest else None
                if current!=request_id:
                    request_id=current;last_started=-math.inf

                active=bool(interest and interest['enabled'] and 0<=(now-interest['stamp_ns'])/1e9<=.5)
                common=(set(self.rgb)&set(self.raw)&set(self.relative)) if active else set()
                ready=[s for s in common if s>self.last and 0<=(now-s)/1e9<=.3]
                rate=10. if interest and interest.get('urgent',False) else self.hz
                remaining=last_started+1./rate-time.monotonic()
                if ready and remaining<=0:
                    s=max(ready);image=self.rgb[s];raw=self.raw[s];relative=self.relative[s];self.last=s
                else:s=None
            if ready and remaining>0:
                # Coalesce arrivals while rate-limited; take the newest exact
                # stamp at the deadline, never queue frames for later inference.
                self.wake.wait(min(remaining,.1));self.wake.set();continue
            if s is not None:
                begun=time.monotonic()
                last_started=begun
                try:
                    if (raw['session'],raw['epoch'])!=(interest['session'],interest['epoch']):raise ValueError('request_generation_mismatch')
                    if (raw['session'],raw['epoch'])!=(relative['session'],relative['epoch']):raise ValueError('generation_mismatch')
                    if not relative['enabled'] or relative['base_frame']!='r680_mapping_floor':raise ValueError('invalid_relative_source')
                    # Geometry remains acquired at this exact RGB stamp, not at inference completion.
                    geometry={r['id']:r for r in relative['objects']}
                    bgr=color_array(image)
                    if bgr is None:raise ValueError('invalid_rgb')
                    selected=person_crops(bgr[:,:,::-1],raw['objects'],allowed_ids=set(interest['allowed_ids']),prefix=f"{raw['session']}:{raw['epoch']}:")
                    vectors=self.encoder.encode([crop for _,crop in selected]);rows=[]
                    for (obj,_),feature in zip(selected,vectors):
                        ident=f"{raw['session']}:{raw['epoch']}:{obj['track_id']}";g=geometry.get(ident)
                        if not g or g.get('label')!='person' or not g.get('detected') or g.get('yaw') is None or g.get('distance') is None:continue
                        if not all(math.isfinite(v) for v in [g['yaw'],g['distance']]):continue
                        rows.append(dict(id=ident,feature=feature.tolist(),yaw=g['yaw'],distance=g['distance']))
                    age=(self.get_clock().now().nanoseconds-s)/1e9
                    if not 0<=age<=.6:raise ValueError('reid_result_expired')
                    packet=dict(schema='wla-reid-features/v1',request_id=interest['request_id'],stamp_ns=s,session=raw['session'],epoch=raw['epoch'],
                        model_id=self.encoder.model_id,base_frame=relative['base_frame'],objects=rows)
                    with self.guard:
                        current=self.interest
                        if not current or not current['enabled'] or current['request_id']!=interest['request_id'] or not 0<=(self.get_clock().now().nanoseconds-current['stamp_ns'])/1e9<=.5:continue
                    self.pub.publish(String(data=json.dumps(packet,allow_nan=False)))
                    self.metrics=dict(state='ready',model_id=self.encoder.model_id,people=len(rows),
                        compute_ms=(time.monotonic()-begun)*1000.,source_age_ms=age*1000.,source_stamp_ns=s)
                except Exception as exc:
                    self.metrics=dict(state='rejected',model_id=self.encoder.model_id,reason=str(exc)[:160])
            if not active:self.metrics=dict(state='idle_no_target',model_id=self.encoder.model_id,people=0)

    def close(self):self.done.set();self.wake.set();self.thread.join(timeout=3.)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--model',required=True);parser.add_argument('--hz',type=float,default=5.)
    args,ros=parser.parse_known_args()
    if not 1<=args.hz<=5:parser.error('hz must be 1..5')
    encoder=AppearanceEncoder(args.model)
    rclpy.init(args=ros);node=ReIDNode(encoder,args.hz)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
