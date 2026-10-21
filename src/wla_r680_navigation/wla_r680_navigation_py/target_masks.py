"""Bounded target mask wire format and exact RGB/aligned-depth pairing.

No mask predicts free space. Removed pixels produce neither obstacle hits nor
clearing rays; missing/invalid/expired masks leave the original depth intact.
"""
import base64
import zlib
import numpy as np


def encode_mask(mask):
    mask=np.asarray(mask,dtype=bool)
    if mask.ndim!=2 or not 0<mask.size<=2_000_000:raise ValueError('invalid_mask_shape')
    return base64.b64encode(zlib.compress(np.packbits(mask.reshape(-1)).tobytes(),1)).decode('ascii')


def decode_mask(bits,height,width):
    if type(height) is not int or type(width) is not int or min(height,width)<=0 or height*width>2_000_000:
        raise ValueError('invalid_mask_shape')
    if not isinstance(bits,str) or len(bits)>350000:raise ValueError('invalid_mask_size')
    size=(height*width+7)//8
    stream=zlib.decompressobj()
    data=stream.decompress(base64.b64decode(bits,validate=True),size+1)
    if len(data)!=size or not stream.eof or stream.unused_data or stream.unconsumed_tail:
        raise ValueError('invalid_mask_payload')
    return np.unpackbits(np.frombuffer(data,dtype=np.uint8),count=height*width).reshape(height,width).astype(bool)


def filter_target_depth(depth,packet,*,target_id,request_id,session,epoch,depth_stamp_ns,frame_id,now_ns):
    status=dict(applied=False,removed_pixels=0,reason='no_confirmed_target_mask')
    if not packet or not target_id:return depth,status
    if (packet.get('schema')!='wla-target-mask/v1' or packet.get('target_id')!=target_id
            or packet.get('request_id')!=request_id or packet.get('session')!=session or packet.get('epoch')!=epoch
            or packet.get('depth_stamp_ns')!=depth_stamp_ns or packet.get('frame_id')!=frame_id
            or not 0<=(now_ns-packet.get('stamp_ns',0))/1e9<=.30
            or abs(packet['stamp_ns']-depth_stamp_ns)>35_000_000
            or (packet.get('height'),packet.get('width'))!=depth.shape):
        status['reason']='mask_identity_or_stamp_mismatch';return depth,status
    if not packet.get('bits'):return depth,status
    try:
        mask=decode_mask(packet['bits'],packet['height'],packet['width'])
    except (ValueError,TypeError,zlib.error):
        status['reason']='invalid_mask_payload';return depth,status
    # Use the actual silhouette, not its box or an eroded rim that would
    # leave target-outline points marked as persistent obstacles.
    valid=mask&np.isfinite(depth)&(depth>0)
    # Disconnected depths inside a silhouette can be foreground objects or
    # background leaks. Only the dominant target surface band is removed.
    if np.count_nonzero(valid)<10:
        status['reason']='insufficient_target_depth';return depth,status
    median=float(np.median(depth[valid]))
    valid &= np.abs(depth-median)<=.35
    result=depth.copy();result[valid]=np.nan
    status.update(applied=True,removed_pixels=int(np.count_nonzero(valid)),reason='current_target_segmentation',surface_depth_m=median)
    return result,status


class MaskDepthPairer:
    """Newest matched acquisition wins; bounded .12 s unmasked fallback.

    Keeping a small acquisition cache is necessary: inference finishes after
    the matching depth callback. Reusing a mask on a newer depth is forbidden.
    """
    def __init__(self):
        from collections import OrderedDict
        self.depths=OrderedDict();self.masks=OrderedDict();self.last=0

    def put_depth(self,stamp,payload):
        if stamp>self.last:self.depths[stamp]=payload
        while len(self.depths)>16:self.depths.popitem(last=False)

    def put_mask(self,packet):
        stamp=packet.get('depth_stamp_ns')
        if type(stamp) is not int or stamp<=self.last:return
        self.masks[stamp]=packet
        while len(self.masks)>16:self.masks.popitem(last=False)

    def take(self,now_ns,active):
        ready=[s for s in self.depths if s>self.last and 0<=(now_ns-s)/1e9<=.30
               and (not active or s in self.masks or now_ns-s>=120_000_000)]
        if not ready:return None
        # Prefer the newest paired frame; an unpaired newer frame waits only
        # for its own bounded deadline, never reuses the previous segmentation.
        stamp=max(ready);payload=self.depths[stamp];mask=self.masks.get(stamp)
        self.last=stamp
        for cache in (self.depths,self.masks):
            for key in list(cache):
                if key<=stamp:cache.pop(key)
        return payload,mask
