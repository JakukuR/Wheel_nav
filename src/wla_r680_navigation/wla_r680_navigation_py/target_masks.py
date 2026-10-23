"""Bounded target mask/box wire format and exact RGB/aligned-depth pairing.

Filtered pixels do not predict free space. Removed pixels produce neither
obstacle hits nor clearing rays; only mask-confirmed target pixels may retire
old hits. Missing/invalid/expired target packets leave depth unchanged.
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


def filter_target_depth(depth,packet,*,target_id,request_id,session,epoch,depth_stamp_ns,frame_id,now_ns,
                        mode='segmentation',return_confirmed=False):
    """Drop same-frame target pixels; only mask-confirmed pixels may retire old hits.

    In bbox mode the full current target box is considered, while a distinctly
    nearer surface remains an obstacle. This does not certify free space.
    """
    status=dict(applied=False,removed_pixels=0,reason='no_confirmed_target_mask')
    confirmed=np.zeros(depth.shape,dtype=bool)
    def finish(result):
        return (result,status,confirmed) if return_confirmed else (result,status)
    if mode not in ('segmentation','bbox'):raise ValueError('invalid_target_filter_mode')
    if not packet or not target_id:return finish(depth)
    if (packet.get('schema')!='wla-target-mask/v1' or packet.get('target_id')!=target_id
            or packet.get('request_id')!=request_id or packet.get('session')!=session or packet.get('epoch')!=epoch
            or packet.get('depth_stamp_ns')!=depth_stamp_ns or packet.get('frame_id')!=frame_id
            or not 0<=(now_ns-packet.get('stamp_ns',0))/1e9<=.30
            or abs(packet['stamp_ns']-depth_stamp_ns)>35_000_000
            or (packet.get('height'),packet.get('width'))!=depth.shape):
        status['reason']='mask_identity_or_stamp_mismatch';return finish(depth)
    if packet.get('bits'):
        try:
            mask=decode_mask(packet['bits'],packet['height'],packet['width'])
        except (ValueError,TypeError,zlib.error):
            status['reason']='invalid_mask_payload'
        else:
            valid=mask&np.isfinite(depth)&(depth>0)
            # Only the dominant target surface band can retire old map hits.
            if np.count_nonzero(valid)>=10:
                median=float(np.median(depth[valid]))
                confirmed=valid&(np.abs(depth-median)<=.35)
                status['surface_depth_m']=median
            else:
                status['reason']='insufficient_target_depth'
    if mode=='bbox':
        box=packet.get('bbox_xyxy')
        if isinstance(box,(list,tuple)) and len(box)==4:
            try:
                x1,y1,x2,y2=(float(v) for v in box)
                h,w=depth.shape
                if (all(np.isfinite((x1,y1,x2,y2))) and 0<=x1<x2<=w and 0<=y1<y2<=h):
                    region=np.zeros(depth.shape,dtype=bool)
                    ix1,iy1,ix2,iy2=int(np.floor(x1)),int(np.floor(y1)),int(np.ceil(x2)),int(np.ceil(y2))
                    region[iy1:iy2,ix1:ix2]=True
                    reference=status.get('surface_depth_m')
                    reference_source='target_segmentation' if reference is not None else 'bbox_center'
                    if reference is None:
                        # Broad center patch is less sensitive to one small
                        # foreground object than a single center depth pixel.
                        xs=ix1+(ix2-ix1)//4;xe=ix2-(ix2-ix1)//4
                        ys=iy1+(iy2-iy1)//4;ye=iy2-(iy2-iy1)//4
                        center=depth[ys:ye,xs:xe]
                        samples=center[np.isfinite(center)&(center>0)]
                        if len(samples)>=10:reference=float(np.median(samples))
                    if reference is None:
                        status['reason']='insufficient_bbox_depth'
                        return finish(depth)
                    present=region&np.isfinite(depth)&(depth>0)
                    near=present&(depth<reference-.35)
                    remove=present&~near
                    if np.any(remove):
                        confirmed &= remove
                        result=depth.copy();result[remove]=np.nan
                        status.update(applied=True,removed_pixels=int(np.count_nonzero(remove)),
                                      confirmed_pixels=int(np.count_nonzero(confirmed)),
                                      foreground_preserved_pixels=int(np.count_nonzero(near)),
                                      reference_depth_m=reference,reference_source=reference_source,
                                      reason='current_target_bbox',source='target_bbox')
                        return finish(result)
            except (TypeError,ValueError,OverflowError):
                pass
        status['reason']='invalid_or_missing_target_bbox'
    if np.any(confirmed):
        result=depth.copy();result[confirmed]=np.nan
        status.update(applied=True,removed_pixels=int(np.count_nonzero(confirmed)),
                      confirmed_pixels=int(np.count_nonzero(confirmed)),
                      reason='current_target_segmentation',source='target_segmentation')
        return finish(result)
    return finish(depth)


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
