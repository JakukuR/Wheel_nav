import base64,zlib
import numpy as np
import pytest
from wla_r680_navigation_py.target_masks import encode_mask,decode_mask,filter_target_depth,MaskDepthPairer


def fixture():
    depth=np.full((48,64),3.,np.float32)
    mask=np.zeros(depth.shape,bool);mask[10:40,10:30]=True
    depth[mask]=1.2
    # Foreground obstacle within the target silhouette remains an obstacle.
    depth[20:25,15:20]=.5
    p=dict(schema='wla-target-mask/v1',stamp_ns=1_000_000_000,depth_stamp_ns=1_000_000_000,
        height=48,width=64,frame_id='optical',target_id='s:1:9',request_id='task',session='s',epoch=1,bits=encode_mask(mask))
    args=dict(target_id='s:1:9',request_id='task',session='s',epoch=1,depth_stamp_ns=1_000_000_000,frame_id='optical',now_ns=1_050_000_000)
    return depth,mask,p,args


def test_target_silhouette_only_preserves_background_and_front_object():
    d,m,p,a=fixture();result,status=filter_target_depth(d,p,**a)
    assert status['applied'] and status['removed_pixels']>300
    np.testing.assert_equal(result[~m],d[~m])
    assert np.all(result[20:25,15:20]==.5)
    assert np.isnan(result[10,10])
    assert np.isnan(result[15,15]) and np.isfinite(d).all()


@pytest.mark.parametrize('change',[{'target_id':'s:1:8'},{'target_id':None},{'request_id':'other'},
    {'epoch':2},{'session':'retired'},{'depth_stamp_ns':1_001_000_000},{'frame_id':'other'}, {'now_ns':1_301_000_000}])
def test_lost_stale_mispaired_and_other_target_never_remove(change):
    d,m,p,a=fixture();a.update(change);result,status=filter_target_depth(d,p,**a)
    assert not status['applied'];np.testing.assert_equal(result,d)


def test_codec_bounded_and_roundtrip():
    d,m,p,a=fixture();np.testing.assert_equal(decode_mask(p['bits'],48,64),m)
    bomb=base64.b64encode(zlib.compress(bytes(2_000_000))).decode()
    with pytest.raises(ValueError):decode_mask(bomb,48,64)
    with pytest.raises(ValueError):decode_mask(p['bits'],4800,6400)
    p['bits']='corrupt';result,status=filter_target_depth(d,p,**a)
    assert not status['applied'];np.testing.assert_equal(result,d)


def test_exact_pair_out_of_order_and_bounded_fallback():
    q=MaskDepthPairer();s=1_000_000_000
    q.put_depth(s,'first');q.put_depth(s+33_000_000,'second')
    assert q.take(s+60_000_000,True) is None
    q.put_mask({'depth_stamp_ns':s,'target_id':'a'})
    assert q.take(s+70_000_000,True)==('first',{'depth_stamp_ns':s,'target_id':'a'})
    assert q.take(s+100_000_000,True) is None
    assert q.take(s+160_000_000,True)==('second',None)
    q.put_mask({'depth_stamp_ns':s})
    assert not q.masks
    q.put_mask({'depth_stamp_ns':s+200_000_000})
    q.put_depth(s+200_000_000,'third')
    assert q.take(s+210_000_000,True)[0]=='third'


def test_idle_does_not_wait_and_expired_depth_is_not_processed():
    q=MaskDepthPairer();q.put_depth(1_000_000_000,'frame')
    assert q.take(1_010_000_000,False)==('frame',None)
    q.put_depth(2_000_000_000,'stale')
    assert q.take(2_400_000_000,True) is None
