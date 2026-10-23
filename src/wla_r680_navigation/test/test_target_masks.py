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


def test_v2_target_box_excludes_all_live_depth_in_box_but_not_outside():
    d,m,p,a=fixture()
    p['bbox_xyxy']=[8.,8.,34.,42.]
    # A segmentation hole and a farther surface both lie in the selected box.
    d[12,31]=1.2
    result,status,confirmed=filter_target_depth(d,p,mode='bbox',return_confirmed=True,**a)
    assert status['applied'] and status['source']=='target_bbox'
    assert np.isnan(result[12,31])
    assert np.isnan(result[20,20]) and result[20,16]==.5
    assert np.isnan(result[9,9])  # Even uncertain background is not a live obstacle hit.
    np.testing.assert_equal(result[~_box_region(d.shape,p['bbox_xyxy'])],d[~_box_region(d.shape,p['bbox_xyxy'])])
    assert confirmed[20,20] and not confirmed[9,9]
    assert not confirmed[20,16]  # Closer, non-target surface is never ghost-cleared.
    assert status['foreground_preserved_pixels']==25


def _box_region(shape,box):
    region=np.zeros(shape,bool)
    region[int(box[1]):int(box[3]),int(box[0]):int(box[2])]=True
    return region


def test_v2_box_works_without_segmentation_but_cannot_retire_old_hits():
    d,m,p,a=fixture();p['bbox_xyxy']=[10,10,30,40];p['bits']=''
    result,status,confirmed=filter_target_depth(d,p,mode='bbox',return_confirmed=True,**a)
    assert status['applied'] and status['source']=='target_bbox'
    assert np.isnan(result[15,15])
    assert result[20,16]==.5
    assert not confirmed.any()


def test_v2_box_rejects_bad_geometry_and_stale_identity():
    d,m,p,a=fixture();p['bbox_xyxy']=[-100,10,30,40]
    result,status=filter_target_depth(d,p,mode='bbox',**a)
    assert status['applied'] and status['source']=='target_segmentation'
    np.testing.assert_equal(result[~m],d[~m])
    p['bbox_xyxy']=[10,10,30,40];a['target_id']='other'
    result,status=filter_target_depth(d,p,mode='bbox',**a)
    assert not status['applied'];np.testing.assert_equal(result,d)


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
