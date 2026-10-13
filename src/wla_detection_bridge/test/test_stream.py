import copy
import json
from types import SimpleNamespace as NS
import pytest
from wla_detection_bridge.stream import DetectionInbox, decode, encode_batch, select


def image(ns=1_000_000_000, frame='camera'):
    return NS(width=64, height=48, header=NS(frame_id=frame,
        stamp=NS(sec=ns//10**9, nanosec=ns%10**9)), k=[32.,0.,32.,0.,32.,24.,0.,0.,1.])


def value(session='a', epoch=0, ns=1_000_000_000):
    return json.loads(encode_batch(image(ns), session, epoch,
        [dict(track_id=i, label=label, confidence=.9, bbox_xyxy=[10.,8.,50.,44.])
         for i,label in enumerate(['person', 'chair', 'cup'])]))


def test_independent_white_lists():
    batch=decode(value(), 1_010_000_000)
    assert [o['label'] for o in select(batch,['chair'])] == ['chair']
    assert [o['label'] for o in select(batch,['person','cup'])] == ['person','cup']
    assert [o['label'] for o in select(batch,['person'])] == ['person']
    assert len(batch['objects']) == 3


@pytest.mark.parametrize('mutate', [
    lambda b: b.update(stamp_ns=0), lambda b: b.update(stamp_ns=2_000_000_000),
    lambda b: b.update(stamp_ns=100_000_000), lambda b: b.update(width=0),
    lambda b: b['objects'][0].update(confidence=float('nan')),
    lambda b: b['objects'][0].update(bbox_xyxy=[0,0,1000,20]),
    lambda b: b['objects'][1].update(track_id=0),
    lambda b: b.update(epoch=True), lambda b: b.update(objects=None),
])
def test_invalid_evidence(mutate):
    batch=value();mutate(batch)
    with pytest.raises(ValueError):decode(batch, 1_010_000_000)


def test_join_requires_original_frame_and_consumes_once():
    inbox=DetectionInbox()
    inbox.push(value(), 1_010_000_000, 3.)
    for queue in (inbox.depth,inbox.info):queue.append(image())
    inbox.rgb.append(image(1_010_000_000))
    assert inbox.take(1_020_000_000,3.02) is None
    inbox.rgb.append(image())
    assert inbox.take(1_020_000_000,3.02)[0]['stamp_ns'] == 1_000_000_000
    assert inbox.take(1_020_000_000,3.02) is None


def test_mapping_does_not_need_color_but_checks_alignment():
    inbox=DetectionInbox()
    inbox.push(value(), 1_010_000_000, 3.)
    inbox.depth.append(image(frame='unaligned_depth'))
    inbox.info.append(image())
    assert inbox.take(1_020_000_000,3.02,False) is None
    inbox.depth.clear();inbox.depth.append(image())
    assert inbox.take(1_020_000_000,3.02,False) is not None


def test_restart_and_retired_messages_cannot_reuse_identity():
    inbox=DetectionInbox()
    assert inbox.push(value(),1_000_000_000,3.)
    assert not inbox.push(value(ns=1_100_000_000),1_100_000_000,3.1)
    assert inbox.push(value(epoch=1, ns=1_200_000_000),1_200_000_000,3.2)
    with pytest.raises(ValueError): inbox.push(value(ns=1_210_000_000),1_210_000_000,3.21)
    assert inbox.push(value('b', ns=1_300_000_000),1_300_000_000,3.3)
    with pytest.raises(ValueError): inbox.push(value(epoch=1, ns=1_310_000_000),1_310_000_000,3.31)


def test_expiry_covers_ros_and_receive_clocks():
    for source_now,mono_now in ((1_500_000_000,3.02),(1_020_000_000,3.5)):
        inbox=DetectionInbox()
        inbox.push(value(),1_000_000_000,3.)
        for queue in (inbox.rgb,inbox.depth,inbox.info):queue.append(image())
        assert inbox.take(source_now,mono_now) is None


def test_duplicate_and_empty_result():
    inbox=DetectionInbox()
    batch=value();batch['objects']=[]
    assert inbox.push(batch,1_000_000_000,3.)
    with pytest.raises(ValueError): inbox.push(batch,1_000_000_000,3.)
    for queue in (inbox.rgb,inbox.depth,inbox.info):queue.append(image())
    assert inbox.take(1_020_000_000,3.02)[0]['objects'] == []
