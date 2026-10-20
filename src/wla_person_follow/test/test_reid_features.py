"""Crop selection and actual pinned-model smoke tests; no ROS or motion."""
import os
import numpy as np
import pytest
from wla_person_follow.reid_features import person_crops,AppearanceEncoder


def box(i,b):return dict(track_id=i,label='person',confidence=.9,bbox_xyxy=b)


def test_only_requested_person_is_cropped():
    image=np.zeros((480,640,3),np.uint8)
    objects=[box(1,[20,20,100,300]),box(2,[200,20,280,300])]
    assert [r['track_id'] for r,_ in person_crops(image,objects,allowed_ids={'s:1:2'},prefix='s:1:')]==[2]
    assert person_crops(image,objects,allowed_ids=set(),prefix='s:1:')==[]


def test_overlap_and_degenerate_crops_are_rejected():
    image=np.zeros((480,640,3),np.uint8)
    objects=[box(1,[20,20,100,300]),box(2,[30,30,105,305]),box(3,[300,20,310,40])]
    assert person_crops(image,objects,allowed_ids={'s:1:1'},prefix='s:1:')==[]


def test_official_model_descriptor_is_finite_normalized_repeatable():
    path=os.environ.get('REID_MODEL')
    if not path:pytest.skip('pinned model smoke test opt-in')
    encoder=AppearanceEncoder(path)
    crop=np.random.default_rng(1).integers(0,255,(256,128,3),dtype=np.uint8)
    features=encoder.encode([crop,crop])
    assert features.shape==(2,512)
    np.testing.assert_allclose(features[0],features[1],atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(features,axis=1),1.,atol=1e-6)


def test_frozen_descriptor_matches_eager_fp32():
    import torch
    from wla_person_follow.vendor.osnet import osnet_x0_25
    path=os.environ.get('REID_MODEL')
    if not path:pytest.skip('pinned model parity test opt-in')
    encoder=AppearanceEncoder(path);frozen=encoder.model
    eager=osnet_x0_25(num_classes=1,pretrained=False).eval()
    weights=torch.load(path,map_location='cpu',weights_only=True)
    weights=weights.get('state_dict',weights)
    eager.load_state_dict({k.removeprefix('module.'):v for k,v in weights.items()
        if not k.removeprefix('module.').startswith('classifier.')},strict=False)
    rng=np.random.default_rng(187)
    crops=[rng.integers(0,256,(h,w,3),np.uint8) for h,w in [(128,64),(256,128),(370,115),(92,50)]]
    for n in (1,2,4):
        encoder.model=eager;a=encoder.encode(crops[:n])
        encoder.model=frozen;b=encoder.encode(crops[:n])
        np.testing.assert_allclose(a,b,rtol=0,atol=2e-6)
        assert np.min(np.sum(a*b,axis=1))>1.-1e-6
