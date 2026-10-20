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
