"""OSNet crop upload and FP16 engine selection."""
from pathlib import Path

import numpy as np
import pytest

from wla_r680_navigation_py.boxmot_reid import CachedBoxMOTReID

WEIGHTS = Path.home()/'Wheel_Legged_Agent/models/perception/reid/osnet_x0_25_msmt17.pth'
needs_weights = pytest.mark.skipif(not WEIGHTS.is_file(), reason='OSNet weights are not on this machine')


@needs_weights
def test_batched_crops_match_boxmot_exactly():
    reid = CachedBoxMOTReID(WEIGHTS, 'cpu')
    assert 'get_crops' in vars(reid.backend)
    image = np.random.default_rng(0).integers(0, 255, (480, 640, 3), dtype=np.uint8)
    boxes = np.array([[10.4, 20.6, 200.2, 400.7], [600, 300, 700, 500], [-30, -20, 40, 60],
                      [700, 500, 800, 600], [50, 50, 50, 50]], np.float32)
    batched = reid.backend.get_crops(boxes, image)
    original = type(reid.backend).get_crops(reid.backend, boxes, image)
    assert batched.is_contiguous() and batched.dtype == original.dtype
    assert (batched == original).all()


@needs_weights
def test_environment_selects_the_boxmot_crops(monkeypatch):
    monkeypatch.setenv('WLA_REID_CROPS', 'boxmot')
    assert 'get_crops' not in vars(CachedBoxMOTReID(WEIGHTS, 'cpu').backend)
