"""OSNet crop upload and FP16 engine selection."""
from pathlib import Path

import json

import numpy as np
import pytest
import torch

from wla_r680_navigation_py import reid_engine
from wla_r680_navigation_py.boxmot_reid import CachedBoxMOTReID
from wla_r680_navigation_py.reid_engine import EngineUnavailable, engine_dir, engine_file, fixed_batches, load_engines, sidecar, verify, write_sidecar

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


def test_fixed_batches_pad_and_keep_box_order():
    seen = []

    def run(size, part):
        seen.append((size, len(part)))
        assert len(part) == size
        return part.mean(dim=(2, 3))

    for count, sizes in ((1, [(1, 1)]), (2, [(4, 4)]), (4, [(4, 4)]), (5, [(4, 4), (1, 1)]), (7, [(4, 4), (4, 4)])):
        seen.clear()
        batch = torch.arange(count, dtype=torch.float32).reshape(-1, 1, 1, 1)*torch.ones(count, 3, 2, 2)
        output = fixed_batches(batch, run)
        assert seen == sizes and output.shape == (count, 3)
        assert output[:, 0].tolist() == list(range(count))


@pytest.fixture
def built(tmp_path):
    weights = tmp_path/'osnet.pth'
    weights.write_bytes(b'weights')
    for batch in reid_engine.BATCHES:
        engine_file(tmp_path, batch).write_bytes(b'engine%d' % batch)
    write_sidecar(tmp_path, weights=weights, tensorrt_version='test')
    return tmp_path, weights


def test_matching_sidecar_passes_and_any_change_refuses(built):
    folder, weights = built
    verify(folder, weights)
    weights.write_bytes(b'other')
    with pytest.raises(EngineUnavailable, match='weights_sha256'):
        verify(folder, weights)


def test_changed_engine_missing_engine_and_bad_sidecar_are_refused(built):
    folder, weights = built
    engine_file(folder, 4).write_bytes(b'tampered')
    with pytest.raises(EngineUnavailable, match='engine_sha256'):
        load_engines(folder, weights, 'cpu')
    engine_file(folder, 4).unlink()
    with pytest.raises(EngineUnavailable, match='engine_missing'):
        load_engines(folder, weights, 'cpu')
    sidecar(folder).write_text('{')
    with pytest.raises(EngineUnavailable, match='sidecar_unreadable'):
        load_engines(folder, weights, 'cpu')
    assert json


def test_empty_environment_value_selects_the_fp32_graph(monkeypatch):
    monkeypatch.setenv('WLA_REID_ENGINE', '')
    assert engine_dir() is None
    monkeypatch.delenv('WLA_REID_ENGINE')
    assert engine_dir() == reid_engine.DEFAULT_DIR


deployed = engine_file(reid_engine.DEFAULT_DIR, 4).is_file() and torch.cuda.is_available()


@needs_weights
@pytest.mark.skipif(not deployed, reason='needs the deployed engines and a GPU')
def test_deployed_engine_features_match_the_fp32_graph(monkeypatch):
    image = np.random.default_rng(1).integers(0, 255, (480, 640, 3), dtype=np.uint8)
    boxes = np.array([[10, 20, 200, 400], [300, 40, 500, 470], [50, 60, 180, 300], [400, 100, 620, 460], [5, 5, 90, 200]], np.float32)
    engine = CachedBoxMOTReID(WEIGHTS)
    assert engine.backend_name == 'tensorrt_fp16'
    engine.warm()
    monkeypatch.setenv('WLA_REID_ENGINE', '')
    graph = CachedBoxMOTReID(WEIGHTS)
    assert graph.backend_name == 'cuda_graph_fp32' and graph.model_id == engine.model_id
    graph.warm()
    for count in (1, 2, 5):
        cosine = (engine.get_features(boxes[:count], image)*graph.get_features(boxes[:count], image)).sum(1)
        assert cosine.min() > .999
