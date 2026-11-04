"""FP16 engine selection and runtime options; no GPU, camera or motion."""
import json
import sys
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest
import torch

from wla_r680_navigation_py.boxmot_runtime import TrackingRuntime
from wla_r680_navigation_py import yoloe_engine
from wla_r680_navigation_py.yoloe_engine import ENGINE_IMGSZ, EngineUnavailable, engine_path, load_engine, write_sidecar

LABELS = ('person', 'chair')


class Encoder:
    model_id = 'fixture'
    last_ms = 0.
    def warm(self): self.last_features = np.empty((0, 512), np.float32)
    def get_features(self, boxes, image):
        self.last_features = np.full((len(boxes), 512), 1/np.sqrt(512), np.float32)
        return self.last_features


class Engine:
    wla_engine_imgsz = ENGINE_IMGSZ

    def __init__(self):
        self.calls = []
        # A real engine backend has no .model to take a graph from.
        self.predictor = NS(model=NS(device=NS(type='cuda')), inference='untouched')

    def predict(self, frame, **options):
        self.calls.append(options)
        data = torch.tensor([[10, 10, 50, 90, .9, 0]], dtype=torch.float32)
        return [NS(boxes=NS(data=data), names=dict(enumerate(LABELS)), masks=None)]


def built(tmp_path):
    for name in ('e.engine', 'w.pt', 't.ts'):
        (tmp_path/name).write_bytes(name.encode())
    write_sidecar(tmp_path/'e.engine', weights=tmp_path/'w.pt', text_encoder=tmp_path/'t.ts', classes=LABELS,
                  prompts=LABELS, vocabulary_sha256='v', tensorrt_version='10')
    return dict(weights=tmp_path/'w.pt', text_encoder=tmp_path/'t.ts', labels=LABELS, vocabulary_sha256='v')


def fake_ultralytics(monkeypatch, names):
    module = ModuleType('ultralytics')
    module.YOLO = lambda path, task: NS(names=names, path=path, task=task)
    monkeypatch.setitem(sys.modules, 'ultralytics', module)


def test_engine_runs_with_class_agnostic_nms_and_no_fp32_graph():
    engine = Engine()
    runtime = TrackingRuntime(engine, reid_backend=Encoder())
    frame = np.zeros((480, 640, 3), np.uint8)
    assert runtime.process(frame, imgsz=416, device='0', conf=.1) is None
    assert runtime.graph is None and engine.predictor.inference == 'untouched'
    assert runtime.last_metrics['backend'] == 'tensorrt_fp16'
    runtime.process(frame, imgsz=416, device='0', conf=.1)
    # Without agnostic_nms the engine keeps person boxes the .pt path suppresses.
    assert all(call['agnostic_nms'] is True and call['imgsz'] == ENGINE_IMGSZ for call in engine.calls)
    assert runtime.last_metrics['backend'] == 'tensorrt_fp16'


def test_matching_engine_loads_with_provenance(tmp_path, monkeypatch):
    fake_ultralytics(monkeypatch, dict(enumerate(LABELS)))
    model = load_engine(tmp_path/'e.engine', **built(tmp_path))
    assert model.wla_engine_imgsz == ENGINE_IMGSZ
    assert model.wla_yoloe_provenance['backend'] == 'tensorrt_fp16'


@pytest.mark.parametrize('change', ['labels', 'vocabulary_sha256', 'weights', 'text_encoder', 'engine'])
def test_any_changed_input_refuses_the_engine(tmp_path, monkeypatch, change):
    fake_ultralytics(monkeypatch, dict(enumerate(LABELS)))
    inputs = built(tmp_path)
    if change == 'labels':
        inputs['labels'] = ('chair', 'person')
    elif change == 'vocabulary_sha256':
        inputs['vocabulary_sha256'] = 'other'
    elif change == 'engine':
        (tmp_path/'e.engine').write_bytes(b'rebuilt')
    else:
        inputs[change].write_bytes(b'changed')
    with pytest.raises(EngineUnavailable, match='sidecar_mismatch'):
        load_engine(tmp_path/'e.engine', **inputs)


def test_missing_engine_sidecar_and_class_order_are_refused(tmp_path, monkeypatch):
    with pytest.raises(EngineUnavailable, match='engine_missing'):
        load_engine(tmp_path/'none.engine', weights='w', text_encoder='t', labels=LABELS, vocabulary_sha256='v')
    inputs = built(tmp_path)
    fake_ultralytics(monkeypatch, {0: 'chair', 1: 'person'})
    with pytest.raises(EngineUnavailable, match='class_order_mismatch'):
        load_engine(tmp_path/'e.engine', **inputs)
    (tmp_path/'e.json').write_text('not json')
    with pytest.raises(EngineUnavailable, match='sidecar_unreadable'):
        load_engine(tmp_path/'e.engine', **inputs)


def test_empty_environment_value_selects_the_pt_path(monkeypatch):
    monkeypatch.setenv('WLA_YOLOE_ENGINE', '')
    assert engine_path() is None
    monkeypatch.delenv('WLA_YOLOE_ENGINE')
    assert engine_path() == yoloe_engine.DEFAULT_ENGINE
