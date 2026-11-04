"""TensorRT FP16 YOLOE engine, bound to the weights and vocabulary it was built from.

The engine bakes in the text embeddings and the input shape, so a sidecar records
what went in. Anything that no longer matches is refused here; the caller decides
what to run instead. Set WLA_YOLOE_ENGINE to an empty value to use the .pt path.
"""
import hashlib
import json
import os
from pathlib import Path

ENGINE_IMGSZ = (320, 416)  # What imgsz=416 gives a 640x480 D455 frame.
DEFAULT_ENGINE = Path(os.environ.get('WLA_ROS_WS', str(Path.home()/'ros2_ws')))/'models/perception/yoloe/yoloe-26s-seg-fp16.engine'
SCHEMA = 'wla-yoloe-engine/v1'


class EngineUnavailable(RuntimeError):
    pass


def engine_path():
    value = os.environ.get('WLA_YOLOE_ENGINE')
    if value is None:
        return DEFAULT_ENGINE
    return Path(value) if value else None


def sidecar(engine):
    return Path(engine).with_suffix('.json')


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def write_sidecar(engine, *, weights, text_encoder, classes, prompts, vocabulary_sha256, tensorrt_version):
    record = dict(schema=SCHEMA, precision='fp16', imgsz=list(ENGINE_IMGSZ), engine_sha256=sha256(engine),
                  weights_sha256=sha256(weights), text_encoder_sha256=sha256(text_encoder),
                  vocabulary_sha256=vocabulary_sha256, classes=list(classes), prompts=list(prompts),
                  tensorrt=tensorrt_version)
    sidecar(engine).write_text(json.dumps(record, indent=1)+'\n')
    return record


def load_engine(engine, *, weights, text_encoder, labels, vocabulary_sha256):
    """Return a ready model, or raise EngineUnavailable naming the mismatch."""
    engine = Path(engine)
    if not engine.is_file():
        raise EngineUnavailable(f'engine_missing:{engine}')
    try:
        record = json.loads(sidecar(engine).read_text())
    except (OSError, ValueError) as exc:
        raise EngineUnavailable(f'sidecar_unreadable:{exc}') from exc
    expected = dict(schema=SCHEMA, imgsz=list(ENGINE_IMGSZ), classes=list(labels), vocabulary_sha256=vocabulary_sha256,
                    engine_sha256=sha256(engine), weights_sha256=sha256(weights),
                    text_encoder_sha256=sha256(text_encoder))
    for key, value in expected.items():
        if record.get(key) != value:
            raise EngineUnavailable(f'sidecar_mismatch:{key}')
    # A .engine load must never try to install TensorRT into the shared venv.
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    os.environ['YOLO_OFFLINE'] = 'True'
    try:
        from ultralytics import YOLO
        model = YOLO(str(engine), task='segment')
        names = model.names
    except Exception as exc:
        raise EngineUnavailable(f'engine_load_failed:{type(exc).__name__}:{exc}') from exc
    actual = tuple(names[i] for i in range(len(names))) if isinstance(names, dict) else tuple(names)
    if actual != tuple(labels):
        raise EngineUnavailable('class_order_mismatch')
    model.wla_engine_imgsz = ENGINE_IMGSZ
    model.wla_yoloe_provenance = dict(
        detector='yoloe', backend='tensorrt_fp16', weights=str(weights), engine=str(engine),
        weights_sha256=record['weights_sha256'], engine_sha256=record['engine_sha256'],
        text_encoder_sha256=record['text_encoder_sha256'], classes=list(labels), prompts=record['prompts'],
        vocabulary_sha256=vocabulary_sha256, physical_acceptance='not_evaluated')
    return model
