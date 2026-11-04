"""Fixed-batch TensorRT FP16 OSNet engines, bound to the weights they were built from.

The same network and weights as the FP32 graph, so the descriptor's model id does
not change. Anything that no longer matches is refused here; the caller runs the
FP32 graph instead. Set WLA_REID_ENGINE to an empty value to choose that on purpose.
"""
import json
import os
from pathlib import Path

from .yoloe_engine import EngineUnavailable, sha256

BATCHES = (1, 4)
DEFAULT_DIR = Path(os.environ.get('WLA_ROS_WS', str(Path.home()/'ros2_ws')))/'models/perception/reid'
SCHEMA = 'wla-osnet-engine/v1'


def engine_dir():
    value = os.environ.get('WLA_REID_ENGINE')
    if value is None:
        return DEFAULT_DIR
    return Path(value) if value else None


def engine_file(folder, batch):
    return Path(folder)/f'osnet-x025-msmt17-fp16-b{batch}.engine'


def sidecar(folder):
    return Path(folder)/'osnet-x025-msmt17-fp16.json'


def write_sidecar(folder, *, weights, tensorrt_version):
    record = dict(schema=SCHEMA, precision='fp16', weights_sha256=sha256(weights), tensorrt=tensorrt_version,
                  engines={str(batch): sha256(engine_file(folder, batch)) for batch in BATCHES})
    sidecar(folder).write_text(json.dumps(record, indent=1)+'\n')
    return record


def verify(folder, weights):
    """Raise EngineUnavailable naming the first mismatch."""
    try:
        record = json.loads(sidecar(folder).read_text())
    except (OSError, ValueError) as exc:
        raise EngineUnavailable(f'sidecar_unreadable:{exc}') from exc
    if record.get('schema') != SCHEMA:
        raise EngineUnavailable('sidecar_mismatch:schema')
    if record.get('weights_sha256') != sha256(weights):
        raise EngineUnavailable('sidecar_mismatch:weights_sha256')
    for batch in BATCHES:
        path = engine_file(folder, batch)
        if not path.is_file():
            raise EngineUnavailable(f'engine_missing:{path}')
        if record.get('engines', {}).get(str(batch)) != sha256(path):
            raise EngineUnavailable(f'sidecar_mismatch:engine_sha256:{batch}')


def fixed_batches(batch, run):
    """Run in batches of 1 and 4, zero-padding a short one; run(size, part) returns one row per box."""
    import torch
    result = []
    for offset in range(0, len(batch), 4):
        part = batch[offset:offset+4]
        size = 1 if len(part) == 1 else 4
        if len(part) != size:
            padded = torch.zeros(size, *part.shape[1:], device=part.device, dtype=part.dtype)
            padded[:len(part)] = part
        else:
            padded = part
        result.append(run(size, padded)[:len(part)])
    return torch.cat(result)


class EngineForward:
    def __init__(self, folder, device):
        import tensorrt
        import torch
        self.device = device
        runtime = tensorrt.Runtime(tensorrt.Logger(tensorrt.Logger.ERROR))
        self.engines = {}
        for batch in BATCHES:
            engine = runtime.deserialize_cuda_engine(engine_file(folder, batch).read_bytes())
            context = engine.create_execution_context()
            names = engine.get_tensor_name(0), engine.get_tensor_name(1)
            if tuple(engine.get_tensor_shape(names[1])) != (batch, 512):
                raise ValueError(f'unexpected_output_shape:{engine.get_tensor_shape(names[1])}')
            # The context holds pointers into this; keep the engine alive beside it.
            self.engines[batch] = (engine, context, names, torch.empty(batch, 512, device=device))

    def run(self, size, part):
        import torch
        _, context, (source, target), output = self.engines[size]
        part = part.contiguous()
        context.set_tensor_address(source, int(part.data_ptr()))
        context.set_tensor_address(target, int(output.data_ptr()))
        context.execute_async_v3(torch.cuda.current_stream(self.device).cuda_stream)
        return output.clone()

    def __call__(self, batch):
        return fixed_batches(batch, self.run)

    def warm(self):
        import torch
        with torch.inference_mode():
            for batch in BATCHES:
                self.run(batch, torch.zeros(batch, 3, 256, 128, device=self.device))
        torch.cuda.current_stream(self.device).synchronize()


def load_engines(folder, weights, device):
    """Return a callable for a ready engine pair, or raise EngineUnavailable."""
    verify(folder, weights)
    try:
        return EngineForward(folder, device)
    except Exception as exc:
        raise EngineUnavailable(f'engine_load_failed:{type(exc).__name__}:{exc}') from exc
