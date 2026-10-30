#!/usr/bin/env python3
"""Metadata parsing and frame routing of emitter_frame_splitter (no ROS graph)."""
import importlib.machinery
import importlib.util
from pathlib import Path

script = Path(__file__).resolve().parents[1] / 'scripts' / 'emitter_frame_splitter'
loader = importlib.machinery.SourceFileLoader('emitter_frame_splitter', str(script))
spec = importlib.util.spec_from_loader('emitter_frame_splitter', loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)

# Real D455 metadata (captured 2026-10-29, projector on), trimmed.
lit = '{"frame_number":339279,"frame_laser_power":150,"frame_laser_power_mode":1,"frame_emitter_mode":1}'
assert module.emitter_on(lit) is True
assert module.emitter_on('{"frame_emitter_mode":0,"frame_laser_power_mode":1}') is False
assert module.emitter_on('{"frame_laser_power_mode":0}') is False
assert module.emitter_on('{"frame_number":1}') is None
assert module.emitter_on('not json') is None and module.emitter_on('[1]') is None
assert module.emitter_on('{"frame_emitter_mode":true}') is None
print('PASS metadata parsing', flush=True)

sent = []
router = module.Router(lambda i, m: sent.append((i, m)), window=2)
router.metadata(100, False)
router.image(1, 100, 'dark1')
router.image(2, 100, 'dark2')
router.image(1, 200, 'lit1')           # image before its metadata
router.metadata(200, True)
router.image(2, 200, 'lit2')
router.image(1, 300, 'dark-late')      # metadata arrives after the image
router.metadata(300, False)
assert sent == [(1, 'dark1'), (2, 'dark2'), (1, 'dark-late')], sent
for stamp in (400, 500, 600):
    router.image(1, stamp, 'orphan')   # no metadata: held, then evicted
assert len(router.pending[1]) == 2 and router.dropped == 3, (router.pending, router.dropped)
print('PASS only projector-off pairs are forwarded, in either arrival order', flush=True)
