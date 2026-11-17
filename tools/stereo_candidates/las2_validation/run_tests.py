"""Bounded offline tests. No camera, robot, or service controls."""
import argparse
import json
import signal
import subprocess
import time
from pathlib import Path

import numpy as np

p = argparse.ArgumentParser()
p.add_argument('--root', type=Path, required=True)
p.add_argument('--size', choices=['s', 'm'], required=True)
p.add_argument('--width', type=int, default=640)
p.add_argument('--engine', type=Path, required=True)
p.add_argument('--rate-hz', type=float, default=10)
p.add_argument('--iterations', type=int, default=300)
p.add_argument('--recorded-only', action='store_true')
p.add_argument('--out-name')
a = p.parse_args()
root = a.root.resolve()
reference = root/f'reference_{a.size}{a.width}'
meta = json.loads((reference/'validation.json').read_text())
out = root/(a.out_name or f'test_{a.size}{a.width}')
out.mkdir(exist_ok=True)
spec = dict(model=f'LAS2-{a.size.upper()}', engine=str(a.engine.resolve()),
            width=meta['width'], height=meta['height'], content_width=meta['width'],
            content_height=meta['content_height'], pad_top=meta['pad_top'],
            pad_bottom=meta['height']-meta['content_height']-meta['pad_top'],
            input_left='left', input_right='right', output_disparity='disparity',
            normalization='raw_rgb', bf_640=17.288570242494362)

def temps():
    return [int(Path(f'/sys/class/thermal/thermal_zone{i}/temp').read_text())/1000 for i in (0, 1)]

rows = []
for case in meta['cases']:
    name = case['case']
    if a.recorded_only and name != 'recorded':
        continue
    dest = out/name
    dest.mkdir(exist_ok=True)
    current = dict(spec, iterations=a.iterations if name=='recorded' else 10, warmup=10, rate_hz=a.rate_hz)
    (dest/'spec.json').write_text(json.dumps(current, indent=2)+'\n')
    deadline = time.monotonic()+180
    while max(temps()) > 90:
        if time.monotonic() > deadline:
            raise RuntimeError('Board did not cool to 90 C; no test started')
        time.sleep(2)
    command = [str(root/'model_probe'), str(dest/'spec.json'), str(reference/name/'left_rect.png'), str(reference/name/'right_rect.png'), str(dest)]
    samples = []
    reason = 'completed'
    started = time.monotonic()
    with (dest/'probe.log').open('w') as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            while child.poll() is None:
                row = dict(seconds=time.monotonic()-started, temperature_c=temps())
                samples.append(row)
                if max(row['temperature_c']) >= 97 or row['seconds'] > 120:
                    reason = 'temperature_abort' if max(row['temperature_c']) >= 97 else 'timeout'
                    break
                time.sleep(.5)
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
    (dest/'telemetry.json').write_text(json.dumps(dict(reason=reason, return_code=child.returncode, samples=samples), indent=2)+'\n')
    if reason != 'completed' or child.returncode:
        raise RuntimeError(f'{name}: {reason}, code={child.returncode}')
    result = json.loads((dest/'result.json').read_text())
    pred = np.fromfile(dest/'disparity_native.bin', np.float32).reshape(meta['height'], meta['width'])
    ref = np.fromfile(reference/name/'onnx_disparity_native.bin', np.float32).reshape(pred.shape)
    top = meta['pad_top']
    delta = np.abs(pred-ref)[top:top+meta['content_height']]
    finite = bool(np.isfinite(pred).all())
    accuracy = dict(finite=finite, mean_abs_px=float(delta.mean()), p99_abs_px=float(np.quantile(delta,.99)), max_abs_px=float(delta.max()), fraction_over_1px=float((delta>1).mean()))
    # Numeric export/backend agreement, not depth ground-truth accuracy.
    accuracy['pass'] = finite and accuracy['mean_abs_px'] < .1 and accuracy['fraction_over_1px'] < .01
    (dest/'fp16_vs_onnx.json').write_text(json.dumps(accuracy, indent=2)+'\n')
    row = dict(case=name, accuracy=accuracy, gpu_median_ms=float(np.median(result['gpu_ms'])), gpu_p95_ms=float(np.quantile(result['gpu_ms'],.95)), wall_median_ms=float(np.median(result['wall_ms'])), wall_p95_ms=float(np.quantile(result['wall_ms'],.95)), cpu_one_core_percent=result['cpu_one_core_percent'], achieved_hz=result['achieved_hz'], engine_bytes=result['engine_bytes'], context_bytes=result['execution_context_bytes'], max_temperature_c=max(max(t['temperature_c']) for t in samples))
    rows.append(row)
    print(json.dumps(row), flush=True)
(out/'summary.json').write_text(json.dumps(dict(spec=spec, cases=rows), indent=2)+'\n')
if not all(r['accuracy']['pass'] for r in rows):
    raise SystemExit('FP16 numeric validation failed')
