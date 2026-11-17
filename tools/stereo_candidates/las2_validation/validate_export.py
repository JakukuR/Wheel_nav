"""Compare original PyTorch and exported ONNX on recorded and upstream pairs."""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import torch

p = argparse.ArgumentParser()
p.add_argument('--repo', type=Path, required=True)
p.add_argument('--weights', type=Path, required=True)
p.add_argument('--size', choices=['s', 'm'], required=True)
p.add_argument('--onnx', type=Path, required=True)
p.add_argument('--recorded', type=Path, required=True)
p.add_argument('--out', type=Path, required=True)
a = p.parse_args()
sys.path.insert(0, str(a.repo.resolve()))
from core.models import build_model, load_model_weights

torch.set_num_threads(4)
cv2.setNumThreads(1)
model = build_model('las2', model_size=a.size, max_disp=192).eval()
load_model_weights(model, torch.load(a.weights, map_location='cpu', weights_only=True))
options = ort.SessionOptions()
options.intra_op_num_threads = 4
options.inter_op_num_threads = 1
session = ort.InferenceSession(str(a.onnx), sess_options=options, providers=['CPUExecutionProvider'])
_, _, height, width = session.get_inputs()[0].shape
content_height = round(width * 360 / 640)
pad_top = (height - content_height) // 2
cases = [('recorded', cv2.imread(str(a.recorded/'left_rect.png')), cv2.imread(str(a.recorded/'right_rect.png')))]
for path in sorted((a.repo/'assets').glob('Explorer*.png'))[:2]:
    im = cv2.imread(str(path))
    half = im.shape[1] // 2
    cases.append((path.stem, cv2.resize(im[:, :half], (640, 360)), cv2.resize(im[:, half:], (640, 360))))
summary = []
for name, left, right in cases:
    dest = a.out/name
    dest.mkdir(parents=True, exist_ok=True)
    tensors = []
    for eye, im in [('left', left), ('right', right)]:
        assert im is not None and im.shape == (360, 640, 3)
        cv2.imwrite(str(dest/f'{eye}_rect.png'), im)
        rgb = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (width, content_height))
        rgb = cv2.copyMakeBorder(rgb, pad_top, height-content_height-pad_top, 0, 0, cv2.BORDER_REPLICATE)
        tensors.append(np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32))
    with torch.inference_mode():
        reference = model(*(torch.from_numpy(t) for t in tensors), max_disp=192, test_mode=True).numpy()
    prediction = session.run(None, dict(zip(['left', 'right'], tensors)))[0]
    delta = np.abs(reference - prediction)
    row = dict(case=name, mean_abs_px=float(delta.mean()), p99_abs_px=float(np.quantile(delta, .99)), max_abs_px=float(delta.max()), finite=bool(np.isfinite(prediction).all()))
    prediction.tofile(dest/'onnx_disparity_native.bin')
    reference.tofile(dest/'torch_disparity_native.bin')
    summary.append(row)
    print(json.dumps(row), flush=True)
report = dict(model=f'LAS2-{a.size.upper()}', width=width, height=height, content_height=content_height, pad_top=pad_top, onnx=str(a.onnx.resolve()), cases=summary)
(a.out/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
assert all(r['finite'] and r['mean_abs_px'] < .001 and r['max_abs_px'] < .05 for r in summary)
