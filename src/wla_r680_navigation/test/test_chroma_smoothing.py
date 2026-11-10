"""Detector-input chroma smoothing; CPU tensors only, no camera, GPU or motion."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
import torch
from wla_r680_navigation_py.boxmot_runtime import LUMA_RGB, TrackingRuntime, install_chroma_smoothing, smooth_chroma


def luma(image):
    return (image.double()*torch.tensor(LUMA_RGB, dtype=torch.float64).view(1, 3, 1, 1)).sum(1)


def noisy(height=48, width=64, seed=0):
    generator = torch.Generator().manual_seed(seed)
    base = torch.linspace(.2, .8, width).expand(1, 3, height, width).clone()
    return (base+.02*torch.randn(1, 3, height, width, generator=generator)).float()


@pytest.mark.parametrize('shape', [(48, 64), (47, 63)])
def test_luma_is_kept_and_only_colour_noise_is_averaged(shape):
    image = noisy(*shape)
    out = smooth_chroma(image)
    assert out.shape == image.shape and out.dtype == image.dtype
    assert float((luma(out)-luma(image)).abs().max()) < 1e-5
    colour = lambda x: x.double()-luma(x).unsqueeze(1)
    assert float(colour(out).std()) < .7*float(colour(image).std())
    assert not torch.equal(out, image)


def test_grey_and_flat_colour_images_are_unchanged():
    grey = torch.rand(1, 1, 32, 32).expand(1, 3, 32, 32).contiguous()
    assert float((smooth_chroma(grey)-grey).abs().max()) < 1e-6
    flat = torch.tensor([.9, .3, .1]).view(1, 3, 1, 1).expand(1, 3, 32, 32).contiguous()
    assert float((smooth_chroma(flat)-flat).abs().max()) < 1e-6


def test_half_input_stays_half():
    image = noisy().half()
    out = smooth_chroma(image)
    assert out.dtype == torch.float16
    assert float((luma(out)-luma(image)).abs().max()) < 2e-3


def test_install_wraps_once():
    calls = []
    predictor = NS(preprocess=lambda image: calls.append(1) or noisy())
    assert install_chroma_smoothing(predictor) and install_chroma_smoothing(predictor)
    predictor.preprocess([np.zeros((4, 4, 3), np.uint8)])
    assert calls == [1]
    assert install_chroma_smoothing(NS()) is False


class Encoder:
    model_id = 'fixture'
    last_ms = 0.
    def warm(self): self.last_features = np.empty((0, 512), np.float32)
    def get_features(self, boxes, image):
        self.last_features = np.full((len(boxes), 512), 1/np.sqrt(512), np.float32)
        return self.last_features


class Detector:
    """Runs the predictor's preprocess like Ultralytics and keeps what the network got."""
    def __init__(self):
        self.predictor = None
        self.seen = []

    def predict(self, frame, **options):
        if self.predictor is None:
            self.predictor = NS(model=NS(device=NS(type='cpu')),
                                preprocess=lambda images: torch.from_numpy(images[0][..., ::-1].copy()).permute(2, 0, 1)[None].float()/255)
        self.seen.append(self.predictor.preprocess([frame]))
        data = torch.tensor([[10, 10, 50, 90, .9, 0]], dtype=torch.float32)
        return [NS(boxes=NS(data=data), names={0: 'person'}, masks=None)]


def frame():
    return np.random.default_rng(1).integers(0, 256, (60, 80, 3), dtype=np.uint8)


def run(enabled):
    model = Detector()
    runtime = TrackingRuntime(model, reid_backend=Encoder(), chroma_smoothing=enabled)
    image = frame()
    assert runtime.process(image, device='cpu') is None
    assert runtime.last_metrics['chroma_smoothing'] is enabled
    output, rows = runtime.process(image, device='cpu')
    assert runtime.last_metrics['chroma_smoothing'] is enabled and rows[0]['bbox_xyxy'] == [10, 10, 50, 90]
    return model.seen, image


def test_switch_off_feeds_the_network_byte_identical_input():
    seen, image = run(False)
    reference = torch.from_numpy(image[..., ::-1].copy()).permute(2, 0, 1)[None].float()/255
    assert all(x.numpy().tobytes() == reference.numpy().tobytes() for x in seen)


def test_switch_on_smooths_after_the_first_warmup_call():
    seen, image = run(True)
    reference = torch.from_numpy(image[..., ::-1].copy()).permute(2, 0, 1)[None].float()/255
    assert torch.equal(seen[0], reference)
    for x in seen[1:]:
        assert torch.equal(x, smooth_chroma(reference))
