"""BoxMOT's OSNet backend; bounded CUDA replay and same-frame feature reuse.

Network, weights, crop preprocessing and descriptor normalization belong to
BoxMOT. This adapter caches its returned batch; it never performs a second
inference for follow-target identity verification.
"""
from pathlib import Path
import hashlib
import os
import sys
import time
import numpy as np
from .reid_engine import EngineUnavailable,engine_dir,fixed_batches,load_engines


class BatchedGraph:
    def __init__(self,forward,device):
        self.forward=forward;self.device=device;self.graphs={}

    def warm(self):
        import torch
        from wla_sim.perception.yoloe_graph import FP32GraphForward
        with torch.inference_mode():
            for size in (1,4):
                graph=FP32GraphForward(self.forward)
                graph(torch.zeros(size,3,256,128,device=self.device))
                graph.allow_capture=False
                self.graphs[size]=graph

    def __call__(self,batch):
        if not self.graphs:raise ValueError('ReID graph requires explicit startup warmup')
        return fixed_batches(batch,lambda size,part:self.graphs[size](part))


def batched_crops(backend,boxes,image):
    """BoxMOT's crops, resize and normalization, with one GPU upload for the batch."""
    import cv2
    import torch
    from boxmot.reid.core.crops import extract_crops
    shape=backend.input_shape
    blank=np.zeros((*shape,3),np.uint8)
    array=np.stack([cv2.cvtColor(backend.preprocess_fn(crop if crop.size else blank,shape),cv2.COLOR_BGR2RGB)
        for crop in extract_crops(boxes,image,shape)])
    batch=torch.from_numpy(np.ascontiguousarray(array.transpose(0,3,1,2))).to(backend.device).float()
    return batch.div_(255.).sub_(backend.mean_array).div_(backend.std_array)


class CachedBoxMOTReID:
    def __init__(self,weights,device='cuda:0'):
        import torch
        from boxmot.reid.backends.pytorch_backend import PyTorchBackend
        weights=Path(weights).expanduser()
        if not weights.is_file():raise FileNotFoundError(weights)
        self.backend=PyTorchBackend(weights,torch.device(device),False)
        digest=hashlib.sha256(weights.read_bytes()).hexdigest()[:16]
        self.model_id='boxmot25-osnet-x025-msmt17:'+digest
        self.last_features=np.empty((0,512),np.float32);self.last_ms=0.
        self.graph=None;self.backend_name='eager'
        if self.backend.device.type=='cuda':
            self.graph=BatchedGraph(self.backend.forward,self.backend.device);self.backend_name='cuda_graph_fp32'
            folder=engine_dir()
            if folder is not None:
                try:
                    self.graph=load_engines(folder,weights,self.backend.device);self.backend_name='tensorrt_fp16'
                except EngineUnavailable as exc:
                    # Slower but correct. Rebuild with build_osnet_engine to get FP16 back.
                    print(f'ERROR: FP16 OSNet refused, running the FP32 graph: {exc}',file=sys.stderr,flush=True)
        if self.graph:self.backend.forward=self.graph
        if os.environ.get('WLA_REID_CROPS','batched')=='batched':
            self.backend.get_crops=lambda boxes,image:batched_crops(self.backend,boxes,image)

    def warm(self):
        if self.graph:self.graph.warm()
        self.get_features(np.array([[0,0,128,256]],np.float32),np.zeros((256,128,3),np.uint8))
        self.last_features=np.empty((0,512),np.float32)

    def get_features(self,boxes,image):
        started=time.perf_counter()
        self.last_features=self.backend.get_features(boxes,image)
        self.last_ms=(time.perf_counter()-started)*1000.
        return self.last_features
