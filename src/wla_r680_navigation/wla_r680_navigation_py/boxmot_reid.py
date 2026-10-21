"""BoxMOT's OSNet backend; bounded CUDA replay and same-frame feature reuse.

Network, weights, crop preprocessing and descriptor normalization belong to
BoxMOT. This adapter caches its returned batch; it never performs a second
inference for follow-target identity verification.
"""
from pathlib import Path
import hashlib
import time
import numpy as np


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
        import torch
        if not self.graphs:raise ValueError('ReID graph requires explicit startup warmup')
        result=[]
        for offset in range(0,len(batch),4):
            part=batch[offset:offset+4];size=1 if len(part)==1 else 4
            if len(part)!=size:
                padded=torch.zeros(size,*part.shape[1:],device=part.device,dtype=part.dtype)
                padded[:len(part)]=part
            else:padded=part
            result.append(self.graphs[size](padded)[:len(part)])
        return torch.cat(result)


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
        self.graph=BatchedGraph(self.backend.forward,self.backend.device) if self.backend.device.type=='cuda' else None
        if self.graph:self.backend.forward=self.graph

    def warm(self):
        if self.graph:self.graph.warm()
        self.get_features(np.array([[0,0,128,256]],np.float32),np.zeros((256,128,3),np.uint8))
        self.last_features=np.empty((0,512),np.float32)

    def get_features(self,boxes,image):
        started=time.perf_counter()
        self.last_features=self.backend.get_features(boxes,image)
        self.last_ms=(time.perf_counter()-started)*1000.
        return self.last_features
