"""YOLOE segmentation -> native BoxMOT BoT-SORT appearance/motion association.

People use BoxMOT OSNet and optional built-in CMC. Other object classes use
class-separated geometry association (a person ReID network is not an object
identity model). No Ultralytics tracker or custom GMC gate is instantiated.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

# BotSort's constructor defaults. The native adapter would otherwise read
# BoxMOT's tuned YAML, which associates differently.
NATIVE_ALIGNED=dict(frame_rate=30,fuse_first_associate=False,second_match_thresh=.5,
    unconfirmed_match_thresh=.7,unconfirmed_emb_scale=2.,asso_func='iou')
CLASS_SLOTS=256
_LIBRARY=None
# BT.601 luma weights in the RGB order Ultralytics feeds the network.
LUMA_RGB=(.299,.587,.114)


def smooth_chroma(image):
    """Average the colour difference over 2x2 pixels and keep the luma.

    image is the (N,3,H,W) RGB 0-1 tensor Ultralytics' preprocess returns. Raw
    D455 RGB8 frames carry pixel-level colour noise that YOLOE-26s reads as a
    much weaker person: on 906 raw frames (2026-11-10) person p50 was 0.58
    without this step and 0.89 with it, the value the 4:2:0 debug JPEG gives.
    Only linear operations on the colour difference, so the luma is unchanged up
    to float rounding. No clamp: clamping saturated pixels would change the luma.
    """
    import torch.nn.functional as F
    source=image.float()
    luma=(source*source.new_tensor(LUMA_RGB).view(1,3,1,1)).sum(1,keepdim=True)
    colour=F.avg_pool2d(source-luma,2,ceil_mode=True)
    colour=F.interpolate(colour,size=source.shape[-2:],mode='bilinear',align_corners=False)
    return (luma+colour).to(image.dtype)


def install_chroma_smoothing(predictor):
    """Wrap the predictor's preprocess once, on the GPU tensor it already built.

    Returns whether the wrap is in place (False for a predictor without one).
    """
    preprocess=getattr(predictor,'preprocess',None)
    if preprocess is None:return False
    if getattr(preprocess,'wla_chroma_smoothing',False):return True
    def smoothed(image):return smooth_chroma(preprocess(image))
    smoothed.wla_chroma_smoothing=True
    predictor.preprocess=smoothed
    return True


def bounded_library():
    """BoxMOT's native BoT-SORT with its removed tracks capped; see build_native_botsort.

    The stock library keeps every removed track until reset, so memory grows with
    each new track. Raises OSError if the build is missing or for another BoxMOT.
    """
    global _LIBRARY
    if _LIBRARY is None:
        import boxmot
        from boxmot.native.trackers.botsort import BotSortLibrary
        root=Path(os.environ.get('WLA_ROS_WS',str(Path.home()/'ros2_ws')))
        path=Path(os.environ.get('WLA_BOTSORT_LIBRARY',str(root/'models/perception/tracking/botsort_capi_bounded.so')))
        try:record=json.loads(path.with_suffix('.json').read_text())
        except ValueError as exc:raise OSError(f'bad_sidecar:{exc}') from exc
        if record.get('boxmot')!=boxmot.__version__:
            raise OSError(f"built_for_boxmot_{record.get('boxmot')}_not_{boxmot.__version__}")
        _LIBRARY=BotSortLibrary(path)
    return _LIBRARY


class NativeObjectTracks:
    """BotSort(per_class=True) for the native library, which has no such option.

    Output rows keep the Python layout: frame-level detection indices and track
    ids that are unique across classes, without a growing remapping table.
    """
    def __init__(self,options):
        self.options=options;self.trackers={}

    def reset(self):
        self.trackers.clear()

    def update(self,dets,frame=None):
        from boxmot.trackers.box.botsort.native import NativeBotSortTracker
        classes=dets[:,5].astype(int)
        rows=[]
        for name in sorted(self.trackers.keys()|set(classes.tolist())):
            if not 0<=name<CLASS_SLOTS:raise ValueError('invalid_detection_class')
            tracker=self.trackers.get(name)
            if tracker is None:tracker=self.trackers[name]=NativeBotSortTracker(dict(self.options),library=bounded_library())
            local=np.flatnonzero(classes==name)
            for row in tracker.update(dets[local],None):
                row=np.array(row,dtype=np.float64)
                row[4]=row[4]*CLASS_SLOTS+name;row[7]=local[int(row[7])]
                rows.append(row)
        return np.array(rows,dtype=np.float64).reshape(-1,8)


class TrackingRuntime:
    def __init__(self,model,config=None,reid_backend=None,chroma_smoothing=True):
        from boxmot import BotSort
        self.model=model
        # Detector input only: ReID crops, masks and boxes keep the camera frame.
        self.chroma_smoothing=chroma_smoothing;self.chroma_active=False
        options=dict(track_high_thresh=.4,track_low_thresh=.1,new_track_thresh=.5,
            track_buffer=15,match_thresh=.8,proximity_thresh=.5,appearance_thresh=.25,
            use_cmc=True,cmc_method='sof',use_embeddings=True)
        weights=os.environ.get('WLA_REID_MODEL','/home/orin/Wheel_Legged_Agent/models/perception/reid/osnet_x0_25_msmt17.pth')
        device=os.environ.get('WLA_REID_DEVICE','cuda:0')
        if config:
            import yaml
            with open(config) as stream:loaded=yaml.safe_load(stream)
            if loaded.pop('tracker_type',None)!='boxmot_botsort':raise ValueError('BoxMOT config required')
            if loaded.get('use_embeddings') is not True:raise ValueError('full BoxMOT appearance association required')
            options.update(loaded)
        self.use_cmc=options['use_cmc']
        if reid_backend is None:
            from .boxmot_reid import CachedBoxMOTReID
            reid_backend=CachedBoxMOTReID(weights,device)
        self.reid=reid_backend
        object_options=dict(options,use_cmc=False,use_embeddings=False)
        self.tracker_backend='python'
        if os.environ.get('WLA_TRACKER_BACKEND','cpp')=='cpp':
            try:
                from boxmot.trackers.box.botsort.native import NativeBotSortTracker
                self.tracker=NativeBotSortTracker(dict(NATIVE_ALIGNED,**options),reid_model=self.reid,library=bounded_library())
                self.object_tracker=NativeObjectTracks(dict(NATIVE_ALIGNED,**object_options))
                self.tracker_backend='cpp'
            except (ImportError,OSError,RuntimeError,subprocess.SubprocessError) as exc:
                # Slower but equivalent association; timing reports the backend in use.
                print(f'ERROR: native BoT-SORT unavailable, running the Python tracker: {exc}',file=sys.stderr,flush=True)
        if self.tracker_backend=='python':
            self.tracker=BotSort(**options,reid_model=self.reid)
            self.object_tracker=BotSort(**object_options,per_class=True)
        self.signature=None;self.graph=None;self.last_metrics={};self.features={}
        self.engine_imgsz=getattr(model,'wla_engine_imgsz',None)

    @property
    def backend(self):
        if self.engine_imgsz:return 'tensorrt_fp16'
        return 'cuda_graph_fp32' if self.graph else 'eager'

    def reset(self):
        self.tracker.reset();self.object_tracker.reset()
        self.features={}

    def _public_id(self,group,identifier):
        # Disjoint, stable namespaces without a growing remapping table.
        return 2*identifier+(group=='object')

    def associate(self,output,frame):
        dets=output.boxes.data.detach().cpu().numpy().astype(np.float32)
        persons=np.array([output.names[int(row[5])]=='person' for row in dets],dtype=bool)
        self.features={};rows=[];self.reid.last_ms=0.
        for group,indices,tracker in (('person',np.flatnonzero(persons),self.tracker),
                                      ('object',np.flatnonzero(~persons),self.object_tracker)):
            tracked=tracker.update(dets[indices],frame if group=='person' else None)
            vectors=self.reid.last_features if group=='person' and len(indices) else None
            for track in tracked:
                local=int(track[7])
                if not 0<=local<len(indices):raise ValueError('invalid_boxmot_detection_index')
                index=int(indices[local]);box=dets[index]
                ident=self._public_id(group,int(track[4]))
                rows.append(dict(track_id=ident,label=output.names[int(box[5])],confidence=float(box[4]),
                    bbox_xyxy=box[:4].tolist(),detection_index=index))
                if vectors is not None:self.features[ident]=vectors[local].copy()
        return rows,int(np.count_nonzero(persons))

    def process(self,frame,**options):
        options.pop('persist',None);options.pop('tracker',None)
        options.update(retina_masks=True,verbose=False)
        # The YOLOE class defaults to agnostic NMS; an engine loads through plain YOLO.
        if self.engine_imgsz:options.update(imgsz=self.engine_imgsz,agnostic_nms=True)
        signature=(frame.shape,str(frame.dtype),repr(sorted(options.items())))
        started=time.perf_counter()
        if self.signature is None:
            self.model.predict(frame,**options)
            predictor=self.model.predictor
            if self.chroma_smoothing:self.chroma_active=install_chroma_smoothing(predictor)
            if predictor.model.device.type=='cuda' and not self.engine_imgsz:
                from wla_sim.perception.yoloe_graph import FP32GraphForward
                network=predictor.model.model;network.pe=network.pe.to(predictor.model.device)
                self.graph=FP32GraphForward(predictor.inference);predictor.inference=self.graph
            for _ in range(2):self.model.predict(frame,**options)
            if self.graph:self.graph.allow_capture=False
            self.reid.warm();self.reset();self.signature=signature
            self.last_metrics=dict(warmup_ms=(time.perf_counter()-started)*1000,
                backend=self.backend,cmc=self.use_cmc,chroma_smoothing=self.chroma_active)
            return None
        if signature!=self.signature:raise ValueError('changed_image_or_options_require_new_detector')
        # Idempotent; re-wraps if Ultralytics ever rebuilt its predictor.
        if self.chroma_smoothing:self.chroma_active=install_chroma_smoothing(self.model.predictor)
        output=self.model.predict(frame,**options)[0]
        detected=time.perf_counter()
        rows,people=self.associate(output,frame)
        self.last_metrics=dict(detector_ms=(detected-started)*1000,
            tracking_reid_ms=(time.perf_counter()-detected)*1000,reid_ms=self.reid.last_ms,
            total_ms=(time.perf_counter()-started)*1000,backend=self.backend,
            cmc=self.use_cmc,cmc_method='sof' if self.use_cmc else None,tracker='boxmot-25.0.0-botsort',
            tracker_backend=self.tracker_backend,reid_backend=getattr(self.reid,'backend_name',None),
            appearance='native_all_persons',people=people,detections=len(output.boxes.data),tracks=len(rows),
            chroma_smoothing=self.chroma_active)
        return output,rows
