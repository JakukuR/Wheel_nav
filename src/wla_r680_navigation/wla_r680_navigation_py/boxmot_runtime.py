"""YOLOE segmentation -> native BoxMOT BoT-SORT appearance/motion association.

People use BoxMOT OSNet and optional built-in CMC. Other object classes use
class-separated geometry association (a person ReID network is not an object
identity model). No Ultralytics tracker or custom GMC gate is instantiated.
"""
import os
import time
import numpy as np


class TrackingRuntime:
    def __init__(self,model,config=None,reid_backend=None):
        from boxmot import BotSort
        self.model=model
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
        self.tracker=BotSort(**options,reid_model=self.reid)
        object_options=dict(options,per_class=True,use_cmc=False,use_embeddings=False)
        self.object_tracker=BotSort(**object_options)
        self.signature=None;self.graph=None;self.last_metrics={};self.features={}

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
        signature=(frame.shape,str(frame.dtype),repr(sorted(options.items())))
        started=time.perf_counter()
        if self.signature is None:
            self.model.predict(frame,**options)
            predictor=self.model.predictor
            if predictor.model.device.type=='cuda':
                from wla_sim.perception.yoloe_graph import FP32GraphForward
                network=predictor.model.model;network.pe=network.pe.to(predictor.model.device)
                self.graph=FP32GraphForward(predictor.inference);predictor.inference=self.graph
            for _ in range(2):self.model.predict(frame,**options)
            if self.graph:self.graph.allow_capture=False
            self.reid.warm();self.reset();self.signature=signature
            self.last_metrics=dict(warmup_ms=(time.perf_counter()-started)*1000,
                backend='cuda_graph_fp32' if self.graph else 'eager',cmc=self.use_cmc)
            return None
        if signature!=self.signature:raise ValueError('changed_image_or_options_require_new_detector')
        output=self.model.predict(frame,**options)[0]
        detected=time.perf_counter()
        rows,people=self.associate(output,frame)
        self.last_metrics=dict(detector_ms=(detected-started)*1000,
            tracking_reid_ms=(time.perf_counter()-detected)*1000,reid_ms=self.reid.last_ms,
            total_ms=(time.perf_counter()-started)*1000,backend='cuda_graph_fp32' if self.graph else 'eager',
            cmc=self.use_cmc,cmc_method='sof' if self.use_cmc else None,tracker='boxmot-25.0.0-botsort',
            appearance='native_all_persons',people=people,detections=len(output.boxes.data),tracks=len(rows))
        return output,rows
