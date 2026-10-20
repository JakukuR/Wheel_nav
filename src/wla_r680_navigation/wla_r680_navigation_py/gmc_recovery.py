"""Optional visual GMC: quality failure uses unwarped Kalman association.

Fresh detections/geometry continue. AssociationIds isolates mode transitions
so consumers cannot silently reuse a previously verified identity.
"""
import time
from collections import OrderedDict
import numpy as np

from wla_sim.perception.yoloe_runtime import BoundedORB, TrackingRuntime


class AssociationIds:
    """Monotonic public IDs; a compensation transition retires old bindings."""
    def __init__(self):
        self.mode=None;self.revision=0;self.next_id=0;self.ids=OrderedDict();self.good_frames=0

    def begin_frame(self, mode):
        if mode not in ('visual', 'uncompensated'):
            raise ValueError('invalid_association_mode')
        if self.mode is None:
            self.mode=mode;self.revision+=1
        elif mode=='uncompensated' and self.mode=='visual':
            self.mode=mode;self.revision+=1;self.ids.clear()
        if mode=='uncompensated':
            self.good_frames=0
        elif self.mode=='uncompensated':
            self.good_frames+=1
            if self.good_frames>=10:
                # Recovery adds assistance; it does not change track identity.
                self.mode='visual';self.good_frames=0

    def resolve(self, track_id):
        if self.mode is None:raise RuntimeError('association_frame_not_started')
        if track_id not in self.ids:
            self.next_id+=1
            if self.next_id>=1_000_000_000:raise RuntimeError('association_id_exhausted')
            self.ids[track_id]=self.next_id
            if len(self.ids)>4096:self.ids.popitem(last=False)
        self.ids.move_to_end(track_id)
        return self.ids[track_id]


class PreparedORB(BoundedORB):
    def reset_params(self):
        super().reset_params()
        self._prepared = None

    def prepare(self, image, detections):
        self._prepared = None
        matrix = super().apply(image, detections)
        if self.diagnostics.get('reason') == 'initialized' and (
                self.previous[1] is None or len(self.previous[1]) < 6):
            self.diagnostics.update(valid=False, reason='insufficient_features')
        if not self.diagnostics.get('valid', False):
            # The tracker WILL update on this image, so retain this image as
            # the next reference. Identity means "no visual compensation",
            # never a claim that the visual transform was verified.
            matrix = np.eye(2, 3, dtype=np.float32)
            self.diagnostics['fallback'] = 'identity_transform'
        self._prepared = matrix

    def apply(self, image, detections=None):
        if self._prepared is None:
            # Initialization/warmup uses the original implementation.
            return super().apply(image, detections)
        matrix, self._prepared = self._prepared, None
        return matrix


class RecoverableTrackingRuntime(TrackingRuntime):
    """Single-image, single-tracker adapter for the deployed nav detector."""
    _preflight = False

    def _install(self, predictor):
        super()._install(predictor)
        for tracker in predictor.trackers:
            if not isinstance(tracker.gmc, PreparedORB):
                tracker.gmc = PreparedORB(self.max_points)
        if not self._preflight:
            return
        if len(predictor.trackers) != 1 or len(predictor.results) != 1:
            raise RuntimeError('gmc_preflight_requires_single_image')
        tracker = predictor.trackers[0]
        result = predictor.results[0]
        boxes = result.boxes.cpu().numpy()
        # Match BYTETracker.update's high-score mask exactly. This callback
        # runs before upstream's tracking callback and Kalman prediction.
        detections = boxes.xyxy[boxes.conf >= tracker.args.track_high_thresh]
        tracker.gmc.prepare(result.orig_img, detections)

    def _track(self, frame, kwargs):
        trackers = tuple(self.model.predictor.trackers)
        for tracker in trackers:
            tracker.gmc.diagnostics = {}
        started = time.perf_counter()
        self.last_metrics = {}
        self._preflight = True
        try:
            result = self.model.track(frame, **kwargs)
            current = self.model.predictor.trackers
            if len(current) != len(trackers) or any(a is not b for a, b in zip(trackers, current)):
                raise RuntimeError('tracker_replaced_during_inference')
            if any('valid' not in t.gmc.diagnostics or t.gmc._prepared is not None for t in trackers):
                raise RuntimeError('gmc_preflight_contract_broken')
            return result
        except Exception as exc:
            self._failure(exc)
            self._reset_trackers()
            raise
        finally:
            self._preflight = False
            self.last_metrics.update(
                track_ms=(time.perf_counter()-started)*1000,
                gmc=[dict(t.gmc.diagnostics) for t in trackers],
                backend=self.backend, generation=self.generation,
                graph_disabled_reason=self.graph_disabled_reason,
                gmc_policy='optional_visual_compensation',
                association_mode=('visual' if all(t.gmc.diagnostics.get('valid') is True for t in trackers)
                                  else 'uncompensated'))
