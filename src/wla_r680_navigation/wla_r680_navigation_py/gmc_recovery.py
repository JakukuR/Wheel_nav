"""Reject unreliable GMC before BoT-SORT mutates tracks; retain task identity.

Only this quality failure is recoverable. Inference/contract exceptions retain
the existing hard reset. A rejected image supplies no fresh target evidence.
"""
import time

from wla_sim.perception.yoloe_runtime import BoundedORB, TrackingRuntime


class GMCQualityRejected(RuntimeError):
    def __init__(self):
        super().__init__('gmc_quality_rejected')


class PreparedORB(BoundedORB):
    def reset_params(self):
        super().reset_params()
        self._prepared = None

    def prepare(self, image, detections):
        previous = self.previous
        self._prepared = None
        matrix = super().apply(image, detections)
        if self.diagnostics.get('reason') == 'initialized' and (
                self.previous[1] is None or len(self.previous[1]) < 6):
            self.diagnostics.update(valid=False, reason='insufficient_features')
        if not self.diagnostics.get('valid', False):
            # The transform must span from the last accepted tracker frame,
            # not from a rejected image whose tracker update never happened.
            self.previous = previous
            raise GMCQualityRejected()
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
            if any(t.gmc.diagnostics.get('valid') is not True for t in trackers):
                # Failure here is a programming/contract fault: an update may
                # already have occurred, so do not classify it as recoverable.
                raise RuntimeError('gmc_preflight_contract_broken')
            return result
        except GMCQualityRejected:
            # No tracker update has run, so no rollback/reset or ID reuse.
            self.last_metrics['outcome'] = 'gmc_degraded'
            raise
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
                gmc_policy='preassociation_reject_preserve_identity')
