"""Appearance supervision and reacquisition, separate from geometry ownership."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
from .reid_identity import TargetIdentity


@dataclass(frozen=True)
class IdentitySupervisorConfig:
    similarity: float = .80
    margin: float = .08
    confirmations: int = 3
    supervision_window: int = 10
    suspect_negative_frames: int = 6
    suspect_recovery_frames: int = 6
    suspect_timeout_s: float = 12.
    neighbor_ambiguity_radius_m: float = 1.50
    neighbor_clear_hold_s: float = .50
    enrollment_timeout_s: float = 10.
    active_search_s: float = 8.
    release_s: float = 30.
    view_expansion_similarity: float = .75
    view_expansion_neighbor_radius_m: float = 1.50
    view_expansion_min_interval_s: float = 1.
    view_expansion_initial_slots: int = 2
    gallery_slots: int = 8


class IdentitySupervisor:
    """Events in this class never revoke a continuous geometry binding."""

    def __init__(self, target_id, *, model_id, start_time, config=IdentitySupervisorConfig()):
        if not self._valid_config(config):
            raise ValueError('invalid_identity_supervisor_config')
        self.c=config
        self.identity=TargetIdentity(target_id,max_lost=config.release_s,
            similarity=config.similarity,margin=config.margin,
            confirmations=config.confirmations,model_id=model_id,start_time=start_time)
        self.locked_id=target_id;self.resolved_id=target_id;self.model_id=model_id
        self.start_time=float(start_time);self.enrolled_at=None;self.enrollment_timed_out=False
        self.state='enrolling';self.reason='waiting_for_target_features'
        self.suspect_type=None;self.suspect_since=None;self.suspect_trigger=None
        self.ambiguous_since=None;self.clear_since=None
        self.window=deque(maxlen=config.supervision_window)
        self.last_similarity=None;self.last_positive_frame=None
        self.last_template_update=None;self.template_update_log=[]
        self.template_origins=[];self.last_expansion=-math.inf
        self.pending_id=None;self.pending_count=0;self.pending_stamp=None
        self.reacquired=0;self.end_reason=None
        self.last_error=None;self.last_ranked=[]
        self.feature_frames=0;self.quality_evidence_frames=0
        self.no_evidence_frames=0;self.feature_diagnostics={}
        self.last_position=None;self.last_seen=None;self.velocity=np.zeros(2)
        self.last_candidates=[]
        self.last_frame_stamp=-math.inf
        self.reacquire_proposal=None

    @staticmethod
    def _valid_config(c):
        vals=(c.similarity,c.margin,c.suspect_timeout_s,c.neighbor_ambiguity_radius_m,
              c.neighbor_clear_hold_s,c.enrollment_timeout_s,c.active_search_s,c.release_s,
              c.view_expansion_similarity,c.view_expansion_neighbor_radius_m,
              c.view_expansion_min_interval_s)
        return (all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in vals)
            and .7<=c.similarity<=.99 and .03<=c.margin<=.3
            and isinstance(c.confirmations,int) and 2<=c.confirmations<=5
            and 3<=c.supervision_window<=30 and 1<=c.suspect_negative_frames<=c.supervision_window
            and 1<=c.suspect_recovery_frames<=c.supervision_window
            and 0<c.suspect_timeout_s<=60 and c.neighbor_ambiguity_radius_m>0
            and 0<=c.neighbor_clear_hold_s<=2 and c.enrollment_timeout_s>0
            and 0<c.active_search_s<c.release_s<=60
            and .5<=c.view_expansion_similarity<=c.similarity
            and c.view_expansion_neighbor_radius_m>0 and c.view_expansion_min_interval_s>0
            and c.gallery_slots==8 and 1<=c.view_expansion_initial_slots<c.gallery_slots)

    @property
    def complete(self):
        return self.identity.seed_count>=2

    def active_speed_cap(self, configured_cap):
        if (not self.complete and not self.enrollment_timed_out
                and isinstance(configured_cap,(int,float)) and configured_cap>0):
            return float(configured_cap)
        return None

    def _sanitize_diagnostics(self, diagnostics):
        if not isinstance(diagnostics,dict) or len(diagnostics)>4:return {}
        return {k:v for k,v in diagnostics.items() if isinstance(k,str) and len(k)<=128
                and isinstance(v,str) and len(v)<=96}

    def _normalize_rows(self,rows):
        result=[];seen=set()
        for row in rows:
            try:
                ident=row['id'];f=np.asarray(row['feature'],dtype=float);p=np.asarray(row['position'],dtype=float)
                if (not isinstance(ident,str) or ident in seen or f.shape!=(512,) or p.shape!=(2,)
                        or not np.isfinite(f).all() or not np.isfinite(p).all() or np.linalg.norm(f)<1e-6):
                    continue
                seen.add(ident);result.append(dict(id=ident,feature=f/np.linalg.norm(f),position=p.copy()))
            except (KeyError,TypeError,ValueError):
                continue
        return result

    def _sim(self,row):
        if not self.identity.gallery:return None
        return float(max(row['feature']@g for g in self.identity.gallery))

    def _log_template(self,t,action,reason,sim):
        event=dict(stamp_s=float(t),action=action,reason=reason,
            similarity=None if sim is None else float(sim),slots=len(self.identity.gallery),
            initial_protected=sum(1 for x in self.template_origins if x=='initial'))
        self.last_template_update=event;self.template_update_log.append(event)
        self.template_update_log=self.template_update_log[-16:]

    def _append_template(self,feature,origin,t,reason,sim):
        gallery=self.identity.gallery
        if len(gallery)>=self.c.gallery_slots:
            protected=self.template_origins[:self.c.view_expansion_initial_slots]
            evict=None
            for i,kind in enumerate(self.template_origins):
                if not (i<len(protected) and kind=='initial'):
                    evict=i;break
            if evict is None:
                self._log_template(t,'frozen','gallery_full_protected_initial',sim);return False
            expansion=sum(1 for kind in self.template_origins if kind=='expansion')
            if expansion>=self.c.gallery_slots-self.c.view_expansion_initial_slots:
                self._log_template(t,'frozen','gallery_full_expansion_slots',sim);return False
            gallery.rotate(-evict);gallery.popleft();gallery.rotate(evict)
            self.template_origins.pop(evict)
        elif len([x for x in self.template_origins if x=='expansion'])>=self.c.gallery_slots-self.c.view_expansion_initial_slots:
            self._log_template(t,'frozen','gallery_full_expansion_slots',sim);return False
        gallery.append(feature.copy());self.template_origins.append(origin)
        self._log_template(t,'updated',reason,sim);return True

    def _rank(self,rows):
        ranked=[]
        for row in rows:
            sim=self._sim(row)
            if sim is None:continue
            ranked.append((sim,row))
        ranked.sort(key=lambda item:item[0],reverse=True)
        self.last_ranked=ranked
        return ranked

    def _target_margin_ok(self,target_id):
        ranked=self.last_ranked
        qualified=[(s,r) for s,r in ranked if s>=self.c.similarity]
        if not qualified:return False
        if qualified[0][1]['id']!=target_id:return False
        return len(qualified)<2 or qualified[0][0]-qualified[1][0]>=self.c.margin

    def _start_suspect(self,t,kind,trigger):
        if self.state.startswith('suspect_'):
            if kind!='ambiguous' or self.suspect_type=='ambiguous':
                return
            # A nearby person is a more immediate safety ambiguity than
            # accumulated negative appearance evidence. Replace that exit path
            # and require proximity to clear before appearance re-verification.
            self.state='suspect_ambiguous';self.suspect_type='ambiguous'
            self.suspect_since=float(t);self.suspect_trigger=trigger
            self.reason='suspect_ambiguous';self.pending_count=0;self.pending_id=None
            self._log_template(t,'frozen','suspect_ambiguous_override',self.last_similarity)
            return
        self.state='suspect_'+kind;self.suspect_type=kind
        self.suspect_since=float(t);self.suspect_trigger=trigger
        self.reason='suspect_'+kind;self.pending_count=0;self.pending_id=None
        self._log_template(t,'frozen','suspect_'+kind,self.last_similarity)


    def _enter_neighbor_ambiguity(self,t):
        self._start_suspect(t,'ambiguous','neighbor_within_radius')
        self.ambiguous_since=float(t) if self.ambiguous_since is None else self.ambiguous_since
        self.clear_since=None

    def _recover_suspect(self,t,reason):
        self.state='tracking';self.reason=reason
        self.suspect_type=None;self.suspect_since=None;self.suspect_trigger=None
        self.ambiguous_since=None;self.clear_since=None

    def _enroll(self,rows,t,diagnostics):
        target=[r for r in rows if r['id']==self.locked_id]
        if self.identity.model_id and self.identity.model_id==self.model_id:
            self.identity.consume(target,float(t),self.model_id,diagnostics)
        if self.identity.seed_count>=2:
            self.enrolled_at=float(t);self.enrollment_timed_out=False
            if self.suspect_since is None:
                self.state='tracking';self.reason='identity_verified'
            self.template_origins=['initial']*min(self.identity.seed_count,len(self.identity.gallery))
            while len(self.template_origins)<len(self.identity.gallery):self.template_origins.append('initial')

    def _process_healthy(self,rows,t,diagnostics,geometry_continuous,neighbor_ambiguity):
        target_id=self.resolved_id
        target=next((r for r in rows if r['id']==target_id),None)
        self._rank(rows)
        sim=None if target is None else self._sim(target)
        self.last_similarity=sim
        self.feature_diagnostics=diagnostics
        high_quality=(target is not None and diagnostics.get(target_id,'feature_ready')=='feature_ready'
                      and sim is not None)
        if high_quality:
            self.quality_evidence_frames+=1
            self.last_positive_frame=float(t)
            self.window.append(float(sim))
            self.last_position=target['position'].copy();self.last_seen=float(t)
            # Strong match remains exactly >=0.90.  Neighbor ambiguity freezes all
            # template writes, including a strong match.
            suspect_before=self.state.startswith('suspect_')
            if neighbor_ambiguity:
                if sim>=.90:self._log_template(t,'frozen','frozen_neighbor_ambiguity',sim)
            elif self.state=='suspect_negative':
                # A clean, geometrically continuous moderate view is the only
                # template update allowed while resolving negative evidence.
                if sim>=self.c.view_expansion_similarity and float(t)-self.last_expansion>=self.c.view_expansion_min_interval_s:
                    if self._append_template(target['feature'],'expansion',t,'view_expansion_suspect_recovery',sim):
                        self.last_expansion=float(t)
                else:
                    self._log_template(t,'frozen','frozen_suspect_negative',sim)
            elif suspect_before:
                self._log_template(t,'frozen','frozen_suspect_ambiguous',sim)
            elif sim>=.90:
                self._append_template(target['feature'],'strong',t,'strong_match',sim)
            elif sim>=self.c.view_expansion_similarity and float(t)-self.last_expansion>=self.c.view_expansion_min_interval_s:
                if self._append_template(target['feature'],'expansion',t,'view_expansion',sim):
                    self.last_expansion=float(t)
        else:
            self.no_evidence_frames+=1
            self.reason='no_supervision_evidence'

        if neighbor_ambiguity:
            self._enter_neighbor_ambiguity(t)
            return
        if self.state=='suspect_ambiguous':
            if self.clear_since is None:self.clear_since=float(t)
            clear_elapsed=float(t)-self.clear_since
            if (clear_elapsed>=self.c.neighbor_clear_hold_s and high_quality
                    and sim>=self.c.similarity and self._target_margin_ok(target_id)):
                self._recover_suspect(t,'ambiguity_cleared_and_reverified')
            elif float(t)-self.suspect_since>=self.c.suspect_timeout_s:
                self._suspect_timeout(t)
            return
        if self.state=='suspect_negative':
            positive=sum(v>=self.c.similarity for v in self.window)
            if positive>=self.c.suspect_recovery_frames:
                self._recover_suspect(t,'negative_evidence_window_recovered')
            elif float(t)-self.suspect_since>=self.c.suspect_timeout_s:
                self._suspect_timeout(t)
            return
        negative=sum(v<self.c.similarity for v in self.window)
        if len(self.window)>=self.c.supervision_window and negative>=self.c.suspect_negative_frames:
            self._start_suspect(t,'negative','negative_frames_in_window')
            return
        self.state='tracking';self.reason='identity_verified'

    def _suspect_timeout(self,t):
        self.state='ended';self.suspect_timeout_at=float(t)
        self.end_reason=('suspect_timeout_before_enrollment' if not self.complete
                         else 'suspect_timeout_after_enrollment')

    def advance_time(self,now,*,geometry_state,binding_snapshot=None):
        """Advance lifecycle deadlines without consuming a feature timestamp."""
        if self.end_reason:return
        if self.suspect_since is not None and now-self.suspect_since>=self.c.suspect_timeout_s:
            self._suspect_timeout(now);return
        if geometry_state=='lost':
            if not self.complete:
                self.state='ended';self.end_reason='target_lost_before_enrollment';return
            snap=binding_snapshot or {}
            age=max(0.,now-snap['lost_since']) if snap.get('lost_since') is not None else float(snap.get('lost_s') or 0.)
            self.state='searching' if age<self.c.active_search_s else 'passive_reacquire'
            self.reason=self.state
            if age>=self.c.release_s:
                self.state='ended';self.end_reason='lost_task_release'

    def _observe_lost(self,rows,t,binding_snapshot):
        self.advance_time(t,geometry_state='lost',binding_snapshot=binding_snapshot)
        if self.end_reason:return
        lost_s=float(binding_snapshot.get('lost_s') or 0.)
        predicted=binding_snapshot.get('prediction')
        predicted=np.asarray(predicted,dtype=float) if predicted is not None else self.last_position
        if predicted is None:return
        age=max(0.,lost_s)
        radius=min(1.5,.45+.25*age)
        ranked=[]
        for row in rows:
            sim=self._sim(row)
            if sim is None:continue
            error=float(np.linalg.norm(row['position']-predicted))
            ranked.append((sim,error,row))
        ranked.sort(key=lambda item:item[0],reverse=True)
        qualified=[item for item in ranked if item[0]>=self.c.similarity and item[1]<=radius]
        self.last_candidates=[dict(id=r['id'],similarity=float(s),position_error_m=float(e),
            qualified=bool(s>=self.c.similarity and e<=radius)) for s,e,r in ranked]
        if not qualified:return
        if len(qualified)>1 and qualified[0][0]-qualified[1][0]<self.c.margin:
            self.reason='reacquire_multiple_similar_people';self.pending_count=0;return
        sim,error,row=qualified[0]
        if row['id']==self.pending_id and self.pending_stamp is not None and 0<float(t)-self.pending_stamp<=.6:
            self.pending_count+=1
        else:self.pending_id=row['id'];self.pending_count=1
        self.pending_stamp=float(t);self.last_similarity=sim;self.last_error=error
        if self.pending_count>=self.c.confirmations:
            self.reacquire_proposal=dict(id=row['id'],stamp=float(t),count=self.pending_count,
                lost_since=binding_snapshot.get('lost_since'))
            self.reason='reacquisition_confirmation_ready'
            self.pending_count=0;self.pending_id=None;self.pending_stamp=None
            self.last_position=row['position'].copy();self.last_seen=float(t)

    def observe(self,rows,t,diagnostics,*,geometry_state,neighbor_ambiguity=False,binding_snapshot=None):
        self.advance_time(float(t),geometry_state=geometry_state,binding_snapshot=binding_snapshot)
        if self.end_reason:return
        if not math.isfinite(float(t)) or float(t)<=self.last_frame_stamp:return
        self.last_frame_stamp=float(t);self.feature_frames+=1
        diagnostics=self._sanitize_diagnostics(diagnostics)
        rows=self._normalize_rows(rows)
        if geometry_state in ('lost','ended') or binding_snapshot and binding_snapshot.get('state')=='lost':
            self._observe_lost(rows,float(t),binding_snapshot or {})
            return
        if not self.complete:
            self._enroll(rows,float(t),diagnostics)
            if (not self.complete
                    and float(t)-self.start_time>=self.c.enrollment_timeout_s):
                self.enrollment_timed_out=True
            if neighbor_ambiguity:
                # Enrollment keeps consuming the user-selected ID's features,
                # while motion is held and templates remain frozen.
                self._enter_neighbor_ambiguity(float(t))
                if self.enrollment_timed_out:
                    self.reason='suspect_ambiguous_after_enrollment_timeout'
                return
            if self.enrollment_timed_out:
                self.state='unverified';self.reason='enrollment_timeout_follow_unverified'
            return
        self._process_healthy(rows,float(t),diagnostics,True,neighbor_ambiguity)

    def rebind(self,target_id,t):
        self.resolved_id=target_id;self.state='tracking';self.reason='reid_reacquired'
        self.reacquired+=1;self.reacquire_proposal=None
        self.suspect_type=None;self.suspect_since=None;self.suspect_trigger=None
        self.window.clear();self.pending_count=0;self.pending_id=None;self.pending_stamp=None
        self.last_seen=float(t)

    def request_ids(self,objects,pose,*,binding_snapshot,center_offset=.28):
        if not isinstance(objects,dict):return []
        if binding_snapshot.get('state')=='lost':
            pred=binding_snapshot.get('prediction')
            if pred is None:return []
            age=float(binding_snapshot.get('lost_s') or 0.)
            radius=min(1.5,.45+.25*age)
            candidates=[];too_many=False
            for ident,row in objects.items():
                if not row.get('detected') or row.get('label')!='person' or row.get('yaw') is None:continue
                d=row['distance']+center_offset;a=pose[2]+row['yaw']
                p=np.asarray(pose[:2])+d*np.array([math.cos(a),math.sin(a)])
                if np.linalg.norm(p-np.asarray(pred))<=radius:candidates.append(ident)
            if len(candidates)>4:
                self.reason='too_many_spatial_candidates';too_many=True
            return [] if too_many else candidates[:4]
        target=binding_snapshot.get('resolved_id') or self.resolved_id
        ids=[target] if target in objects else []
        if self.state.startswith('suspect_') or self.neighbor_near(objects,pose,target,self.c.view_expansion_neighbor_radius_m,center_offset):
            for ident in self.neighbor_ids(objects,pose,target,self.c.view_expansion_neighbor_radius_m,center_offset):
                if ident not in ids:ids.append(ident)
        return ids[:4]

    @staticmethod
    def _positions(objects,pose,center_offset):
        result={}
        if pose is None:return result
        for ident,row in objects.items():
            if not row.get('detected') or row.get('label')!='person' or row.get('yaw') is None or row.get('distance') is None:continue
            d=row['distance']+center_offset;a=pose[2]+row['yaw']
            result[ident]=np.asarray(pose[:2])+d*np.array([math.cos(a),math.sin(a)])
        return result

    def neighbor_ids(self,objects,pose,target,radius,center_offset=.28):
        positions=self._positions(objects,pose,center_offset)
        if target not in positions:return []
        return [ident for ident,p in sorted(positions.items(),key=lambda item:float(np.linalg.norm(item[1]-positions[target])))
                if ident!=target and float(np.linalg.norm(p-positions[target]))<radius]

    def neighbor_near(self,objects,pose,target,radius,center_offset=.28):
        return bool(self.neighbor_ids(objects,pose,target,radius,center_offset))

    def snapshot(self,now):
        enrollment_elapsed=max(0.,float(now)-self.start_time)
        return dict(state=self.state,reason=self.reason,locked_id=self.locked_id,
            resolved_id=self.resolved_id,completed_enrollment=self.complete,
            enrollment_elapsed_s=enrollment_elapsed,
            enrollment_timed_out=self.enrollment_timed_out,
            enrollment_completed_s=None if self.enrolled_at is None else max(0.,self.enrolled_at-self.start_time),
            suspect_type=self.suspect_type,suspect_trigger=self.suspect_trigger,
            suspect_since=self.suspect_since, suspect_age_s=None if self.suspect_since is None else max(0.,float(now)-self.suspect_since),
            suspect_timeout_s=self.c.suspect_timeout_s,
            similarity_window=[float(v) for v in self.window],
            similarity=self.last_similarity,quality_frames=self.quality_evidence_frames,
            no_evidence_frames=self.no_evidence_frames,feature_frames=self.feature_frames,
            template_slots=len(self.identity.gallery),template_origins=list(self.template_origins),
            initial_templates_protected=sum(kind=='initial' for kind in self.template_origins),
            last_template_update=self.last_template_update,
            template_update_log=list(self.template_update_log[-8:]),
            pending_id=self.pending_id,pending_confirmations=self.pending_count,
            reacquired=self.reacquired,candidates=list(self.last_candidates),
            feature_diagnostics=dict(self.feature_diagnostics),end_reason=self.end_reason,
            thresholds=dict(similarity=self.c.similarity,margin=self.c.margin,
                confirmations=self.c.confirmations,view_expansion_similarity=self.c.view_expansion_similarity,
                neighbor_ambiguity_radius_m=self.c.neighbor_ambiguity_radius_m,
                neighbor_clear_hold_s=self.c.neighbor_clear_hold_s,
                suspect_timeout_s=self.c.suspect_timeout_s,
                supervision_window=self.c.supervision_window,
                suspect_negative_frames=self.c.suspect_negative_frames,
                suspect_recovery_frames=self.c.suspect_recovery_frames,
                enrollment_timeout_s=self.c.enrollment_timeout_s,
                active_search_s=self.c.active_search_s,release_s=self.c.release_s,
                view_expansion_neighbor_radius_m=self.c.view_expansion_neighbor_radius_m,
                view_expansion_min_interval_s=self.c.view_expansion_min_interval_s))
