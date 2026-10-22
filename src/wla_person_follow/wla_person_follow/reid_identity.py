"""Target-only appearance/space/time association. No ROS or actuator I/O."""
from collections import deque
import math
import numpy as np


class TargetIdentity:
    def __init__(self, target_id, *, max_lost=8., similarity=.80, margin=.08,
                 confirmations=3, model_id='', start_time=0.):
        self.locked_id=target_id; self.resolved_id=target_id
        self.max_lost=max_lost; self.threshold=similarity; self.margin=margin
        self.confirmations=confirmations; self.model_id=model_id
        self.gallery=deque(maxlen=8); self.last_seen=None; self.position=None
        self.velocity=np.zeros(2); self.frame_stamp=-math.inf; self.start_time=start_time
        self.pending_id=None; self.pending_count=0; self.pending_stamp=None
        self.seed_count=0; self.state='enrolling'; self.reason='waiting_for_target_features'
        self.last_similarity=None; self.position_error=None; self.reacquired=0
        self.enrollment_restarts=0;self.feature_frames=0;self.feature_diagnostics={}
        self.last_candidate_similarity=None;self.last_candidate_error=None
        self.candidate_frame=None
        self.selection_diagnostics=dict(accepted=False,reason='not_selected')
        self.request_diagnostics=None

    def _expire(self,now):
        # Enrollment has its own fixed task-start deadline. Renewing a
        # provisional pair must not extend the search indefinitely.
        if self.state=='expired':return True
        anchor=self.start_time if self.seed_count<2 else self.last_seen
        if anchor is not None and now-anchor>self.max_lost:
            self.state='expired'
            self.reason='reid_enrollment_timeout' if self.seed_count<2 else 'reid_search_timeout'
            return True
        return False

    def _missing_target_reason(self):
        reason=self.feature_diagnostics.get(self.locked_id,'target_feature_absent')
        self.reason='enrollment_'+reason

    def prediction(self,t):
        if self.position is None:return None
        return self.position+self.velocity*min(max(0.,t-self.last_seen),1.5)

    def radius(self,age):return min(1.5,.45+.25*max(0.,age))

    def _accept(self,row,t,sim,seed=False):
        pos=np.asarray(row['position'],dtype=float)
        if self.last_seen is not None and .05<t-self.last_seen<.8:
            v=(pos-self.position)/(t-self.last_seen)
            v*=min(1.,1.5/max(np.linalg.norm(v),1e-9))
            self.velocity=.6*self.velocity+.4*v
        elif self.last_seen is not None and t-self.last_seen>=.8:
            self.velocity*=0.
        self.position=pos.copy();self.last_seen=t;self.resolved_id=row['id']
        # Only explicit enrollment or strong matches may update the gallery.
        if seed or sim>=.90:self.gallery.append(row['feature'].copy())
        self.seed_count+=1
        self.pending_id=None;self.pending_count=0;self.pending_stamp=None
        self.last_similarity=float(sim)
        self.state='tracking' if self.seed_count>=2 else 'enrolling'
        self.reason='identity_verified' if self.seed_count>=2 else 'confirm_initial_appearance'

    def _audit(self,ident,**fields):
        if self.candidate_frame is None:return
        entry=next((r for r in self.candidate_frame['candidates'] if r['id']==ident),None)
        if entry is None:
            entry=dict(id=ident,quality=self.feature_diagnostics.get(ident,'not_reported'),
                evaluated=False,similarity=None,position_error_m=None,reason='not_evaluated')
            self.candidate_frame['candidates'].append(entry)
        entry.update(fields)

    def consume(self,rows,t,model_id,diagnostics=None):
        if not math.isfinite(t) or t<=self.frame_stamp:return
        self.frame_stamp=t;self.feature_frames+=1
        self.feature_diagnostics={k:v for k,v in (diagnostics or {}).items()
            if isinstance(k,str) and len(k)<=128 and isinstance(v,str) and len(v)<=96} if isinstance(diagnostics,dict) and len(diagnostics)<=4 else {}
        self.candidate_frame=dict(stamp_s=float(t),model_id=model_id,evaluated=False,
            reason='pending',candidates=[],similarity_threshold=self.threshold,
            ambiguity_margin=self.margin)
        for ident in self.feature_diagnostics:
            self._audit(ident,reason=self.feature_diagnostics[ident] if self.feature_diagnostics[ident]!='feature_ready' else 'feature_not_received')
        for row in rows:
            if isinstance(row,dict) and isinstance(row.get('id'),str):self._audit(row['id'])
        if self._expire(t):
            self.candidate_frame['reason']='identity_expired_no_evaluation'
            for row in self.candidate_frame['candidates']:row['reason']='identity_expired_no_evaluation'
            return
        if model_id!=self.model_id:
            self.candidate_frame['reason']='feature_model_mismatch'
            for row in self.candidate_frame['candidates']:row['reason']='feature_model_mismatch'
            self.reason='feature_model_mismatch';return
        self.candidate_frame.update(evaluated=True,reason='processed')
        valid=[];ids=set()
        for row in rows:
            try:
                f=np.asarray(row['feature'],dtype=float);p=np.asarray(row['position'],dtype=float)
                if (not isinstance(row['id'],str) or row['id'] in ids or f.shape!=(512,) or p.shape!=(2,)
                        or not np.isfinite(f).all() or not np.isfinite(p).all() or np.linalg.norm(f)<1e-6):
                    if isinstance(row.get('id'),str):self._audit(row['id'],reason='invalid_or_duplicate_feature_position')
                    continue
                ids.add(row['id']);valid.append(dict(id=row['id'],position=p,feature=f/np.linalg.norm(f)))
            except (ValueError,TypeError,KeyError):
                if isinstance(row,dict) and isinstance(row.get('id'),str):self._audit(row['id'],reason='malformed_candidate')
                continue
        if not self.gallery:
            for candidate in valid:self._audit(candidate['id'],reason='enrollment_requires_locked_id')
            row=next((r for r in valid if r['id']==self.locked_id),None)
            if row is not None:
                self._audit(row['id'],reason='enrollment_seed',accepted=True)
                self._accept(row,t,1.,seed=True)
            else:self._missing_target_reason()
            return
        age=t-self.last_seen
        if age<0:return
        predicted=self.prediction(t)
        if self.seed_count<2:
            for candidate in valid:self._audit(candidate['id'],reason='enrollment_requires_locked_id')
            # A single provisional descriptor is not an enrolled identity.
            # Only the explicitly selected ID may provide either pair member.
            row=next((r for r in valid if r['id']==self.locked_id),None)
            if row is None:self._missing_target_reason();return
            sim=max(float(row['feature']@g) for g in self.gallery)
            error=float(np.linalg.norm(row['position']-predicted))
            self._audit(row['id'],evaluated=True,similarity=sim,position_error_m=error,
                position_limit_m=self.radius(age),reason=('enrollment_appearance_mismatch' if sim<self.threshold else
                    'enrollment_position_mismatch' if error>self.radius(age) else 'enrollment_pair_accepted'))
            self.last_candidate_similarity=sim;self.last_candidate_error=error
            if sim<self.threshold:self.reason='enrollment_appearance_mismatch';return
            if error>self.radius(age):self.reason='enrollment_position_mismatch';return
            if age>.6:
                # Start a fresh pair from this compatible same-ID sample; do
                # not accept a late frame as the missing second confirmation.
                self.gallery.clear();self.seed_count=0;self.velocity[:]=0.
                self.last_seen=None;self.enrollment_restarts+=1
                self._accept(row,t,sim,seed=True)
                self.reason='enrollment_pair_restarted';return
            self.position_error=error;self._accept(row,t,sim);return
        ranked=[]
        for row in valid:
            sim=max(float(row['feature']@g) for g in self.gallery)
            error=float(np.linalg.norm(row['position']-predicted))
            self._audit(row['id'],evaluated=True,similarity=sim,position_error_m=error,
                position_limit_m=self.radius(age),reason=('appearance_below_threshold' if sim<self.threshold else
                    'position_outside_gate' if error>self.radius(age) else 'qualified_candidate'))
            if sim>=self.threshold and error<=self.radius(age):ranked.append((sim,error,row))
        ranked.sort(key=lambda r:r[0],reverse=True)
        direct=next((r for r in ranked if r[2]['id']==self.resolved_id),None)
        if direct is not None and age<=.6:
            for _,_,row in ranked:self._audit(row['id'],reason='not_selected_same_id_priority')
            self._audit(direct[2]['id'],reason='same_id_accepted',accepted=True)
            self.position_error=direct[1];self._accept(direct[2],t,direct[0]);return
        self.state='searching';self.reason='no_appearance_position_match'
        if not ranked:
            self.pending_id=None;self.pending_count=0;return
        if len(ranked)>1 and ranked[0][0]-ranked[1][0]<self.margin:
            for _,_,row in ranked:self._audit(row['id'],reason='multiple_similar_people')
            self.state='ambiguous';self.reason='multiple_similar_people'
            self.pending_id=None;self.pending_count=0;return
        for _,_,candidate in ranked[1:]:self._audit(candidate['id'],reason='lower_similarity_rank')
        sim,error,row=ranked[0]
        if row['id']==self.pending_id and self.pending_stamp is not None and 0<t-self.pending_stamp<=.6:
            self.pending_count+=1
        else:self.pending_id=row['id'];self.pending_count=1
        self.pending_stamp=t;self.last_similarity=sim;self.position_error=error
        self._audit(row['id'],reason='awaiting_distinct_frames',confirm_count=self.pending_count)
        self.state='confirming';self.reason='awaiting_distinct_frames'
        if self.pending_count>=self.confirmations:
            self._audit(row['id'],reason='reacquired',accepted=True)
            self.reacquired+=1;self._accept(row,t,sim)
            self.reason='reacquired_by_appearance_time_position'

    def _selection(self,reason,value=None):
        self.selection_diagnostics.update(reason=reason,accepted=value is not None,selected_id=value)
        return value

    def select(self,objects,pose,stamp,now,center_offset=.28):
        """Only current RGB-D measurements can drive; embeddings never renew geometry."""
        self.selection_diagnostics=dict(observation_stamp_s=stamp,control_stamp_s=now,
            source='TargetIdentity.select',resolved_id_before=self.resolved_id,state_before=self.state)
        if self._expire(now):return self._selection('identity_expired')
        if self.last_seen is None or self.seed_count<2:return self._selection('enrollment_incomplete')
        age=now-self.last_seen
        if age>.8 or self.state not in ('tracking',):
            if self.state=='tracking':self.state='searching';self.reason='verified_target_not_visible'
            return self._selection('identity_not_tracking_or_stale')
        row=objects.get(self.resolved_id)
        if not row or not row.get('detected') or row.get('label')!='person' or row.get('yaw') is None:return self._selection('geometry_unavailable')
        if not 0<=now-stamp<=.30:return self._selection('geometry_stale')
        distance=row['distance']+center_offset;yaw=row['yaw']+pose[2]
        p=np.asarray(pose[:2])+distance*np.array([np.cos(yaw),np.sin(yaw)])
        if np.linalg.norm(p-self.prediction(stamp))>self.radius(stamp-self.last_seen):
            self.reason='current_position_jump';return self._selection('current_position_jump')
        return self._selection('current_identity_and_geometry_accepted',self.resolved_id)

    def candidate_ids(self,objects,pose,stamp,now,center_offset=.28):
        """Feature work is target-only; only a lost target opens the spatial gate."""
        self.request_diagnostics=dict(stamp_s=stamp,control_stamp_s=now,evaluated=False,
            reason='not_evaluated',candidates=[])
        if self._expire(now):
            self.request_diagnostics['reason']='identity_expired_no_evaluation';return []
        if pose is None or not 0<=now-stamp<=.3:
            self.request_diagnostics['reason']='pose_or_geometry_unavailable';return []
        if not self.gallery or self.seed_count<2:
            return [self.locked_id] if self.locked_id in objects else []
        age=now-self.last_seen
        if age>self.max_lost:return []
        self.request_diagnostics.update(evaluated=True,reason='spatial_gate')
        eligible=[]
        predicted=self.prediction(stamp)
        for ident,row in objects.items():
            if not row.get('detected') or row.get('label')!='person' or row.get('yaw') is None:continue
            d=row['distance']+center_offset;a=pose[2]+row['yaw']
            position=np.asarray(pose[:2])+d*np.array([np.cos(a),np.sin(a)])
            error=float(np.linalg.norm(position-predicted))
            self.request_diagnostics['candidates'].append(dict(id=ident,position_error_m=error,
                position_limit_m=self.radius(age),eligible=bool(error<=self.radius(age))))
            if error<=self.radius(age):eligible.append((error,ident))
        if self.state=='tracking' and age<=.6 and any(ident==self.resolved_id for _,ident in eligible):
            return [self.resolved_id]
        # Do not silently ignore a fifth ambiguous person because of the CPU cap.
        if len(eligible)>4:
            self.state='ambiguous';self.reason='too_many_spatial_candidates';return []
        return [ident for _,ident in sorted(eligible)]

    def snapshot(self,now):
        self._expire(now)
        return dict(state=self.state,reason=self.reason,locked_id=self.locked_id,
            resolved_id=self.resolved_id,reference_samples=len(self.gallery),
            lost_s=None if self.last_seen is None else max(0.,now-self.last_seen),
            similarity=self.last_similarity,position_error_m=self.position_error,
            confirm_count=self.pending_count,reacquired=self.reacquired,max_lost_s=self.max_lost,
            enrollment_samples=min(self.seed_count,2),enrollment_restarts=self.enrollment_restarts,
            enrollment_remaining_s=max(0.,self.start_time+self.max_lost-now) if self.seed_count<2 else None,
            feature_frames=self.feature_frames,
            feature_age_s=None if not math.isfinite(self.frame_stamp) else max(0.,now-self.frame_stamp),
            last_candidate_similarity=self.last_candidate_similarity,last_candidate_error_m=self.last_candidate_error,
            feature_diagnostics=self.feature_diagnostics,
            predicted_odom=None if self.position is None else self.prediction(now).tolist(),
            candidate_frame=self.candidate_frame,
            candidate_frame_age_s=None if self.candidate_frame is None else max(0.,now-self.candidate_frame['stamp_s']),
            selection=self.selection_diagnostics,candidate_request=self.request_diagnostics)
