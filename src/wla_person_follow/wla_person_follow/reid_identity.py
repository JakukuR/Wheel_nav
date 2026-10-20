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

    def consume(self,rows,t,model_id):
        if not math.isfinite(t) or t<=self.frame_stamp:return
        self.frame_stamp=t
        if model_id!=self.model_id:
            self.reason='feature_model_mismatch';return
        valid=[];ids=set()
        for row in rows:
            try:
                f=np.asarray(row['feature'],dtype=float);p=np.asarray(row['position'],dtype=float)
                if (not isinstance(row['id'],str) or row['id'] in ids or f.shape!=(512,) or p.shape!=(2,)
                        or not np.isfinite(f).all() or not np.isfinite(p).all() or np.linalg.norm(f)<1e-6):continue
                ids.add(row['id']);valid.append(dict(id=row['id'],position=p,feature=f/np.linalg.norm(f)))
            except (ValueError,TypeError,KeyError):continue
        if not self.gallery:
            row=next((r for r in valid if r['id']==self.locked_id),None)
            if row is not None:self._accept(row,t,1.,seed=True)
            return
        age=t-self.last_seen
        if age<0:return
        if age>self.max_lost:
            self.state='expired';self.reason='reid_search_timeout';return
        predicted=self.prediction(t);ranked=[]
        for row in valid:
            sim=max(float(row['feature']@g) for g in self.gallery)
            error=float(np.linalg.norm(row['position']-predicted))
            if sim>=self.threshold and error<=self.radius(age):ranked.append((sim,error,row))
        ranked.sort(key=lambda r:r[0],reverse=True)
        direct=next((r for r in ranked if r[2]['id']==self.resolved_id),None)
        if direct is not None and age<=.6:
            self.position_error=direct[1];self._accept(direct[2],t,direct[0]);return
        # Initial enrollment can never adopt a different ID.
        if self.seed_count<2:
            self.reason='initial_identity_not_confirmed';return
        self.state='searching';self.reason='no_appearance_position_match'
        if not ranked:
            self.pending_id=None;self.pending_count=0;return
        if len(ranked)>1 and ranked[0][0]-ranked[1][0]<self.margin:
            self.state='ambiguous';self.reason='multiple_similar_people'
            self.pending_id=None;self.pending_count=0;return
        sim,error,row=ranked[0]
        if row['id']==self.pending_id and self.pending_stamp is not None and 0<t-self.pending_stamp<=.6:
            self.pending_count+=1
        else:self.pending_id=row['id'];self.pending_count=1
        self.pending_stamp=t;self.last_similarity=sim;self.position_error=error
        self.state='confirming';self.reason='awaiting_distinct_frames'
        if self.pending_count>=self.confirmations:
            self.reacquired+=1;self._accept(row,t,sim)
            self.reason='reacquired_by_appearance_time_position'

    def select(self,objects,pose,stamp,now,center_offset=.28):
        """Only current RGB-D measurements can drive; embeddings never renew geometry."""
        if self.last_seen is None or self.seed_count<2:return None
        age=now-self.last_seen
        if age>self.max_lost:
            self.state='expired';self.reason='reid_search_timeout';return None
        if age>.8 or self.state not in ('tracking',):
            if self.state=='tracking':self.state='searching';self.reason='verified_target_not_visible'
            return None
        row=objects.get(self.resolved_id)
        if not row or not row.get('detected') or row.get('label')!='person' or row.get('yaw') is None:return None
        if not 0<=now-stamp<=.30:return None
        distance=row['distance']+center_offset;yaw=row['yaw']+pose[2]
        p=np.asarray(pose[:2])+distance*np.array([np.cos(yaw),np.sin(yaw)])
        if np.linalg.norm(p-self.prediction(stamp))>self.radius(stamp-self.last_seen):
            self.reason='current_position_jump';return None
        return self.resolved_id

    def candidate_ids(self,objects,pose,stamp,now,center_offset=.28):
        """Feature work is target-only; only a lost target opens the spatial gate."""
        if pose is None or not 0<=now-stamp<=.3:return []
        if not self.gallery or self.seed_count<2:
            return [self.locked_id] if self.locked_id in objects else []
        age=now-self.last_seen
        if age>self.max_lost:return []
        eligible=[]
        predicted=self.prediction(stamp)
        for ident,row in objects.items():
            if not row.get('detected') or row.get('label')!='person' or row.get('yaw') is None:continue
            d=row['distance']+center_offset;a=pose[2]+row['yaw']
            position=np.asarray(pose[:2])+d*np.array([np.cos(a),np.sin(a)])
            error=float(np.linalg.norm(position-predicted))
            if error<=self.radius(age):eligible.append((error,ident))
        if self.state=='tracking' and age<=.6 and any(ident==self.resolved_id for _,ident in eligible):
            return [self.resolved_id]
        # Do not silently ignore a fifth ambiguous person because of the CPU cap.
        if len(eligible)>4:
            self.state='ambiguous';self.reason='too_many_spatial_candidates';return []
        return [ident for _,ident in sorted(eligible)]

    def snapshot(self,now):
        return dict(state=self.state,reason=self.reason,locked_id=self.locked_id,
            resolved_id=self.resolved_id,reference_samples=len(self.gallery),
            lost_s=None if self.last_seen is None else max(0.,now-self.last_seen),
            similarity=self.last_similarity,position_error_m=self.position_error,
            confirm_count=self.pending_count,reacquired=self.reacquired,max_lost_s=self.max_lost,
            predicted_odom=None if self.position is None else self.prediction(now).tolist())
