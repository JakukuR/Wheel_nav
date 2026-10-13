"""Object-specific task policy using shared identity/lease and native kernels."""
import math
from _wla_follow_local import close_object
from wla_person_follow.relative_control import RelativeFollower

class ObjectFollower(RelativeFollower):
    def __init__(self,*args,clearance=.15,front=.24,rear=.24,half_width=.24,close_max=.20,close_gain=.8,reaction=.7,deceleration=.3,**kwargs):
        super().__init__(*args,**kwargs)
        if not .15<=clearance<=.5 or not all(.1<=v<=1. for v in (front,rear,half_width)):
            raise ValueError('invalid_object_clearance_or_footprint')
        if not (0<close_max<=.20 and 0<close_gain<=2 and .1<=reaction<=2 and .05<=deceleration<=2):
            raise ValueError('invalid_object_speed_configuration')
        self.close_max,self.close_gain,self.reaction,self.deceleration=close_max,close_gain,reaction,deceleration
        self.measured_velocity=(0.,0.)
        self.follow_mode='follow_object' ;self.approach_mode='approach_object'
        self.clearance,self.front,self.rear,self.half_width=clearance,front,rear,half_width
        self.geometry=None;self.geometry_until=-math.inf;self.surface_gap=None

    def command(self,command,request_id='',mode='',target_id='',lease_duration_s=1.,now=0.):
        previous=self.target_label
        if command=='start' and request_id not in self.requests:
            row=self.objects.get(target_id)
            if row is None or row['label']=='person':return False,'non_person_object_id_required'
            self.target_label=row['label']
        accepted,reason=super().command(command,request_id,mode,target_id,lease_duration_s,now)
        if not accepted:self.target_label=previous
        if reason=='person_id_not_visible':reason='object_id_not_visible'
        return accepted,reason

    def tick(self,now,safety=None,scene=None,estimate=None):
        self._expire(now)
        previous,self.previous_tick=self.previous_tick,now
        dt=.05 if previous is None else max(.001,min(.1,now-previous))
        if not self.active:return self._zero()
        row=self.objects.get(self.target_id)
        reason=('observations_stale' if now>=self.expires_at else
                'object_not_visible' if row is None or not row['detected'] or row['label']!=self.target_label else
                'object_geometry_unavailable' if row['yaw'] is None or row['distance'] is None else
                'object_estimate_unavailable' if estimate is None or not estimate.observed or estimate.target_id!=self.target_id else
                'depth_safety_unavailable' if safety is None or not safety.valid or scene is None else
                'object_surface_unavailable' if self.geometry is None or now>=self.geometry_until else '')
        if reason:
            self.state,self.reason='waiting',reason;self.arrival_count=0;self.surface_gap=None
            return self._zero()
        points,target,source_stamp=self.geometry
        result=close_object(points,target,estimate.yaw,self.front,self.rear,self.half_width,self.clearance,
            min(self.close_max,self.max_linear,self.previous_v+.4*dt),self.close_gain,
            *self.measured_velocity,self.reaction,self.deceleration,min(.25,self.max_angular))
        self.surface_gap=result.gap if math.isfinite(result.gap) else None
        if not result.valid:
            self.state,self.reason='waiting',result.reason;self.arrival_count=0
            return self._zero()
        if estimate.distance>1.2:
            # Far field retains the shared conservative detour policy.
            if safety.front_distance<self.emergency_distance:
                self.state,self.reason='waiting','emergency_obstacle';return self._zero()
            v,w=self.local_planner.command(scene,estimate.yaw,estimate.distance,.9,self.max_linear,self.max_angular)
            if not self.local_planner.last.path:
                self.state,self.reason='waiting',self.local_planner.last.reason;return self._zero()
            reason=self.local_planner.last.reason
            if not scene.command_clear(v,w):return self._zero()
            self.arrival_count=0
        else:
            self.local_planner.reset()
            v,w=result.v,result.w;reason=result.reason
            if source_stamp!=self.arrival_stamp:
                self.arrival_count=self.arrival_count+1 if result.arrived else 0
                self.arrival_stamp=source_stamp
            if self.mode==self.approach_mode and self.arrival_count>=3:
                self._terminate('arrived','object_clearance_reached');return self._zero()
        # Preserve the exact native terminal arc. Only the far-field branch
        # ramps velocity and subsequently checks that modified arc again.
        if estimate.distance>1.2:
            v=min(v,self.previous_v+.4*dt) if v else 0.
            if not scene.command_clear(v,w):v=w=0.
        self.previous_v,self.previous_w=v,w
        self.state='approaching' if self.mode==self.approach_mode else 'following'
        self.reason=reason
        return v,w

    def snapshot(self,now):
        value=super().snapshot(now)
        value.update(target_label=self.target_label,object_clearance_m=self.clearance,
                     object_surface_gap_m=self.surface_gap,
                     object_surface_fresh=now<self.geometry_until,
                     close_max_linear_mps=self.close_max,close_gain=self.close_gain,
                     measured_linear_mps=self.measured_velocity[0],
                     response_delay_s=self.reaction,braking_deceleration_mps2=self.deceleration)
        return value
