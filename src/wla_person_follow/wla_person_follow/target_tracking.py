"""Identity and robot-frame adapter around the native Kalman estimator."""
from dataclasses import dataclass
import math
from _wla_follow_local import TargetKalman

@dataclass(frozen=True)
class TargetEstimate:
    target_id: str
    yaw: float
    distance: float
    age: float
    std: float
    observed: bool
    last_seen_yaw: float = 0.
    last_seen_distance: float = 0.

class TargetTracker:
    def __init__(self,horizon=2.,max_std=.75):
        self.filter=TargetKalman(horizon,max_std)
        self.key=None
        self.frame_stamp=0
        self.last_position=None

    def reset(self,key=None):
        self.filter.reset();self.key=key;self.frame_stamp=0;self.last_position=None

    def update(self,stamp,yaw,distance,pose):
        x,y,theta=pose
        position=(x+distance*math.cos(theta+yaw),y+distance*math.sin(theta+yaw))
        accepted=self.filter.update(stamp,*position)
        if accepted:self.last_position=position
        return accepted

    def estimate(self,stamp,pose,target_id,observation_stamp):
        values=self.filter.estimate(stamp)
        if not values or self.last_position is None:return None
        x,y,_,_,age,std=values
        dx,dy=x-pose[0],y-pose[1]
        yaw=math.atan2(dy,dx)-pose[2]
        yaw=math.atan2(math.sin(yaw),math.cos(yaw))
        gx,gy=self.last_position[0]-pose[0],self.last_position[1]-pose[1]
        goal_yaw=math.atan2(gy,gx)-pose[2]
        goal_yaw=math.atan2(math.sin(goal_yaw),math.cos(goal_yaw))
        return TargetEstimate(target_id,yaw,math.hypot(dx,dy),age,std,
                              abs(self.filter.stamp-observation_stamp)<1e-7,
                              goal_yaw,math.hypot(gx,gy))
