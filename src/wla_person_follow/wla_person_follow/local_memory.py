"""Bounded body-twist motion compensation; no global odometry pose or map TF."""
from collections import deque
import math
import numpy as np
from .local_avoidance import inflate_scene


def advance(pose, twist, dt):
    x, y, angle = pose
    vx, vy, w = twist
    mid = angle+w*dt*.5
    return (x+(vx*math.cos(mid)-vy*math.sin(mid))*dt,
            y+(vx*math.sin(mid)+vy*math.cos(mid))*dt, angle+w*dt)


class LocalMotion:
    def __init__(self):
        self.samples = deque(maxlen=128)
        self.generation = 0

    def update(self, stamp, twist, now):
        if not all(math.isfinite(v) for v in (stamp, now, *twist)):
            self.samples.clear(); self.generation += 1
            return False
        if not 0 <= now-stamp <= .20 or abs(twist[0]) > 1.5 or abs(twist[1]) > .3 or abs(twist[2]) > 2.:
            self.samples.clear(); self.generation += 1
            return False
        pose = (0., 0., 0.)
        if self.samples:
            previous, prior, velocity = self.samples[-1]
            dt = stamp-previous
            if dt <= 0:
                return False
            if dt > .20:
                self.samples.clear(); self.generation += 1
            else:
                pose = advance(prior, tuple((a+b)*.5 for a,b in zip(velocity,twist)), dt)
        self.samples.append((stamp, pose, tuple(twist)))
        return True

    def healthy(self, now):
        return bool(self.samples and 0 <= now-self.samples[-1][0] <= .20)

    def pose_at(self, stamp):
        if not self.samples:
            return None
        values = list(self.samples)
        if stamp < values[0][0] or stamp-values[-1][0] > .05:
            return None
        for t, pose, twist in reversed(values):
            if t <= stamp:
                return advance(pose, twist, stamp-t)
        return None


class LocalMemory:
    def __init__(self, ttl=2.):
        self.ttl = ttl
        self.frames = deque(maxlen=9)
        self.generation = None
        self.latest = None

    def reset(self):
        self.frames.clear()
        self.generation = None
        self.latest = None

    @staticmethod
    def world_points(scene, mask, pose):
        iy, ix = np.nonzero(mask)
        local = np.column_stack(((ix-scene.center)*scene.resolution, (iy-scene.center)*scene.resolution))
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return local @ np.array([[c, s], [-s, c]]) + np.asarray(pose[:2])

    def apply(self, scene, pose, stamp, generation):
        if generation != self.generation:
            self.reset(); self.generation = generation
        while self.frames and stamp-self.frames[0][0] > self.ttl:
            self.frames.popleft()
        if self.frames and stamp <= self.frames[-1][0]:
            self.reset(); self.generation = generation
        known = scene.observed.copy(); obstacles = scene.occupied.copy()
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rotation = np.array([[c, -s], [s, c]])
        for _, observed, occupied in self.frames:
            for values, mask in [(observed, known), (occupied, obstacles)]:
                local = (values-np.asarray(pose[:2])) @ rotation
                cells = np.rint(local/scene.resolution).astype(int)+scene.center
                valid = (cells >= 0).all(axis=1) & (cells < mask.shape[0]).all(axis=1)
                cells = cells[valid]
                mask[cells[:, 1], cells[:, 0]] = True
        # Store only this frame's observations, never recursively store merged
        # evidence. Old geometry therefore cannot have its lifetime renewed.
        if not self.frames or stamp-self.frames[-1][0] >= .25:
            self.frames.append((stamp,self.world_points(scene,scene.observed,pose),
                                 self.world_points(scene,scene.occupied,pose)))
        else:
            # Never miss a newly seen obstacle between free-space keyframes.
            # Retain the original keyframe stamp, so this cannot renew old data.
            old_stamp, old_known, old_obstacles = self.frames[-1]
            combined = np.vstack([old_obstacles,self.world_points(scene,scene.occupied,pose)])
            combined = np.unique(np.rint(combined/scene.resolution),axis=0)*scene.resolution
            self.frames[-1] = (old_stamp,old_known,combined)
        self.latest=(stamp,self.world_points(scene,scene.observed,pose),
                     self.world_points(scene,scene.occupied,pose),scene.free.shape,
                     scene.resolution,scene.radius)
        return inflate_scene(known, obstacles, scene.resolution, scene.radius)


    def project(self,pose,stamp,generation):
        """Re-express raw evidence at control time without renewing its age."""
        if self.latest is None or generation!=self.generation:return None
        t,observed,occupied,shape,resolution,radius=self.latest
        if not 0<=stamp-t<=self.ttl:return None
        known=np.zeros(shape,dtype=bool);obstacles=known.copy();center=shape[0]//2
        c,s=math.cos(pose[2]),math.sin(pose[2]);rotation=np.array([[c,-s],[s,c]])
        evidence=list(self.frames)+[(t,observed,occupied)]
        for source_stamp,free_points,obstacle_points in evidence:
            if not 0<=stamp-source_stamp<=self.ttl:continue
            for values,mask in [(free_points,known),(obstacle_points,obstacles)]:
                local=(values-np.asarray(pose[:2])) @ rotation
                cells=np.rint(local/resolution).astype(int)+center
                valid=(cells>=0).all(axis=1)&(cells<shape[0]).all(axis=1)
                cells=cells[valid];mask[cells[:,1],cells[:,0]]=True
        known[center,center]=True
        known[center-1:center+2,center]=True
        known[center,center-1:center+2]=True
        return inflate_scene(known,obstacles,resolution,radius)


def camera_in_reference(base_pose, camera_rotation, camera_translation):
    """Image-time local_reference<-base<-camera SE(3) composition.

    Base motion is currently planar wheel integration. The camera extrinsic
    may be dynamic and is supplied by the image-time TF lookup, not cached.
    """
    x,y,yaw=base_pose
    c,s=math.cos(yaw),math.sin(yaw)
    reference_from_base=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    rotation=reference_from_base @ np.asarray(camera_rotation)
    translation=reference_from_base @ np.asarray(camera_translation)+np.array([x,y,0.])
    return rotation,translation
