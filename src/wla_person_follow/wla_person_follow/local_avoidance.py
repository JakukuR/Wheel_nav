"""Native local perception/planning kernels; no Python search fallback."""
import numpy as np
from _wla_follow_local import LocalScene, LocalAvoidance, LocalPlan, inflate_scene
from _wla_follow_local import scene_from_points as _scene_from_points

def scene_from_points(points, origin=(0., 0.), **kwargs):
    return _scene_from_points(np.asarray(points,dtype=np.float32).reshape(-1,3),origin,**kwargs)
