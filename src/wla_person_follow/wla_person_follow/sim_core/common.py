from dataclasses import dataclass
import numpy as np

# User-specified circular chassis footprint, in metres.
ROBOT_RADIUS_M = .19

def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))

@dataclass(frozen=True)
class Tuning:
    step: float = .1
    horizon: int = 24
    # Follow cruise ceiling (walking pace); approach/object/search modes override it lower.
    max_v: float = 1.2
    max_w: float = .9
    accel: float = .65
    yaw_accel: float = 1.5
    radius: float = ROBOT_RADIUS_M
    margin: float = .10
    # Static obstacles: only penetration (clearance < 0) rejects a rollout; the
    # margin band is an inflation cost, so tight passages stay feasible at a price.
    # People keep margin as a hard limit.
    margin_weight: float = 400.
    desired_gap: float = ROBOT_RADIUS_M+.28+.40
    target_radius: float = .28
    target_stop_gap: float = ROBOT_RADIUS_M+.28+.36
    target_hard_gap: float = ROBOT_RADIUS_M+.28+.10
    target_clearance_max: float = .50
    camera_half_angle: float = float(np.arctan(.8))
    # The person only has to stay inside the detector's image, not near its centre:
    # the near-horizon hard limit is the camera half angle less a small edge margin.
    fov_edge_margin: float = .05
    centering_weight: float = .5
    coast_timeout: float = .8

def rotation(yaw):
    return np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
