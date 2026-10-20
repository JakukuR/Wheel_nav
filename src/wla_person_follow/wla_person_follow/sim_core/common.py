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
    max_v: float = .65
    max_w: float = .9
    accel: float = .65
    yaw_accel: float = 1.5
    radius: float = ROBOT_RADIUS_M
    margin: float = .20
    desired_gap: float = 1.6
    camera_half_angle: float = float(np.arctan(.8))
    central_half_angle: float = float(np.arctan(.64))
    coast_timeout: float = .8

def rotation(yaw):
    return np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
