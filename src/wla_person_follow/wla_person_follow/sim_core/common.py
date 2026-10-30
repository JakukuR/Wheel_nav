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
    # 1.5 let a 0.05 s control cycle change the turn rate by only 0.075 rad/s, so
    # reaching the chassis' usable 0.3 rad/s took 0.2 s before its own 0.44 s lag
    # (J02 2026-10-29 17:47: planner |w| median 0.26 when 0.9 was needed).
    yaw_accel: float = 3.
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
    # Weak pull to the image centre plus an edge cost starting well inside the
    # image: with only the hard edge limit the person drifted to ~35 deg at 1 m/s and
    # left the frame on the next sidestep (J02 2026-10-29 15:37).
    centering_weight: float = 2.
    fov_soft_angle: float = .45
    # Between this bearing and fov_soft_angle the planner shifts from the person's
    # footprints to the person (trail weight 6 -> 1.5, heading blend .25 -> 1) and
    # caps speed down to (1 - lateral_slowdown) * max_v (J02 2026-10-29 17:25: the
    # person passed the robot on its right at 1.4 m while it chased the trail at
    # 0.9 m/s, turning the other way).
    lateral_bearing_start: float = .15
    lateral_slowdown: float = .5
    # Turn first: above turn_slow_angle between the heading and the point being
    # steered to (detour waypoint, else the person) the speed cap falls linearly,
    # reaching 0 (turn in place) at turn_first_angle. 17:47 J02: the detour point
    # was 0.7-1.2 rad off while the robot kept 0.65-0.79 m/s and pinned itself
    # on the desk it had to go round.
    turn_slow_angle: float = .3
    turn_first_angle: float = 1.
    # Chassis yaw deadband (measured: 0.5 s mean |w| 0.1-0.2 -> ~20 % achieved,
    # >= 0.3 -> 80-100 %): commanded turns in [yaw_deadband_min, yaw_floor) are
    # raised to yaw_floor at the follow output (manual control is untouched).
    yaw_deadband_min: float = .04
    yaw_floor: float = .28
    # Stuck with only flank-looking left: back up along the robot's own swept
    # path (known free: the body was just there) instead of turning in place.
    backup_speed: float = .2
    # Chassis response inside the rollout (measured on 4 J02 runs, 2026-10-29):
    # sustained yaw commands reach 63 % after 0.44 s; mean |w| below 0.2 rad/s
    # gives ~20 %, 0.2-0.3 partly, >= 0.3 fully. Without it the rollouts assumed
    # instant turns, chose too-small rates and, in the chassis-model sim, grazed
    # obstacles. 0 disables (commands act instantly, as before).
    chassis_tau_w: float = .40
    chassis_tau_v: float = .18
    chassis_deadband_lo: float = .2
    chassis_deadband_hi: float = .3
    backup_max_m: float = .6

def rotation(yaw):
    return np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
