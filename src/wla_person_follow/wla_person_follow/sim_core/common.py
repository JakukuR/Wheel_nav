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
    # Reference polylines (shortcut, detour, grid route) get their corners rounded
    # by a cubic B-spline within this distance of each vertex; it falls back to
    # 0.6x, 0.3x, then the raw polyline when the curve would lose clearance.
    # 0 disables.
    reference_corner_m: float = .35
    # Static obstacles: only penetration (clearance < 0) rejects a rollout; the
    # margin band is an inflation cost, so tight passages stay feasible at a price.
    # People keep margin as a hard limit.
    margin_weight: float = 400.
    # Robot centre to person centre. 0.87 m (0.40 m clear) let a turning person
    # leave the image at corridor corners; 1.2 m by operator choice 2026-11-02.
    desired_gap: float = ROBOT_RADIUS_M+.28+.73
    target_radius: float = .28
    target_stop_gap: float = ROBOT_RADIUS_M+.28+.69
    target_hard_gap: float = ROBOT_RADIUS_M+.28+.10
    target_clearance_max: float = .50
    camera_half_angle: float = float(np.arctan(.8))
    # The person only has to stay inside the detector's image, not near its centre:
    # the near-horizon hard limit is the camera half angle less a small edge margin.
    fov_edge_margin: float = .05
    # Weak pull to the image centre plus an edge cost starting well inside the
    # image: with only the hard edge limit the person drifted to ~35 deg at 1 m/s and
    # left the frame on the next sidestep (J02 2026-10-29 15:37). 6 rather than 2:
    # circling at 0.3 rad/s, 2 settled into a stop-go turn inside the chassis
    # deadband and let the person reach the edge (simulated, 5 seeds).
    centering_weight: float = 6.
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
    # No output yaw floor: the rollout models the chassis deadband (chassis_yaw),
    # so the MPC itself picks turns the chassis executes. A floor that lifted
    # |w| in [0.04, 0.28) to 0.28 locked turns: the first step is bounded from
    # the measured yaw rate, which the floor had raised, so the raw command stayed
    # above 0.04 and was lifted again (J02 2026-10-30 16:45: planner |w| median
    # 0.28 at every bearing, only ~50 % of turns toward the person).
    # Stuck with only flank-looking left: back up along the robot's own swept
    # path (known free: the body was just there) instead of turning in place.
    backup_speed: float = .2
    # Backing out reverses at >= backup_min_speed while steering the tail onto
    # the trail. Reversing at backup_speed*cos(err) gave 0.03-0.05 m/s, which
    # the chassis does not move at (11-02 16:47: 9 s of backing_out, 0 m). No
    # 5 cm within backup_stall_s of reversing ends the backing for this stuck
    # episode. (The 11-02 turn-the-tail-first step was removed 11-03.)
    backup_min_speed: float = .12
    backup_stall_s: float = 2.
    # Chassis response inside the rollout (measured on 4 J02 runs, 2026-10-29):
    # sustained yaw commands reach 63 % after 0.44 s; mean |w| below 0.2 rad/s
    # gives ~20 %, 0.2-0.3 partly, >= 0.3 fully. Without it the rollouts assumed
    # instant turns, chose too-small rates and, in the chassis-model sim, grazed
    # obstacles. 0 disables (commands act instantly, as before).
    # Refit 2026-11-02 on four J02 runs (10-29 17:25/17:47, 10-30 16:45/18:41,
    # command vs wheel odometry at 50 Hz): steady gain 1.0, no dead time,
    # tau 0.5-0.6 s (rms 0.0995 -> 0.0948 rad/s); 0.40 made the rollouts expect
    # turns sooner than the chassis delivers them.
    chassis_tau_w: float = .50
    chassis_tau_v: float = .18
    chassis_deadband_lo: float = .2
    chassis_deadband_hi: float = .3
    backup_max_m: float = .6
    # After reaching the last-seen point the robot turns to the direction the
    # person left (walking heading, else last bearing) and sweeps +-0.6 rad.
    # 0.3 rad/s took 3-12 s per loss (normal follow 2026-10-30 19:08) and could
    # not follow the 8 s sweep (peak 0.47 rad/s).
    search_turn_rate: float = .6
    # Search at the last-seen point, relative to the walking direction: first
    # ~90 deg to the side the person left on (the last-seen point is before a
    # corridor corner; 11-02 14:53), then keep rotating the same way until found.
    # The former +1.5/-0.8 rad swing never looked behind the robot: a person who
    # turned back past it stayed unseen for good (closed-loop simulation 11-04/05).
    search_exit_turn: float = 1.5
    # Driving to the last-seen point (usually the corner the person turned at).
    # 0.20 m/s took 5-10 s to get there after a corner loss.
    search_speed: float = .4

def rotation(yaw):
    return np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
