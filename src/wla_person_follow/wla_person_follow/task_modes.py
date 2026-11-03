"""Canonical task modes shared by the command ledger and the MPC adapter."""
import math

MODE_PERSON_FOLLOW = '0'
MODE_PERSON_APPROACH = '1'
MODE_OBJECT_APPROACH = '2'
# Exploration primitives: no tracked target, the goal is given relative to the
# robot when the task starts and fixed in the wheel-odometry frame from then on.
MODE_POINT_GOAL = '3'
MODE_ORIENT = '4'

MODES = {
    MODE_PERSON_FOLLOW: dict(label='person', terminal=False, reid=True),
    MODE_PERSON_APPROACH: dict(label='person', terminal=True, reid=True),
    MODE_OBJECT_APPROACH: dict(label='object', terminal=True, reid=False),
    MODE_POINT_GOAL: dict(label='point', terminal=True, reid=False),
    MODE_ORIENT: dict(label='orient', terminal=True, reid=False),
}
GOAL_MODES = (MODE_POINT_GOAL, MODE_ORIENT)
# Point goals stay inside the rolling map window (+-4 m).
POINT_GOAL_MAX_M = 3.
ORIENT_MAX_RAD = 2*math.pi+.5


def normalize_mode(value):
    """Return a canonical numeric mode; old English aliases are rejected."""
    if isinstance(value, int) and not isinstance(value, bool):
        value = str(value)
    if value not in MODES:
        raise ValueError('mode_must_be_0_person_follow_1_person_approach_2_object_approach_3_point_goal_or_4_orient')
    return value


def mode_config(value):
    return MODES[normalize_mode(value)]


def parse_goal(mode, target_id):
    """Mode 3: "dx,dy" metres in the robot frame. Mode 4: "dyaw" radians (left positive)."""
    try:
        values = [float(part) for part in target_id.split(',')]
    except (AttributeError, ValueError):
        raise ValueError('invalid_goal_target_id')
    if not all(math.isfinite(v) for v in values):
        raise ValueError('invalid_goal_target_id')
    if mode == MODE_POINT_GOAL:
        if len(values) != 2 or not .05 <= math.hypot(*values) <= POINT_GOAL_MAX_M:
            raise ValueError('point_goal_must_be_dx_dy_within_3m')
        return dict(kind='point', relative=values)
    if mode == MODE_ORIENT:
        if len(values) != 1 or not .05 <= abs(values[0]) <= ORIENT_MAX_RAD:
            raise ValueError('orient_goal_must_be_dyaw_within_2pi')
        return dict(kind='orient', relative=values[0])
    raise ValueError('not_a_goal_mode')
