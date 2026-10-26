"""Canonical task modes shared by the command ledger and the MPC adapter."""

MODE_PERSON_FOLLOW = '0'
MODE_PERSON_APPROACH = '1'
MODE_OBJECT_APPROACH = '2'

MODES = {
    MODE_PERSON_FOLLOW: dict(label='person', terminal=False, reid=True),
    MODE_PERSON_APPROACH: dict(label='person', terminal=True, reid=True),
    MODE_OBJECT_APPROACH: dict(label='object', terminal=True, reid=False),
}


def normalize_mode(value):
    """Return a canonical numeric mode; old English aliases are rejected."""
    if isinstance(value, int) and not isinstance(value, bool):
        value = str(value)
    if value not in MODES:
        raise ValueError('mode_must_be_0_person_follow_1_person_approach_or_2_object_approach')
    return value


def mode_config(value):
    return MODES[normalize_mode(value)]
