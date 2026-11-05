"""Short input hiccups must not zero the command of a following robot."""
import pytest
from wla_person_follow.follow_session import same_output_phase
from wla_person_follow.motion_memory import LocalMotion


def _motion():
    m = LocalMotion()
    for k in range(5):
        assert m.update(10.+.02*k, (1., 0., 0.), 10.+.02*k)
    return m  # newest sample at 10.08 s


def test_pose_bridges_a_late_wheel_sample_with_the_twist():
    pose = _motion().pose_at(10.08+.12)
    assert pose is not None and pose[0] == pytest.approx(.08+.12)


def test_pose_is_refused_once_the_wheel_sample_is_too_old():
    assert _motion().pose_at(10.08+.16) is None


def test_pose_before_the_history_is_refused():
    assert _motion().pose_at(9.9) is None


@pytest.mark.parametrize("live,plan", [("coasting", "search_last_seen"), ("search_last_seen", "coasting"),
                                       ("tracking", "coasting"), ("holding", "coasting"),
                                       ("search_last_seen", "search_last_seen")])
def test_driving_phases_keep_the_command_in_person_modes(live, plan):
    assert same_output_phase(live, plan, "0")


@pytest.mark.parametrize("live,plan", [("search_heading", "search_last_seen"), ("search_last_seen", "search_heading"),
                                       ("search_heading", "tracking"), ("binding", "tracking")])
def test_the_sweep_and_unbound_phases_never_reuse_a_driving_plan(live, plan):
    assert not same_output_phase(live, plan, "0")


@pytest.mark.parametrize("live,plan", [("holding", "tracking"), ("search_last_seen", "coasting")])
def test_object_approach_keeps_its_stricter_rule(live, plan):
    assert not same_output_phase(live, plan, "2")
