import math
import stat
import sys

import numpy as np
import pytest

from wla_person_follow.neupan_follower import NeuPANFollower

FAKE = r'''#!{python}
import sys
print('READY', flush=True)
n = 0
script = {script!r}
for line in sys.stdin:
    fields = line.split()
    sys.stdin.readline()
    sys.stdin.readline()
    with open({log!r}, 'a') as log:
        log.write(line)
    reply = script[min(n, len(script)-1)]
    n += 1
    if reply == 'exit':
        sys.exit(0)
    if reply == 'hang':
        import time
        time.sleep(5)
    print(reply, flush=True)
'''
GOOD = 'RESULT 1 0 0 0.5 3.0 0.7 0.2'


@pytest.fixture
def make(tmp_path):
    made = []

    def build(*script, timeout_s=.3):
        path = tmp_path/f'fake{len(made)}.py'
        log = tmp_path/f'log{len(made)}'
        path.write_text(FAKE.format(python=sys.executable, script=list(script), log=str(log)))
        path.chmod(path.stat().st_mode | stat.S_IEXEC)
        planner = NeuPANFollower(directory=str(tmp_path), timeout_s=timeout_s, command=[str(path)])
        planner.log = log
        made.append(planner)
        return planner
    yield build
    for planner in made:
        planner.close()


def obstacles():
    return np.zeros((0, 2))


def test_plans_a_straight_line_to_the_following_distance(make):
    planner = make(GOOD)
    assert planner.plan((0., 0., 0.), (3., 0.), obstacles(), 1.2) == pytest.approx([.7, .2])
    assert planner.diagnostic['solved'] is True
    assert planner.log.read_text().split()[:6] == ['0.0', '0.0', '0.0', '1', '2', '0']


def test_starts_from_where_the_last_command_will_have_taken_the_robot(make):
    planner = make(GOOD)
    planner.plan((0., 0., 0.), (3., 0.), obstacles(), 1.2)
    planner.plan((0., 0., 0.), (3., 0.), obstacles(), 1.2)
    second = planner.log.read_text().splitlines()[1].split()
    assert float(second[0]) == pytest.approx(.7*.12)
    assert float(second[2]) == pytest.approx(.2*.12)
    assert second[3] == '0'


def test_at_the_following_distance_it_faces_the_person_without_planning(make):
    planner = make(GOOD)
    command = planner.plan((0., 0., 0.), (0., 1.2), obstacles(), 1.2)
    assert command == pytest.approx([0., .9])
    assert not planner.log.exists()


@pytest.mark.parametrize('reply', ['RESULT 1 1 0 0.01 3.0 0.7 0.2', 'RESULT 1 0 1 inf 3.0 0.7 0.2'])
def test_stop_and_arrive_output_zero(make, reply):
    planner = make(reply)
    assert planner.plan((0., 0., 0.), (3., 0.), obstacles(), 1.2) == pytest.approx([0., 0.])


def test_infinite_distance_stays_json_safe(make):
    planner = make('RESULT 1 0 0 inf 3.0 0.7 0.2')
    planner.plan((0., 0., 0.), (3., 0.), obstacles(), 1.2)
    assert planner.diagnostic['min_distance_m'] is None


def test_stops_after_more_than_five_unsolved_plans(make):
    planner = make('RESULT 0 0 0 0.5 3.0 0.7 0.2')
    commands = [planner.plan((0., 0., 0.), (3., 0.), obstacles(), 1.2) for _ in range(6)]
    assert commands[4] == pytest.approx([.7, .2])
    assert commands[5] == pytest.approx([0., 0.])


def test_late_process_raises_instead_of_falling_back(make):
    planner = make('hang', timeout_s=.05)
    with pytest.raises(RuntimeError, match='timeout'):
        planner.plan((0., 0., 0.), (3., 0.), obstacles(), 1.2)


def test_dead_process_raises_instead_of_falling_back(make):
    planner = make('exit')
    with pytest.raises(RuntimeError, match='exited'):
        planner.plan((0., 0., 0.), (3., 0.), obstacles(), 1.2)


def test_reset_clears_the_last_command_and_restarts_the_path(make):
    planner = make(GOOD)
    planner.plan((0., 0., 0.), (3., 0.), obstacles(), 1.2)
    planner.reset()
    planner.plan((1., 0., 0.), (4., 0.), obstacles(), 1.2)
    second = planner.log.read_text().splitlines()[1].split()
    assert float(second[0]) == pytest.approx(1.)
    assert second[3] == '1'


def test_engine_uses_neupan_only_for_tracked_person_following():
    from wla_person_follow.sim_runtime import SimulationEngine

    class Stub:
        diagnostic = dict(active='plan')

        def plan(self, pose, target, points, gap):
            self.call = (tuple(target), gap)
            return np.array([.3, .1])

    engine = SimulationEngine(np.zeros(3), 0.)
    assert engine.neupan_command(np.zeros(3), dict(state='tracking', prediction=[3., 0.]), '0', False) is None
    engine.neupan = Stub()
    snapshot = dict(state='tracking', prediction=[3., 0.])
    assert engine.neupan_command(np.zeros(3), snapshot, '0', False) == pytest.approx([.3, .1])
    assert engine.neupan.call == ((3., 0.), pytest.approx(1.2))
    for args in ((snapshot, '1', False), (snapshot, '0', True), (None, '0', False),
                 (dict(state='search_last_seen', prediction=[3., 0.]), '0', False),
                 (dict(state='tracking', prediction=None), '0', False)):
        assert engine.neupan_command(np.zeros(3), *args) is None
