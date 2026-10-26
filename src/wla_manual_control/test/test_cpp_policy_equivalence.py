"""Differential test: C++ VelocityPolicy against scripts/velocity_policy.py.

Random event sequences (valid, stale, malformed and non-finite inputs) go to the
Python policy and to velocity_policy_replay; every field and result must match
after every event.
Usage: test_cpp_policy_equivalence.py <velocity_policy_replay> [sequences] [seed]
"""
import json
import math
from pathlib import Path
import random
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from velocity_policy import VelocityPolicy

BASE_NS = 1_792_000_000 * 10**9
NUMBERS = [0., -0., .005, .2, .65, -.25, 4., -4., 1, 0, True, None, 'a', math.nan, math.inf]
STATUSES = ['ACTIVE', 'SPIN', 'STANDBY', 'ARMING', 'ESTOP', 'DISCONNECTED', 'BOGUS',
            '[ACTIVE] 档位:mid(60%)', '[[STANDBY]]x', '', ']', '[DISCONNECTED] 死人开关:OFF']


def enc(v):
    if isinstance(v, float) and not math.isfinite(v):
        return 'nan' if math.isnan(v) else ('inf' if v > 0 else '-inf')
    return v


def dec(v):
    return {'nan': math.nan, 'inf': math.inf, '-inf': -math.inf}.get(v, v) if isinstance(v, str) else v


def decode(data):  # command_mux.py Mux.decode
    try:
        return json.loads(data)
    except (ValueError, TypeError):
        return None


def stamp(rng, now_ns):
    lag = rng.choice([0, 0, 0, 10**7, 10**8, 149 * 10**6, 150 * 10**6, 151 * 10**6, 199 * 10**6,
                      200 * 10**6, 201 * 10**6, -4 * 10**7, -5 * 10**7, -6 * 10**7, 2 * 10**9])
    return rng.choice([now_ns - lag] * 8 + [None, 1.5e18, True, -5, 2**70, 'x'])


def raw_json(rng, d):
    r = rng.random()
    if r < .04:
        return 'not json'
    if r < .07:
        return json.dumps([d])
    text = json.dumps(d)
    if r < .10 and d:  # duplicate key: last one wins in both parsers
        k = rng.choice(list(d))
        text = text[:-1] + ', ' + json.dumps(k) + ': ' + json.dumps(rng.choice(NUMBERS)) + '}'
    return text


def candidate(rng, now_ns):
    d = dict(schema=rng.choice(['wla-person-candidate/v1'] * 6 + ['wla-nav-candidate/v1', 5]),
             stamp_ns=stamp(rng, now_ns), linear=rng.choice(NUMBERS), angular=rng.choice(NUMBERS))
    for k in list(d):
        if rng.random() < .05:
            del d[k]
    return raw_json(rng, d)


def person_status(rng, now_ns):
    d = dict(schema=rng.choice(['wla-person-status/v1'] * 6 + ['other']),
             stamp_ns=stamp(rng, now_ns), request_id=rng.choice(['', 'r1', 'r2', 'r1', 5, None]),
             follow_session=rng.choice(['s1', 's2', 's1', '', None]),
             enabled=rng.choice([True, True, False, 1, None]),
             candidate_only=rng.choice([False, False, True, 0]),
             motion_permitted=rng.choice([True, False, None]))
    if rng.random() < .5:
        d['fault'] = rng.choice([False, False, True, 0])
    for k in list(d):
        if rng.random() < .04:
            del d[k]
    return raw_json(rng, d)


def sequence(rng, n):
    # Clean sequences stay mostly unlatched so every priority path is exercised;
    # hostile ones mix in malformed, stale and non-finite inputs.
    clean = rng.random() < .6
    # Half the sequences run on a 1/256 s grid so exact boundaries such as .5 s occur.
    grid = rng.random() < .5
    t = 100. + (0. if grid else rng.random())
    yield dict(op='reset', limits=[1., .3, 1.5])
    last = {}
    for _ in range(n):
        t += 0. if rng.random() < .05 else rng.choice([.001, .005, .02, .02, .02, .05, .1, .149, .151, .2, .21, .3, .6] if not clean else
                        [.005, .02, .02, .02, .02, .05, .1, .149, .151, .21])
        if grid:
            t = round(t * 256) / 256
        now_ns = BASE_NS + int(t * 1e9) + rng.choice([0, 0, 1, -1, 10**6])
        op = rng.choices(['output', 'manual', 'auto', 'person', 'person_status', 'status', 'wheel',
                          'estop', 'mode', 'source'], [30, 12, 12, 12, 6, 8, 10, 4, 2, 2])[0]
        e = dict(op=op, t=t, now_ns=now_ns)
        values = [.0, .0, .005, .3, -.2, 2.] + ([] if clean else [math.nan, math.inf])
        if op in ('manual', 'auto'):
            e['v'] = [enc(rng.choice(values)), enc(rng.choice(values))]
        elif op == 'person':
            e['data'] = candidate(rng, now_ns) if not clean else json.dumps(dict(
                schema='wla-person-candidate/v1', stamp_ns=now_ns - rng.choice([0, 10**7, 2 * 10**8]),
                linear=rng.choice([0., .2, .65, 4., -4.]), angular=rng.choice([0., .1, -.45])))
        elif op == 'person_status':
            e['data'] = person_status(rng, now_ns)
        if op in ('person', 'person_status'):
            # Re-deliver an earlier message now and then: equal stamps must be rejected.
            if op in last and rng.random() < .15:
                e['data'] = last[op]
            last[op] = e['data']
        elif op == 'status':
            e['data'] = rng.choice(STATUSES if not clean else
                                   ['ACTIVE'] * 20 + ['SPIN', 'STANDBY', 'ARMING', 'DISCONNECTED', 'ESTOP'])
        elif op == 'wheel':
            e['v'] = [enc(rng.choice([0.] * (8 if clean else 2) + [.005, .05] + ([] if clean else [math.nan]))),
                      enc(rng.choice([0.] * (8 if clean else 2) + [.02] + ([] if clean else [math.nan])))]
            e['fresh'] = rng.random() < .85
        elif op in ('estop', 'mode', 'source'):
            e['data'] = rng.random() < ((.05 if clean else .3) if op == 'estop' else .5)
        yield e


def py_snapshot(p, t):
    pair = lambda v: [enc(float(v[0])), enc(float(v[1]))]
    return dict(
        mode=p.mode, armed=p.armed, estop_latched=p.estop_latched, estop_reason=p.estop_reason,
        gamepad_stop_asserted=p.gamepad_stop_asserted, manual_ready_after=enc(p.manual_ready_after),
        manual=pair(p.manual), manual_at=enc(p.manual_at), status=p.status, status_at=enc(p.status_at),
        wheel=pair(p.wheel), wheel_at=enc(p.wheel_at), automatic_source=p.automatic_source,
        source_generation=p.source_generation, person_revoke_generation=p.person_revoke_generation,
        output_source=p.output_source, person=pair(p.person), person_at=enc(p.person_at),
        person_status_at=enc(p.person_status_at), person_until=enc(p.person_until),
        person_status_until=enc(p.person_status_until), person_stamp_ns=p.person_stamp_ns,
        person_status_stamp_ns=p.person_status_stamp_ns, person_request_id=p.person_request_id,
        person_enabled=p.person_enabled, person_session=p.person_session,
        person_activation_ns=p.person_activation_ns, person_candidate_only=p.person_candidate_only,
        person_motion_permitted=p.person_motion_permitted, person_fault=p.person_fault,
        blocked_person_request_id=p.blocked_person_request_id,
        blocked_person_session=p.blocked_person_session,
        commands={k: pair(v) + [enc(r), enc(u)] for k, (v, r, u) in p.commands.items()},
        last_stamps=p.last_stamps, manual_override=p.manual_override,
        resume_after=enc(p.resume_after), manual_ready=p.manual_ready(t), connected=p.connected(t),
        stopped=p.stopped(t), nav_fresh=p.command_fresh('NAV', t),
        person_fresh=p.command_fresh('PERSON', t))


def py_apply(p, e):
    op, t, now_ns = e['op'], e.get('t', 0.), e.get('now_ns', 0)
    v = [dec(x) for x in e.get('v', [])]
    if op == 'reset':
        return VelocityPolicy(*e['limits']), None
    if op == 'manual':
        return p, p.manual_input(tuple(v), t)
    if op == 'auto':
        return p, p.automatic_input(tuple(v), t)
    if op == 'person':
        return p, p.person_input(decode(e['data']), t, now_ns)
    if op == 'person_status':
        return p, p.person_status_input(decode(e['data']), t, now_ns)
    if op == 'status':
        return p, p.gamepad_status_input(e['data'].split(']', 1)[0].lstrip('['), t)
    if op == 'wheel':
        p.wheel = tuple(v)
        if e['fresh']:
            p.wheel_at = t
        return p, None
    if op == 'estop':
        return p, list(p.set_estop(e['data'], t))
    if op == 'mode':
        if e['data']:
            return p, list(p.set_auto(t, now_ns))
        p.set_manual()
        return p, [True, 'cleared']
    if op == 'source':
        return p, list(p.set_person_source(e['data'], t, now_ns))
    if op == 'output':
        return p, [enc(float(x)) for x in p.output(t)]
    raise ValueError(op)


def same(a, b):
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    return a == b


def main():
    replay = sys.argv[1]
    sequences = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    seed = int(sys.argv[3]) if len(sys.argv) > 3 else 20261026
    rng = random.Random(seed)
    events = [e for _ in range(sequences) for e in sequence(rng, 400)]
    lines = '\n'.join(json.dumps(e, ensure_ascii=False) for e in events) + '\n'
    cpp = subprocess.run([replay], input=lines.encode(), capture_output=True, check=True).stdout.decode().splitlines()
    assert len(cpp) == len(events), (len(cpp), len(events))
    p, counts = None, {}
    for i, (e, line) in enumerate(zip(events, cpp)):
        p, result = py_apply(p, e)
        expected = py_snapshot(p, e.get('t', 0.)) if e['op'] != 'reset' else None
        got = json.loads(line)
        if expected is not None:
            expected['result'] = result
            if not same(expected, got):
                diff = {k: (expected[k], got.get(k)) for k in expected if not same(expected[k], got.get(k))}
                raise AssertionError(f'event {i} {json.dumps(e, ensure_ascii=False)}\n{diff}')
        if e['op'] == 'output':
            counts[p.output_source] = counts.get(p.output_source, 0) + 1
        if p is not None and p.estop_latched:
            counts['latched_events'] = counts.get('latched_events', 0) + 1
    print(f'PASS: {len(events)} events in {sequences} sequences identical; output sources {counts}')


if __name__ == '__main__':
    main()
