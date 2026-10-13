"""Execute the deployed Gateway methods with controlled callback ordering.

These tests use the real TaskGateway state machine; ROS/DDS is separately smoked.
GATEWAY_SCRIPT can point at the installed executable to verify the deployed file.
"""
import ast
import asyncio
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS
import uuid

from wla_sim.behavior_bridge.tasks import TaskGateway, TaskRequest
from wla_sim.behavior_bridge.semantic import ApproachPose, finite


class Clock:
    def __init__(self):
        self.value = 10.

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class Deferred:
    def __init__(self, result=None):
        self.value = result
        self.callbacks = []

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


def status_array(*entries):
    return NS(status_list=[NS(status=state, goal_info=NS(goal_id=NS(uuid=list(key))))
                           for key, state in entries])


def fixture(submit=True):
    clock = Clock()
    path = Path(os.environ.get('GATEWAY_SCRIPT', str(
        Path(__file__).resolve().parents[1] / 'scripts/navigation_task_gateway')))
    cls = next(n for n in ast.parse(path.read_text(encoding='utf8')).body
               if isinstance(n, ast.ClassDef) and n.name == 'Gateway')

    def nav_goal():
        return NS(pose=NS(header=NS(), pose=NS(position=NS(), orientation=NS())))

    env = dict(Node=object, time=NS(monotonic=clock), math=math, uuid=uuid,
               TaskLease=NS, GoalInfo=lambda: NS(goal_id=NS(uuid=[])),
               NavigateToPose=NS(Goal=nav_goal), finite=finite,
               Future=asyncio.Future, CancelGoal=NS(Request=NS))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), env)
    gateway = env['Gateway'].__new__(env['Gateway'])
    gateway.core = TaskGateway(clock)
    gateway.core.health(True)
    gateway.core.motion(0., 0.)
    clock.advance(.21)
    gateway.core.motion(0., 0.)
    request = TaskRequest('r1', 'instance', 1, 'agent', 'map-version', 'feature',
                          'fingerprint', ApproachPose(1., 2., 0.))
    token = gateway.core.submit(request)['token'] if submit else None
    gateway.handles = {}
    gateway.goal_ids = {}
    gateway.waiters = {}
    gateway.plan_waits = []
    gateway.foreign_active = False
    gateway.foreign_generation = 0
    gateway.reconciled = True
    gateway.claiming = False
    gateway.status_seen = clock()
    gateway.map = NS(map_revision=request.map_revision,
                     targets={request.feature_id: NS(fingerprint=request.target_fingerprint)})
    gateway.executor_lease = ((request.request_id, request.owner, request.generation),
                              clock(), True)
    gateway.dispatch_clear = lambda _: True
    gateway.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: None))
    gateway.logs = []
    gateway.get_logger = lambda: NS(warning=gateway.logs.append)
    gateway.published = []
    gateway.lease_pub = NS(publish=gateway.published.append)
    gateway.sent = []
    gateway.canceled = []

    def send(goal, *, goal_uuid, feedback_callback):
        key = bytes(goal_uuid.uuid)
        gateway.sent.append(key)
        handle = NS(accepted=True, goal_id=goal_uuid,
                    get_result_async=lambda: Deferred(),
                    cancel_goal_async=lambda: gateway.canceled.append(key))
        return Deferred(handle)

    gateway.nav = NS(send_goal_async=send)
    return clock, gateway, request, token


def start_running(gateway, token):
    original = gateway.nav.send_goal_async
    result = []

    def send(*args, **kwargs):
        future = original(*args, **kwargs)
        result.append(future)
        return future

    gateway.nav.send_goal_async = send
    gateway.tick()
    gateway.nav_accepted(result[0], token)
    gateway.tick()
    assert gateway.core.permit()
    return gateway.sent[0]


def test_own_status_before_acceptance_is_recognized():
    _, gateway, request, token = fixture()
    original = gateway.nav.send_goal_async

    def send(*args, **kwargs):
        key = bytes(kwargs['goal_uuid'].uuid)
        assert gateway.goal_ids[token] == key
        assert not gateway.handles
        gateway.status(status_array((key, 2)))
        return original(*args, **kwargs)

    gateway.nav.send_goal_async = send
    gateway.tick()
    assert gateway.reconciled and not gateway.foreign_active
    assert not gateway.core.records[request.request_id]['cancel']


def test_foreign_goal_revokes_immediately_and_only_cancels_owned_goal():
    _, gateway, request, token = fixture()
    own = start_running(gateway, token)
    foreign = uuid.uuid4().bytes
    gateway.status(status_array((own, 2), (foreign, 2)))
    assert not gateway.published[-1].allowed  # before the next timer
    assert not gateway.core.permit() and not gateway.reconciled
    assert gateway.core.active == request.request_id
    gateway.tick()
    assert gateway.canceled == [own]
    assert foreign not in gateway.canceled
    assert not gateway.published[-1].allowed


def test_foreign_goal_before_own_acceptance_cancels_late_owned_handle():
    _, gateway, request, token = fixture()
    original = gateway.nav.send_goal_async
    pending = []

    def send(*args, **kwargs):
        result = original(*args, **kwargs)
        pending.append(result)
        return result

    gateway.nav.send_goal_async = send
    gateway.tick()
    own = gateway.sent[0]
    gateway.status(status_array((own, 1), (uuid.uuid4().bytes, 2)))
    gateway.nav_accepted(pending[0], token)
    gateway.tick()
    assert gateway.canceled == [own]
    assert gateway.core.active == request.request_id
    assert not gateway.published[-1].allowed


def test_empty_status_does_not_restore_cancelled_lease_or_control_claim():
    _, gateway, request, token = fixture()
    own = start_running(gateway, token)
    gateway.status(status_array((own, 2), (uuid.uuid4().bytes, 2)))
    gateway.status(status_array((own, 3)))
    assert not gateway.foreign_active and not gateway.reconciled
    gateway.core.heartbeat(request.request_id, request.owner, request.generation)
    gateway.tick()
    assert not gateway.published[-1].allowed
    assert gateway.core.records[request.request_id]['cancel']


def test_foreign_terminal_history_does_not_revoke_an_owned_active_task():
    _, gateway, _, token = fixture()
    own = start_running(gateway, token)
    gateway.status(status_array((own, 2), (uuid.uuid4().bytes, 4),
                                (uuid.uuid4().bytes, 5), (uuid.uuid4().bytes, 6)))
    gateway.tick()
    assert not gateway.foreign_active and gateway.reconciled
    assert gateway.published[-1].allowed and not gateway.canceled


def test_unknown_send_retains_registered_uuid_without_reopening_lease():
    _, gateway, request, token = fixture()

    def failed_send(*args, **kwargs):
        raise RuntimeError('acceptance unknown')

    gateway.nav.send_goal_async = failed_send
    gateway.tick()
    own = gateway.goal_ids[token]
    gateway.status(status_array((own, 2)))
    gateway.tick()
    assert not gateway.foreign_active
    assert gateway.core.active == request.request_id
    assert gateway.core.records[request.request_id]['cancel']
    assert not gateway.published[-1].allowed


def test_claim_response_cannot_overwrite_new_foreign_activity():
    async def scenario():
        _, gateway, _, _ = fixture(submit=False)
        reply = Deferred(NS(return_code=0, goals_canceling=[]))
        gateway.cancel_all = NS(service_is_ready=lambda: True, call_async=lambda _: reply)
        response = NS(success=None, message='')
        task = asyncio.create_task(gateway.claim_control(NS(), response))
        await asyncio.sleep(0)
        assert gateway.claiming and not gateway.reconciled
        gateway.status(status_array((uuid.uuid4().bytes, 2)))
        reply.callbacks[0](reply)
        await task
        assert response.success is False
        assert not gateway.reconciled and gateway.foreign_active

    asyncio.run(scenario())
