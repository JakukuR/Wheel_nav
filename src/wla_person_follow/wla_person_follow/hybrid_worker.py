"""ROS-free worker runtime. ROS message classes are data codecs, never a Node.

C++ owns DDS, input pairing, integrated wheel history and the final output gate.
The existing FollowEngine owns task/tracking/numerical state. Large buffers use
one acknowledged slot per direction; there is no frame queue behind the solver.
"""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import logging
import mmap
import os
import select
import signal
import socket
import struct
import threading
import time
from types import SimpleNamespace as NS

from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from wla_behavior_interfaces.srv import PersonCommand
from .sim_node import FollowEngine

SLOT_BYTES = 16 * 1024 * 1024
MAX_METADATA = 1024 * 1024


def encode_packet(metadata, blobs):
    metadata = dict(metadata)
    metadata['blobs'] = {}
    payload = bytearray()
    for name, data in blobs.items():
        metadata['blobs'][name] = dict(offset=len(payload), size=len(data))
        payload.extend(data)
    text = json.dumps(metadata, separators=(',', ':'), allow_nan=False).encode()
    if len(text) > MAX_METADATA or 4 + len(text) + len(payload) > SLOT_BYTES:
        raise ValueError('IPC packet exceeds capacity')
    return struct.pack('<I', len(text)) + text + payload


def decode_packet(data):
    if not 4 <= len(data) <= SLOT_BYTES:
        raise ValueError('invalid IPC size')
    size, = struct.unpack_from('<I', data)
    if size > MAX_METADATA or size > len(data) - 4:
        raise ValueError('invalid IPC metadata size')
    metadata = json.loads(data[4:4 + size])
    payload = memoryview(data)[4 + size:]
    blobs = {}
    for name, item in metadata.get('blobs', {}).items():
        offset, length = item['offset'], item['size']
        if type(offset) is not int or type(length) is not int or min(offset, length) < 0 or offset > len(payload) or length > len(payload) - offset:
            raise ValueError('invalid IPC blob range')
        blobs[name] = bytes(payload[offset:offset + length])
    return metadata, blobs


class TransformCache:
    def __init__(self):
        self.values = {}
        self.lock = threading.Lock()

    def update(self, rows, blobs):
        with self.lock:
            for key, name in rows.items():
                self.values[key] = deserialize_message(blobs[name], TransformStamped)
            while len(self.values) > 128:
                del self.values[next(iter(self.values))]

    def lookup_transform(self, target, source, at):
        with self.lock:
            value = self.values.get(f'{source}:{at.nanoseconds}')
            if value is None or value.header.frame_id != target:
                raise ValueError('cpp_acquisition_transform_unavailable')
            return value


class Publisher:
    def __init__(self, owner, topic):
        self.owner, self.topic = owner, topic

    def publish(self, message):
        self.owner.publish(self.topic, message)


class WorkerTransport:
    def __init__(self, sock, memory, hello):
        if hello.get('protocol') != 1:
            raise ValueError('unsupported follow IPC protocol')
        self.sock, self.memory = sock, memory
        self.config, self.session = hello['config'], hello['session']
        self.generation = hello['generation']
        self.engine = None
        self.running = True
        self.context = NS(ok=lambda: self.running)
        self.callbacks, self.services = {}, {}
        self.tf = TransformCache()
        self.timers = []
        self.guard_ready = threading.Event()
        self.guard_callback = None
        self.outbox = {}
        self.lock = threading.RLock()
        self.result_sequence = self.input_sequence = self.geometry_sequence = 0
        self.geometry_busy = False
        self.candidate = dict(linear=0., angular=0.)
        self.preview = dict(linear=0., angular=0.)

    def send(self, message):
        data = json.dumps(message, separators=(',', ':'), allow_nan=False).encode()
        if len(data) > 128 * 1024:
            raise ValueError('IPC notification too large')
        with self.lock:
            if self.sock.send(data) != len(data):
                raise ConnectionError('short IPC packet')

    def declare_parameter(self, name, default):
        return NS(value=self.config.get(name, default))

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=time.time_ns()))

    def get_logger(self):
        return logging.getLogger('follow_compute')

    def transform_buffer(self):
        return self.tf

    def compute_pool(self):
        # With ROS dispatch in C++, numerical work stays in this Python process.
        # Single worker preserves process_step's state ownership and bounded job count.
        return ThreadPoolExecutor(max_workers=1, thread_name_prefix='follow-numeric')

    def create_publisher(self, _kind, topic, _qos):
        return Publisher(self, topic)

    def create_subscription(self, kind, topic, callback, _qos, **_kwargs):
        self.callbacks[topic] = (kind, callback)

    def create_service(self, kind, topic, callback):
        self.services[topic] = (kind, callback)

    def create_guard_condition(self, callback):
        self.guard_callback = callback
        return NS(trigger=self.guard_ready.set)

    def create_timer(self, period, callback):
        self.timers.append([time.monotonic() + period, period, callback])
        return self.timers[-1]

    def publish(self, topic, message):
        prefix = self.engine.p['prefix'] if self.engine else self.config.get('prefix', '/r680/person_follow')
        suffix = topic[len(prefix):]
        if suffix == '/command_candidate':
            self.candidate = json.loads(message.data)
        elif suffix == '/cmd_vel_candidate':
            self.preview = dict(linear=message.linear.x, angular=message.angular.z)
        elif suffix == '/status':
            self.publish_result(json.loads(message.data))
        elif suffix == '/reid_request':
            self.send(dict(kind='interest', generation=self.generation, data=message.data))
        elif suffix in ('/local_path', '/local_obstacles', '/depth_obstacles'):
            name = {'/local_path': 'path', '/local_obstacles': 'grid', '/depth_obstacles': 'cloud'}[suffix]
            with self.lock:
                self.outbox[name] = (message, self.generation, time.monotonic())
        else:
            raise ValueError('unsupported hybrid publisher: ' + topic)

    def publish_result(self, status):
        e = self.engine
        result = e.result
        self.result_sequence += 1
        proof = dict(sequence=self.result_sequence, generation=result.get('transport_generation', -1) if result else self.generation,
                     request_id=status['request_id'], motion_generation=result['key'][-1] if result else e.motion.generation,
                     detector_session=status['session'], detector_epoch=status['epoch'],
                     observation_ns=result.get('observation_stamp_ns', 0) if result else 0,
                     frame_ns=result['stamp_ns'] if result else 0,
                     valid_until=result['valid_until'] if result else 0., generated_at=time.monotonic(),
                     permitted=bool(status['motion_permitted']),
                     linear=float(self.candidate.get('linear', 0.)), angular=float(self.candidate.get('angular', 0.)))
        self.send(dict(kind='result', proof=proof, status=status, preview=self.preview))

    def flush_geometry(self):
        with self.lock:
            if self.geometry_busy or not self.outbox:
                return
            now = time.monotonic()
            rows = {k: v for k, v in self.outbox.items() if v[1] == self.generation and now - v[2] <= .30}
            self.outbox.clear()
            if not rows:
                return
            packet = encode_packet(dict(generation=self.generation, generated_at=min(v[2] for v in rows.values())),
                                   {k: serialize_message(v[0]) for k, v in rows.items()})
            self.memory[SLOT_BYTES:SLOT_BYTES + len(packet)] = packet
            self.geometry_sequence += 1
            self.send(dict(kind='geometry', sequence=self.geometry_sequence, size=len(packet)))
            self.geometry_busy = True

    def input(self, notification):
        seq, size = notification['sequence'], notification['size']
        if type(seq) is not int or seq <= self.input_sequence or type(size) is not int or not 4 <= size <= SLOT_BYTES:
            raise ValueError('invalid input sequence/size')
        # Copy before acknowledgement; the writer cannot reuse the slot earlier.
        metadata, blobs = decode_packet(self.memory[:size])
        self.input_sequence = seq
        self.send(dict(kind='input_ack', sequence=seq))
        if metadata['schema'] != 'wla-follow-input/v1':
            raise ValueError('invalid input schema')
        if metadata['generation'] < self.generation:
            return
        self.tf.update(metadata['transforms'], blobs)
        e = self.engine
        with e.guard:
            self.generation = metadata['generation']
            if e.motion.generation != metadata['motion_generation']:
                e.motion.samples.clear()
                e.motion.generation = metadata['motion_generation']
                e.result = None
            for stamp, pose, twist in metadata['motion']:
                if not e.motion.samples or stamp > e.motion.samples[-1][0]:
                    e.motion.samples.append((stamp, tuple(pose), tuple(twist)))
            e.wheel_received = metadata['wheel_received']
            diagnostic = metadata['wheel_diagnostic']
            if diagnostic:
                stamp = diagnostic['stamp_ns']
                p, q, v = diagnostic['position'], diagnostic['orientation'], diagnostic['twist']
                msg = NS(header=NS(stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9), frame_id=diagnostic['frame_id']),
                         child_frame_id=diagnostic['child_frame_id'],
                         pose=NS(pose=NS(position=NS(x=p[0], y=p[1], z=p[2]), orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))),
                         twist=NS(twist=NS(linear=NS(x=v[0], y=v[1]), angular=NS(z=v[2]))))
                e.wheel_diagnostic = (msg, diagnostic['received'], diagnostic['accepted'], e.motion.generation)
            for topic, data in metadata['events'].items():
                if topic in self.callbacks:
                    self.callbacks[topic][1](String(data=json.dumps(data, separators=(',', ':'))))
            if 'info' in blobs:
                e.on_info(deserialize_message(blobs['info'], CameraInfo))
            for topic, key in metadata['tof_topics']:
                if topic in self.callbacks:
                    self.callbacks[topic][1](deserialize_message(blobs[key], PointCloud2))
            if 'depth' in blobs:
                # Still serialized until the engine consumes this selected frame.
                e.on_depth(blobs['depth'])

    def receive(self, message):
        kind = message['kind']
        if kind == 'input':
            self.input(message)
        elif kind == 'barrier':
            with self.engine.guard:
                self.generation = message['generation']
                self.engine.result = None
                # C++ also acknowledges a terminal state produced by Python.
                # Do not turn an already completed arrival into cancellation.
                if message['cancel'] and self.engine.core.active:
                    self.engine.core._terminate('cancelled', message['reason'])
                self.guard_ready.set()
        elif kind == 'estop':
            with self.engine.guard:
                self.generation = message['generation']
                self.engine.on_estop(SetBool.Request(data=message['data']), SetBool.Response())
        elif kind == 'rpc':
            with self.engine.guard:
                self.generation = message['generation']
                req = PersonCommand.Request(**{k: message[k] for k in ('command', 'request_id', 'mode', 'target_id', 'lease_duration_s')})
                response = self.engine.on_command(req, PersonCommand.Response())
                if message['id']:
                    self.send(dict(kind='rpc_reply', id=message['id'], response={k: getattr(response, k) for k in
                        ('accepted', 'reason', 'request_id', 'state', 'mode', 'target_id', 'enabled')}))
                self.guard_ready.set()
        elif kind == 'geometry_ack':
            if message['sequence'] != self.geometry_sequence:
                raise ValueError('geometry acknowledgement mismatch')
            self.geometry_busy = False
        else:
            raise ValueError('unknown C++ transport message')

    def run(self):
        self.engine = FollowEngine(self)
        self.engine.follow_session = self.session
        self.send(dict(kind='ready'))
        while self.running:
            if select.select([self.sock], [], [], .005)[0]:
                self.receive(receive_json(self.sock))
            now = time.monotonic()
            for timer in self.timers:
                if now >= timer[0]:
                    timer[0] = now + timer[1]
                    timer[2]()
            if self.guard_ready.is_set():
                self.guard_ready.clear()
                self.guard_callback()
            self.flush_geometry()


def receive_json(sock):
    data, _, flags, _ = sock.recvmsg(128 * 1024)
    if not data or flags & socket.MSG_TRUNC:
        raise ConnectionError('closed or truncated follow IPC message')
    return json.loads(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--socket', required=True)
    parser.add_argument('--memory', required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sock.settimeout(.2)
    sock.connect(args.socket)
    fd = os.open(args.memory, os.O_RDWR | os.O_NOFOLLOW)
    if os.fstat(fd).st_size != 2 * SLOT_BYTES or os.fstat(fd).st_uid != os.getuid():
        raise ValueError('invalid shared frame file')
    memory = mmap.mmap(fd, 0)
    transport = None
    try:
        hello = receive_json(sock)
        if hello.get('kind') != 'hello':
            raise ValueError('missing C++ handshake')
        transport = WorkerTransport(sock, memory, hello)
        def stop(_signum, _frame):
            transport.running = False
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        transport.run()
    finally:
        if transport and transport.engine:
            transport.running = False
            transport.engine.close()
        memory.close()
        os.close(fd)
        sock.close()


if __name__ == '__main__':
    main()
