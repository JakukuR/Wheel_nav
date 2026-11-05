"""Mode 0 local planner backed by unmodified neupan_cpp in its own process (R_LA).

Called the way neupan_node calls the library: straight reference to the following
distance, stop on arrive/collision threshold/more than five unsolved plans. A dead or
late process raises, so the output gate stops the robot; it never falls back to the MPC.
"""
import math
import os
import select
import subprocess

import numpy as np

DEFAULT_DIRECTORY = os.path.expanduser('~/ros2_ws/.runtime/neupan')
TRACKED_STATES = ('tracking', 'coasting', 'holding')
ARRIVE_M = .1  # ipath.arrive_threshold
LOOKAHEAD_S = .12  # neupan_node `lookahead`: command delay
MAX_UNSOLVED = 5  # neupan_node `solver_fail_grace`
HOLD_GAIN, HOLD_MAX_W = 2., .9


class NeuPANFollower:
    def __init__(self, directory=DEFAULT_DIRECTORY, timeout_s=.1, command=None):
        self.timeout_s = timeout_s
        command = command or [os.path.join(directory, 'neupan_bridge'), os.path.join(directory, 'r_la.yaml'),
                              os.path.join(directory, 'car_50x40.bin')]
        environment = dict(os.environ, LD_LIBRARY_PATH=os.path.join(directory, 'lib'))
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0, env=environment)
        self.buffer = b''
        if not self._line(5.).startswith('READY'):
            raise RuntimeError('neupan_bridge_not_ready')
        self.reset()

    def reset(self):
        self.reset_next = True
        self.unsolved = 0
        self.last = np.zeros(2)
        self.diagnostic = None

    def _line(self, timeout_s):
        while b'\n' not in self.buffer:
            if not select.select([self.process.stdout], [], [], timeout_s)[0]:
                raise RuntimeError('neupan_bridge_timeout')
            data = os.read(self.process.stdout.fileno(), 4096)
            if not data:
                raise RuntimeError('neupan_bridge_exited')
            self.buffer += data
        line, self.buffer = self.buffer.split(b'\n', 1)
        return line.decode()

    def plan(self, pose, target, obstacles, gap):
        """pose (x, y, yaw), target (x, y), obstacles (n, 2) in the same frame. Returns (v, w)."""
        pose, target = np.asarray(pose, dtype=float), np.asarray(target, dtype=float)
        delta = target-pose[:2]
        distance = float(np.linalg.norm(delta))
        if distance <= gap+ARRIVE_M:
            # At the following distance there is nothing to plan: face the person.
            error = (math.atan2(delta[1], delta[0])-pose[2]+math.pi) % (2*math.pi)-math.pi
            self.last = np.array([0., float(np.clip(HOLD_GAIN*error, -HOLD_MAX_W, HOLD_MAX_W))])
            self.diagnostic = dict(active='hold')
            return self.last.copy()
        end = pose[:2]+delta*(distance-gap)/distance
        v, w = self.last
        start = (pose[0]+v*math.cos(pose[2])*LOOKAHEAD_S, pose[1]+v*math.sin(pose[2])*LOOKAHEAD_S, pose[2]+w*LOOKAHEAD_S)
        message = f'{start[0]} {start[1]} {start[2]} {int(self.reset_next)} 2 {len(obstacles)}\n'
        message += f'{pose[0]} {pose[1]} {end[0]} {end[1]}\n'
        message += ' '.join(f'{float(a)} {float(b)}' for a, b in obstacles)+'\n'
        try:
            self.process.stdin.write(message.encode())
        except BrokenPipeError:
            raise RuntimeError('neupan_bridge_exited') from None
        fields = self._line(self.timeout_s).split()
        if len(fields) != 8 or fields[0] != 'RESULT':
            raise RuntimeError('neupan_bridge_bad_reply')
        self.reset_next = False
        solved, stop, arrive = fields[1] == '1', fields[2] == '1', fields[3] == '1'
        command = np.array([float(fields[6]), float(fields[7])])
        self.unsolved = 0 if solved or arrive else self.unsolved+1
        if arrive or stop or self.unsolved > MAX_UNSOLVED:
            command = np.zeros(2)
        if not np.isfinite(command).all():
            raise ValueError('nonfinite_neupan_command')
        self.last = command
        minimum = float(fields[4])
        self.diagnostic = dict(active='plan', solved=solved, stop=stop, arrive=arrive, unsolved=self.unsolved,
                               min_distance_m=round(minimum, 3) if math.isfinite(minimum) else None, solve_ms=round(float(fields[5]), 2))
        return command.copy()

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(1.)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
