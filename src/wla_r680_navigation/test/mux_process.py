"""The installed C++ command_mux as a test subprocess, seen only through its public interface.

Tests used to run the Python mux in-process and poke its policy object; that node
is gone. Reset is a process restart, estop is the real service, and state comes
from /r680_base/control_state (5 Hz, immediate on estop or mode changes).
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from std_msgs.msg import String
from std_srvs.srv import SetBool


class MuxProcess:
    def __init__(self, node, log_path='/tmp/test-command-mux.log'):
        self.node, self.log_path = node, log_path
        self.process, self.log, self.state = None, None, {}
        node.create_subscription(String, '/r680_base/control_state', self._on_state, 10)
        self.estop_client = node.create_client(SetBool, '/r680_base/set_estop')

    def _on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
        except ValueError:
            pass

    def start(self):
        executable = Path(get_package_prefix('wla_manual_control')) / 'lib/wla_manual_control/command_mux'
        self.log = open(self.log_path, 'a')
        self.state = {}
        self.process = subprocess.Popen([str(executable)], stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)

    def wait_ready(self, spin, timeout=5.):
        end = time.monotonic() + timeout
        while not self.state and time.monotonic() < end:
            spin()
        assert self.state, 'command_mux published no control_state; see ' + self.log_path

    def latch_estop(self):
        self.estop_client.call_async(SetBool.Request(data=True))

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        if self.log is not None:
            self.log.close()
        self.process, self.log = None, None

    def restart(self, spin=None):
        """A new process means a new control_session: the authority change sessions must notice."""
        self.stop()
        self.start()
        if spin is not None:
            self.wait_ready(spin)
