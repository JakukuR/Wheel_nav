"""Read startup completion from the lifecycle owner, not the static-map manager."""
import os
from pathlib import Path

# Readiness has no localization deadline, so a dead safety/localization node must
# end the wait. Launch names a process "<executable>-<n>"; respawned frontends are absent.
CRITICAL_PROCESSES = ('rtabmap', 'vo_watchdog', 'interface_monitor', 'command_guard',
                      'vio_initializer', 'lifecycle_manager')


class StartupEvidence:
    def __init__(self, path, odom_source, bringup_pid=None):
        self.path = Path(path)
        self.odom_source = odom_source
        self.bringup_pid = bringup_pid
        self.offset = 0
        self.tail = ''
        self.completed = False
        self.failure = None

    def read(self):
        with self.path.open(errors='replace') as stream:
            stream.seek(self.offset)
            lines = (self.tail + stream.read()).splitlines(keepends=True)
            self.offset = stream.tell()
        self.tail = ''
        for line in lines:
            if not line.endswith(('\n', '\r')):
                self.tail = line
                continue
            if any(marker in line for marker in (
                    'Failed to bring up all requested nodes',
                    'OpenVINS Nav2 startup failed:',
                    'malformed launch argument',
                    '[ERROR] [launch]: Caught exception in launch')):
                self.failure = '导航启动失败；查看本次 logs/bringup.log: ' + line.strip()
            if 'process has died' in line and any(f'[{name}-' in line for name in CRITICAL_PROCESSES):
                self.failure = '关键节点已退出；查看本次 logs/bringup.log: ' + line.strip()[:300]
            if self.odom_source == 'openvins':
                if '[openvins_nav_startup]' in line and 'OpenVINS Nav2 startup completed;' in line:
                    self.completed = True
            elif '[lifecycle_manager_navigation]' in line and 'Managed nodes are active' in line:
                self.completed = True
        if self.bringup_pid is not None and not self._bringup_alive():
            # Even a silent exit (or an exit after a success line) must fail closed.
            detail = self.path.read_text(errors='replace')[-2000:].strip()
            self.failure = (self.failure or
                f'导航主进程已退出 (pid={self.bringup_pid})；查看 {self.path}\n{detail}')
        if self.failure:
            raise SystemExit(self.failure)
        return self.completed

    def _bringup_alive(self):
        try:
            os.kill(self.bringup_pid, 0)
            # kill(pid, 0) also succeeds for a zombie whose parent has not reaped it.
            stat = Path(f'/proc/{self.bringup_pid}/stat').read_text()
            return stat.rpartition(')')[2].split()[0] not in ('Z', 'X')
        except (ProcessLookupError, FileNotFoundError):
            return False
