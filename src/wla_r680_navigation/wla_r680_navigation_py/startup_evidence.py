"""Read startup completion from the lifecycle owner, not the static-map manager."""
from pathlib import Path


class StartupEvidence:
    def __init__(self, path, odom_source):
        self.path = Path(path)
        self.odom_source = odom_source
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
            if 'Failed to bring up all requested nodes' in line or 'OpenVINS Nav2 startup failed:' in line:
                self.failure = 'Nav2 启动失败；查看本次 logs/bringup.log: ' + line.strip()
            if self.odom_source == 'openvins':
                if '[openvins_nav_startup]' in line and 'OpenVINS Nav2 startup completed;' in line:
                    self.completed = True
            elif '[lifecycle_manager_navigation]' in line and 'Managed nodes are active' in line:
                self.completed = True
        if self.failure:
            raise SystemExit(self.failure)
        return self.completed
