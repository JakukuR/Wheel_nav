"""Bounded GPU-only test cap; restores original sysfs limits on exit. No CPU/motor changes."""
from pathlib import Path
import subprocess,time,signal,json
base=Path('/sys/devices/platform/17000000.gpu/devfreq/17000000.gpu')
low=base/'min_freq';high=base/'max_freq';saved=[low.read_text(),high.read_text()]
Path('/tmp/wla_depth_gpu_limits_before.json').write_text(json.dumps(saved))
def stop(*_):raise SystemExit(0)
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
try:
 low.write_text('306000000\n');high.write_text('612000000\n')
 for _ in range(900):
  print(time.time(),low.read_text().strip(),high.read_text().strip(),(base/'cur_freq').read_text().strip(),flush=True);time.sleep(1)
finally:
 high.write_text(saved[1]);low.write_text(saved[0])
