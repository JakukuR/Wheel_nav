"""Bounded temporary full fan, restoring nvfancontrol automatically. No configuration edits."""
import subprocess,time,signal
from pathlib import Path

def stop(*_):
    raise SystemExit(0)
for sig in (signal.SIGTERM,signal.SIGINT,signal.SIGHUP): signal.signal(sig,stop)
was_active=subprocess.run(['systemctl','is-active','--quiet','nvfancontrol']).returncode==0
try:
    subprocess.run(['systemctl','stop','nvfancontrol'],check=True)
    Path('/sys/class/hwmon/hwmon0/pwm1').write_text('255\n')
    for _ in range(900):
        Path("/sys/class/hwmon/hwmon0/pwm1").write_text("255\n")
        temps=[int(Path(f'/sys/class/thermal/thermal_zone{i}/temp').read_text())/1000 for i in (0,1)]
        rpm=Path('/sys/class/hwmon/hwmon3/rpm').read_text().strip()
        print(time.time(),temps,rpm,flush=True)
        time.sleep(1)
finally:
    if was_active: subprocess.run(['systemctl','start','nvfancontrol'],check=True)
