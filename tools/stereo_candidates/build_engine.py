"""Bounded offline FP16 TensorRT build. Does not start cameras or alter power settings."""
import argparse,json,subprocess,time,signal
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--onnx',type=Path,required=True);p.add_argument('--engine',type=Path,required=True);p.add_argument('--plugin',type=Path);p.add_argument('--report',type=Path,required=True);p.add_argument('--trtexec',default='/usr/src/tensorrt/bin/trtexec');a=p.parse_args()
if not a.onnx.is_file():p.error('ONNX does not exist')
if a.engine.exists():p.error('Output already exists; choose a new file to preserve the previous engine')
if a.plugin and not a.plugin.is_file():p.error('Plugin does not exist')
def temperatures():return [int(Path(f'/sys/class/thermal/thermal_zone{i}/temp').read_text())/1000 for i in (0,1)]
start=temperatures()
# Observed throttling is ~99C. These are explicit test budgets, not sensor accuracy limits.
if max(start)>90:p.error(f'Cooling required before build: {start}')
a.engine.parent.mkdir(parents=True,exist_ok=True);a.report.parent.mkdir(parents=True,exist_ok=True)
cmd=[a.trtexec,'--onnx='+str(a.onnx.resolve()),'--saveEngine='+str(a.engine.resolve()),'--fp16','--builderOptimizationLevel=0','--memPoolSize=workspace:1024M','--profilingVerbosity=detailed','--skipInference']
if a.plugin:cmd+=['--staticPlugins='+str(a.plugin.resolve())]
rows=[];reason='completed';started=time.monotonic()
with a.report.with_suffix('.log').open('w') as log:
 child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  while child.poll() is None:
   available=int(next(s.split()[1] for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:')))
   row={'seconds':time.monotonic()-started,'temperature_c':temperatures(),'available_memory_kib':available};rows.append(row)
   if max(row['temperature_c'])>=97 or available<1500000 or row['seconds']>600:
    reason='temperature_abort' if max(row['temperature_c'])>=97 else 'memory_abort' if available<1500000 else 'build_timeout';break
   time.sleep(1)
 finally:
  if child.poll() is None:
   child.send_signal(signal.SIGINT)
   try:child.wait(timeout=5)
   except subprocess.TimeoutExpired:child.kill();child.wait()
ok=reason=='completed' and child.returncode==0 and a.engine.is_file() and a.engine.stat().st_size>0
j={'command':cmd,'success':ok,'return_code':child.returncode,'stop_reason':reason,'start_temperature_c':start,'engine_bytes':a.engine.stat().st_size if a.engine.exists() else 0,'samples':rows};a.report.write_text(json.dumps(j,indent=2)+'\n');print(json.dumps({k:v for k,v in j.items() if k!='samples'}))
if not ok and a.engine.exists() and a.engine.stat().st_size==0:a.engine.unlink()
raise SystemExit(0 if ok else 1)
