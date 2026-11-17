"""Offline TensorRT build with explicit profile and observed-temperature abort."""
import subprocess,time,json,signal
from pathlib import Path
root=Path('/home/onana/ros2_ws')
model=root/'models/depth_comparison/c_fast_foundationstereo_dynamic.onnx'
engine=root/'models/depth_comparison/c_fast_stereo_384x640_fp32.engine'
log=root/'.runtime/c_fast_stereo_build_retry.log'
report=root/'.runtime/c_fast_stereo_build_retry.json'
def temps():
 return [int(Path(f'/sys/class/thermal/thermal_zone{i}/temp').read_text())/1000 for i in (0,1)]
# 90C start, 97C abort are conservative test budgets, below observed 99C throttling.
start=temps()
if max(start)>90: raise SystemExit(f'cooling_not_ready:{start}')
cmd=['/usr/src/tensorrt/bin/trtexec',f'--onnx={model}',f'--saveEngine={engine}',
 '--optShapes=left_image:1x3x384x640,right_image:1x3x384x640',
 '--builderOptimizationLevel=1','--memPoolSize=workspace:4096M','--skipInference']
rows=[];reason='completed';started=time.monotonic()
with log.open('w') as output:
 p=subprocess.Popen(cmd,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  while p.poll() is None:
   row={'seconds':time.monotonic()-started,'temperature_c':temps()}; rows.append(row)
   row['available_memory_kib']=int(next(s.split()[1] for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:')))
   if max(row['temperature_c'])>=97 or row['seconds']>600 or row['available_memory_kib']<1500000:
    reason='temperature_abort' if max(row['temperature_c'])>=97 else 'memory_abort' if row['available_memory_kib']<1500000 else 'build_timeout'
    p.send_signal(signal.SIGINT)
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:p.kill();p.wait()
    break
   time.sleep(1)
 finally:
  if p.poll() is None:p.terminate();p.wait()
result={'command':cmd,'start_temperature_c':start,'stop_reason':reason,'return_code':p.returncode,'samples':rows}
report.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='samples'}))
raise SystemExit(p.returncode or (1 if reason!='completed' else 0))
