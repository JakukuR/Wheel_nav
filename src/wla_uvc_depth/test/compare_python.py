"""Compare native OpenCV core and retained Python reference on recorded pixels."""
import argparse,json,subprocess,time
from pathlib import Path
import cv2,numpy as np
from wla_uvc_camera.stereo import StereoRig
p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);cv2.setNumThreads(1)
root=Path('/home/onana/calibration_data/uvc_joint_20261116_172823_174231');m=json.loads((root/'input_manifest.json').read_text());g=next(g for g in m['groups'] if g['session']==0 and g['slot']==22)
paths={e:Path('/home/onana/calibration_data')/m['source_sessions'][g['session']]/'official_inputs_1s'/r['file'] for e,r in g['frames'].items()}
rig=a.workspace/'config/uvc_rig/calibrated_20261116/rig.yaml';images={e:cv2.imread(str(f)) for e,f in paths.items()};results={}
for algorithm in ['sgbm','bm']:
 prefix=a.output/algorithm
 completed=subprocess.run([str(a.workspace/'build/wla_uvc_depth/compare_reference'),str(rig),str(paths['left']),str(paths['right']),str(paths['rgb']),str(prefix),algorithm],check=True,text=True,capture_output=True)
 native_version=completed.stdout.strip().split()[-1]
 native_depth=cv2.imread(str(prefix)+'_depth.png',cv2.IMREAD_UNCHANGED);native_color=cv2.imread(str(prefix)+'_color.png');file=cv2.FileStorage(str(prefix)+'_disparity.yaml',cv2.FILE_STORAGE_READ);native_disparity=file.getNode('disparity').mat();native_r1=file.getNode('R1').mat();native_q=file.getNode('Q').mat();file.release()
 reference=StereoRig(rig,algorithm=algorithm,rgb_mode='crop');start=time.monotonic();depth,disparity=reference.depth(images['left'],images['right']);color=reference.color(images['rgb']);elapsed=(time.monotonic()-start)*1000
 result={'disparity_max_abs_px':float(np.max(np.abs(native_disparity-disparity))),'color_max_abs':int(np.max(np.abs(native_color.astype(int)-color.astype(int)))),'depth_max_abs_mm':int(np.max(np.abs(native_depth.astype(int)-depth.astype(int)))),'depth_different_pixels':int(np.count_nonzero(native_depth!=depth)),'reference_compute_ms':elapsed,'valid_depth_fraction':float(np.count_nonzero(native_depth)/native_depth.size)}
 result['python_opencv_version']=cv2.__version__;result['native_opencv_version']=native_version;result['rectification']={}
 for eye in ['left','right']:
  rect=cv2.remap(images[eye],*getattr(reference,eye+'_maps'),cv2.INTER_LINEAR);rect=cv2.cvtColor(rect,cv2.COLOR_BGR2GRAY)
  native_rect=cv2.imread(str(prefix)+'_'+eye+'.png',cv2.IMREAD_GRAYSCALE)
  delta=np.abs(rect.astype(int)-native_rect.astype(int))
  result['rectification'][eye]={'different_pixels':int(np.count_nonzero(delta)),'max_gray_level_difference':int(delta.max()),'mean_gray_level_difference':float(delta.mean())}
  if native_version==cv2.__version__:assert np.array_equal(rect,native_rect),eye+' same-version rectification differs'
 class FrozenMatcher:
  def compute(self,left,right):return np.rint(native_disparity*16).astype(np.int16)
 result['r1_max_abs_difference']=float(np.max(np.abs(reference.r1-native_r1)));result['q_max_abs_difference']=float(np.max(np.abs(reference.q-native_q)))
 reference.matcher=FrozenMatcher();reference.r1=native_r1;reference.q=native_q
 frozen_depth,_=reference.depth(images['left'],images['right'])
 result['frozen_disparity_geometry_max_abs_mm']=int(np.max(np.abs(native_depth.astype(int)-frozen_depth.astype(int))))
 result['frozen_disparity_geometry_different_pixels']=int(np.count_nonzero(native_depth!=frozen_depth))
 assert result['frozen_disparity_geometry_max_abs_mm']==0,result
 assert result['color_max_abs']==0,result
 # The independent 2m-plane native test covers metric matching. Record raw 4.6/4.8 matcher differences rather than labeling them bit-identical.
 results[algorithm]=result
(a.output/'result.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
