from pathlib import Path
import sys,json
import argparse
p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--timm-dir',type=Path,required=True);a=p.parse_args()
repo=a.repo.resolve();a.output.parent.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(repo));sys.path.insert(0,str(a.timm_dir.resolve()))
import torch,timm,yaml
from easydict import EasyDict
original=timm.create_model
def create_model(*args,**kwargs):
 kwargs['pretrained']=False
 return original(*args,**kwargs)
timm.create_model=create_model
import types
for name in ('stereo','stereo.modeling','stereo.modeling.models','stereo.modeling.models.lightstereo','stereo.modeling.common','stereo.modeling.cost_volume','stereo.modeling.disp_pred','stereo.modeling.disp_refinement'):
 path=repo/name.replace('.', '/')
 module=types.ModuleType(name);module.__path__=[str(path)];sys.modules[name]=module
from stereo.modeling.models.lightstereo.lightstereo import LightStereo
torch.set_num_threads(4)
cfg=EasyDict(yaml.safe_load((repo/'cfgs/lightstereo/lightstereo_s_sceneflow_general.yaml').read_text())['MODEL'])
model=LightStereo(cfg).eval()
checkpoint=torch.load(a.checkpoint,map_location='cpu',weights_only=False)
print('checkpoint keys',list(checkpoint)[:10],flush=True)
state=checkpoint
for key in ('model_state','state_dict','model'):
 if isinstance(checkpoint,dict) and key in checkpoint:state=checkpoint[key];break
if all(k.startswith('module.') for k in state):state={k[7:]:v for k,v in state.items()}
model.load_state_dict(state,strict=True)
class Wrapper(torch.nn.Module):
 def __init__(self,m):super().__init__();self.model=m
 def forward(self,left,right):return self.model({'left':left,'right':right})['disp_pred']
wrapper=Wrapper(model).eval();inputs=(torch.randn(1,3,384,640),torch.randn(1,3,384,640))
with torch.no_grad():
 reference=wrapper(*inputs)
 torch.onnx.export(wrapper,inputs,a.output,input_names=['left','right'],output_names=['disparity'],opset_version=18,dynamo=False,do_constant_folding=True)
import onnx,onnxsim,numpy as np,onnxruntime as ort
m=onnx.load(a.output);m,ok=onnxsim.simplify(m)
if not ok:raise RuntimeError('onnx_simplification_validation_failed')
onnx.save(m,a.output)
session=ort.InferenceSession(str(a.output),providers=['CPUExecutionProvider'])
pred=session.run(None,{'left':inputs[0].numpy(),'right':inputs[1].numpy()})[0]
j={'parameters':sum(p.numel() for p in model.parameters()),'max_abs_export_difference_px':float(np.max(np.abs(pred-reference.numpy()))),'mean_abs_export_difference_px':float(np.mean(np.abs(pred-reference.numpy())))}
(a.output.with_suffix('.validation.json')).write_text(json.dumps(j,indent=2)+'\n');print(j,flush=True)
if j['max_abs_export_difference_px']>.01:raise RuntimeError('onnx_reference_difference_too_large')
