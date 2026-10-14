import importlib.util
from pathlib import Path
from wla_openvins_navigation.msg import EstimatorStatus

p=Path(__file__).resolve().parents[1]/'scripts/quality_gate.py'
spec=importlib.util.spec_from_file_location('gate',p);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
g=module.Gate.__new__(module.Gate)
g.recent_status=lambda:True
s=EstimatorStatus();g.status=s
s.mode=s.STATIONARY;s.zupt_enabled=True;s.zupt_updates=10
s.zupt_update_age_s=.05;s.image_age_s=.05;s.uncertainty_acceptable=True
s.visual_update_age_s=-1;s.recovery_updates=0
assert g.stationary_ready() and g.local_ok() and g.local_estimate_ready()
assert not g.visual_ready(), 'ZUPT must not fabricate visual updates'
for field,value in [('zupt_update_age_s',.6),('image_age_s',.6),('zupt_updates',0),('uncertainty_acceptable',False),('zupt_enabled',False)]:
    old=getattr(s,field);setattr(s,field,value)
    assert not g.stationary_ready() and not g.local_ok(),field
    setattr(s,field,old)
g.recent_status=lambda:False
assert not g.local_estimate_ready()
print('PASS stationary local health, genuine visual evidence kept separate, stale/invalid rejection')
