import json
from pathlib import Path
import subprocess
import yaml
from wla_r680_navigation_py.storage import load_storage_config

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/prepare_mapping_run'


def test_cuvslam_run_and_relative_storage(tmp_path):
    cfg=tmp_path/'storage.yaml'
    cfg.write_text(yaml.safe_dump(dict(run_root='runs',maps_root='maps',map_prefix='map',active_map='latest')))
    completed=subprocess.run([str(SCRIPT),'--storage-config',str(cfg),'--run-id','new-cuvslam',
                              '--odom-source','cuvslam','--imu-mode','chassis-vio','--auto-vio-init'],
                             text=True,capture_output=True,check=True)
    root=Path(completed.stdout.strip()); run=json.loads((root/'run.json').read_text())
    assert run['odom_source']=='cuvslam' and run['auto_vio_init']
    assert run['imu_topic']=='/wheel/imu/data_raw' and not run['hardware_output_enabled']
    assert run['rtabmap_odom_topic']=='/r680_nav/mapping_odom'
    assert load_storage_config(root/'storage.yaml')['maps_root']==tmp_path/'maps'
    assert (root/'source_storage.yaml').read_text()==cfg.read_text()
    assert all((root/n).is_file() for n in ('cuvslam.yaml','d455_cuvslam.yaml','vio_initializer.yaml'))


def test_original_mapping_defaults_and_invalid_combination(tmp_path):
    completed=subprocess.run([str(SCRIPT),'--root',str(tmp_path),'--run-id','old-rgbd'],
                             text=True,capture_output=True,check=True)
    run=json.loads((Path(completed.stdout.strip())/'run.json').read_text())
    assert run['odom_source']=='rgbd' and not run['auto_vio_init']
    assert run['rtabmap_odom_topic']=='/d455_slam/odom' and run['imu_topic']=='/r680_nav/chassis/imu_filtered'
    rejected=subprocess.run([str(SCRIPT),'--auto-vio-init','--root',str(tmp_path)],capture_output=True)
    assert rejected.returncode==2
