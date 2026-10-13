#!/usr/bin/env python3
"""Generate fixed stereo calibration. All transforms map child into parent."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml


class IndentedDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)


def transform(value, name):
    x = np.asarray(value, dtype=float)
    if (x.shape != (4, 4) or not np.isfinite(x).all() or
            not np.allclose(x[3], [0, 0, 0, 1], atol=1e-7) or
            not np.allclose(x[:3, :3].T @ x[:3, :3], np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(x[:3, :3]), 1, atol=1e-5)):
        raise ValueError(f'{name}: expected rigid 4x4 transform')
    return x


def quaternion(r):
    # Eigenvector formulation, xyzw, including rotations near pi.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                  [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                  [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], r.trace()]]) / 3
    q = np.linalg.eigh(k)[1][:, -1]
    return (q if q[3] >= 0 else -q).tolist()


def generate(manifest, template, output, allow_test=False, allow_commissioning=False):
    m = yaml.safe_load(Path(manifest).read_text())
    commissioning=m.get('kind')=='commissioning'
    if commissioning and (not allow_commissioning or m.get('calibration_validated') is not False):
        raise ValueError('commissioning requires explicit opt-in and calibration_validated: false')
    if not commissioning and m.get('calibration_validated') is not True:
        raise ValueError('calibration_validated must be true after measured calibration review')
    if m.get('kind') not in ('calibrated', 'synthetic', 'commissioning'):
        raise ValueError('kind must be calibrated, synthetic or commissioning')
    if m['kind'] == 'synthetic' and not allow_test:
        raise ValueError('synthetic calibration requires --allow-test-config')
    if not m.get('provenance'):
        raise ValueError('calibration provenance is required')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    t_ib = transform(m['T_imu_base'], 'T_imu_base')
    cams = {}
    shift = float(m['timeshift_cam_imu'])
    if not np.isfinite(shift) or abs(shift)>0.5:
        raise ValueError('invalid camera to IMU time offset')
    for i in range(2):
        c = m[f'cam{i}']
        intr = np.asarray(c['intrinsics'], dtype=float)
        resolution = np.asarray(c['resolution'])
        if (intr.shape != (4,) or not np.isfinite(intr).all() or (intr[:2]<=0).any() or
                resolution.shape != (2,) or (resolution<=0).any() or
                not np.equal(resolution, resolution.astype(int)).all()):
            raise ValueError('invalid intrinsics/resolution')
        if c.get('image_model') != 'rectified_pinhole':
            raise ValueError('v1 requires rectified pinhole intrinsics and rectified camera extrinsics')
        t_ic = transform(c['T_imu_camera_rectified'], f'cam{i} extrinsic')
        cams[f'cam{i}'] = dict(T_imu_cam=t_ic.tolist(), cam_overlaps=[1-i], camera_model='pinhole',
                              distortion_model='radtan', distortion_coeffs=[0.,0.,0.,0.],
                              intrinsics=intr.tolist(), resolution=resolution.astype(int).tolist(),
                              rostopic=c['topic'], timeshift_cam_imu=shift)
    if np.linalg.norm(np.array(cams['cam0']['T_imu_cam'])[:3,3]-np.array(cams['cam1']['T_imu_cam'])[:3,3]) < 0.001:
        raise ValueError('stereo baseline is missing')
    imu = dict(m['imu'])
    for key in ('accelerometer_noise_density','accelerometer_random_walk','gyroscope_noise_density',
                'gyroscope_random_walk','update_rate'):
        if not np.isfinite(float(imu[key])) or float(imu[key])<=0:
            raise ValueError(f'IMU {key} must come from a positive calibration value')
    frame = imu.pop('frame_id')
    imu.update(T_i_b=t_ib.tolist(), time_offset=0., model='kalibr', Tw=np.eye(3).tolist(),
               Ta=np.eye(3).tolist(), R_IMUtoGYRO=np.eye(3).tolist(), R_IMUtoACC=np.eye(3).tolist(),
               Tg=np.zeros((3,3)).tolist())
    content = Path(template).read_text()
    for filename, value in [('kalibr_imucam_chain.yaml', cams), ('kalibr_imu_chain.yaml', {'imu0':imu})]:
        (output/filename).write_text('%YAML:1.0\n---\n'+yaml.dump(value, Dumper=IndentedDumper,
            sort_keys=False, default_flow_style=None), newline='\n')
    (output/'estimator_config.yaml').write_text(content, newline='\n')
    params = dict(config_path=str(output/'estimator_config.yaml'), imu_topic=imu['rostopic'], imu_frame=frame,
                  left_topic=m['cam0']['topic'], right_topic=m['cam1']['topic'],
                  left_info_topic=m['cam0'].get('camera_info_topic','/r680/d455/infra1/camera_info'),
                  right_info_topic=m['cam1'].get('camera_info_topic','/r680/d455/infra2/camera_info'),
                  left_frame=m['cam0']['frame_id'], right_frame=m['cam1']['frame_id'],
                  left_info_frame=m['cam0'].get('camera_info_frame_id',m['cam0']['frame_id']),
                  right_info_frame=m['cam1'].get('camera_info_frame_id',m['cam1']['frame_id']),
                  p_base_in_imu=t_ib[:3,3].tolist(), q_base_to_imu_xyzw=quaternion(t_ib[:3,:3]))
    (output/'frontend.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False),newline='\n')
    (output/'calibration.json').write_text(json.dumps(dict(kind=m['kind'], calibration_validated=m['calibration_validated'],
        provenance=m['provenance'], source_sha256=hashlib.sha256(Path(manifest).read_bytes()).hexdigest()),indent=2),newline='\n')
    return output


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('manifest');p.add_argument('output');p.add_argument('--template')
    p.add_argument('--allow-test-config',action='store_true')
    p.add_argument('--allow-commissioning',action='store_true');a=p.parse_args()
    if not a.template:
        from ament_index_python.packages import get_package_share_directory
        a.template=str(Path(get_package_share_directory('wla_openvins_navigation'))/'config/estimator_template.yaml')
    print(generate(a.manifest,a.template,a.output,a.allow_test_config,a.allow_commissioning))
