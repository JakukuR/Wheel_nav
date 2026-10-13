#!/usr/bin/env python3
"""Validate a complete frontend configuration before any runtime side effects."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml


def inspect(folder, hardware=False):
    folder=Path(folder).resolve(strict=True)
    metadata=json.loads((folder/'calibration.json').read_text())
    commissioning=metadata.get('kind')=='commissioning'
    if commissioning:
        if metadata.get('calibration_validated') is not False:
            raise ValueError('Commissioning metadata must remain unvalidated')
    elif metadata.get('kind')!='calibrated' or metadata.get('calibration_validated') is not True:
        raise ValueError('A calibrated or explicitly commissioning configuration is required')
    if hardware:
        raise ValueError('OpenVINS hardware output is not yet accepted: omit --enable-motion; candidate calibration/contact/physical acceptance is pending')
    for name in ('frontend.yaml','estimator_config.yaml','kalibr_imu_chain.yaml','kalibr_imucam_chain.yaml'):
        data=(folder/name).read_bytes()
        expected=metadata.get('config_sha256',{}).get(name)
        if expected and hashlib.sha256(data).hexdigest()!=expected:
            raise ValueError(f'Configuration changed without provenance update: {name}')
    params=yaml.safe_load((folder/'frontend.yaml').read_text())['/**']['ros__parameters']
    topic=params['imu_topic'];frame=params['imu_frame']
    expected={'/r680/d455/imu':'d455_imu_optical_frame','/wheel/imu/data_raw':'gyro_link'}
    if topic not in expected or frame!=expected[topic]:
        raise ValueError(f'Unsupported or inconsistent IMU topic/frame: {topic} {frame}')
    imu=yaml.safe_load('\n'.join(line for line in (folder/'kalibr_imu_chain.yaml').read_text().splitlines() if not line.startswith('%YAML')))['imu0']
    if imu['rostopic']!=topic:raise ValueError('Frontend IMU topic differs from noise configuration')
    return commissioning,topic=='/r680/d455/imu'


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder')
    parser.add_argument('--hardware-output',choices=['true','false'],default='false')
    args=parser.parse_args()
    try:
        print(*('true' if x else 'false' for x in inspect(args.folder,args.hardware_output=='true')))
    except (OSError,ValueError,KeyError) as error:parser.exit(2,f'{error}\n')
