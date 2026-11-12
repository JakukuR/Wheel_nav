#!/usr/bin/env python3
"""Extract CAD sensor mounts; no ROS publication or optical/IMU calibration."""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def quaternion(rpy):
    r, p, y = (v / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.source
    csv_path = source.with_suffix('.csv')
    root = ET.parse(source).getroot()
    rows = {row['Link Name']: row for row in csv.DictReader(io.StringIO(csv_path.read_text(encoding='utf-8-sig')))}
    links = {x.get('name') for x in root.findall('link')}
    joints = root.findall('joint')
    children = [j.find('child').get('link') for j in joints]
    assert len(children) == len(set(children)), 'multiple parents'
    assert links-set(children) == {'base_link'}, 'unexpected root'
    assert len(joints) == len(links)-1, 'disconnected graph'
    parents = {j.find('child').get('link'): j.find('parent').get('link') for j in joints}
    for link in links:
        seen = set()
        while link in parents:
            assert link not in seen, 'cycle'
            seen.add(link)
            link = parents[link]
        assert link == 'base_link', 'unconnected link'
    assert rows['base_link']['Coordinate System'] == 'base_point'
    names = ['upper_camera_R', 'upper_camera_L', 'bottom_camera_R', 'bottom_camera_L',
             'front_tof_R', 'front_tof_L', 'back_tof_R', 'back_tof_L']
    data = {
        'schema_version': 1,
        'purpose': 'CAD mount geometry only; not runtime optical/IMU calibration',
        'source_urdf_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'source_csv_sha256': hashlib.sha256(csv_path.read_bytes()).hexdigest(),
        'parent_frame': 'base_link',
        'cad_root_coordinate_system': 'base_point',
        'imu_location': 'base_point; user confirmed location between and below four ToFs',
        'imu_raw_measurement_axes_verified': False,
        'runtime_tf_enabled': False,
        'device_to_mount_mapping': {
            'DICOTA_4K_RGB': 'upper_camera_R',
            'Sunplus_stereo_left': 'bottom_camera_L',
            'Sunplus_stereo_right': 'bottom_camera_R',
        },
        'device_to_mount_mapping_source': 'user confirmed 2026-11-12',
        'sunplus_image_half_order_physically_verified': False,
        'transform_convention': 'p_base = R_base_sensor @ p_sensor + t_base_sensor; R=Rz(yaw)Ry(pitch)Rx(roll)',
        'units': {'translation': 'm', 'rpy': 'rad', 'quaternion_order': 'xyzw'},
        'sensors': {},
    }
    out_root = ET.Element('robot', name='new_orin_sensor_mounts_cad')
    out_root.append(ET.Comment('CAD mounts only. No optical frames, no IMU raw-axis transform. upper_camera_L absent on current robot.'))
    ET.SubElement(out_root, 'link', name='base_link')
    for name in names:
        matches = [j for j in joints if j.find('child').get('link') == name]
        assert len(matches) == 1
        joint = matches[0]
        assert joint.get('type') == 'fixed'
        assert joint.find('parent').get('link') == 'base_link'
        origin = joint.find('origin')
        xyz = list(map(float, origin.get('xyz').split()))
        rpy = list(map(float, origin.get('rpy').split()))
        assert all(math.isfinite(v) for v in xyz+rpy)
        for value, col in zip(xyz+rpy, ['Joint Origin X', 'Joint Origin Y', 'Joint Origin Z',
                                      'Joint Origin Roll', 'Joint Origin Pitch', 'Joint Origin Yaw']):
            assert math.isclose(value, float(rows[name][col]), abs_tol=1e-12), (name, col)
        q = quaternion(rpy)
        assert abs(sum(v*v for v in q)-1) < 1e-12
        active = name != 'upper_camera_L'
        data['sensors'][name] = {'installed': active, 'parent': 'base_link', 'xyz_m': xyz,
                                  'rpy_rad': rpy, 'quaternion_xyzw': q}
        if active:
            ET.SubElement(out_root, 'link', name=name)
            dst = ET.SubElement(out_root, 'joint', name=name+'_mount', type='fixed')
            ET.SubElement(dst, 'parent', link='base_link')
            ET.SubElement(dst, 'child', link=name)
            ET.SubElement(dst, 'origin', xyz=origin.get('xyz'), rpy=origin.get('rpy'))
    invalid_limits = []
    for j in joints:
        limit = j.find('limit')
        if limit is not None and float(limit.get('lower', '0')) > float(limit.get('upper', '0')):
            invalid_limits.append(j.get('name'))
    data['source_full_robot_limit_lower_gt_upper'] = invalid_limits
    left, right = [data['sensors'][f'bottom_camera_{s}']['xyz_m'] for s in ['L', 'R']]
    data['bottom_mount_separation_m'] = math.dist(left, right)
    assert math.isclose(data['bottom_mount_separation_m'], .037, abs_tol=1e-12)
    out = args.output
    cad = out/'config/uvc_rig/cad_20261112'
    cad.mkdir(parents=True, exist_ok=True)
    for src, dst in [(source, cad/'source_robot.urdf'), (csv_path, cad/'source_robot.csv')]:
        if src.resolve() != dst.resolve():
            shutil.copyfile(src, dst)
    (cad/'sensor_mounts.cad.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    ET.indent(out_root)
    ET.ElementTree(out_root).write(cad/'sensor_mounts.urdf', encoding='utf-8', xml_declaration=True)
    parsed = ET.parse(cad/'sensor_mounts.urdf').getroot()
    assert len(parsed.findall('link')) == 8 and len(parsed.findall('joint')) == 7
    assert all(j.get('type') == 'fixed' for j in parsed.findall('joint'))
    print(json.dumps({'source_links': len(links), 'source_joints': len(joints),
                      'cad_root': rows['base_link']['Coordinate System'],
                      'exported_active_sensors': 7, 'inactive': ['upper_camera_L'],
                      'bottom_mount_separation_m': data['bottom_mount_separation_m'],
                      'invalid_source_joint_limits': invalid_limits,
                      'source_urdf_sha256': data['source_urdf_sha256']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
