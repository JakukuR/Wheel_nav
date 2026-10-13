#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile
import json
import unittest
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('generator',ROOT/'scripts/generate_config.py')
generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)


class Calibration(unittest.TestCase):
    def test_commissioning_keeps_unvalidated_provenance(self):
        with tempfile.TemporaryDirectory() as d:
            m=yaml.safe_load((ROOT/'config/calibration_manifest.example.yaml').read_text())
            m.update(kind='commissioning',provenance='test only')
            path=Path(d)/'manifest.yaml';path.write_text(yaml.safe_dump(m))
            with self.assertRaises(ValueError):
                generator.generate(path,ROOT/'config/estimator_template.yaml',Path(d)/'rejected')
            out=generator.generate(path,ROOT/'config/estimator_template.yaml',Path(d)/'accepted',allow_commissioning=True)
            self.assertIs(json.loads((out/'calibration.json').read_text())['calibration_validated'],False)

    def test_rejects_unverified_and_implicit_synthetic(self):
        with tempfile.TemporaryDirectory() as d:
            for source in ('calibration_manifest.example.yaml','synthetic_manifest.yaml'):
                with self.assertRaises(ValueError):
                    generator.generate(ROOT/'config'/source,ROOT/'config/estimator_template.yaml',Path(d)/source)

    def test_transform_and_output(self):
        with tempfile.TemporaryDirectory() as d:
            out=generator.generate(ROOT/'config/synthetic_manifest.yaml',ROOT/'config/estimator_template.yaml',Path(d)/'out',True)
            cams=yaml.safe_load((out/'kalibr_imucam_chain.yaml').read_text().replace('%YAML:1.0',''))
            self.assertEqual(cams['cam0']['distortion_coeffs'],[0.,0.,0.,0.])
            self.assertAlmostEqual(cams['cam1']['T_imu_cam'][0][3],.095)
            r=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
            np.testing.assert_allclose(generator.quaternion(r),[0,0,2**-.5,2**-.5],atol=1e-12)
            invalid=np.eye(4);invalid[0,0]=2
            with self.assertRaises(ValueError):generator.transform(invalid,'invalid')


if __name__=='__main__':unittest.main()
