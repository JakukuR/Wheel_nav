import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from velocity_policy import VelocityPolicy


def msg(source, t, v=.65, w=.9):
    return dict(schema='wla-'+source.lower()+'-candidate/v1',
                stamp_ns=round((100+t)*1e9), linear=v, angular=w)


class VelocityTests(unittest.TestCase):
    def make(self):
        return VelocityPolicy(1., .3, 1.5)

    def feed(self, p, source, t, v=.65, w=.9):
        return p.candidate(source,msg(source,t,v,w),t,round((100+t)*1e9))

    def test_person_without_status_mode_or_source(self):
        p=self.make()
        self.assertTrue(self.feed(p,'PERSON',1.))
        self.assertEqual(p.output(1.01),(.65,.9))
        self.assertEqual(p.output_source,'PERSON')
        self.assertEqual(p.automatic_source,'NAV')
        self.assertEqual(p.output(1.16),(0.,0.))

    def test_priority_zero_release_and_expiry(self):
        p=self.make();p.automatic_input((.2,0.),1.)
        self.feed(p,'OBJECT',1.,.1,0.);self.feed(p,'PERSON',1.,.4,0.)
        self.assertEqual(p.output(1.01),(.4,0.))
        self.feed(p,'PERSON',1.02,0.,0.)
        self.assertEqual(p.output(1.03),(.1,0.))
        self.feed(p,'OBJECT',1.04,0.,0.)
        self.assertEqual(p.output(1.05),(.2,0.))
        self.assertEqual(p.output(1.2),(0.,0.))

    def test_manual_release_requires_post_release_sample(self):
        p=self.make();p.gamepad_status_input('ACTIVE',1.)
        p.manual_input((.1,0.),1.01);self.feed(p,'PERSON',1.01)
        self.assertEqual(p.output(1.02),(.1,0.))
        p.manual_input((0.,0.),1.03)
        self.assertEqual(p.output(1.04),(0.,0.))
        self.feed(p,'PERSON',1.05)
        self.assertEqual(p.output(1.06),(.65,.9))
        self.assertEqual(p.person_revoke_generation,0)

    def test_estop_retained_and_no_cached_resume(self):
        p=self.make();self.feed(p,'PERSON',1.)
        p.set_estop(True,1.01)
        self.assertEqual(p.output(1.02),(0.,0.))
        self.feed(p,'PERSON',1.03)
        self.assertEqual(p.output(1.04),(0.,0.))
        p.wheel_at=1.05;p.wheel=(0.,0.)
        self.assertTrue(p.set_estop(False,1.05)[0])
        self.assertEqual(p.output(1.06),(0.,0.))
        self.feed(p,'PERSON',1.07)
        self.assertEqual(p.output(1.08),(.65,.9))

    def test_invalid_replay_and_source_time(self):
        p=self.make();self.feed(p,'PERSON',1.)
        self.assertFalse(p.candidate('PERSON',msg('PERSON',.5),1.1,round(101.1e9)))
        self.assertEqual(p.output(1.1),(0.,0.))
        self.assertFalse(p.candidate('PERSON',msg('PERSON',1.1,math.nan),1.1,round(101.1e9)))
        self.assertFalse(p.candidate('PERSON',msg('PERSON',1.1,True),1.1,round(101.1e9)))
        self.feed(p,'PERSON',1.2)
        self.assertFalse(p.candidate('PERSON',msg('PERSON',1.19,0.,0.),1.2,round(101.2e9)))
        self.assertEqual(p.output(1.21),(.65,.9))

    def test_limits_and_compatibility_services_do_not_gate(self):
        p=self.make();p.set_tracking_source('OBJECT',1.,round(101e9))
        self.feed(p,'PERSON',1.01,4.,-3.)
        self.assertEqual(p.output(1.02),(1.,-1.5))
        p.set_manual();self.assertEqual(p.output(1.03),(0.,0.))
        self.feed(p,'PERSON',1.04,-4.,0.)
        self.assertEqual(p.output(1.05),(-.3,0.))


if __name__=='__main__':unittest.main()
