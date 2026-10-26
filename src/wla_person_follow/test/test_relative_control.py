import unittest

from wla_person_follow.task_session import TaskSession

def batch(stamp=10., session='camera', epoch=0, enabled=True, distance=2., yaw=0.,
          label='person', identifier=7, objects=None):
    return dict(schema='wla-relative-detections/v1', stamp_ns=round(stamp*1e9),
        session=session, epoch=epoch, enabled=enabled, base_frame='r680_mapping_floor',
        objects=([dict(detected=True, label=label, id=f'{session}:{epoch}:{identifier}',
                       yaw=yaw, distance=distance)] if objects is None else objects))

class TaskSessionTests(unittest.TestCase):
    def setUp(self):
        self.c = TaskSession()

    def receive(self, stamp=10., now=10., **kwargs):
        return self.c.receive(batch(stamp, **kwargs), now, round(now*1e9))

    def start(self, mode='0', request='one', target='camera:0:7', now=10., lease=1.):
        return self.c.command('start', request, mode, target, lease, now)

    def test_session_restart_retires_task_and_old_session(self):
        self.receive()
        self.start()
        self.receive(10.1, 10.1, session='new')
        self.assertFalse(self.c.active)
        self.assertEqual(self.c.reason, 'detector_generation_changed')
        self.assertFalse(self.receive(10.2, 10.2))
        self.assertFalse(self.start(request='two', now=10.2)[0])

    def test_epoch_restart_cannot_reuse_previous_id(self):
        self.receive()
        self.start()
        self.receive(10.1, 10.1, epoch=1)
        self.assertFalse(self.c.active)
        self.assertFalse(self.start(request='two', now=10.1)[0])
        self.assertTrue(self.start(request='two', target='camera:1:7', now=10.1)[0])

    def test_gate_disable_cancels_and_old_frame_cannot_reenable(self):
        self.receive()
        self.start()
        self.c.receive_status(dict(schema='wla-relative-perception-status/v1',
            stamp_ns=10_150_000_000, session='camera', epoch=0, enabled=False), 10.15, 10_150_000_000)
        self.assertFalse(self.c.active)
        self.assertFalse(self.receive(10.1, 10.16))
        self.assertFalse(self.c.gate_enabled)

    def test_bad_geometry_and_duplicate_ids_rejected(self):
        for bad in (float('nan'), float('inf'), -1., True):
            self.assertFalse(self.receive(distance=bad))
        obj = batch()['objects'][0]
        self.assertFalse(self.receive(objects=[obj, obj]))

    def test_non_person_target_rejected(self):
        self.receive(label='chair')
        self.assertEqual(self.start(), (False, 'target_id_not_visible'))

    def test_disable_after_completed_stop_does_not_relabel_as_fault(self):
        self.receive()
        self.start()
        self.c.command('stop',now=10.05)
        self.receive(10.1,10.1,enabled=False,objects=[])
        self.assertEqual(self.c.state,'stopped')

    def test_start_idempotent_does_not_renew_lease(self):
        self.receive()
        self.start()
        self.assertEqual(self.start(now=10.1),(True,'already_active'))
        self.assertEqual(self.c.lease_until,11.)
        self.assertEqual(self.start(mode='1'),(False,'request_id_conflict'))

    def test_named_stop_tombstones_late_start_without_stopping_current_task(self):
        self.receive()
        self.start()
        self.assertEqual(self.c.command('stop','in-flight',now=10.1),
                         (True,'request_cancelled_before_start'))
        self.assertTrue(self.c.active)
        self.assertEqual(self.c.request_id,'one')
        self.assertEqual(self.start(request='in-flight',now=10.1),
                         (False,'request_already_terminal'))

    def test_named_stop_before_any_start(self):
        self.assertEqual(self.c.command('stop','in-flight',now=10.),
                         (True,'request_cancelled_before_start'))
        self.receive()
        self.assertEqual(self.start(request='in-flight'),(False,'request_already_terminal'))

    def test_lease_limits(self):
        self.receive()
        for value in (0.,.09,2.01,float('nan'),True):
            self.assertEqual(self.start(lease=value),(False,'invalid_lease_duration'))

    def test_status_no_side_effect(self):
        self.receive()
        self.start()
        until = self.c.lease_until
        self.assertEqual(self.c.command('status',now=10.1),(True,'status'))
        self.assertEqual(self.c.lease_until,until)

    def test_stamp_object_compatibility_and_conflict(self):
        data=batch()
        data['stamp']=dict(sec=10,nanosec=0)
        del data['stamp_ns']
        self.assertTrue(self.c.receive(data,10.,10_000_000_000))
        data['stamp_ns']=10_100_000_000
        self.assertFalse(self.c.receive(data,10.1,10_100_000_000))

if __name__ == '__main__':
    unittest.main()
