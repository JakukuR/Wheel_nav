import math
import unittest

from wla_person_follow.relative_control import RelativeFollower, Safety


SAFE = Safety(True, math.inf, True)


def batch(stamp=10., session='camera', epoch=0, enabled=True, distance=2., yaw=0.,
          label='person', identifier=7, objects=None):
    return dict(schema='wla-relative-detections/v1', stamp_ns=round(stamp*1e9),
        session=session, epoch=epoch, enabled=enabled, base_frame='r680_mapping_floor',
        objects=([dict(detected=True, label=label, id=f'{session}:{epoch}:{identifier}',
                       yaw=yaw, distance=distance)] if objects is None else objects))


class RelativeControlTests(unittest.TestCase):
    def setUp(self):
        self.c = RelativeFollower()

    def receive(self, stamp=10., now=10., **kwargs):
        return self.c.receive(batch(stamp, **kwargs), now, round(now*1e9))

    def start(self, mode='follow', request='one', target='camera:0:7', now=10., lease=1.):
        return self.c.command('start', request, mode, target, lease, now)

    def test_no_input_zero_and_no_automatic_acquisition(self):
        self.assertEqual(self.c.tick(10., SAFE), (0., 0.))
        self.receive()
        self.assertEqual(self.c.tick(10., SAFE), (0., 0.))

    def test_relative_follow_requires_no_map_or_odometry(self):
        self.receive()
        self.assertTrue(self.start()[0])
        v, w = self.c.tick(10.05, SAFE)
        self.assertGreater(v, 0.)
        self.assertEqual(w, 0.)

    def test_source_age_consumes_ttl_at_receive(self):
        self.receive(stamp=10., now=10.25)
        self.assertTrue(self.start(now=10.25)[0])
        self.assertGreater(self.c.tick(10.29, SAFE)[0], 0.)
        self.assertEqual(self.c.tick(10.301, SAFE), (0., 0.))

    def test_source_ttl_cannot_be_refreshed_by_status(self):
        self.receive()
        self.start()
        self.c.receive_status(dict(schema='wla-relative-perception-status/v1',
            stamp_ns=10_290_000_000, session='camera', epoch=0, enabled=True), 10.29, 10_290_000_000)
        self.assertEqual(self.c.tick(10.31, SAFE), (0., 0.))

    def test_empty_frame_stops_without_predicting(self):
        self.receive()
        self.start()
        self.assertGreater(self.c.tick(10.05, SAFE)[0], 0.)
        self.receive(10.1, 10.1, objects=[])
        self.assertEqual(self.c.tick(10.1, SAFE), (0., 0.))
        self.assertEqual(self.c.reason, 'target_not_visible')

    def test_no_automatic_reidentification(self):
        self.receive()
        self.start()
        self.receive(10.1, 10.1, identifier=8)
        self.assertEqual(self.c.tick(10.1, SAFE), (0., 0.))
        self.assertEqual(self.c.target_id, 'camera:0:7')

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

    def test_invalid_geometry_stops_but_keeps_detection(self):
        self.receive()
        self.start()
        self.receive(10.1, 10.1, yaw=None, distance=None)
        self.assertEqual(self.c.tick(10.1, SAFE), (0., 0.))
        self.assertEqual(self.c.reason, 'target_geometry_unavailable')

    def test_bad_geometry_and_duplicate_ids_rejected(self):
        for bad in (float('nan'), float('inf'), -1., True):
            self.assertFalse(self.receive(distance=bad))
        obj = batch()['objects'][0]
        self.assertFalse(self.receive(objects=[obj, obj]))

    def test_non_person_target_rejected(self):
        self.receive(label='chair')
        self.assertEqual(self.start(), (False, 'person_id_not_visible'))

    def test_replayed_frame_does_not_refresh(self):
        self.receive()
        self.start()
        self.assertFalse(self.receive(stamp=10., now=10.1))
        self.assertEqual(self.c.tick(10.1, SAFE), (0., 0.))

    def test_large_yaw_rotates_in_place_left_positive(self):
        self.receive(yaw=.8)
        self.start()
        v, w = self.c.tick(10.05, SAFE)
        self.assertEqual(v, 0.)
        self.assertGreater(w, 0.)

    def test_negative_yaw_turns_right(self):
        self.receive(yaw=-.8)
        self.start()
        self.assertLess(self.c.tick(10.05, SAFE)[1], 0.)

    def test_bearing_reversal_never_commands_previous_direction(self):
        self.receive(yaw=.8)
        self.start()
        self.assertGreater(self.c.tick(10.05,SAFE)[1],0.)
        self.receive(10.1,10.1,yaw=-.2)
        self.assertLess(self.c.tick(10.1,SAFE)[1],0.)

    def test_disable_after_completed_stop_does_not_relabel_as_fault(self):
        self.receive()
        self.start()
        self.c.command('stop',now=10.05)
        self.receive(10.1,10.1,enabled=False,objects=[])
        self.assertEqual(self.c.state,'stopped')

    def test_close_target_never_reverses(self):
        self.receive(distance=1.1)
        self.start()
        self.assertEqual(self.c.tick(10.05, SAFE), (0., 0.))

    def test_depth_is_required_and_near_obstacle_stops_rotation(self):
        self.receive(yaw=.5)
        self.start()
        for safety in (None, Safety(), Safety(True,.4,True)):
            self.assertEqual(self.c.tick(10.05,safety), (0.,0.))

    def test_forward_collision_stops_translation(self):
        self.receive()
        self.start()
        self.assertEqual(self.c.tick(10.05, Safety(True, 1., False)), (0., 0.))

    def test_expired_lease_cannot_resurrect(self):
        self.receive()
        self.start(lease=.1)
        self.c.tick(10.11, SAFE)
        self.assertEqual(self.c.state, 'expired')
        self.assertFalse(self.c.command('heartbeat','one',lease_duration_s=1.,now=10.12)[0])
        self.assertEqual(self.start(now=10.12), (False, 'request_already_terminal'))

    def test_matching_heartbeat_extends_only_lease(self):
        self.receive()
        self.start(lease=.2)
        self.assertFalse(self.c.command('heartbeat','other',lease_duration_s=1.,now=10.1)[0])
        self.assertTrue(self.c.command('heartbeat','one',lease_duration_s=1.,now=10.1)[0])
        self.assertEqual(self.c.lease_until,11.1)
        self.assertEqual(self.c.tick(10.31,SAFE),(0.,0.))

    def test_start_idempotent_does_not_renew_lease(self):
        self.receive()
        self.start()
        self.assertEqual(self.start(now=10.1),(True,'already_active'))
        self.assertEqual(self.c.lease_until,11.)
        self.assertEqual(self.start(mode='approach_interact'),(False,'request_id_conflict'))

    def test_atomic_mode_switch_resets_motion(self):
        self.receive()
        self.start()
        self.c.tick(10.05, SAFE)
        self.assertTrue(self.start(mode='approach_interact',request='two',now=10.06)[0])
        self.assertEqual(self.c.previous_v,0.)
        self.assertEqual(self.c.mode,'approach_interact')

    def test_approach_arrives_after_three_distinct_measurements_and_latches(self):
        self.receive(distance=1.)
        self.start(mode='approach_interact')
        self.c.tick(10.01,SAFE)
        self.c.tick(10.02,SAFE)
        self.assertTrue(self.c.active)
        for timestamp in (10.1,10.2):
            self.receive(timestamp,timestamp,distance=1.)
            self.c.tick(timestamp,SAFE)
        self.assertEqual(self.c.state,'arrived')
        self.assertFalse(self.c.active)
        self.receive(10.25,10.25,distance=3.)
        self.assertEqual(self.c.tick(10.25,SAFE),(0.,0.))

    def test_approach_needs_bearing_alignment(self):
        self.receive(distance=1.,yaw=.3)
        self.start(mode='approach_interact')
        for timestamp in (10.01,10.1,10.2):
            self.receive(timestamp,timestamp,distance=1.,yaw=.3)
            self.c.tick(timestamp,SAFE)
        self.assertTrue(self.c.active)

    def test_stop_is_immediate_and_old_stop_does_not_stop_new_task(self):
        self.receive()
        self.start()
        self.start(request='two')
        self.assertTrue(self.c.command('stop','one',now=10.1)[0])
        self.assertTrue(self.c.active)
        self.assertEqual(self.c.request_id,'two')
        self.assertTrue(self.c.command('stop','',now=10.1)[0])
        self.assertEqual(self.c.tick(10.1,SAFE),(0.,0.))

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
