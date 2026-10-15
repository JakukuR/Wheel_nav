#!/usr/bin/env python3
"""Initialization liveness is distinct from healthy-running odometry freshness."""
from test_rgbd_watchdog import exercise


def test_cuvslam_startup_timeout():
    for bounded in (False, True):
        for case in ('live', 'frontend_dead') + (('continuity_dead',) if bounded else ()):
            exercise('cuvslam', bounded, startup_timeout_case=case)


if __name__ == '__main__':
    test_cuvslam_startup_timeout()
