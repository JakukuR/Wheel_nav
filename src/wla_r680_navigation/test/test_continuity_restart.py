#!/usr/bin/env python3
"""Isolated synthetic inputs only; no motion and frontend restart disabled."""
from test_rgbd_watchdog import exercise


def test_continuity_restart():
    exercise('cuvslam', True, continuity_recovery=True)


if __name__ == '__main__':
    test_continuity_restart()
