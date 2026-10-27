#!/usr/bin/env python3
"""Synthetic startup relocalization gate; no RTAB-Map, no motor topics."""
from test_rgbd_watchdog import exercise


def test_startup_view_consistency():
    exercise('cuvslam', startup_views='views')
    exercise('cuvslam', startup_views='manual')


if __name__ == '__main__':
    test_startup_view_consistency()
