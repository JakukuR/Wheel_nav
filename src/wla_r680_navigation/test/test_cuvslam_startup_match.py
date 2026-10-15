#!/usr/bin/env python3
"""Synthetic startup ordering regression; restart disabled, no motor topics."""
from test_rgbd_watchdog import exercise


def test_cuvslam_preserves_initial_map_match():
    for bounded in (False, True):
        exercise('cuvslam', bounded, initial_match=True)


if __name__ == '__main__':
    test_cuvslam_preserves_initial_map_match()
