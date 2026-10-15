#!/usr/bin/env python3
"""Real initializer, final guard and base mux; synthetic sensors, no driver."""
import os
import rclpy
from test_vio_init_ros import run

if __name__ == '__main__':
    assert os.environ.get('ROS_DOMAIN_ID') == '198'
    rclpy.init()
    try:
        for case in ('success','manual_shared','manual_invalid','manual_legacy','startup_no_authority','authority_exit','base_takeover','base_estop','obstacle'):
            run(case,base_authority=True)
        run('success',base_authority=True,guard_delay=3.)
    finally:
        rclpy.shutdown()
