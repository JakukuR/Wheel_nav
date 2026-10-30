#!/usr/bin/env python3
"""navigation_map_mode and navigation_goal_bridge in an isolated graph, without motion."""
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Harness:
    def __init__(self):
        self.node = rclpy.create_node('mapless_navigation_regression')
        n = self.node
        self.tf = n.create_publisher(TFMessage, '/tf', 50)
        self.source = n.create_publisher(OccupancyGrid, '/r680/d455/map', LATCHED)
        self.localized = n.create_publisher(Bool, '/r680_nav/localization_ready', 10)
        self.motion = n.create_publisher(Bool, '/r680_nav/motion_ready', 10)
        self.epoch = n.create_publisher(UInt32, '/r680_nav/odom_epoch', LATCHED)
        self.goal = n.create_publisher(PoseStamped, '/r680_nav/goal_request', 10)
        self.maps, self.modes, self.goals = [], [], []
        n.create_subscription(OccupancyGrid, '/r680_nav/navigation_map', self.maps.append, LATCHED)
        n.create_subscription(String, '/r680_nav/navigation_map_mode',
                              lambda m: self.modes.append(m.data), LATCHED)
        group = ReentrantCallbackGroup()
        ActionServer(n, NavigateToPose, '/navigate_to_pose', self.execute,
                     goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(n)
        self.map_odom = (0.0, 0.0, 0.0)
        self.base = (1.0, 2.0, 0.0)
        self.flags = (False, False)

    def execute(self, handle):
        record = {'pose': handle.request.pose, 'cancelled': False}
        self.goals.append(record)
        while rclpy.ok():
            if handle.is_cancel_requested:
                record['cancelled'] = True
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        handle.abort()
        return NavigateToPose.Result()

    def transform(self, parent, child, pose):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = pose[0], pose[1]
        t.transform.rotation.z, t.transform.rotation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        return t

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            # base is given in odom; map->odom is the (unverified) correction.
            self.tf.publish(TFMessage(transforms=[
                self.transform('map', 'd455_floor_odom', self.map_odom),
                self.transform('d455_floor_odom', 'r680_mapping_floor', self.base)]))
            self.localized.publish(Bool(data=self.flags[0]))
            self.motion.publish(Bool(data=self.flags[1]))
            self.executor.spin_once(timeout_sec=0.02)

    def wait(self, predicate, seconds, what):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.spin(0.05)
            if predicate():
                return
        raise AssertionError(f'timeout: {what}')

    def send_goal(self, frame, x, y):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.w = 1.0
        self.goal.publish(msg)


def start(script, *args):
    return subprocess.Popen([sys.executable, str(ROOT/'scripts'/script), '--ros-args', *args])


def real_map():
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.05
    grid.info.width, grid.info.height = 20, 10
    grid.info.origin.orientation.w = 1.0
    grid.data = [100]*200
    return grid


def blank(grid):
    return grid.info.width == 800 and not any(grid.data)


def main():
    assert 100 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232
    # ros2 launch runs them directly from the install symlinks.
    for script in ('navigation_map_mode', 'navigation_goal_bridge'):
        assert os.access(ROOT/'scripts'/script, os.X_OK), script + ' is not executable'
    rclpy.init()
    h = Harness()
    mode = start('navigation_map_mode', '-p', 'lost_hold_s:=1.0')
    bridge = start('navigation_goal_bridge')
    try:
        h.source.publish(real_map())
        h.epoch.publish(UInt32(data=0))
        h.flags = (False, True)
        h.wait(lambda: h.modes and h.modes[-1] == 'mapless' and h.maps and blank(h.maps[-1]),
               10, 'mapless blank grid')
        grid = h.maps[-1]
        cx = grid.info.origin.position.x + 20.0
        cy = grid.info.origin.position.y + 20.0
        assert abs(cx-1.0) < 0.1 and abs(cy-2.0) < 0.1, (cx, cy)
        print('PASS startup is mapless with a blank grid centred on the robot', flush=True)

        # Unverified map->odom movement rebuilds the grid (clears marked obstacles).
        count = len(h.maps)
        h.map_odom = (0.5, 0.0, 0.0)
        h.wait(lambda: len(h.maps) > count and blank(h.maps[-1]), 3, 'grid rebuilt after jump')
        assert h.maps[-1].info.origin.position.x != grid.info.origin.position.x
        print('PASS map->odom jump while mapless rebuilds the blank grid', flush=True)

        # Goals: map frame refused, odometry frame forwarded, robot frame converted.
        h.wait(lambda: bridge.poll() is None and h.node.count_subscribers('/r680_nav/goal_request') > 0,
               10, 'goal bridge')
        h.send_goal('map', 3.0, 0.0)
        h.spin(1.0)
        assert not h.goals, 'map-frame goal reached Nav2 while mapless'
        h.send_goal('r680_mapping_floor', 1.0, 0.0)
        h.wait(lambda: h.goals, 5, 'robot-frame goal')
        pose = h.goals[-1]['pose']
        assert pose.header.frame_id == 'd455_floor_odom', pose.header.frame_id
        assert abs(pose.pose.position.x-2.0) < 0.05 and abs(pose.pose.position.y-2.0) < 0.05
        print('PASS mapless refuses map goals and converts robot-frame goals to odom', flush=True)

        # A new odometry epoch cancels the odometry-frame goal.
        h.epoch.publish(UInt32(data=1))
        h.wait(lambda: h.goals[-1]['cancelled'], 5, 'goal cancelled on epoch change')
        print('PASS odometry epoch change cancels the odom goal', flush=True)

        # Verified match switches to the real map; map goals are accepted again.
        h.flags = (True, True)
        h.wait(lambda: h.modes[-1] == 'map' and h.maps[-1].info.width == 20, 5, 'map mode')
        before = len(h.goals)
        h.send_goal('map', 3.0, 0.0)
        h.wait(lambda: len(h.goals) > before, 5, 'map goal in map mode')
        assert h.goals[-1]['pose'].header.frame_id == 'map'
        print('PASS verified match republishes the real map and admits map goals', flush=True)

        # A short loss holds the map; a sustained one returns to mapless.
        h.flags = (False, True)
        h.spin(0.6)
        assert h.modes[-1] == 'map', 'short localization drop switched mode'
        h.wait(lambda: h.modes[-1] == 'mapless' and blank(h.maps[-1]), 4, 'mapless after loss')
        # Stale motion and localization keep the current mode.
        h.flags = (False, False)
        h.spin(1.5)
        assert h.modes[-1] == 'mapless'
        print('PASS sustained loss returns to mapless; short drops and stale inputs hold', flush=True)
    finally:
        for p in (mode, bridge):
            p.terminate()
            p.wait(timeout=5)
        h.node.destroy_node()
        rclpy.shutdown()

    # Mapping mode passes the source map through unchanged.
    rclpy.init()
    h = Harness()
    mode = start('navigation_map_mode', '-p', 'always_map:=true')
    try:
        h.source.publish(real_map())
        h.wait(lambda: h.modes and h.modes[-1] == 'map' and h.maps and h.maps[-1].info.width == 20,
               10, 'passthrough')
        h.flags = (False, True)
        h.spin(1.5)
        assert h.modes[-1] == 'map' and h.maps[-1].info.width == 20
        print('PASS always_map passes the source map through', flush=True)
    finally:
        mode.terminate()
        mode.wait(timeout=5)
        h.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
