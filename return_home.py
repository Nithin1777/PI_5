#!/usr/bin/env python3
"""return_home.py [--map-name NAME] [--x X --y Y --yaw YAW] [--timeout S] [--no-seed]

Return-to-origin demo driver.

Pipeline:
  1. Seed AMCL by publishing /initialpose from a DEDICATED node with TRANSIENT_LOCAL
     QoS (matches AMCL's latched subscription). We do NOT use BasicNavigator's
     setInitialPose -- its /initialpose publisher uses a QoS AMCL rejects
     ("incompatible QoS ... DURABILITY") so the seed never arrives.
  2. Confirm AMCL echoed back /amcl_pose. ABORT (do not drive) if not confirmed.
  3. waitUntilNav2Active() via BasicNavigator.
  4. goToPose(goal) -- default goal is the map origin (0,0,0): slam_toolbox starts
     the robot at map (0,0,0) when mapping begins, so the origin IS your drop point.

Safety: on ANY exit (success/fail/Ctrl-C/crash-via-finally) the active goal is
CANCELLED so Nav2 can't keep driving after this script dies.

Run via:  ~/rover nav
"""
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

MAPS_DIR = "/root/maps"


def yaw_to_quat(yaw):
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_goal(navigator, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    z, w = yaw_to_quat(yaw)
    p.pose.orientation.z = z
    p.pose.orientation.w = w
    return p


def make_initial_pose_msg(clock, x, y, yaw):
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.header.stamp = clock.now().to_msg()
    m.pose.pose.position.x = float(x)
    m.pose.pose.position.y = float(y)
    m.pose.pose.position.z = 0.0
    z, w = yaw_to_quat(yaw)
    m.pose.pose.orientation.z = z
    m.pose.pose.orientation.w = w
    cov = [0.0] * 36
    cov[0] = cov[7] = cov[35] = 0.05      # x, y, yaw variance
    m.pose.covariance = cov
    return m


def seed_and_confirm_amcl(x, y, yaw, timeout=30.0):
    """Publish /initialpose REPEATEDLY from a dedicated node until AMCL echoes
    /amcl_pose. AMCL rejects the seed while still 'inactive' (a common nav2
    lifecycle bringup race: 'amcl/get_state async_send_request failed' -> it
    aborts and leaves AMCL inactive), so we keep republishing ~every 0.4 s -- the
    moment AMCL goes active it accepts the next one. Returns True on /amcl_pose."""
    node = rclpy.create_node('rh_seed')
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
    got = {'f': False}
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                             lambda _m: got.__setitem__('f', True), 10)
    t0 = time.time()
    last_pub = 0.0
    while time.time() - t0 < timeout:
        if time.time() - last_pub > 0.4:     # republish through AMCL's activation window
            msg = make_initial_pose_msg(node.get_clock(), x, y, yaw)
            pub.publish(msg)
            last_pub = time.time()
        rclpy.spin_once(node, timeout_sec=0.1)
        if got['f']:
            node.destroy_node()
            return True
    node.destroy_node()
    return False


def parse_args():
    ap = argparse.ArgumentParser(description="Return-to-origin Nav2 driver")
    ap.add_argument('--map-name', default='room')
    ap.add_argument('--x', type=float, default=0.0, help='goal x (default 0 = origin)')
    ap.add_argument('--y', type=float, default=0.0, help='goal y (default 0 = origin)')
    ap.add_argument('--yaw', type=float, default=0.0, help='goal yaw rad (default 0)')
    ap.add_argument('--timeout', type=float, default=240.0,
                    help='give up after this many seconds (default 240)')
    ap.add_argument('--no-seed', action='store_true',
                    help='skip seeding AMCL (use global localization)')
    ap.add_argument('--no-amcl', action='store_true',
                    help='slam_toolbox localization mode: skip AMCL seeding/confirmation '
                         '(reseed.py already published /initialpose; stack.sh verified map->odom)')
    return ap.parse_args()


def wait_for_bt_navigator_active(nav, timeout=120.0):
    """No-AMCL mode: wait until the Nav2 BT navigator lifecycle node is ACTIVE.

    BasicNavigator.waitUntilNav2Active() hardcodes a wait on '/amcl/get_state',
    which can never exist once AMCL is retired (slam_toolbox localization owns
    map->odom) -- it would spin forever on "amcl/get_state service not available".
    We instead poll the bt_navigator lifecycle directly; stack.sh already proved
    localization is up (it waited for the map->odom TF before launching Nav2)."""
    cli = nav.create_client(GetState, '/bt_navigator/get_state')
    req = GetState.Request()
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(nav, timeout_sec=0.1)
        if not cli.service_is_ready():
            continue
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(nav, fut, timeout_sec=1.0)
        if fut.result() is not None and \
                fut.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
            return True
    return False


def main():
    args = parse_args()
    rclpy.init()
    nav = BasicNavigator()
    goal_sent = False
    rc = 1

    try:
        # --- 1. seed + confirm localization ---
        # AMCL mode: publish /initialpose from a dedicated node and confirm via /amcl_pose.
        # slam_toolbox localization mode (--no-amcl): reseed.py already published /initialpose
        # and stack.sh verified the map->odom TF, so there is nothing to do here.
        if not args.no_amcl:
            seed_file = os.path.join(MAPS_DIR, f"{args.map_name}.seed")
            if not args.no_seed and os.path.exists(seed_file):
                sx, sy, syaw = (float(v) for v in open(seed_file).read().split())
                print(f"[return_home] seeding AMCL at ({sx:.2f}, {sy:.2f}, {syaw:.2f}) from {seed_file}")
                print("[return_home] published /initialpose (TRANSIENT_LOCAL); waiting for amcl_pose ...")
                if not seed_and_confirm_amcl(sx, sy, syaw):
                    print("[return_home] ABORT: AMCL never published /amcl_pose -- localization NOT "
                          "confirmed. Will NOT drive. Seed manually in Foxglove (2D Pose Estimate) or "
                          "check /tmp/nav.log for AMCL errors.", file=sys.stderr)
                    return 6
                print("[return_home] AMCL localized (amcl_pose received).")
            elif not args.no_seed:
                print("[return_home] WARNING: no seed file -- AMCL will do global localization "
                      "(may be unreliable).", file=sys.stderr)

        # --- 2. wait for Nav2 active ---
        print("[return_home] waiting for Nav2 to activate ...")
        if args.no_amcl:
            if not wait_for_bt_navigator_active(nav):
                print("[return_home] ABORT: bt_navigator never became ACTIVE in 120s "
                      "-- check /tmp/nav.log.", file=sys.stderr)
                return 6
        else:
            nav.waitUntilNav2Active()   # waits on AMCL lifecycle (present in AMCL mode)
        print("[return_home] Nav2 active.")

        # --- 3. goal (default: map origin = your drop point) ---
        goal = make_goal(nav, args.x, args.y, args.yaw)
        print(f"[return_home] goal = ({args.x:.2f}, {args.y:.2f}, {args.yaw:.2f}) -- navigating ...")
        nav.goToPose(goal)
        goal_sent = True

        # --- monitor ---
        t0 = time.time()
        last = None
        while not nav.isTaskComplete():
            if time.time() - t0 > args.timeout:
                print(f"[return_home] TIMEOUT after {args.timeout:.0f}s -- cancelling.",
                      file=sys.stderr)
                nav.cancelTask()
                break
            fb = nav.getFeedback()
            if fb is not None:
                d = fb.distance_remaining
                if last is None or abs(d - last) > 0.05:
                    last = d
                    try:
                        eta = f"{fb.estimated_time_remaining.sec}s"
                    except Exception:
                        eta = "?"
                    print(f"[return_home] {d:.2f} m to goal (est {eta})")
            rclpy.spin_once(nav, timeout_sec=0.2)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print("[return_home] ARRIVED at goal. :-)")
            rc = 0
        elif result == TaskResult.CANCELED:
            print("[return_home] navigation canceled.", file=sys.stderr)
            rc = 4
        else:
            print("[return_home] navigation FAILED.", file=sys.stderr)
            rc = 5

    except KeyboardInterrupt:
        print("[return_home] interrupted -- cancelling goal.", file=sys.stderr)
        rc = 4
    finally:
        # CRITICAL: never leave Nav2 executing a goal after this script exits
        if goal_sent:
            try:
                if not nav.isTaskComplete():
                    nav.cancelTask()
                    t = time.time()
                    while not nav.isTaskComplete() and time.time() - t < 3:
                        rclpy.spin_once(nav, timeout_sec=0.1)
            except Exception:
                pass
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
