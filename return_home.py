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
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
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
    ap.add_argument('--verify-attempts', type=int, default=2,
                    help='after Nav2 reports success, re-localize against the saved map and '
                         're-drive if the TRUE pose is outside tolerance (0 disables)')
    ap.add_argument('--xy-tol', type=float, default=0.15,
                    help='accept-position tolerance for the verify pass '
                         '(match xy_goal_tolerance in nav2_params.yaml)')
    ap.add_argument('--yaw-tol', type=float, default=0.20,
                    help='accept-heading tolerance in rad for the verify pass '
                         '(match yaw_goal_tolerance in nav2_params.yaml)')
    return ap.parse_args()


def absolute_pose(map_name, publish=True):
    """Scan-match the CURRENT lidar frame against the saved map -> (x, y, yaw) or None.

    Runs reseed.py, which is deliberately independent of the running localization: it
    matches the live scan against the saved .pgm from scratch. That independence is the
    entire point. Verifying with map->base_link would consult the very estimate that is
    suspected of being wrong, and would happily confirm a pose that is 22 deg off -- the
    exact failure this pass exists to catch. Measured across four runs, the final heading
    error tracked reseed's INITIAL yaw error almost one-for-one (0.4 deg in -> 3.0 out,
    8.9 -> 9.0), because slam_loc contributes no correction over a short run; Nav2 drives
    to "believed yaw = 0" and the belief is offset by exactly the seed error.

    With publish=True reseed also republishes /initialpose, so a bad belief is CORRECTED
    here, not merely detected -- which is what makes a second attempt worth anything.
    """
    cmd = [sys.executable, '/root/reseed.py', '--map-name', map_name]
    if not publish:
        cmd.append('--no-publish')
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=120).stdout
    except Exception as e:
        print(f"[return_home] verify: reseed failed to run ({e})", file=sys.stderr)
        return None
    for line in out.splitlines():
        if line.startswith('POSE '):
            try:
                x, y, yaw = (float(v) for v in line.split()[1:4])
                return (x, y, yaw)
            except Exception:
                return None
    print("[return_home] verify: reseed produced no POSE line (gate failed?)", file=sys.stderr)
    return None


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pose_error(pose, gx, gy, gyaw):
    """(distance_m, |heading error| rad) of `pose` from the goal."""
    dxy = math.hypot(pose[0] - gx, pose[1] - gy)
    dyaw = abs(wrap(pose[2] - gyaw))
    return dxy, dyaw


# Fastest yaw rate this chassis has been measured doing at the lowest usable duty.
# Used to size trim pulses so they UNDERSHOOT: assuming the fast end means the pulse is
# too short whenever the rover happens to respond slowly, which is the safe direction.
TRIM_ASSUMED_RATE = 1.5      # rad/s
TRIM_W = 0.2                 # commanded yaw rate for trim pulses (lowest usable duty)
TRIM_UNDERSHOOT = 0.5        # fraction of the "ideal" duration to actually apply


def trim_heading(nav, pub, map_name, goal_yaw, yaw_tol, max_pulses=6):
    """Close the loop on final heading: measure, nudge, measure again.

    Nav2 cannot do this on its own here. Its heading correction is open-loop within a
    control cycle, and this chassis's rotation is NOT repeatable -- the same command,
    at the same duty, was measured producing 0.62 rad/s and 1.42 rad/s on different
    attempts (2.3x), because 4 of 6 tyres are taped and the skid-steer sticks and then
    breaks free. Open-loop control of an actuator like that cannot hit an 11.5 deg
    window: the verify pass previously tried to fix a -21 deg error and landed at
    +18 deg, bouncing across the goal.

    Measuring between pulses removes the need for repeatability. Each pulse is sized
    from the FAST end of the observed range and then halved, so it reliably undershoots
    and the error converges from one side instead of oscillating. Absolute scan-match is
    used for the measurement -- checking against map->base_link would consult the very
    estimate that has been wrong.

    Returns the last measured pose, or None if it could not be measured.
    """
    pose = None
    for i in range(1, max_pulses + 1):
        pose = absolute_pose(map_name, publish=True)
        if pose is None:
            return None
        err = wrap(goal_yaw - pose[2])
        print(f"[return_home] trim {i}/{max_pulses}: heading off by "
              f"{math.degrees(err):+.1f} deg")
        if abs(err) <= yaw_tol:
            print("[return_home] trim: heading within tolerance.")
            return pose
        dur = abs(err) / TRIM_ASSUMED_RATE * TRIM_UNDERSHOOT
        dur = max(0.06, min(dur, 0.5))       # floor: shorter than this does nothing
        twist = Twist()
        twist.angular.z = math.copysign(TRIM_W, err)
        t0 = time.time()
        while time.time() - t0 < dur:
            pub.publish(twist)
            time.sleep(0.02)
        stop = Twist()
        for _ in range(30):                  # latching driver: stop must be insisted on
            pub.publish(stop)
            time.sleep(0.02)
        time.sleep(1.5)                      # let the chassis settle before re-measuring
    print(f"[return_home] trim: did not converge in {max_pulses} pulses.", file=sys.stderr)
    return pose


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
        print(f"[return_home] goal = ({args.x:.2f}, {args.y:.2f}, {args.yaw:.2f}) -- navigating ...")

        def drive_once():
            """Send the goal and monitor to completion. Returns a TaskResult."""
            nonlocal goal_sent
            goal = make_goal(nav, args.x, args.y, args.yaw)
            nav.goToPose(goal)
            goal_sent = True
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
            return nav.getResult()

        trim_pub = nav.create_publisher(Twist, '/cmd_vel', 10)

        # --- 4. DRIVE, VERIFY AND CORRECT ---
        # Nav2 reporting SUCCEEDED only means its BELIEF reached the goal. Measured on
        # this rover: a run finished with `Goal succeeded` while physically sitting
        # 22.0 deg off (tolerance 11.5), and an earlier one 38 deg off, because the
        # heading belief was wrong by that much at the instant the goal checker sampled
        # and converged to the truth only afterwards. So re-localize from scratch and
        # check the REAL pose. reseed also republishes /initialpose, so a wrong belief is
        # corrected before the retry -- otherwise the retry would just drive to the same
        # wrong place again, convinced it was already home.
        # A Nav2 FAILURE used to end the run right here, and that was the single biggest
        # gap in this script. The two things that actually go wrong on this rover are both
        # recoverable and neither is recovered by giving up:
        #   - it wedges itself on furniture (a chair leg), stops moving, and therefore stops
        #     feeding slam the travel it needs to keep map->odom fresh, so nav2 loses the
        #     transform and aborts follow_path;
        #   - it gets picked up and put down somewhere else (freeing it by hand counts).
        #     slam_toolbox localization only scan-matches LOCALLY around its current
        #     estimate, so after a manual move its belief is simply wrong and stays wrong,
        #     and nav2 confidently drives to the wrong place -- "I put it back and it still
        #     could not find its way home".
        # reseed.py fixes the second by matching the live scan against the saved map from
        # scratch, and it was already being run after SUCCESS only. Running it after a
        # FAILURE is what turns both of these into a retry instead of a dead run.
        max_attempts = max(1, args.verify_attempts + 1)     # drive attempts, not verifies
        result = None
        for attempt in range(1, max_attempts + 1):
            result = drive_once()

            if result != TaskResult.SUCCEEDED:
                print(f"[return_home] attempt {attempt}/{max_attempts}: Nav2 did not reach "
                      f"the goal ({result}).", file=sys.stderr)
                if attempt == max_attempts:
                    break
                print("[return_home] re-localizing from scratch ...")
                if absolute_pose(args.map_name, publish=True) is None:
                    print("[return_home] could not re-localize -- giving up.", file=sys.stderr)
                    break
                time.sleep(2.0)                 # let slam absorb the corrected /initialpose
                continue

            if args.verify_attempts <= 0:
                break

            time.sleep(2.0)                     # let the pose settle before measuring
            pose = absolute_pose(args.map_name, publish=True)
            if pose is None:
                print("[return_home] verify: could not establish a true pose -- "
                      "reporting Nav2's result unchanged.", file=sys.stderr)
                break
            dxy, dyaw = pose_error(pose, args.x, args.y, args.yaw)
            print(f"[return_home] verify {attempt}/{max_attempts}: true pose "
                  f"({pose[0]:.3f}, {pose[1]:.3f}, yaw {math.degrees(pose[2]):.1f} deg) "
                  f"-> off by {dxy:.3f} m / {math.degrees(dyaw):.1f} deg")
            if dxy <= args.xy_tol and dyaw <= args.yaw_tol:
                print("[return_home] verify: within tolerance.")
                break

            # Position and heading are corrected by DIFFERENT mechanisms on purpose.
            # Re-issuing a Nav2 goal to fix a heading-only error is what made this
            # oscillate before (-21 deg -> +18 deg): DWB does one open-loop rotation
            # per cycle, and on this non-repeatable chassis that rotation overshoots.
            # So Nav2 is used only when the POSITION is actually wrong, and the final
            # heading is closed-loop trimmed with measured pulses.
            if dxy > args.xy_tol:
                if attempt == max_attempts:
                    print(f"[return_home] verify: STILL off by {dxy:.3f} m / "
                          f"{math.degrees(dyaw):.1f} deg after {attempt} attempt(s) -- "
                          f"giving up rather than driving indefinitely.", file=sys.stderr)
                    result = TaskResult.FAILED
                    break
                print("[return_home] verify: position outside tolerance -- "
                      "belief corrected, re-driving ...")
                time.sleep(2.0)                 # let slam absorb the corrected /initialpose
                continue

            print("[return_home] verify: position OK, heading off -- trimming ...")
            pose = trim_heading(nav, trim_pub, args.map_name, args.yaw, args.yaw_tol)
            if pose is None:
                result = TaskResult.FAILED
                break
            dxy, dyaw = pose_error(pose, args.x, args.y, args.yaw)
            print(f"[return_home] verify: after trim, off by {dxy:.3f} m / "
                  f"{math.degrees(dyaw):.1f} deg")
            if dxy <= args.xy_tol and dyaw <= args.yaw_tol:
                print("[return_home] verify: within tolerance.")
                break
            if attempt == max_attempts:
                print("[return_home] verify: heading trim did not converge.", file=sys.stderr)
                result = TaskResult.FAILED
                break

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
