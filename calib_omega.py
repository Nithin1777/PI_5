#!/usr/bin/env python3
"""calib_omega.py [reps]  --  measure ACTUAL vs COMMANDED angular velocity.

Sweeps a set of commanded yaw rates through the normal /cmd_vel path (motor node in
its usual proportional mode -- nothing is pinned or overridden), measures what the
rover really did from rf2o, and reports the actual/commanded ratio plus the single
best-fit scale factor over the range Nav2 actually uses.

Why a scale factor is the thing to find
---------------------------------------
After the proportional-duty fix, commanded 0.5 rad/s still produced ~0.79 rad/s. DWB
plans trajectories assuming it gets what it asks for, so a consistent over-response
makes every heading correction overshoot. Measured directly: a verify pass tried to
correct a -21 deg heading error and landed at +18 deg -- ~39 deg of rotation applied
to fix 21 deg, bouncing across the goal instead of settling inside an 11.5 deg window.

Repeats matter here. Two identical wz=-0.15 pulses previously measured -51.9 and -69.4
deg (34% apart) because 4 of the 6 tyres are taped to cut grip, so a skid-steer turn
slips differently every time. Fitting a curve to one sample per point would be fitting
noise; this takes `reps` samples per point and reports the spread so you can see how
much of the residual is calibration and how much is simply not repeatable.

Direction alternates every pulse so the rover unwinds itself instead of spinning up.
"""
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

PULSE_S = 0.8
SETTLE_S = 1.5
# spans the useful band: below ~0.2 the chassis barely moves, above ~1.2 is faster than
# nav2's max_vel_theta (0.6) ever asks for outside recovery spins
COMMANDS = [0.2, 0.3, 0.4, 0.5, 0.7, 1.0]


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    reps = int(sys.argv[1]) if len(sys.argv) > 1 else 3

    rclpy.init()
    node = rclpy.create_node("calib_omega")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    buf = Buffer()
    TransformListener(buf, node, spin_thread=True)
    time.sleep(2.0)

    def yaw():
        try:
            t = buf.lookup_transform("odom", "base_link", rclpy.time.Time())
            return quat_yaw(t.transform.rotation)
        except Exception:
            return None

    t_disc = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t_disc < 5.0:
        time.sleep(0.02)

    sign = -1.0
    rows = []
    print(f"{reps} reps x {PULSE_S}s per point, actual rate from rf2o\n")
    print(f"  {'cmd':>6} {'actual (rad/s)':>26}  {'mean':>7} {'ratio':>7} {'spread':>7}")
    try:
        for c in COMMANDS:
            got = []
            for _ in range(reps):
                y0 = yaw()
                msg = Twist(); msg.angular.z = sign * c
                t0 = time.time()
                while time.time() - t0 < PULSE_S:
                    pub.publish(msg)
                    time.sleep(0.05)
                elapsed = time.time() - t0
                stop = Twist()
                for _ in range(30):
                    pub.publish(stop)
                    time.sleep(0.02)
                time.sleep(SETTLE_S)
                y1 = yaw()
                sign = -sign
                if None in (y0, y1):
                    continue
                got.append(abs(wrap(y1 - y0)) / elapsed)
            if not got:
                print(f"  {c:6.2f}   <no tf>")
                continue
            m = statistics.mean(got)
            spread = (max(got) - min(got)) / m if m else 0.0
            rows.append((c, m))
            vals = " ".join(f"{g:5.2f}" for g in got)
            print(f"  {c:6.2f} {vals:>26}  {m:7.2f} {m / c:7.2f} {spread * 100:6.0f}%")
    finally:
        stop = Twist()
        for _ in range(40):
            pub.publish(stop)
            time.sleep(0.02)

    if rows:
        # least-squares slope through the origin: actual ~= gain * commanded.
        # Through the origin because zero command must mean zero motion -- an intercept
        # would be physically meaningless and would skew the correction at small errors,
        # which is exactly where the heading loop lives.
        num = sum(c * a for c, a in rows)
        den = sum(c * c for c, _ in rows)
        gain = num / den if den else float("nan")
        print(f"\n  actual = {gain:.3f} x commanded   over {rows[0][0]}..{rows[-1][0]} rad/s")
        print(f"  => set w_scale = {1.0 / gain:.3f} to make commanded match actual")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
