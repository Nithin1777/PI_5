#!/usr/bin/env python3
"""speed_check.py [reps]  --  what linear speed does a COMMANDED linear speed produce?

pos_check.py measured commanded 0.10 m/s x 3.0 s = 0.300 m producing 0.796 m by rf2o
and 0.802 m by absolute scan-match. The two methods agreeing to 6 mm rules out an
estimation error: the chassis really does travel ~2.7x what it is asked for. DWB plans
every trajectory on the assumption that a commanded velocity is the velocity it gets,
so a 2.7x gain makes every final approach overshoot -- the goal-hunting and the 0.42 m
lurch past a 0.24 m target both follow from it.

Fixing that needs the real curve, not one point, and it needs the constant term
separated from the slope:

  distance(t) = v_actual * t + coast

`coast` is what the rover adds after the stop burst (the motor node latches, the chassis
has mass, and the taped tyres slide). Two durations per commanded speed separate them:
    v_actual = (d_long - d_short) / (t_long - t_short)
    coast    = d_short - v_actual * t_short
A single duration cannot tell a fast rover from a slow rover that coasts a long way, and
those want different fixes -- one is a scale factor on the duty mapping, the other is a
deceleration limit.

Distance is measured from rf2o odom->base_link. pos_check.py cross-checked rf2o against
an absolute scan-match over the same 0.8 m motion and they agreed to 0.006 m, so rf2o is
trusted here and is far faster than re-running a scan-match per pulse.

Direction alternates every pulse so the rover oscillates about its starting point instead
of marching across the room; pulses are kept short for the same reason.
"""
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Kept short deliberately: at the measured ~2.7x gain a 1.2 s pulse at 0.15 m/s is
# already ~0.5 m, which is a large fraction of this room.
DURATIONS = [0.6, 1.2]
COMMANDS = [0.05, 0.10, 0.15]
SETTLE_S = 2.5

# Mirrors l298n_node.py so the report can state the duty each command actually produced.
PWM_MIN, PWM_MAX = 0.08, 0.94


def level_to_duty(lvl):
    return PWM_MIN + (lvl / 9.0) * (PWM_MAX - PWM_MIN)


def main():
    reps = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    max_level = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    min_duty = float(sys.argv[3]) if len(sys.argv) > 3 else 0.10
    max_speed = float(sys.argv[4]) if len(sys.argv) > 4 else 0.30
    max_duty = level_to_duty(max_level)

    rclpy.init()
    node = rclpy.create_node("speed_check")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    buf = Buffer()
    TransformListener(buf, node, spin_thread=True)
    time.sleep(2.0)

    def pos():
        try:
            t = buf.lookup_transform("odom", "base_link", rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    t_disc = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t_disc < 5.0:
        time.sleep(0.02)

    def pulse(vx, dur):
        p0 = pos()
        msg = Twist()
        msg.linear.x = vx
        t0 = time.time()
        while time.time() - t0 < dur:
            pub.publish(msg)
            time.sleep(0.02)
        elapsed = time.time() - t0
        stop = Twist()
        for _ in range(40):
            pub.publish(stop)
            time.sleep(0.02)
        time.sleep(SETTLE_S)
        p1 = pos()
        if p0 is None or p1 is None:
            return None, elapsed
        return math.hypot(p1[0] - p0[0], p1[1] - p0[1]), elapsed

    sign = 1.0
    table = {}
    print(f"{reps} reps per point; distance from rf2o; direction alternates\n")
    print(f"  {'cmd m/s':>8} {'duty':>6} {'dur s':>6} {'distance m':>22} {'mean':>7}")
    try:
        for c in COMMANDS:
            frac = min(abs(c) / max_speed, 1.0)
            duty = min_duty + frac * (max(max_duty, min_duty) - min_duty)
            for d in DURATIONS:
                got = []
                for _ in range(reps):
                    dist, el = pulse(sign * c, d)
                    sign = -sign
                    if dist is not None:
                        got.append(dist)
                if not got:
                    print(f"  {c:8.2f} {duty:6.3f} {d:6.2f}   <no tf>")
                    continue
                m = statistics.mean(got)
                table[(c, d)] = m
                vals = " ".join(f"{g:6.3f}" for g in got)
                print(f"  {c:8.2f} {duty:6.3f} {d:6.2f} {vals:>22} {m:7.3f}")
    finally:
        stop = Twist()
        for _ in range(40):
            pub.publish(stop)
            time.sleep(0.02)

    print("\n=== steady speed and coast, separated ===")
    print(f"  {'cmd m/s':>8} {'actual m/s':>11} {'gain':>6} {'coast m':>8}")
    fits = []
    for c in COMMANDS:
        a = table.get((c, DURATIONS[0]))
        b = table.get((c, DURATIONS[1]))
        if a is None or b is None:
            continue
        v = (b - a) / (DURATIONS[1] - DURATIONS[0])
        coast = a - v * DURATIONS[0]
        fits.append((c, v))
        print(f"  {c:8.2f} {v:11.3f} {v / c if c else float('nan'):6.2f} {coast:8.3f}")

    if fits:
        num = sum(c * v for c, v in fits)
        den = sum(c * c for c, _ in fits)
        gain = num / den if den else float("nan")
        print(f"\n  actual = {gain:.2f} x commanded over "
              f"{COMMANDS[0]}..{COMMANDS[-1]} m/s")
        # The motor node maps commanded speed -> duty through max_speed. Scaling
        # max_speed by the gain makes a commanded 0.10 m/s pick the duty that really
        # produces 0.10 m/s, without touching min_duty (the stiction floor) or the
        # level ceiling the user set.
        print(f"  => max_speed {max_speed:.2f} should become "
              f"{max_speed * gain:.2f} to make commanded match actual")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
