#!/usr/bin/env python3
"""gate_probe.py  --  does map->odom advance when the rover ROTATES, or only when it
TRANSLATES?

During nav the map->odom stamp was measured freezing for up to 22 s while climbing at
exactly 1.0 s per second, and slam_toolbox peaked at 58% of one core out of four. A
backlog cannot produce either number: a process falling behind lags at LESS than 1 s/s
and needs to be saturated to do it. 1.0 s/s means the stamp is not moving at all, i.e.
slam_toolbox is being handed scans and choosing to discard every one of them.

slam_toolbox's own gate (`shouldProcessScan`) is a chain of rejections, not a set of
alternatives, and the last link is a TRANSLATION test against minimum_travel_distance.
`minimum_time_interval` sits above it and only rejects scans that arrive too SOON -- it
is a floor, not a heartbeat, so it can never rescue a robot that has stopped moving.
If that reading is right then pure rotation advances nothing, and the rotate-in-place
that DWB does before every straight run, and again at the goal, starves localization
exactly when navigation needs it most.

This settles it with three phases against the same stack, watching the stamp:
    PARKED     nothing commanded          -- baseline; the stamp should freeze
    ROTATE     in place, no translation   -- the phase under test
    TRANSLATE  forward/back, no rotation  -- the control; the stamp must advance

Reports, per phase, how many times the stamp advanced and how far the age drifted.
Rotation resembling PARKED rather than TRANSLATE confirms the translation gate.
Motion is in place or alternating, so the rover ends roughly where it started.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

PHASE_S = 20.0


def main():
    rclpy.init()
    node = rclpy.create_node("gate_probe")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    buf = Buffer()
    TransformListener(buf, node, spin_thread=True)
    time.sleep(2.0)

    t_disc = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t_disc < 5.0:
        time.sleep(0.02)

    def stamp():
        try:
            tr = buf.lookup_transform("map", "odom", rclpy.time.Time())
            return tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        except Exception:
            return None

    def odom():
        try:
            tr = buf.lookup_transform("odom", "base_link", rclpy.time.Time())
            t, q = tr.transform.translation, tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.x, t.y, yaw
        except Exception:
            return None

    def stop_burst():
        s = Twist()
        for _ in range(40):
            pub.publish(s)
            time.sleep(0.02)

    def run_phase(name, vx, wz, flip_every=None):
        print(f"\n=== {name} (vx={vx:+.2f} wz={wz:+.2f}) for {PHASE_S:.0f}s ===",
              flush=True)
        o0 = odom()
        advances, last = 0, stamp()
        ages = []
        msg = Twist()
        sign = 1.0
        t0 = time.time()
        t_flip = t0
        while time.time() - t0 < PHASE_S:
            if flip_every and time.time() - t_flip > flip_every:
                sign = -sign
                t_flip = time.time()
            msg.linear.x = vx * sign
            msg.angular.z = wz * sign if flip_every else wz
            pub.publish(msg)
            st = stamp()
            if st is not None:
                ages.append(node.get_clock().now().nanoseconds * 1e-9 - st)
                if last is None or abs(st - last) > 1e-6:
                    advances += 1
                    last = st
            time.sleep(0.05)
        stop_burst()
        time.sleep(1.5)
        o1 = odom()
        moved = rot = float("nan")
        if o0 and o1:
            moved = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
            rot = abs(math.degrees(o1[2] - o0[2]))
            rot = min(rot, 360.0 - rot) if rot > 180 else rot
        print(f"  odom travelled {moved:.3f} m, turned {rot:.1f} deg")
        print(f"  stamp advanced {advances} times in {PHASE_S:.0f}s "
              f"({advances / PHASE_S:.2f} Hz)")
        if ages:
            print(f"  age min {min(ages):+.2f}s  max {max(ages):+.2f}s  "
                  f"drift {max(ages) - min(ages):.2f}s")
        return advances, moved, rot

    try:
        r = {}
        r["PARKED"] = run_phase("PARKED", 0.0, 0.0)
        r["ROTATE"] = run_phase("ROTATE in place", 0.0, 0.5, flip_every=5.0)
        r["TRANSLATE"] = run_phase("TRANSLATE only", 0.10, 0.0, flip_every=5.0)
    finally:
        stop_burst()

    print("\n=== verdict ===")
    for k, (adv, moved, rot) in r.items():
        print(f"  {k:10s} stamp advances {adv:4d}   odom {moved:.3f} m / {rot:5.1f} deg")
    if r["ROTATE"][0] <= max(1, r["PARKED"][0]) and r["TRANSLATE"][0] > 5:
        print("\n  CONFIRMED: rotation does not advance map->odom; only translation does.")
        print("  slam_toolbox is gating on minimum_travel_distance with no time escape,")
        print("  so every rotate-in-place is a localization blackout.")
    else:
        print("\n  NOT confirmed -- rotation advanced the stamp; look elsewhere.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
