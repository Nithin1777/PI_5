#!/usr/bin/env python3
"""tf_age.py [seconds] [hz]  --  how STALE is map->odom, measured against the clock.

slam_toolbox does not publish a live map->odom. It stamps the transform as
(time of the last PROCESSED scan + transform_timeout) and then republishes that
same frozen stamp at 50 Hz until another scan is processed. A scan is only
processed once the robot has travelled minimum_travel_distance /
minimum_travel_heading, so while the rover creeps or sits still the stamp stops
advancing and the transform silently ages at one second per second.

Nav2 does not tolerate that. tf_help refuses any lookup past the newest stamp it
holds and logs "Transform data too old when converting from map to odom"; the
controller then cannot put the robot pose in the global plan's frame, patience
runs out, and follow_path aborts. That is a deadlock, not a slowdown: no valid
transform -> no velocity command -> no travel -> no scan processed -> no fresh
transform.

This samples the newest map->odom stamp the TF buffer holds and compares it with
wall time, so the number it prints is exactly the quantity nav2 checks. Anything
above nav2's transform_tolerance (0.5 s in the costmaps) is a window in which
navigation cannot run. Run it DURING a nav to see whether the deadlock is what
is happening on this particular run.

Reports median / p95 / worst age and the fraction of samples past each tolerance,
plus the longest continuous stretch with no new stamp at all -- the stall length,
which is the number that has to stay under transform_timeout.
"""
import sys
import time

import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    # Frames are arguments because map->odom turned out NOT to be the first link that
    # breaks. slam_toolbox's scan callback runs behind a tf2 MessageFilter waiting on
    # odom->base_laser, and that filter was observed evicting scans with "queue is
    # full" -- so odom->base_link, published by rf2o, needs the same scrutiny.
    parent = sys.argv[3] if len(sys.argv) > 3 else "map"
    child = sys.argv[4] if len(sys.argv) > 4 else "odom"

    rclpy.init()
    node = rclpy.create_node("tf_age")
    buf = Buffer()
    TransformListener(buf, node, spin_thread=True)
    time.sleep(2.0)

    ages, stamps = [], []
    misses = 0
    t0 = time.time()
    while time.time() - t0 < dur:
        try:
            # Time() = "latest available", so .header.stamp is the newest stamp held.
            tr = buf.lookup_transform(parent, child, rclpy.time.Time())
            st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            age = node.get_clock().now().nanoseconds * 1e-9 - st
            ages.append(age)
            stamps.append((time.time() - t0, st, age))
        except Exception:
            misses += 1
        time.sleep(1.0 / hz)

    node.destroy_node()
    rclpy.shutdown()

    if not ages:
        print(f"tf_age: NO {parent}->{child} at all ({misses} failed lookups).")
        return 1

    ages.sort()
    n = len(ages)
    med = ages[n // 2]
    p95 = ages[min(n - 1, int(0.95 * n))]
    worst = ages[-1]

    # longest stretch during which the stamp never changed = a scan-processing stall
    longest, run_start, prev = 0.0, stamps[0][0], stamps[0][1]
    for t, st, _ in stamps:
        if st != prev:
            longest = max(longest, t - run_start)
            run_start, prev = t, st
    longest = max(longest, stamps[-1][0] - run_start)

    def frac(lim):
        return 100.0 * sum(1 for a in ages if a > lim) / n

    print(f"{parent}->{child} stamp age over {dur:.0f}s, {n} samples ({misses} lookup misses)")
    print(f"  median {med:6.2f}s   p95 {p95:6.2f}s   worst {worst:6.2f}s")
    print(f"  past 0.5s (costmap transform_tolerance): {frac(0.5):5.1f}%")
    print(f"  past 1.0s (controller transform_tolerance): {frac(1.0):5.1f}%")
    print(f"  longest stall with NO new stamp: {longest:.2f}s")

    # The summary above cannot tell the two failure shapes apart, and they have
    # opposite fixes:
    #   SAWTOOTH  age climbs, then drops back to ~0 when a scan is finally processed.
    #             slam is keeping up; it is simply not being ASKED to run often enough
    #             (a motion/interval gate). Fix: loosen the gate.
    #   RAMP      age climbs and never comes back -- slam is consuming 10 Hz of scans
    #             more slowly than they arrive, so the stamp it publishes is the
    #             timestamp of an ever-older scan and the backlog grows without bound.
    #             Fix: feed it fewer scans (throttle_scans) or make each one cheaper.
    # One column per second of wall time, showing the age at that moment, makes the
    # shape obvious at a glance.
    print("\n  age timeline (one sample per second, s):")
    per_sec = {}
    for t, _, age in stamps:
        per_sec.setdefault(int(t), []).append(age)
    keys = sorted(per_sec)
    for row in range(0, len(keys), 30):
        chunk = keys[row:row + 30]
        print(f"    t={chunk[0]:4d}s  " +
              " ".join(f"{max(per_sec[k]):4.1f}" for k in chunk))
    return 0


if __name__ == "__main__":
    sys.exit(main())
