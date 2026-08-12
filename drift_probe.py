#!/usr/bin/env python3
"""drift_probe.py [seconds] [hz]  --  measure pose drift on a STATIONARY rover.

Samples map->base_link, odom->base_link and map->odom from one long-lived node and
reports how far each moved. Run it with the rover physically parked and untouched:
anything it reports is error, by definition, because the true pose is constant.

Why this exists
---------------
Repeated `tf_pose.py` calls cannot measure this. Each call spins up a node, waits for
TF discovery and exits (~5 s), so the sample cadence is coarse and irregular, and the
process churn sits right on top of the thing being measured. This holds one listener
open and samples on a fixed clock.

Splitting the chain matters more than the total:
  map->base_link  = what Nav2's goal checker actually evaluates
  odom->base_link = rf2o, the dead-reckoning source
  map->odom       = slam_toolbox's correction
If map->odom is CONSTANT while odom->base_link moves, rf2o is drifting and slam is not
correcting it -- which is the expected state while parked, since slam_loc gates scan
processing on minimum_travel_distance/minimum_travel_heading and a parked robot never
trips either. The drift is invisible to the corrector by construction.
"""
import math
import sys
import time

import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

PAIRS = [("map", "base_link"), ("odom", "base_link"), ("map", "odom")]


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    rclpy.init()
    node = rclpy.create_node("drift_probe")
    buf = Buffer()
    # spin_thread=True for the same reason tf_pose.py needs it: the buffer is only
    # filled by callbacks, and the main thread here is busy sleeping between samples.
    TransformListener(buf, node, spin_thread=True)
    time.sleep(2.0)                       # let /tf and /tf_static populate

    series = {p: [] for p in PAIRS}
    t0 = time.time()
    while time.time() - t0 < dur:
        t = time.time() - t0
        for pair in PAIRS:
            try:
                tr = buf.lookup_transform(pair[0], pair[1], rclpy.time.Time())
                series[pair].append((t,
                                     tr.transform.translation.x,
                                     tr.transform.translation.y,
                                     quat_yaw(tr.transform.rotation)))
            except Exception:
                pass
        time.sleep(1.0 / hz)

    print(f"drift_probe: {dur:.0f}s @ {hz:.1f}Hz, rover assumed STATIONARY\n")
    for pair in PAIRS:
        s = series[pair]
        if len(s) < 2:
            print(f"{pair[0]:>5} -> {pair[1]:<10}  NO DATA")
            continue
        (_, x0, y0, a0), (tn, xn, yn, an) = s[0], s[-1]
        dxy = math.hypot(xn - x0, yn - y0)
        dyaw = wrap(an - a0)
        yaws = [w[3] for w in s]
        span = wrap(max(yaws) - min(yaws))
        frozen = all(abs(w[1] - x0) < 1e-9 and abs(w[2] - y0) < 1e-9
                     and abs(w[3] - a0) < 1e-9 for w in s)
        print(f"{pair[0]:>5} -> {pair[1]:<10} n={len(s):4d}  "
              f"{'FROZEN (bit-identical)' if frozen else ''}")
        print(f"        net xy {dxy*100:7.2f} cm   net yaw {math.degrees(dyaw):7.2f} deg   "
              f"yaw peak-to-peak {math.degrees(span):6.2f} deg")
        if tn > 0:
            print(f"        rate    {dxy/tn*60*100:7.2f} cm/min  "
                  f"{math.degrees(dyaw)/tn*60:7.2f} deg/min")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
