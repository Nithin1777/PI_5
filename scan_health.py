#!/usr/bin/env python3
"""scan_health.py [seconds] [--topics /scan_raw,/scan]  --  is the lidar dropping out?

The rover was reported to "read 0.0 at random intervals when nav is on". Three very
different faults produce that symptom and they have nothing in common:

  DRIVER      the LD19 itself emits frames whose ranges are literally 0.0 (its
              protocol uses 0 for "no return" -- fog, black surface, out of range,
              or a dropped serial packet re-assembled wrong). Shows up on /scan_raw.
  NORMALIZER  /scan_raw is clean but /scan is not, i.e. scan_normalize.py is the one
              manufacturing the zeros/holes -- a binning or masking bug.
  STARVATION  neither topic contains 0.0, but frames stop arriving for a while and
              consumers extrapolate or render the gap as zero. Shows up as an
              inter-frame gap, not as a bad value.

Only per-frame accounting separates them, so this records, for EVERY frame on every
watched topic: how many ranges are exactly 0.0, how many are NaN/inf, how many are
finite and positive, the beam count, and the wall gap since the previous frame on
that topic. Both topics are watched simultaneously so a bad frame can be attributed
to one side of scan_normalize or the other.

Prints a line only for ABNORMAL frames (any exact zero, a beam-count change, or a
gap over --gap seconds) so a clean run is quiet and a dropout is obvious, then a
summary. Run it once with the base stack alone and once during nav; the difference
is the answer.
"""
import argparse
import math
import statistics
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seconds", nargs="?", type=float, default=60.0)
    ap.add_argument("--topics", default="/scan_raw,/scan")
    ap.add_argument("--gap", type=float, default=0.35,
                    help="report a frame whose arrival gap exceeds this (s)")
    args = ap.parse_args()
    topics = [t.strip() for t in args.topics.split(",") if t.strip()]

    rclpy.init()
    node = rclpy.create_node("scan_health")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST)

    st = {t: {"n": 0, "zero_frames": 0, "zeros": 0, "gaps": [], "last": None,
              "beams": set(), "valid": [], "worst_gap": 0.0, "empty_frames": 0,
              "stamp_lag": []} for t in topics}
    t_start = time.time()

    def make_cb(topic):
        s = st[topic]

        def cb(msg: LaserScan):
            now = time.time()
            gap = None if s["last"] is None else now - s["last"]
            s["last"] = now
            if gap is not None:
                s["gaps"].append(gap)
                s["worst_gap"] = max(s["worst_gap"], gap)

            zeros = sum(1 for r in msg.ranges if r == 0.0)
            nans = sum(1 for r in msg.ranges if r != r or math.isinf(r))
            valid = len(msg.ranges) - zeros - nans
            s["n"] += 1
            s["zeros"] += zeros
            s["valid"].append(valid)
            s["beams"].add(len(msg.ranges))
            if zeros:
                s["zero_frames"] += 1
            if valid == 0:
                s["empty_frames"] += 1

            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            s["stamp_lag"].append(now - stamp)

            odd = (zeros > 0 or valid == 0 or len(s["beams"]) > 1
                   or (gap is not None and gap > args.gap))
            if odd:
                print(f"  {now - t_start:7.2f}s {topic:10s} beams={len(msg.ranges):4d} "
                      f"zero={zeros:4d} nan={nans:4d} valid={valid:4d} "
                      f"gap={'--' if gap is None else f'{gap:.3f}'}s", flush=True)
        return cb

    subs = [node.create_subscription(LaserScan, t, make_cb(t), qos) for t in topics]

    print(f"watching {topics} for {args.seconds:.0f}s "
          f"(printing only abnormal frames)\n", flush=True)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.2)

    print("\n=== summary ===")
    for t in topics:
        s = st[t]
        if s["n"] == 0:
            print(f"  {t:10s} NO FRAMES RECEIVED")
            continue
        hz = s["n"] / (time.time() - t0)
        gmean = statistics.mean(s["gaps"]) if s["gaps"] else float("nan")
        vmean = statistics.mean(s["valid"])
        vmin = min(s["valid"])
        lag = statistics.median(s["stamp_lag"])
        print(f"  {t:10s} {s['n']:5d} frames  {hz:5.2f} Hz  "
              f"beams={sorted(s['beams'])}")
        print(f"             gap mean {gmean:.3f}s worst {s['worst_gap']:.3f}s   "
              f"header lag median {lag:+.3f}s")
        print(f"             valid beams mean {vmean:.1f} min {vmin}   "
              f"frames with a 0.0: {s['zero_frames']}  total zeros {s['zeros']}  "
              f"fully empty frames {s['empty_frames']}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
