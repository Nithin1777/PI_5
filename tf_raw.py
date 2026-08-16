#!/usr/bin/env python3
"""tf_raw.py [seconds] [parent] [child]  --  publish rate vs UNIQUE-STAMP rate on /tf.

Everything measured so far agrees that map->odom's age climbs at exactly 1.0 s per
second and snaps back every 5-22 s, and that this is unaffected by CPU (slam peaks at
58% of one core out of four), by the travel gate (0.05 -> 0.01 m changed nothing), by
motion (parked, rotating and driving all behave identically), and by the odometry it
depends on (odom->base_link is a clean 10 Hz at 0.16 s lag).

Two very different things still produce that trace, and no measurement so far separates
them because the TF *buffer* only ever reports its newest stamp:

  FROZEN STAMP   slam publishes at its configured 50 Hz, but every message carries the
                 SAME stamp until another scan is processed. Publish rate high, unique
                 stamp rate low. The scan pipeline into slam is the problem.
  SLOW PUBLISH   slam publishes rarely, each message carrying a FRESH stamp. Publish
                 rate low, and equal to the unique stamp rate. The publisher thread is
                 the problem.

Subscribing to /tf directly rather than going through a Buffer is what separates them:
it counts the messages themselves, not the buffer's summary of them. Reports both rates
side by side; whichever is low is the one that is broken.
"""
import sys
import time
from collections import Counter

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    parent = sys.argv[2] if len(sys.argv) > 2 else "map"
    child = sys.argv[3] if len(sys.argv) > 3 else "odom"

    rclpy.init()
    node = rclpy.create_node("tf_raw")
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST)

    msgs = 0
    stamps = []
    pairs = Counter()

    def cb(m: TFMessage):
        nonlocal msgs
        for tr in m.transforms:
            pairs[(tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"))] += 1
            if (tr.header.frame_id.lstrip("/") == parent
                    and tr.child_frame_id.lstrip("/") == child):
                msgs += 1
                stamps.append((time.time(),
                               tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9))

    node.create_subscription(TFMessage, "/tf", cb, qos)

    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = time.time() - t0

    print(f"=== /tf traffic over {elapsed:.0f}s ===")
    for (p, c), n in pairs.most_common():
        print(f"  {p:12s} -> {c:12s} {n:6d} msgs  {n / elapsed:7.2f} Hz")

    if not stamps:
        print(f"\n  no {parent}->{child} seen on /tf at all.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    uniq = sorted({s for _, s in stamps})
    print(f"\n=== {parent}->{child} ===")
    print(f"  published        {msgs:6d} msgs   {msgs / elapsed:7.2f} Hz")
    print(f"  UNIQUE stamps    {len(uniq):6d}        {len(uniq) / elapsed:7.2f} Hz")
    if len(uniq) > 1:
        deltas = [b - a for a, b in zip(uniq, uniq[1:])]
        deltas.sort()
        print(f"  stamp advance interval: median {deltas[len(deltas) // 2]:.2f}s  "
              f"max {deltas[-1]:.2f}s")
    if msgs / elapsed > 5 and len(uniq) / elapsed < 2:
        print("\n  FROZEN STAMP: slam is publishing fine but re-sending one old stamp.")
        print("  The fault is upstream of the publisher -- in scan processing.")
    elif msgs / elapsed < 5:
        print("\n  SLOW PUBLISH: the publisher thread itself is not keeping its period.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
