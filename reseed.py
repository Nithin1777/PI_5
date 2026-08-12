#!/usr/bin/env python3
"""reseed.py [--map-name NAME] [--threshold M] [--no-publish] [--timeout S]

Scan-match re-seed for slam_toolbox LOCALIZATION mode (and a consistency gate).

WHY: AMCL failed to converge on this robot's map (wandered / flipped 180 deg), so
the published map->odom transform disagreed with the raw lidar. slam_toolbox
localization is now the map->odom owner, but it needs an accurate initial pose.
This script finds it deterministically by scan-matching the live /scan against the
saved map (the same method that pinpointed the ~2-3 cm match in issue.md):

  1. load /root/maps/<name>.{pgm,yaml}, build a distance-transform likelihood field
  2. capture a fresh /scan
  3. coarse search (0.25 m / 15 deg) then refine (0.02 m / 1 deg) for the pose with
     the lowest mean endpoint-to-wall distance ("cost")
  4. print the pose and publish /initialpose (TRANSIENT_LOCAL, tight cov) repeatedly
  5. consistency gate: if the best cost exceeds --threshold (default 0.12 m), the
     map and the live lidar do not agree anywhere -> exit 2 so nav ABORTS instead
     of driving against a wrong pose.

Exit codes: 0 = seeded OK, 1 = hard error (no map/scan), 2 = gate failed (map vs
lidar inconsistent). Output line "POSE x y yaw" is parseable for logs.
"""
import argparse
import math
import re
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

MAPS = "/root/maps"
FREE, UNK, OCC = 254, 205, 0


def load_map(name):
    yaml_txt = open(f"{MAPS}/{name}.yaml").read()
    res = float(re.search(r"resolution:\s*([\d.]+)", yaml_txt).group(1))
    m = re.search(r"origin:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)", yaml_txt)
    ox, oy = float(m.group(1)), float(m.group(2))
    with open(f"{MAPS}/{name}.pgm", "rb") as f:
        f.readline(); line = f.readline()
        while line.startswith(b"#"): line = f.readline()
        w, h = map(int, line.split()); f.readline()
        d = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    # distance transform: meters from each cell to the nearest occupied cell
    INF = 1e6
    DT = np.full((h, w), INF, dtype=np.float32)
    q = deque()
    for r in range(h):
        for c in range(w):
            if d[r, c] == OCC:
                DT[r, c] = 0.0
                q.append((c, r))
    while q:
        c, r = q.popleft()
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = c + dc, r + dr
            if 0 <= nc < w and 0 <= nr < h and DT[nr, nc] > DT[r, c] + res:
                DT[nr, nc] = DT[r, c] + res
                q.append((nc, nr))
    DT = np.minimum(DT, 2.0)
    return res, ox, oy, w, h, d, DT


def main():
    ap = argparse.ArgumentParser(description="Scan-match re-seed for slam_toolbox localization")
    ap.add_argument("--map-name", default="room")
    ap.add_argument("--threshold", type=float, default=0.12,
                    help="max acceptable mean endpoint-to-wall error (m); gate fails above")
    ap.add_argument("--no-publish", action="store_true", help="only print the pose, do not publish")
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for a /scan")
    args = ap.parse_args()

    res, ox, oy, w, h, d, DT = load_map(args.map_name)
    ymax = oy + h * res

    rclpy.init()
    n = rclpy.create_node("reseed")

    scan = {"msg": None}
    n.create_subscription(LaserScan, "/scan",
                          lambda m: scan.__setitem__("msg", m), 10)
    t0 = time.time()
    while scan["msg"] is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
    if scan["msg"] is None:
        print("[reseed] ERROR: no /scan within %.0f s (base up?)" % args.timeout, file=__import__("sys").stderr)
        return 1
    m = scan["msg"]

    # subsampled beams (every 8th), valid 0.1-2.8 m (LD19 practical range)
    beams = []
    for i in range(0, len(m.ranges), 8):
        r = m.ranges[i]
        if r == r and 0.1 <= r <= 2.8:
            beams.append((m.angle_min + i * m.angle_increment, r))

    # OUT_PENALTY: cost assigned to a beam whose endpoint falls OUTSIDE the mapped
    # area. Skipping such beams (as the original did) let a spurious pose score a
    # tiny mean cost by putting most of the scan outside a small/partial map --
    # e.g. on a degenerate single-wall map it locked 3 m / 284 deg off the truth
    # at 0.046 m. Every beam now counts: inside == DT value (0 at a wall), outside
    # == OUT_PENALTY (same as the 2.0 m DT cap), so a correct match must fit the
    # whole scan inside the map.
    OUT_PENALTY = 2.0

    def cost(x, y, yaw):
        tot = 0.0
        for a, r in beams:
            th = yaw + a
            col = int(round((x + r * math.cos(th) - ox) / res))
            row = int(round((ymax - (y + r * math.sin(th))) / res))
            if 0 <= col < w and 0 <= row < h:
                tot += DT[row, col]
            else:
                tot += OUT_PENALTY
        return tot / len(beams)

    # coarse then fine search
    xmin, xmax = ox + 0.4, ox + w * res - 0.4
    ymin, ymax = oy + 0.4, oy + h * res - 0.4
    best = (1e9, None, None, None)
    for x in np.arange(xmin, xmax, 0.25):
        for y in np.arange(ymin, ymax, 0.25):
            for yaw in np.arange(0.0, 2 * math.pi, math.radians(15)):
                c = cost(x, y, yaw)
                if c < best[0]:
                    best = (c, x, y, yaw)
    c0, x0, y0, y0a = best
    for step, yaw_step, span in ((0.05, math.radians(3), 0.3), (0.02, math.radians(1), 0.1)):
        for x in np.arange(x0 - span, x0 + span + 1e-9, step):
            for y in np.arange(y0 - span, y0 + span + 1e-9, step):
                for yaw in np.arange(y0a - 3 * yaw_step, y0a + 3 * yaw_step + 1e-9, yaw_step):
                    c = cost(x, y, yaw)
                    if c < c0:
                        c0, x0, y0, y0a = c, x, y, yaw

    # coverage: how much of the scan actually falls inside the mapped area at the
    # best pose (out-of-map beams are what the OUT_PENALTY rejects)
    cov_in = 0
    for a, r in beams:
        th = y0a + a
        col = int(round((x0 + r * math.cos(th) - ox) / res))
        row = int(round((ymax - (y0 + r * math.sin(th))) / res))
        if 0 <= col < w and 0 <= row < h:
            cov_in += 1
    print("[reseed] best scan-match cost %.3f m at (%.3f, %.3f, yaw %.1f deg)"
          % (c0, x0, y0, math.degrees(y0a)))
    print("[reseed] scan coverage inside map: %d/%d beams" % (cov_in, len(beams)))
    print("POSE %.4f %.4f %.4f" % (x0, y0, y0a))
    if c0 > args.threshold:
        print("[reseed] GATE FAIL: best match %.2f m > threshold %.2f m -- the live lidar does not"
              % (c0, args.threshold), file=__import__("sys").stderr)
        print("[reseed]   agree with the saved map anywhere. Re-map this room (or check the lidar),"
              " then 'map save' again.", file=__import__("sys").stderr)
        n.destroy_node(); rclpy.shutdown()
        return 2

    if not args.no_publish:
        pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose",
                                 QoSProfile(depth=1,
                                            reliability=QoSReliabilityPolicy.RELIABLE,
                                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = x0, y0
        msg.pose.pose.orientation.z = math.sin(y0a / 2)
        msg.pose.pose.orientation.w = math.cos(y0a / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05
        cov[35] = 0.05
        msg.pose.covariance = cov
        t1 = time.time()
        while time.time() - t1 < 6:          # republish through slam's initialization window
            msg.header.stamp = n.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(n, timeout_sec=0.1)
            time.sleep(0.15)
        print("[reseed] published /initialpose at (%.3f, %.3f, yaw %.1f deg) x~40 (TRANSIENT_LOCAL)"
              % (x0, y0, math.degrees(y0a)))

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
