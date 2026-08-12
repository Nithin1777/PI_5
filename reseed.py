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
    # Distance transform: meters from each cell to the nearest occupied cell.
    # This is a uniform-cost BFS, so the FIRST visit to a cell is already its
    # shortest distance -- `seen` is the correct gate. The previous gate re-tested
    # `DT[nr,nc] > DT[r,c] + res`, which never settles here: under numpy 1.21
    # `float32 + python float` promotes to float64, so the stored float32 never
    # compares equal to the recomputed float64 and every cell got relaxed ~104x.
    # Measured on the Pi, same map, bit-identical output (max diff 0.000 m):
    #     old gate: 977,749 queue pops -> 23.44 s
    #     `seen`  :   9,396 queue pops ->  0.10 s   (248x)
    # That 23 s was over 90 % of every `nav` bring-up's reseed step.
    INF = 1e6
    DT = np.full((h, w), INF, dtype=np.float32)
    seen = np.zeros((h, w), dtype=bool)
    q = deque()
    occ_r, occ_c = np.where(d == OCC)
    for r, c in zip(occ_r.tolist(), occ_c.tolist()):
        DT[r, c] = 0.0
        seen[r, c] = True
        q.append((c, r))
    while q:
        c, r = q.popleft()
        nxt = DT[r, c] + res
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = c + dc, r + dr
            if 0 <= nc < w and 0 <= nr < h and not seen[nr, nc]:
                seen[nr, nc] = True
                DT[nr, nc] = nxt
                q.append((nc, nr))
    DT = np.minimum(DT, 2.0)
    return res, ox, oy, w, h, d, DT


def main():
    ap = argparse.ArgumentParser(description="Scan-match re-seed for slam_toolbox localization")
    ap.add_argument("--map-name", default="room")
    ap.add_argument("--threshold", type=float, default=0.12,
                    help="max acceptable TRIMMED-MEAN endpoint-to-wall error (m) over the "
                         "beams that land inside the map; gate fails above")
    ap.add_argument("--min-coverage", type=float, default=0.60,
                    help="min fraction of beams that must land inside the mapped area. This "
                         "is what stops a pose from scoring well by throwing most of the scan "
                         "outside a small/partial map.")
    ap.add_argument("--trim", type=float, default=0.10,
                    help="fraction of worst-fitting in-map beams to drop before averaging, so "
                         "a few moved/soft objects cannot veto a good pose")
    ap.add_argument("--clearance", type=float, default=0.14,
                    help="reject any candidate pose whose own cell is not mapped-free and at "
                         "least this far (m) from an obstacle. Default 0.14 = the footprint's "
                         "inscribed radius. Pass 0 to disable the check entirely.")
    ap.add_argument("--yaw-scan", action="store_true",
                    help="diagnostic: sweep cost vs yaw about the best pose to show how"
                         " sharply (or not) the map constrains heading")
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

    # FREE-SPACE CHECK. The scan-match cost only scores where the BEAM ENDPOINTS
    # land -- it says nothing about the cell the robot body would occupy. A pose
    # can therefore match the walls beautifully while sitting inside one: a run on
    # this map seeded (1.530, -0.460) at cost 0.039 with 52/52 beams in-map, and
    # that cell was LETHAL (cost 100) in the global costmap. NavFn aborts outright
    # when the start is lethal -> "failed to create plan with tolerance 0.50" on
    # every attempt, and Nav2 fell through to spin/backup recoveries. Requiring the
    # pose cell to be mapped-free and >= one inscribed radius from any obstacle
    # makes such a pose unrepresentable rather than merely unlikely.
    def pose_free(x, y):
        if args.clearance <= 0:
            return True
        col = int(round((x - ox) / res))
        row = int(round((ymax - y) / res))
        if not (0 <= col < w and 0 <= row < h):
            return False
        # must be mapped FREE -- not occupied, and not UNKNOWN either, or the
        # search happily parks the rover in the unmapped void outside the room,
        # where DT is large and nothing contradicts it.
        return d[row, col] == FREE and DT[row, col] >= args.clearance

    # coarse then fine search.
    # NOTE: these are the SEARCH BOUNDS -- they must NOT be named `ymax`. `cost()`
    # reads `ymax` (the map's top edge, set above) at call time to convert a beam
    # endpoint to a pixel row, so shadowing it here shifted the whole map 0.4 m in
    # y for every cost evaluation: the search then returned (true_y - 0.4) with an
    # IDENTICAL cost, so the consistency gate could never catch it and nav started
    # every run half a metre off (often worse, once the shift moved the global
    # optimum into a neighbouring basin).
    xs_min, xs_max = ox + 0.4, ox + w * res - 0.4
    ys_min, ys_max = oy + 0.4, oy + h * res - 0.4
    best = (1e9, None, None, None)
    for x in np.arange(xs_min, xs_max, 0.25):
        for y in np.arange(ys_min, ys_max, 0.25):
            if not pose_free(x, y):
                continue
            for yaw in np.arange(0.0, 2 * math.pi, math.radians(15)):
                c = cost(x, y, yaw)
                if c < best[0]:
                    best = (c, x, y, yaw)
    if best[1] is None:
        print("[reseed] ERROR: no candidate pose sits in mapped free space with %.2f m clearance."
              % args.clearance, file=__import__("sys").stderr)
        print("[reseed]   The map has no open region the rover fits in -- re-map the room, or"
              " lower --clearance if the footprint really is smaller.", file=__import__("sys").stderr)
        n.destroy_node(); rclpy.shutdown()
        return 2
    c0, x0, y0, y0a = best
    for step, yaw_step, span in ((0.05, math.radians(3), 0.3), (0.02, math.radians(1), 0.1)):
        for x in np.arange(x0 - span, x0 + span + 1e-9, step):
            for y in np.arange(y0 - span, y0 + span + 1e-9, step):
                if not pose_free(x, y):
                    continue          # refine must not walk the pose back into a wall
                for yaw in np.arange(y0a - 3 * yaw_step, y0a + 3 * yaw_step + 1e-9, yaw_step):
                    c = cost(x, y, yaw)
                    if c < c0:
                        c0, x0, y0, y0a = c, x, y, yaw

    # coverage: how much of the scan actually falls inside the mapped area at the
    # best pose (out-of-map beams are what the OUT_PENALTY rejects)
    # GATE. The search objective above keeps OUT_PENALTY so a pose still cannot cheat
    # by pushing the scan off-map -- but that same penalty must NOT drive the gate.
    # Averaging a hard 2.0 m into the score makes a CORRECT pose look terrible on a
    # small map: measured on a 3.5x3.7 m single-scan map, 4 out-of-map beams of 41
    # contributed 0.195 m of a 0.296 m score (66%) while the in-map median error was
    # 0.000 m and 25 of 37 beams sat within 0.10 m. The gate rejected a pose that was
    # within 5 cm of the saved seed. A handful of long outliers (0.5-1.1 m: soft
    # furnishings, or something that moved between mapping and nav) skewed it further.
    #
    # So the gate now judges the two things separately, which is also what each one
    # actually means:
    #   coverage      -- did the scan land in the map at all? (the anti-degenerate
    #                    check OUT_PENALTY was really there to enforce)
    #   trimmed mean  -- of the beams that DID land in the map, how well do they fit,
    #                    ignoring the worst `--trim` fraction so a few moved objects
    #                    cannot veto an otherwise perfect match.
    ins = []
    for a, r in beams:
        th = y0a + a
        col = int(round((x0 + r * math.cos(th) - ox) / res))
        row = int(round((ymax - (y0 + r * math.sin(th))) / res))
        if 0 <= col < w and 0 <= row < h:
            ins.append(float(DT[row, col]))
    cov_in = len(ins)
    coverage = cov_in / len(beams)
    ins.sort()
    keep = max(1, int(round(len(ins) * (1.0 - args.trim)))) if ins else 0
    fit = sum(ins[:keep]) / keep if keep else float("inf")

    print("[reseed] best scan-match cost %.3f m at (%.3f, %.3f, yaw %.1f deg)"
          % (c0, x0, y0, math.degrees(y0a)))
    print("[reseed] scan coverage inside map: %d/%d beams (%.0f%%)"
          % (cov_in, len(beams), 100 * coverage))
    print("[reseed] in-map fit: trimmed mean %.3f m (median %.3f m) over %d of %d beams"
          % (fit, ins[len(ins) // 2] if ins else float("nan"), keep, cov_in))
    print("POSE %.4f %.4f %.4f" % (x0, y0, y0a))

    # ROTATIONAL CONDITIONING. A low cost only means "this pose fits". It says nothing
    # about whether NEIGHBOURING poses fit just as well -- and if they do, the match is
    # not actually pinned down, it just happens to have won. Yaw is where that bites on
    # this rover: a small, roughly rectangular room seen from near its centre looks much
    # the same rotated a few degrees, so the cost surface can be almost flat in yaw while
    # being sharp in x/y. Measured on a parked rover, map->odom rotated 23 deg over three
    # minutes with the robot untouched -- slam_loc sliding along exactly such a flat
    # direction -- against a yaw_goal_tolerance of 11.5 deg.
    #
    # This sweeps yaw about the winner with x/y held at the optimum and reports how far
    # you must rotate before the cost rises by a meaningful margin. A wide "flat band"
    # means absolute heading is not observable from this map and no amount of gate
    # tightening will make it so.
    if args.yaw_scan:
        base = cost(x0, y0, y0a)
        margin = max(0.02, 0.5 * args.threshold)     # "meaningfully worse" than the best
        print("[reseed] yaw conditioning sweep about the best pose (cost is mean m):")
        band = []
        for d in range(-30, 31, 2):
            c = cost(x0, y0, y0a + math.radians(d))
            flat = (c - base) < margin
            if flat:
                band.append(d)
            print("           %+4d deg  cost %.4f  %s%s"
                  % (d, c, "flat" if flat else "", "   <-- best" if d == 0 else ""))
        if band:
            print("[reseed] cost stays within %.3f m of the optimum over %+d..%+d deg (%d deg wide)."
                  % (margin, min(band), max(band), max(band) - min(band)))
            if (max(band) - min(band)) > 12:
                print("[reseed]   WARNING: heading is UNDER-CONSTRAINED by this map. The scan match"
                      " cannot pin yaw tighter than that band, so yaw_goal_tolerance and any"
                      " heading-dependent behaviour are running on noise.")

    sys_err = __import__("sys").stderr
    if coverage < args.min_coverage:
        print("[reseed] GATE FAIL: only %.0f%% of the scan lands inside the map (need %.0f%%)."
              % (100 * coverage, 100 * args.min_coverage), file=sys_err)
        print("[reseed]   The rover is probably outside the mapped area, or the map is a"
              " fragment. Re-map this room, then 'map save' again.", file=sys_err)
        n.destroy_node(); rclpy.shutdown()
        return 2
    if fit > args.threshold:
        print("[reseed] GATE FAIL: in-map fit %.2f m > threshold %.2f m -- the live lidar does not"
              % (fit, args.threshold), file=sys_err)
        print("[reseed]   agree with the saved map anywhere. Re-map this room (or check the lidar),"
              " then 'map save' again.", file=sys_err)
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
