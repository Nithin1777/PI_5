#!/usr/bin/env python3
"""pos_check.py [map] [reps] [drive_s] [vx]  --  is the POSITION belief wrong, or is the
rover physically not where it should be?

Nav2 declared "goal reached" three times in a row while an absolute scan-match put the
rover 0.238 m, then 0.216 m, then 0.100 m from the goal, against a 0.15 m tolerance. Two
completely different faults produce that log and they have opposite fixes:

  BELIEF WRONG    map->base_link says (0,0) while the rover is really 0.24 m away.
                  Nav2 is driving correctly to a target that is in the wrong place.
                  Fix lives in localization (slam_loc / reseed / rf2o).
  CONTROL WRONG   the belief is right and the chassis physically cannot stop inside
                  0.15 m -- it stalls on stiction, breaks free, and lurches past.
                  Fix lives in the controller / motor limits.

Deciding between them needs three independent numbers over the SAME motion, which is
what this collects:
  d_belief   change in map->base_link      (what Nav2's goal checker consults)
  d_odom     change in odom->base_link     (rf2o dead-reckoning, no map involved)
  d_reseed   change in reseed's absolute pose (live scan vs the saved .pgm, from scratch)

Reading the result:
  * d_reseed ~= d_odom, but d_belief differs   -> slam's map->odom correction is the liar.
  * d_reseed differs from BOTH                 -> reseed is the liar, and every "true
                                                  pose" printed by return_home is suspect.
  * all three agree, and none is close to the  -> nobody is lying; the chassis simply
    commanded distance                            did not go where it was told.

Also reports reseed's repeatability while PARKED (reps samples at each stop). That
number bounds how much of any disagreement can be blamed on measurement noise -- if
reseed scatters by 0.10 m standing still, a 0.20 m "error" means much less than it looks.

Runs with the normal stack up; reseed is invoked with --no-publish so nothing is
corrected. This measures, it does not change anything.
"""
import math
import statistics
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def reseed_pose(map_name):
    cmd = [sys.executable, "/root/reseed.py", "--map-name", map_name, "--no-publish"]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=150).stdout
    except Exception as e:
        print(f"  reseed failed: {e}", file=sys.stderr)
        return None
    for line in out.splitlines():
        if line.startswith("POSE "):
            try:
                return tuple(float(v) for v in line.split()[1:4])
            except Exception:
                return None
    return None


def main():
    map_name = sys.argv[1] if len(sys.argv) > 1 else "room"
    reps = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    drive_s = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    vx = float(sys.argv[4]) if len(sys.argv) > 4 else 0.10

    rclpy.init()
    node = rclpy.create_node("pos_check")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    buf = Buffer()
    TransformListener(buf, node, spin_thread=True)
    time.sleep(2.0)

    def tf(parent, child):
        try:
            t = buf.lookup_transform(parent, child, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y,
                    quat_yaw(t.transform.rotation))
        except Exception:
            return None

    def stop_burst():
        s = Twist()
        for _ in range(40):
            pub.publish(s)
            time.sleep(0.02)

    def sample(label):
        belief = tf("map", "base_link")
        odom = tf("odom", "base_link")
        poses = []
        for i in range(reps):
            p = reseed_pose(map_name)
            if p is not None:
                poses.append(p)
                print(f"  reseed {i + 1}/{reps}: ({p[0]:+.3f}, {p[1]:+.3f}, "
                      f"{math.degrees(p[2]):+.1f} deg)")
        print(f"[{label}] belief map->base_link : "
              + (f"({belief[0]:+.3f}, {belief[1]:+.3f}, {math.degrees(belief[2]):+.1f} deg)"
                 if belief else "<none>"))
        print(f"[{label}] odom  odom->base_link : "
              + (f"({odom[0]:+.3f}, {odom[1]:+.3f}, {math.degrees(odom[2]):+.1f} deg)"
                 if odom else "<none>"))
        if len(poses) > 1:
            sx = statistics.pstdev([p[0] for p in poses])
            sy = statistics.pstdev([p[1] for p in poses])
            spread = max(math.hypot(a[0] - b[0], a[1] - b[1])
                         for a in poses for b in poses)
            print(f"[{label}] reseed scatter while parked: sd {sx:.3f}/{sy:.3f} m, "
                  f"max pairwise {spread:.3f} m")
        mean = None
        if poses:
            mean = (statistics.mean([p[0] for p in poses]),
                    statistics.mean([p[1] for p in poses]),
                    statistics.mean([p[2] for p in poses]))
            if belief:
                off = math.hypot(mean[0] - belief[0], mean[1] - belief[1])
                print(f"[{label}] BELIEF vs RESEED disagreement: {off:.3f} m  "
                      f"({mean[0] - belief[0]:+.3f}, {mean[1] - belief[1]:+.3f})")
        return belief, odom, mean

    try:
        print(f"=== stop A (parked) ===")
        bA, oA, rA = sample("A")

        print(f"\n=== driving forward {vx:.2f} m/s for {drive_s:.1f}s "
              f"(nominal {vx * drive_s:.2f} m) ===")
        msg = Twist()
        msg.linear.x = vx
        t_disc = time.time()
        while pub.get_subscription_count() == 0 and time.time() - t_disc < 5.0:
            time.sleep(0.02)
        t0 = time.time()
        while time.time() - t0 < min(drive_s, 10.0):
            pub.publish(msg)
            time.sleep(0.05)
        stop_burst()
        time.sleep(3.0)                # settle, and let slam process the travel

        print(f"\n=== stop B (after the drive) ===")
        bB, oB, rB = sample("B")
    finally:
        stop_burst()

    def delta(p, q):
        if p is None or q is None:
            return None
        return math.hypot(q[0] - p[0], q[1] - p[1])

    print("\n=== distance actually travelled, three ways ===")
    for name, d in (("d_belief (map->base_link)", delta(bA, bB)),
                    ("d_odom   (rf2o)", delta(oA, oB)),
                    ("d_reseed (scan vs saved map)", delta(rA, rB))):
        print(f"  {name:32s} " + (f"{d:.3f} m" if d is not None else "<none>"))
    print(f"  {'commanded':32s} {vx * drive_s:.3f} m")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
