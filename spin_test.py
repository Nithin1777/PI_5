#!/usr/bin/env python3
"""spin_test.py [wz] [secs]  --  measure rotational OVERSHOOT after the stop command.

Commands a rotation for `secs`, then publishes zeros, and keeps sampling odom->base_link
for several seconds afterwards. Prints the yaw trace so you can separate:

    commanded rotation   -- what turned while a nonzero Twist was being published
    coast                -- what KEPT turning after the stop command went out

Why this matters here
---------------------
Nav2's SimpleGoalChecker decides success from the pose at the instant it samples. If the
rover coasts a long way after the controller stops commanding, the checker can legitimately
see "within yaw_goal_tolerance", report success, and the machine then rolls to rest well
outside tolerance. That reads to a user as "nav says it arrived but it's pointing the wrong
way", and it is intermittent because the coast depends on speed, floor grip and battery.

The x/y case of this is already known on this rover -- nav2_params.yaml carries the note
that xy_goal_tolerance was once 0.40 "to absorb the 2 s stop overshoot". This measures the
same effect in yaw, which nothing has quantified yet.

Uses rf2o (odom->base_link) rather than map->base_link deliberately: over a few seconds
rf2o is smooth and accurate, while map->base_link carries slam's episodic jumps.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    wz = float(sys.argv[1]) if len(sys.argv) > 1 else -0.5
    secs = min(float(sys.argv[2]) if len(sys.argv) > 2 else 0.6, 2.0)   # hard cap

    rclpy.init()
    node = rclpy.create_node("spin_test")
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

    # Same discovery wait as pub_cmd.py -- without it a short pulse is spent on DDS
    # discovery and the "commanded" phase is silently shorter than requested.
    t_disc = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t_disc < 5.0:
        time.sleep(0.02)

    trace = []
    y_start = yaw()
    msg = Twist(); msg.angular.z = wz
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            pub.publish(msg)
            y = yaw()
            if y is not None:
                trace.append((time.time() - t0, y, "cmd"))
            time.sleep(0.05)
    finally:
        stop = Twist()
        for _ in range(40):
            pub.publish(stop)
            time.sleep(0.02)
    t_stop = time.time() - t0
    y_at_stop = yaw()

    # coast phase: nothing is being commanded, anything that moves is overshoot
    t1 = time.time()
    while time.time() - t1 < 6.0:
        y = yaw()
        if y is not None:
            trace.append((time.time() - t0, y, "coast"))
        time.sleep(0.1)
    y_end = yaw()

    print(f"spin_test: wz={wz} for {secs:.2f}s, stop command sent at t={t_stop:.2f}s\n")
    for t, y, ph in trace[::4]:
        print(f"  t={t:5.2f}s  {ph:5s}  yaw {math.degrees(wrap(y - y_start)):8.2f} deg")
    if None not in (y_start, y_at_stop, y_end):
        cmd = math.degrees(wrap(y_at_stop - y_start))
        coast = math.degrees(wrap(y_end - y_at_stop))
        total = math.degrees(wrap(y_end - y_start))
        print(f"\n  commanded rotation : {cmd:8.2f} deg")
        print(f"  coast after stop   : {coast:8.2f} deg"
              f"   ({abs(coast) / abs(total) * 100:.0f}% of total)" if total else "")
        print(f"  total rotation     : {total:8.2f} deg")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
