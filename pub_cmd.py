#!/usr/bin/env python3
"""Publish a Twist on /cmd_vel for a duration, then stop.
Usage: pub_cmd.py <seconds> <linear_x_m/s> <angular_z_rad/s>
Example: pub_cmd.py 5 0.25 0.0   # forward 5s
         pub_cmd.py 4 0.0 0.5    # spin-in-place 4s
"""
import sys, time, rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('cmd_pulser')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
vx   = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
wz   = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

# DO NOT use node.create_rate()/rate.sleep() here. rclpy's Rate is driven by a timer
# that only fires while the node is being spun, and nothing spins this node -- so the
# FIRST rate.sleep() blocks forever. Verified on the Pi: `timeout 15 pub_cmd.py 3 0 0`
# never returned and left 8 stuck processes behind, and `timeout`'s SIGTERM could not
# kill them because the block is inside rclpy and never processes signals.
#
# That is a genuine hazard, not just an annoyance: the motor node runs with
# deadman:=0.0 (no watchdog) and LATCHES its last command, so hanging here after
# publishing a NONZERO Twist leaves the rover driving indefinitely, un-interruptible
# by Ctrl-C. Hence plain time.sleep(), a hard cap, and the stop burst in a `finally`
# so it is sent on every exit path including an exception or a signal.
DRIVE = min(secs, 10.0)
msg = Twist(); msg.linear.x = vx; msg.angular.z = wz
t0 = time.time()
try:
    while rclpy.ok() and time.time() - t0 < DRIVE:
        pub.publish(msg)
        time.sleep(0.05)
finally:
    stop = Twist()
    for _ in range(40):           # ~0.8 s of zeros so the driver latches STOP
        pub.publish(stop)
        time.sleep(0.02)
    node.destroy_node(); rclpy.shutdown()
    print(f"published vx={vx} wz={wz} for {min(secs, 10.0):.1f}s, then stop")
