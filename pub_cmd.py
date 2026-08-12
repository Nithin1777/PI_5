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

msg = Twist(); msg.linear.x = vx; msg.angular.z = wz
rate = node.create_rate(20)
t0 = time.time()
while rclpy.ok() and time.time() - t0 < secs:
    pub.publish(msg)
    rate.sleep()
stop = Twist()
for _ in range(10):           # send a handful of zero cmds so it stops cleanly
    pub.publish(stop)
    rate.sleep()
node.destroy_node(); rclpy.shutdown()
print(f"published vx={vx} wz={wz} for {secs:.1f}s, then stop")
