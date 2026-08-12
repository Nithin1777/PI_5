#!/usr/bin/env python3
"""One-shot: publish /initialpose from /root/maps/<name>.seed so AMCL starts
broadcasting map->odom. Uses TRANSIENT_LOCAL QoS to match AMCL's latched
subscription (a VOLATILE publisher gets rejected with "incompatible QoS ... DURABILITY").

Diagnostic helper -- return_home.py does the same thing itself before navigating.
Usage: seed_amcl.py [name]   (default name: room)
"""
import sys, math, time, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

name = sys.argv[1] if len(sys.argv) > 1 else "room"
x, y, yaw = (float(v) for v in open(f"/root/maps/{name}.seed").read().split())

rclpy.init()
n = rclpy.create_node("seed_amcl")
qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
p = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)

m = PoseWithCovarianceStamped()
m.header.frame_id = "map"
m.header.stamp = n.get_clock().now().to_msg()
m.pose.pose.position.x = x
m.pose.pose.position.y = y
m.pose.pose.orientation.z = math.sin(yaw / 2.0)
m.pose.pose.orientation.w = math.cos(yaw / 2.0)
cov = [0.0] * 36
cov[0] = cov[7] = cov[35] = 0.05
m.pose.covariance = cov

# confirm AMCL echoed back /amcl_pose
got = {"f": False}
n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda msg: got.__setitem__("f", True), 10)

time.sleep(1.0)
for _ in range(5):
    p.publish(m)
    rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(0.25)
print(f"published /initialpose ({x:.3f}, {y:.3f}, yaw {yaw:.3f}) x5 [TRANSIENT_LOCAL]")
t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
    if got["f"]:
        print("OK: AMCL published /amcl_pose -> localization confirmed.")
        break
else:
    print("WARNING: no /amcl_pose within 10s (AMCL may not have accepted the seed).")
n.destroy_node(); rclpy.shutdown()
