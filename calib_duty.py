#!/usr/bin/env python3
"""calib_duty.py  --  measure the motor duty deadband and the duty->yaw-rate curve.

For each duty in the sweep it pins the motor node to that exact duty (min_duty ==
max_duty, set live via ros2 params), commands a short in-place rotation, and measures
the actual yaw rate from rf2o. Rotation is used rather than forward motion because it
needs no floor space and it is the axis Nav2 is failing on.

What the output is for
----------------------
  deadband  -- the lowest duty that actually moves the chassis. Mapping Nav2's velocity
               range onto duties below this makes the rover silently ignore small
               commands, which reads as "it drifts past the goal and never trims".
  linearity -- whether yaw rate is roughly proportional to duty above the deadband. DWB
               assumes it is. If the curve is flat or wildly non-monotonic, the usable
               band is too narrow to steer in and the ceiling has to come up.

Safety: every pulse is hard-capped, a stop burst is sent in `finally`, and the motor
node's duty params are restored on exit. The node runs deadman:=0.0 (no watchdog) and
LATCHES its last command, so the stop burst is the only thing that halts the rover.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

MOTOR_NODE = "/smart_elex_15d"
PULSE_S = 0.6
SETTLE_S = 1.5


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    duties = [float(x) for x in sys.argv[1].split(",")] if len(sys.argv) > 1 else \
        [0.09, 0.11, 0.13, 0.15, 0.17, 0.20, 0.24, 0.271]

    rclpy.init()
    node = rclpy.create_node("calib_duty")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    buf = Buffer()
    TransformListener(buf, node, spin_thread=True)
    # The parameter client lives on a SEPARATE node on purpose. `node` was handed to
    # TransformListener(spin_thread=True), which puts it in its own dedicated executor;
    # calling rclpy.spin_until_future_complete(node, ...) then fights that executor for
    # the same node and the TF callbacks stop being serviced. The buffer keeps answering
    # lookups from its last cached transform, so every sample reads back an IDENTICAL
    # pose and the sweep reports 0.00 deg at every duty -- a dead-looking motor that is
    # in fact spinning fine. Verified: the same rotation measured -91 deg via spin_test
    # while this script called it 0.00 deg.
    pnode = rclpy.create_node("calib_duty_params")
    cli = pnode.create_client(SetParameters, f"{MOTOR_NODE}/set_parameters")
    if not cli.wait_for_service(timeout_sec=10.0):
        print(f"calib_duty: {MOTOR_NODE} set_parameters unavailable", file=sys.stderr)
        return 1
    time.sleep(2.0)

    def set_duty(d):
        req = SetParameters.Request()
        for name in ("min_duty", "max_duty"):
            req.parameters.append(Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=d)))
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(pnode, fut, timeout_sec=5.0)
        return fut.done()

    def yaw():
        try:
            t = buf.lookup_transform("odom", "base_link", rclpy.time.Time())
            return quat_yaw(t.transform.rotation)
        except Exception:
            return None

    # warm up DDS so the first pulse is not eaten by discovery (measured 1.79 s here)
    t_disc = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t_disc < 5.0:
        time.sleep(0.02)

    print(f"duty sweep, {PULSE_S}s in-place rotation each, rate from rf2o\n")
    print(f"  {'duty':>6}  {'rotated':>9}  {'rate':>10}   moved?")
    results = []
    try:
        for i, d in enumerate(duties):
            if not set_duty(d):
                print(f"  {d:6.3f}   <param set failed>")
                continue
            time.sleep(0.3)
            y0 = yaw()
            # alternate direction each step so the sweep unwinds itself instead of
            # spinning the rover through several full turns in one direction
            msg = Twist(); msg.angular.z = -0.5 if i % 2 == 0 else 0.5
            t0 = time.time()
            while time.time() - t0 < PULSE_S:
                pub.publish(msg)
                time.sleep(0.05)
            stop = Twist()
            for _ in range(30):
                pub.publish(stop)
                time.sleep(0.02)
            time.sleep(SETTLE_S)
            y1 = yaw()
            if None in (y0, y1):
                print(f"  {d:6.3f}   <no tf>")
                continue
            deg = math.degrees(wrap(y1 - y0))
            rate = deg / PULSE_S
            moved = abs(deg) > 3.0                   # 3 deg > rf2o noise over ~2 s
            results.append((d, deg, rate, moved))
            print(f"  {d:6.3f}  {deg:8.2f}d  {rate:8.1f}d/s   {'YES' if moved else 'no'}")
    finally:
        stop = Twist()
        for _ in range(40):
            pub.publish(stop)
            time.sleep(0.02)

    moved = [r for r in results if r[3]]
    print()
    if moved:
        print(f"deadband: lowest duty that moved = {moved[0][0]:.3f}"
              f"  ({moved[0][2]:.1f} deg/s)")
        print(f"ceiling : {results[-1][0]:.3f} -> {results[-1][2]:.1f} deg/s")
        if len(moved) >= 2:
            lo, hi = moved[0], moved[-1]
            print(f"usable band: {lo[0]:.3f}..{hi[0]:.3f} duty  "
                  f"-> {abs(lo[2]):.1f}..{abs(hi[2]):.1f} deg/s "
                  f"({abs(hi[2]) / max(abs(lo[2]), 1e-6):.1f}x range)")
    else:
        print("NOTHING MOVED at any duty in the sweep -- check power/wiring before tuning.")

    pnode.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
