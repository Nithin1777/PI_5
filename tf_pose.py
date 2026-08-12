#!/usr/bin/env python3
"""tf_pose.py <target_frame> <source_frame> [outfile]

Look up the pose of <source_frame> in <target_frame> via TF2 and print
"x y yaw" (yaw in radians). If outfile is given, also write that line to it.

Used by:  stack.sh  'map save'  ->  tf_pose.py map base_link /root/maps/<name>.seed
so the navigation phase can seed AMCL at the exact pose the rover had when
mapping stopped (no manual "2D Pose Estimate" needed).
"""
import sys
import math
import time
import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def quat_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    if len(sys.argv) < 3:
        print("usage: tf_pose.py <target_frame> <source_frame> [outfile]", file=sys.stderr)
        return 2
    target, source = sys.argv[1], sys.argv[2]
    outfile = sys.argv[3] if len(sys.argv) > 3 else None

    rclpy.init()
    node = rclpy.create_node('tf_pose')
    buf = Buffer()
    # spin_thread=True is LOAD-BEARING, not a tidy-up.
    #
    # The buffer is filled only by this listener's /tf and /tf_static subscription
    # callbacks, which need the node spinning. This used to be spin_thread=False
    # with a rclpy.spin_once() call on THIS thread followed by the *blocking* form
    # of lookup_transform (timeout=2 s). That deadlocks: the lookup sleeps waiting
    # for buffer data that can only arrive from a spin, and the spin cannot run
    # until the lookup returns. Every attempt burned the full 20 s and reported
    # "no transform map -> base_link", so 'map save' NEVER captured a localization
    # seed -- issue.md's recurring "could not capture pose", which looked like a
    # TF/slam fault but reproduced 100% while `ros2 run tf2_ros tf2_echo map
    # base_link` resolved instantly on the very same TF tree.
    #
    # With spin_thread=True the listener owns a SingleThreadedExecutor on its own
    # thread, so the buffer fills while the main thread blocks in the lookup.
    tfl = TransformListener(buf, node, spin_thread=True)

    pose = None
    deadline = time.time() + 20.0                 # up to ~20 s, retrying
    while time.time() < deadline:
        try:
            # blocking lookup with 2 s timeout: absorbs a transient map->odom
            # blip (slam_loc/rf2o under load) instead of failing 'map save'
            t = buf.lookup_transform(target, source, rclpy.time.Time(),
                                     timeout=rclpy.duration.Duration(seconds=2.0))
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = quat_yaw(t.transform.rotation)
            pose = (x, y, yaw)
            break
        except Exception:
            time.sleep(0.1)

    # Stop the listener's executor and join its (non-daemon) thread BEFORE
    # rclpy.shutdown(), or spin() raises out of that thread and dumps a traceback
    # onto stderr after the pose has already been printed.
    try:
        tfl.executor.shutdown()
        tfl.dedicated_listener_thread.join(timeout=2.0)
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()

    if pose is None:
        print(f"tf_pose: no transform {target} -> {source} after 20 s", file=sys.stderr)
        return 1

    line = f"{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f}"
    if outfile:
        with open(outfile, 'w') as f:
            f.write(line + "\n")
        print(f"tf_pose: {target}->{source} = {line}  (saved to {outfile})")
    else:
        print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
