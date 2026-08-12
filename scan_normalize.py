#!/usr/bin/env python3
"""
scan_normalize.py  --  LD19 scan resampler/normalizer.

WHY THIS NODE EXISTS
--------------------
The LDROBOT LD19 publishes a VARIABLE number of range readings per 360-deg
frame -- measured 501..505 (and even 500) instead of a fixed count, with a
varying angle_increment and ~40 NaN beams thrown in.  slam_toolbox (karto)
expects a fixed count and prints:

    LaserRangeScan contains 503 range readings, expected 504

and DROPS the mismatched scan, so SLAM only integrates ~9% of the data.
rf2o and AMCL also buck the shifting angle layout.

This node bridges between the lidar driver and the rest of the stack:
it subscribes to the raw scan (default /scan_raw), resamples every frame
onto a CONSTANT 504-beam grid with a fixed angle_increment, fills any
beam that had no measurement with NaN, and republishes the result
(default /scan) for rf2o / slam_toolbox / AMCL / Nav2 to consume.

The number of beams is configurable (param `beams`, default 504) and is
chosen to exceed the sensor's nominal point count so no wall data is
thrown away: with 504 bins over 360 deg each bin is 0.7143 deg wide.

OUTPUT CONVENTION (matches ROS2 LaserScan):
    angle_min = 0
    angle_max = 2*pi
    angle_increment = 2*pi / N
    ranges[i]     = min measured range whose angle fell into bin i (NaN if none)
    intensities[i]= intensity of that winning point

Run (added to stack.sh BASE):   python3 /root/scan_normalize.py
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

TWO_PI = 2.0 * math.pi


class ScanNormalizer(Node):
    def __init__(self):
        super().__init__("scan_normalizer")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("beams", 504)
        # Robot's OWN body mask: the LD19 sees parts of the chassis/stand at
        # ~0.46-0.50 m in two fixed sectors (verified live). Without a mask those
        # readings get baked into every map at the spots where the robot stood
        # (map origin + `map save` seed) AND into the costmaps as obstacles next
        # to the robot -> "Collision Ahead", planner "failed to create plan",
        # recovery spin/backup loops. Masking them out (NaN) at this choke point
        # fixes mapping, localization and Nav2 costmaps at once.
        self.declare_parameter("body_mask", 0.55)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.n = int(self.get_parameter("beams").value)
        self.body_mask = float(self.get_parameter("body_mask").value)
        self.angle_increment = TWO_PI / self.n

        # Same QoS the LD19 driver uses (reliable/volatile default) so
        # downstream rf2o/slam/AMCL compatibility is unchanged.
        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.sub = self.create_subscription(LaserScan, in_topic, self.cb, self.qos)
        self.pub = self.create_publisher(LaserScan, out_topic, self.qos)

        self.get_logger().info(
            "scan_normalize ready: %s -> %s  (%d beams, %.4f deg/beam, body_mask %.2f m)"
            % (in_topic, out_topic, self.n, math.degrees(self.angle_increment), self.body_mask))

    # ------------------------------------------------------------------ #
    def cb(self, msg: LaserScan):
        n = self.n
        inc = self.angle_increment

        best_range = [float("inf")] * n
        best_intensity = [float("nan")] * n

        # The output grid is CENTRED ON ZERO: bin i is the bearing -pi + i*inc.
        #
        # It used to run 0 .. 2*pi, which is mathematically the same set of bearings
        # and is read correctly by anything that does `angle_min + i*angle_increment`
        # (slam_toolbox, the nav2 costmaps, reseed.py). rf2o does NOT do that. From
        # rf2o's CLaserOdometry2D.cpp:
        #     fovh = std::abs(scan.angle_max - scan.angle_min);          // line 71
        #     const float tita = -0.5*fovh + float(u)*fovh/float(cols-1); // 441,544,626,667,821
        # It ignores angle_min completely and ASSUMES the scan is centred on the
        # laser's +x axis. With angle_min=0/angle_max=2pi it placed beam 0 at -pi
        # when that beam really points at 0, i.e. it rotated the entire scan by 180
        # deg -- so every odometry estimate came out rotated 180 deg, inverting both
        # x and y. Measured on the bench with one 1.0 s forward pulse:
        #     map scan-match (no rf2o): dx = +0.260 m   <- rover moved FORWARD
        #     lidar front/rear cones  :                 <- rover moved FORWARD
        #     rf2o odometry           : dx = -0.226 m   <- reported BACKWARD
        # Nav2 therefore drove forward, was told it had gone backward, and corrected
        # by driving further -- and slam had to fight an odometry prior pointing the
        # wrong way on every update, which is what smeared and speckled the maps.
        #
        # angle_max is set to angle_min + (n-1)*inc rather than +pi so that rf2o's
        # fovh/(cols-1) works out to EXACTLY inc, making its beam bearings identical
        # to ours instead of off by one bin. As a bonus this makes
        # round((angle_max-angle_min)/inc)+1 == n, which is the beam count karto
        # checks (the "expected 504" complaint this node was written to silence).
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if r == r and r > self.body_mask:         # skip NaN/invalid + own body
                idx = int(round((angle + math.pi) / inc)) % n   # bin, zero-centred
                if r < best_range[idx]:
                    best_range[idx] = r
                    best_intensity[idx] = msg.intensities[i] if i < len(msg.intensities) else float("nan")
            angle += msg.angle_increment

        out = LaserScan()
        out.header = msg.header
        out.angle_min = -math.pi
        out.angle_max = -math.pi + (n - 1) * inc
        out.angle_increment = inc
        out.time_increment = msg.scan_time / n if msg.scan_time > 0 and n > 1 else 0.0
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, self.body_mask)   # reflect the body mask
        out.range_max = msg.range_max
        out.ranges = [r if r != float("inf") else float("nan") for r in best_range]
        out.intensities = best_intensity
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()