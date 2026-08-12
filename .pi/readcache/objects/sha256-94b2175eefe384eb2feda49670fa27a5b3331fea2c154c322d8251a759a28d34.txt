# Rover — command reference

Mapping + autonomous navigation (lidar-only) on the Pi 5, in Docker container `rover_ros2`.

- **Mapping phase:** LD19 lidar → `rf2o` (laser odometry) → `slam_toolbox` → `foxglove_bridge`
- **Navigation phase:** LD19 → `rf2o` + `map_server` + AMCL + Nav2 (planner/controller/costmaps) → `smart_elex_15d` motors

A persistent **base** (lidar + rf2o + motor + foxglove) is shared by both phases; only the owner
of the `map → odom` transform changes (slam while mapping, AMCL while navigating).

---

## Access

```bash
ssh nithin@10.7.12.41          # password: 6238764749
```

Container `rover_ros2` (image `rover_ros2:saved`) auto-starts on boot (`--restart unless-stopped`).
Raw shell inside it:

```bash
docker exec -it rover_ros2 bash
```

---

## Daily use — the `~/rover` wrapper (preferred)

`~/rover <cmd>` just forwards to `/root/stack.sh` in the container. Every node launches detached
with its own log file. **Map first, then navigate.**

**Base (always up first):**

| Command | Action |
|---|---|
| `~/rover start` | Bring up the **base**: lidar → rf2o → motor → foxglove. (slam is **not** started here.) |
| `~/rover stop` | Tear down everything (base + map + nav). |
| `~/rover restart` | stop → start (base only). |
| `~/rover status` | Phase + processes + ros2 nodes + `/scan`,`/odom_rf2o` rates + `map→base_laser` TF. |
| `~/rover logs <node>` | Tail a node log live (Ctrl-C to exit). Nodes: `lidar` `rf2o` `motor` `foxglove` `slam` `nav`. |

**Mapping phase (after `start`):**

| Command | Action |
|---|---|
| `~/rover map` | Launch `slam_toolbox online_sync` (owns `map→odom`). Drive in Foxglove. |
| `~/rover map save [name]` | Save `/root/maps/<name>.{pgm,yaml}` + capture the localization seed. (default: `room`) |
| `~/rover map stop` | Stop mapping (keep base). |

**Navigation phase (after `map save`):**

| Command | Action |
|---|---|
| `~/rover nav [name]` | Stop slam; bring up `map_server`+AMCL+Nav2; **drive back to the origin** (map 0,0,0). |
| `~/rover nav stop` | Tear down Nav2 (keep base). |

```bash
ssh nithin@10.7.12.41
~/rover start          # base up
~/rover map            # map the room — drive around in Foxglove
~/rover map save       # persist map + localization seed
~/rover nav            # return to origin (your drop point)
~/rover nav stop
~/rover stop
```

Nodes survive the SSH session closing (launched with `nohup setsid`), so you can fire-and-forget:

```bash
ssh nithin@10.7.12.41 '~/rover start'
```

Log files (inside the container): `/tmp/{lidar,rf2o,motor,foxglove,slam,nav}.log`.

---

## Return-to-origin demo (the intended workflow)

The "origin" is map frame `(0,0,0)` — which is exactly where you drop the rover, because
`slam_toolbox` starts the robot there when mapping begins. So:

1. **Drop the rover at point A**, connect the SmartElex + 7–22 V motor power.
2. `~/rover start` → `~/rover map` → drive the room (walking pace).
3. `~/rover map save` (eyeball `/root/maps/room.pgm` if you like).
4. Leave it where you stopped (point B), then `~/rover nav` → it crawls back to A.

`map save` captures the pose at B and `nav` seeds AMCL with it, so localization wakes up already
knowing it's at B (no manual "2D Pose Estimate"). It then drives to `(0,0,0) = A`.

---

## View the map / robot — Foxglove

`~/rover start`, then on your laptop open [Foxglove Studio](https://foxglove.dev) and connect to:

```
ws://10.7.12.41:8765
```

Panels: **Map** (`/map`), **LaserScan** (`/scan`), and during navigation **Costmap**
(`/global_costmap/costmap`, `/local_costmap/costmap`) + **Teleop** (`/cmd_vel`). Avoid spinning
in place too fast while mapping (rf2o is differential scan-matching).

---

## Under the hood — raw launch commands (optional)

Only if running a node by hand instead of via the wrapper. From inside the container
(`docker exec -it rover_ros2 bash`):

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# BASE (kept up by `~/rover start`)
ros2 launch ldlidar_stl_ros2 ld19.launch.py                    # lidar + base_link->base_laser TF (z=0.18)
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py  # owns odom->base_link, topic /odom_rf2o
python3 /root/l298n_node.py --ros-args -p left_is_a:=false -p max_speed:=0.3 -p max_level:=5 -p deadman:=2.0
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# MAPPING (`~/rover map`)  -- base_frame + use_sim_time overrides are MANDATORY (Troubleshooting)
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/root/slam.yaml use_sim_time:=false

# SAVE MAP + SEED (`~/rover map save`)
python3 /root/tf_pose.py map base_link /root/maps/room.seed
ros2 run nav2_map_server map_saver_cli -f /root/maps/room --occ 0.65 --free 0.25

# NAVIGATION (`~/rover nav`)  -- use_composition MUST be capital False (Troubleshooting)
ros2 launch nav2_bringup bringup_launch.py \
  map:=/root/maps/room.yaml params_file:=/root/nav2_params.yaml \
  use_sim_time:=false use_composition:=False autostart:=true
python3 /root/return_home.py --map-name room     # seeds AMCL + drives to (0,0,0)
```

`/root/slam.yaml` (one-time) overrides `base_frame` (defaults to `base_footprint`):

```bash
cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_sync.yaml /root/slam.yaml
sed -i 's/base_frame: base_footprint/base_frame: base_link/' /root/slam.yaml
```

---

## Quick health checks (inside the container)

```bash
ros2 topic hz /scan                       # ~10 Hz
ros2 topic hz /odom_rf2o                  # ~10 Hz (rf2o)
ros2 run tf2_ros tf2_echo odom base_link  # exists (rf2o)
ros2 run tf2_ros tf2_echo map odom        # mapping: slam ;  nav: AMCL
ros2 topic echo /map --once --field info  # resolution/width/height
# during navigation:
ros2 lifecycle get /bt_navigator          # -> active
ros2 topic echo /cmd_vel                  # velocity_smoother -> motor node
```

The mapping slam log must show `Registering sensor: [...]` and **zero** `Failed to compute odom pose`.

---

## Troubleshooting

**Mapping — `/map` stays empty, slam spams `Failed to compute odom pose`** (handled by the wrapper):
1. `use_sim_time` defaults to `true` → frozen clock → pass `use_sim_time:=false`.
2. `base_frame` defaults to `base_footprint`, we only have `base_link` → `/root/slam.yaml` override.

**Navigation — Nav2 nodes stay `inactive`, logs spam `Invalid frame ID "map" ... frame does not exist`:**
This is the AMCL chicken-and-egg: AMCL won't broadcast `map→odom` until it has an **initial pose**,
and the planner/costmap can't finish activating until `map→odom` exists. `~/rover nav` (via
`return_home.py`) seeds AMCL automatically and unblocks it. If you launch Nav2 by hand, seed it:
`python3 /root/seed_amcl.py room` (or publish a 2D Pose Estimate in Foxglove).

**Navigation — `nav2_bringup` crashes with `name 'false' is not defined`:**
You passed `use_composition:=false` (lowercase). The launch file evaluates it as **Python**, so it
must be `use_composition:=False` (capital F). (`use_sim_time:=false` lowercase is fine — it's a param
value, not evaluated.)

**Rover drives the wrong way / spins when told to go straight:** wrong motor channel mapping — flip
`left_is_a` (`false` ↔ `true`) in the `start` motor launch line of `stack.sh`.

**Known, non-blocking:** rf2o yaw drifts (no absolute heading) — AMCL corrects it via `map→odom`.
The LD19 scan-count off-by-one warnings are non-fatal. The motor node has a **2 s serial write
floor** (SmartElex firmware) — hence the crawl speed + `nav2_velocity_smoother` deadband to keep
Nav2's 10 Hz `cmd_vel` from spamming it.

---

## IMU (MPU6050) — wired, detected, NOT used

Sensor is on I2C bus 1, confirmed at address **`0x68`** (`WHO_AM_I` = `0x68` → genuine MPU6050).
User `nithin` is in the `i2c` group, so you can probe it **without root** and without `i2c-tools`
installed — using a raw-ioctl Python scan of `/dev/i2c-1`.

⚠️ **Mounted sideways** — for any yaw-fusion plan the MPU6050 must be **flat, Z-up**, or you must
remap axes before the Madgwick filter. Phase 2 (EKF yaw-fusion) was attempted and **dropped** — the
MPU6050 has no magnetometer, so its yaw drifts too fast to help. Navigation (Phase 3) does not use
the IMU; AMCL corrects yaw via `map→odom`.
