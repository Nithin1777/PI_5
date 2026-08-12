USEFUL COMMANDS:

root@nithin:/# ros2 launch ldlidar_stl_ros2 ld19.launch.py
docker exec -it rover_ros2 bash
root@nithin:/# ros2 launch slam_toolbox online_sync_launch.py
nithin@nithin:~ $ docker start rover_ros2
rover_ros2
nithin@nithin:~ $ docker exec -it rover_ros2 bash
docker exec -it rover_ros2 bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml


SSH COMMANDS:
ssh nithin@10.7.12.41
password:6238764749

WIRING — PI5 to motor driver: SmartElex 15D (PWM INDEPENDENT mode, GPIO — serial RETIRED)
The L298N GPIO driver and the SmartElex TTL/USB-SERIAL mode are both RETIRED. The SmartElex 15D is
now driven over GPIO **PWM in INDEPENDENT mode**: the serial mode's ~2 s per-write floor (§10) made
teleop/nav crawl — PWM applies a new duty within one period (~2 ms @ 490 Hz; ~0.1 ms write latency).

Board setup: DIP = Mode 3 (PWM LINEAR INDEPENDENT: SW1=OFF SW2=OFF SW3=ON SW4=ON)
Power:       Motors need external 7-22 V on VIN/GND screw terminals (USB serial is unused now).
             I/P LOGIC SELECT jumpers -> ON (3V3; Pi GPIO is 3.3 V); Pi GND <-> driver GND MUST be commoned.
Pins:        PWM1/S1 <- GPIO12   DIR1 <- GPIO23   -> ch1 = M1+/M1-
             PWM2/S2 <- GPIO13   DIR2 <- GPIO24   -> ch2 = M2+/M2-
             (left_is_a=False on this build -> ch1 = right; invert_right=True compensates wiring)
Duty map:    DIR HIGH = forward; duty 8% -> 0% motor, 94% -> 100% motor (linear); 0% = STOPPED.
             The old serial levels 0..9 are reproduced: level N -> 0.08 + (N/9)*0.86 (same 9 speeds).
ROS node:    /root/l298n_node.py  (class SmartElex15D, node name `smart_elex_15d`)
             consumes /cmd_vel -> GPIO PWM. Launched by stack.sh as the `motor` node (needs gpiozero+lgpio).
Live params: left_is_a=False, invert_right=True, max_speed=0.3, max_level=3 (=min_level via `rover speed`),
             deadman=0.0 (latched; stop via stop_cmd.py / Nav2 smoother timeout).
Serial guide: smart_elex_15D.md  (the old serial-mode protocol — board is in PWM mode now; kept as reference)











# Plan: Lidar-only SLAM (no wheel encoders) on the Rover

**Hardware:** Raspberry Pi 5 · LDROBOT LD19 lidar (`base_laser`, ~10 Hz) · SmartElex 15D motor driver (GPIO PWM, independent mode) · no wheel encoders · no IMU in the loop
**Stack:** ROS 2 Humble, all running inside Docker container `rover_ros2`
**Goal:** Get `slam_toolbox` to actually build `/map` by providing the missing `odom → base_link` transform using **lidar scan-matching odometry** (rf2o). The original plan also fused an MPU6050 yaw via an EKF as "Phase 2" — that has been **dropped** (see STATUS).

---

## STATUS — what's done (updated Aug 2026)

- 🆕 **Phase 3 — Autonomous navigation (return-to-origin) ✅ WORKING (Aug 2026).** Map the room → `~/rover nav` → the rover localizes and `return_home.py` drives it back to the **map origin `(0,0,0)`** (= the drop point). **Aug 10: localization owner swapped AMCL → slam_toolbox localization** (AMCL never converged on this map — §5b.6 #13); `nav` = `reseed.py` scan-match seed → slam_loc (`map→odom`) → standalone `map_server` → `navigation_launch.py` (no AMCL) → `return_home --no-amcl`. Pipeline **verified parked** (slam_loc up, reseed gates honestly, `map→odom` TF, Nav2 nav stack activates AMCL-free, bt_navigator wait fix, and a zero-distance `navigate_to_pose` succeeds end-to-end) — and three more root causes found & fixed on the floor: dual `/map` publishers (#16), CPU starvation (load 8.7, foxglove at 100% — #17), and the **robot's own body baked into the maps** via the scan (#18, `body_mask`). Final e2e drive test pending a **fresh re-map** with the masked scan. New verbs: `start`(base) `map` `map save` `nav` `nav stop` `speed`. Speed is a **single power level** (`rover speed N`, default 3; floor=ceiling) so turns have enough torque in **both** teleop and nav. ⚠️ The LD19 was found **mounted 180° backwards** (caused a wrong-way drive) — remount forward and re-map. See §5b.
- ✅ **Phase 1 = the architecture.** Lidar-only SLAM is the final, in-use design. `rf2o` owns `odom → base_link`, `slam_toolbox` owns `map → odom`, `/map` builds. Verified live on the Pi.
- ✅ **Motor driver migrated L298N → SmartElex 15D, then control mode SERIAL → PWM (Aug 2026).** The SmartElex is now driven over GPIO **PWM independent mode** (pins 12/13/23/24, gpiozero+lgpio, ~0.1 ms writes) — USB serial (`/dev/ttyACM0`) and its ~2 s per-write floor are retired (§10). Node `smart_elex_15d` (file `/root/l298n_node.py`) verified on the Pi, subscribed to `/cmd_vel`. See top wiring note + §10.
- ✅ **Stack is managed by `~/rover` (host) → `/root/stack.sh` (container) → `/root/rosbg.sh` (per node).** `start` = base (lidar → rf2o → motor → foxglove); `map` / `nav` layer slam or Nav2 on top. All detached; survives SSH disconnect.
- ❌ **Phase 2 (MPU6050 + EKF yaw-fusion) DROPPED.** Tried, results were poor: the MPU6050 has **no magnetometer**, so its yaw has no absolute reference and drifts far too fast to help scan-matching — fusing it gave no real benefit over lidar-only. SLAM's own `map → odom` already corrects yaw drift continuously. §5 is kept below as historical/reference only; it is **not** deployed.
- ⚠️ **LD19 scan-count flapping DID fire during mapping (N=501–505, slam drops the mismatches) — superseded by the `scan_normalize` fix in the next bullet.**
- 🔧 **rf2o "Waiting for laser_scans" spam — cosmetic, now silenced (Aug 2026).** rf2o's update timer ran at `freq: 20 Hz` while `/scan` is 10 Hz → every other timer tick had no new scan and it logged `Waiting for laser_scans....` indefinitely (plus a burst at stack start before the first scan arrives). Odometry was never affected — `/odom_rf2o` stayed at 10 Hz and the log alternated compute/wait 1:1. Fix: patched `rf2o_laser_odometry.launch.py` `'freq' : 20.0 → 10.0` (re-apply if the container is rebuilt — see the NOTE in stack.sh). Verified after patch: 40 odom updates vs 1 wait in the same window.
- 🔧 **LD19 scan-count flapping fixed (Aug 2026) — new `scan_normalize` bridge.** The LD19 driver emits 501–505 readings/frame (+ ~40 NaNs, flapping `angle_increment`) instead of a fixed count; karto expects a fixed 504 and **drops every mismatched scan**, so slam was integrating only ~9% of scans → the saved `room.pgm` was 97% free with **zero walls** → AMCL had nothing to lock onto → nav did "random motion / never reaches goal". Fix: new `scan_normalize.py` resamples every frame onto a fixed **504-beam** grid and republishes as `/scan`; the driver is repointed to `/scan_raw` (one-line patch to `ld19.launch.py` — re-apply if the container is rebuilt). Verified live: `/scan` is 504 every frame with a constant angle layout, slam logs **zero** `expected 504` drops (was spamming), and a live map shows real wall structure. ⚠️ Re-map the room (`~/rover map` → drive → `map save`) and confirm a READY grade before `nav` — the old `room` map is stale.
- 🔧 **Teleop latency FIXED — root cause was the driver, not Foxglove.** The SmartElex 15D accepts ONE serial write every ~2.0 s (measured: `write()` blocks for exactly `max(0, 2000 − gap)` ms). The old motor node re-sent the command every 2.5 s (`REFRESH_PERIOD`), keeping the device busy, AND blocked its control loop on `write()` → every command stalled up to 2 s. Fix: `l298n_node.py` rewritten — serial I/O on a dedicated **writer thread** (the 20 Hz loop never blocks) + **refresh removed** (the SmartElex latches state). Normal driving (commands >2 s apart) is now responsive; rapid back-to-back commands still hit the ~2 s device floor. (The earlier `num_threads:=4` on `foxglove_bridge` was a wrong turn — v3.4.2 ignores that param.) See §10. **Superseded (Aug 2026): serial mode itself is retired — the SmartElex now runs over GPIO PWM, so the 2 s floor is gone entirely (§10).**
- 🔧 **Map-quality debugging + auto-grader (Aug 10).** The "nav spins in place / gets lost after one turn" failure was traced to **a bad map, not the nav stack** (confirmed by a controlled stationary map+nav test that `ARRIVED` instantly). New tooling: `map save` now auto-prints a **map-quality grade** (coverage %, # free components, seed↔goal connectivity, READY/KEEP-MAPPING verdict). Tool: `/root/grade_map.py [name]`. See §11.

---

## 1. The problem (one paragraph)

`slam_toolbox` matches consecutive laser scans to build a map, but to align scans it must know *roughly how the robot moved between scans*. That motion estimate lives in the `odom → base_link` transform. On this robot there is **no odometry source** (no encoders), so `odom → base_link` is missing → SLAM can't process scans → `/map` stays empty. We synthesize that transform two ways: (a) **laser scan-matching** with `rf2o_laser_odometry` (already built in the workspace) produces odometry from the lidar itself, and (b) the **MPU6050's yaw** is fused in through `robot_localization`'s EKF to stabilize rotation. The reference implementation for this exact constraint (Pi + no encoders + lidar + IMU) is `Lochan25526/ros2-lidar-inertial-slam`. **(Update Aug 2026: half (b) — the MPU6050/EKF yaw fusion — was DROPPED after poor results; only the lidar scan-matching (a) is deployed. See STATUS.)**

### TF tree we are building
```
map ──> odom ──> base_link ──> base_laser   (lidar)
                   │
                   └──> imu_link            (MPU6050) — NOT deployed (Phase 2 dropped)
```
| Transform | Owner |
|---|---|
| `map → odom` | `slam_toolbox` |
| `odom → base_link` | **rf2o** (final — EKF dropped) |
| `base_link → base_laser` | static TF — ✅ already exists (z = 0.180) |
| `base_link → imu_link` | static TF — **not added** (Phase 2 dropped) |

---

## 2. Current state (verified on the Pi)

- ✅ **Device issue permanently fixed.** Container recreated with `-v /dev:/dev` + `--privileged` + `--restart unless-stopped`; image saved as `rover_ros2:saved`. Survives USB replugs and Pi reboots. (Live confirm: `--privileged`, `--network host`, `-v /dev:/dev`.)
- ✅ **Phase 1 stack running & verified:** `/LD19`, `/rf2o_laser_odometry`, `/slam_toolbox` (`sync_slam_toolbox_node`), `/foxglove_bridge`, static `base_link → base_laser` (z=0.180), plus the **`smart_elex_15d`** motor node.
- ✅ `/scan`: ~10 Hz, real ranges, frame `base_laser`.
- ✅ `/map`: **building** — slam logs `Registering sensor: [Custom Described Lidar]`; the `map → odom → base_link → base_laser` TF chain resolves.
- ✅ `odom → base_link` owned by **rf2o** (`publish_tf=True`). Verified live: rf2o emitted real odometry (`Robot-base odom [x,y,yaw]=...`) while tracking.
- ✅ **Motor: SmartElex 15D over GPIO PWM (independent mode).** `/tmp/motor.log` shows `smart_elex_15d ready (PWM independent @ 490 Hz)` (launched with `left_is_a=False`, `invert_right=True`, `max_level=3`, `min_level=3`, `deadman=0.0`).
- ✅ Already installed: `robot_localization`, `imu_filter_madgwick`, `rf2o_laser_odometry` (in `/root/ros2_ws`). (The first two are now UNUSED — Phase 2 dropped.)
- ❌ **MPU6050 fusion dropped** — see STATUS. IMU was detected on `i2c-1` @ `0x68` but is mounted sideways and is **not** in the loop.
- ⚠️ slam emits LD19 scan-count warnings (`expected 500`) — see §9.
- rf2o launch defaults: `odom_topic=/odom_rf2o`, `base_frame_id=base_link`, `odom_frame_id=odom`, `laser_scan_topic=/scan`, `publish_tf=True`.

---

## 3. Architecture (final — lidar only)

The original plan proposed two phases; **Phase 2 (IMU/EKF) was dropped** (see STATUS). The deployed architecture is lidar-only — `rf2o` owns `odom → base_link`, `slam_toolbox` owns `map → odom`. The motor node sits outside the TF chain (it just consumes `/cmd_vel`).

```
LD19 ──/scan──┬─→ rf2o ──/odom_rf2o──→ owns odom→base_link
              │        │
              │   slam_toolbox ──/map   (owns map→odom)
              └─→ slam_toolbox

smart_elex_15d :  /cmd_vel ──→ GPIO PWM  (SmartElex 15D independent mode, outside TF chain)
```

---

## 4. Phase 1 — Lidar-only odometry → building map (no IMU needed)

> ✅ **PHASE 1 VERIFIED WORKING (2026-08-01)** — lidar-only odometry builds `/map`. Two non-default slam settings are mandatory; see §4.2.

rf2o already defaults to publishing `odom → base_link`, so this is mostly just launching it and restarting slam.

**4.1 Launch rf2o** (in a new container shell):
```bash
docker exec -it rover_ros2 bash
# inside:
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
# params are already: odom_topic=/odom_rf2o, base_frame_id=base_link, publish_tf=true
```

**4.2 Launch slam_toolbox** (now that `odom → base_link` exists). **Two non-default settings are mandatory** on this robot — the stock launch will NOT build a map without them (see "Phase-1 gotchas" below):
```bash
# (a) create a params file overriding base_frame (default is base_footprint; we only have base_link)
docker exec rover_ros2 bash -c \
  'cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_sync.yaml /root/slam.yaml && \
   sed -i "s/base_frame: base_footprint/base_frame: base_link/" /root/slam.yaml'

# (b) launch with that params file AND use_sim_time:=false
ros2 launch slam_toolbox online_sync_launch.py \
  slam_params_file:=/root/slam.yaml use_sim_time:=false
```

> **Phase-1 gotchas (both hit for real):**
> - **`use_sim_time` defaults to `true`** in `online_sync_launch.py` (it's meant for bag playback). On a real robot with no `/clock`, slam's clock is frozen at 0, so it can never resolve `odom→base_link` at the scan timestamp → it spams `Failed to compute odom pose` forever and `/map` never fills. Always pass `use_sim_time:=false`.
> - **`base_frame` defaults to `base_footprint`**, but this robot's tree only has `base_link`. Every `odom→base_footprint` lookup fails with the identical `Failed to compute odom pose` warning. Override it via `/root/slam.yaml` as above.
> - ⚠️ **The LD19 "scan-count off-by-one" conformance bug DOES fire now** — see §9. Non-fatal (map still builds); recorded for awareness (earlier it was wrongly logged as not firing).

**4.3 Verify:**
```bash
ros2 topic echo /odom_rf2o --once          # rf2o publishing odom
ros2 run tf2_ros tf2_echo odom base_link   # NOW EXISTS  ✅
ros2 run tf2_ros tf2_echo map odom         # slam_toolbox publishing  ✅
ros2 topic echo /map --once --field info   # shows resolution/width/height → map exists ✅
ros2 topic hz /map                         # ~0.2 Hz (slam updates every ~5 s) — fine
# slam log MUST show "Registering sensor: [...]" and ZERO "Failed to compute odom pose"
```
Then **drive the robot** around the room — `/map` fills in. Open `/map` + `/scan` in Foxglove (connect `ws://10.7.12.41:8765`).

**Phase-1 limitations (accepted):** yaw drifts over time (no absolute heading); scan-matching degrades in long featureless corridors. (Phase 2 was meant to address the yaw part but was dropped — see §5; SLAM's own `map → odom` already corrects yaw drift continuously.)

---

## 5. Phase 2 — Add MPU6050 + fuse yaw via EKF   ❌ DROPPED

> **⛔ DROPPED (Aug 2026).** This phase was attempted and abandoned. The MPU6050 has **no magnetometer**, so its yaw has no absolute heading reference and drifts far too quickly to stabilize scan-matching — fusing it via the EKF produced **no improvement over lidar-only**, while adding TF-jitter and calibration headaches (the sensor was also mounted sideways). SLAM's own `map → odom` already corrects yaw drift continuously, so the EKF gave no net benefit. **rf2o keeps `publish_tf=True` and owns `odom → base_link`.** Everything below is kept as historical reference only — do not redeploy without a 9-DoF IMU (with magnetometer).

### 5.1 Wire the MPU6050 (Pi 5 GPIO)
| MPU6050 | Pi 5 pin |
|---|---|
| VCC | 3.3V (pin 1) |
| GND | GND (pin 9) |
| SDA | GPIO2 / SDA (pin 3) |
| SCL | GPIO3 / SCL (pin 5) |

### 5.2 Install i2c-tools + confirm the sensor
```bash
docker exec rover_ros2 bash -c "apt update && apt install -y i2c-tools"
docker exec rover_ros2 i2cdetect -y 1          # expect 0x68 to show up
```
(If `0x68` doesn't appear, re-check wiring / `dtparam=i2c_arm=on` in `/boot/firmware/config.txt`.)

### 5.3 Add a MPU6050 ROS 2 driver
Clone one of these into `/root/ros2_ws/src` and `colcon build`:
- `https://github.com/hiwad-aziz/ros2_mpu6050_driver` (publishes `sensor_msgs/Imu`, auto-calibration)
- `https://github.com/DavDori/mpu6050_ros2`

```bash
docker exec -it rover_ros2 bash
cd ~/ros2_ws/src && git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver
cd ~/ros2_ws && colcon build --symlink-install --packages-select ros2_mpu6050_driver
```
Run it → publishes raw IMU. **Calibration note:** most of these auto-calibrate on startup — the sensor must be **flat, Z-up, and stationary** when the node starts.

### 5.4 Orientation filter (gives a fused quaternion)
The MPU6050 driver gives raw accel + gyro. Feed it to `imu_filter_madgwick` to get a stable orientation:
```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args -p use_mag:=false -p publish_tf:=false \
  -p stateless:=false
# (remap input to the driver's raw topic, output is /imu/data)
```
`use_mag:=false` is required — the MPU6050 has **no magnetometer**, so yaw has no absolute reference (it drifts; SLAM corrects it via `map → odom`).

### 5.5 Turn rf2o's TF off (EKF will own `odom → base_link`)
The repo's key lesson: two nodes publishing `odom → base_link` makes TF jitter. Edit the rf2o launch (or pass as arg) to `publish_tf:=false`. rf2o then publishes **only** `/odom_rf2o` (a topic), which the EKF consumes.

### 5.6 EKF config — save as `~/ros2_ws/ekf.yaml`
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true            # EKF owns odom -> base_link
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /odom_rf2o
    #            X     Y     Z    roll  pitch  yaw   vX   vY   vZ  vroll vpitch vyaw   aX aY aZ
    odom0_config: [true, true, false,
                   false, false, true,
                   true, true, false,
                   false, false, true,
                   false, false, false]

    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,   # yaw
                  false, false, false,
                  false, false, true,   # yaw rate
                  false, false, false]
    imu0_remove_gravitational_acceleration: false
```
**The IMU contributes yaw + yaw-rate only** — that's all an MPU6050 (no magnetometer) is good for. Linear acceleration is unused because double-integration drifts hopelessly.

### 5.7 Launch the EKF
```bash
ros2 run robot_localization ekf_node --ros-args --params-file ~/ros2_ws/ekf.yaml
```

### 5.8 Add the `imu_link` static transform
```bash
# replace x/y/z with the actual IMU mounting offset from base_link
ros2 run tf2_ros static_transform_publisher x y z 0 0 0 base_link imu_link
```

### 5.9 Verify Phase 2
```bash
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo odom base_link      # now published by EKF, smooth
ros2 topic echo /odometry/filtered --once
ros2 topic hz /map
```
Rotate the robot in place — the laser ring should rotate smoothly with minimal jitter (the EKF's whole point).

---

## 5b. Phase 3 — Autonomous navigation (return-to-origin)  ✅ WORKING

> ✅ **Working (Aug 2026).** Return-to-origin: map the room → `~/rover nav` → the rover localizes and drives back to the **map origin `(0,0,0)`** (= the drop point). **Aug 10: AMCL retired** — `map→odom` is now owned by slam_toolbox **localization mode**, seeded by `reseed.py`'s global scan-match (§5b.6 #13–15); verified parked on the Pi, final drive test pending on the floor. Single goal-to-origin in a pre-mapped room (not exploration), tuned for the SmartElex's bang-bang power levels (the old ~2 s serial floor is gone since the PWM migration — see wiring note + §10). The bring-up bugs are catalogued in §5b.6 so they don't have to be re-discovered.

### 5b.1 What "origin" means
`slam_toolbox` starts the robot at map `(0,0,0)` when mapping begins, so the **drop point A *is* the map origin**. `map save` captures the pose at the stop point **B** (the localization seed); `nav` navigates from B back to `(0,0,0) = A`. No markers, no measuring — the goal is hardcoded as the origin.

### 5b.2 Architecture (navigation mode)
The base (lidar, rf2o, motor, foxglove) stays up across both phases. Only the **owner of `map→odom`** changes:

```
MAPPING:     map→odom owned by slam_toolbox online_sync   (/map = live, growing)
NAVIGATION:  map→odom owned by slam_toolbox LOCALIZATION  (/map = static pgm from map_server)
```

> **Update Aug 2026 — AMCL is RETIRED.** AMCL never converged reliably on this map
> (wandered / flipped 180°, so the published `map→odom` disagreed with the raw
> lidar). `nav` now runs slam_toolbox in **localization mode** (Ceres scan-match
> inside the serialized pose-graph): `reseed.py` finds the robot's pose by global
> scan-matching the live `/scan` against the saved `.pgm`, publishes `/initialpose`
> to seed slam_loc, and `stack.sh` waits for the `map→odom` TF before bringing up
> Nav2. This nav2_bringup has **no `use_amcl` arg**, so AMCL is simply never
> launched: `do_nav` runs a standalone `map_server` + `navigation_launch.py`
> (nav stack only) — see §5b.6 #6.

`odom→base_link` is owned by **rf2o throughout** (never stopped → continuous across the swap). Nav2's `velocity_smoother` is the last hop of the command chain — `controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel → smart_elex_15d` — so the motor node needs **zero remapping**.

```
LD19 ──/scan──┬─→ rf2o ──/odom_rf2o──→ owns odom→base_link
              ├─→ slam_loc (localization_slam_toolbox_node) ──→ owns map→odom
              │        seeded by reseed.py's /initialpose (DT scan-match vs pgm)
              └─→ Nav2 (planner/controller/costmaps) ──/cmd_vel──→ smart_elex_15d
map_server :  /root/maps/<name>.yaml ──→ /map (static, feeds the global costmap)
posegraph  :  /root/maps/<name>.{posegraph,data} — serialized by `map save` (slam_loc loads it)
```

### 5b.3 Files (`~/pi5` repo → container `/root/`)
| File | Role |
|---|---|
| `stack.sh` | verbs `start`(base) `map` `map save` `map stop` `nav` `nav stop` `speed` `status` `logs`; `nav` = slam_loc + reseed + map_server + `navigation_launch.py` (NO AMCL) → return_home `--no-amcl`; `map save` also serializes the slam pose-graph; kills publish a STOP first |
| `slam_loc.yaml` | slam_toolbox **localization** params: `mode: localization`, `map_file_name: /root/maps/<name>` (sed'ed per `nav`), `base_frame: base_link`, `scan_topic: /scan` |
| `reseed.py` | **scan-match re-seed + consistency gate** (new): builds a distance-transform likelihood field from `<name>.pgm`, global-search matches the live `/scan`, publishes `/initialpose` (TRANSIENT_LOCAL) to seed slam_loc; ABORTS nav (exit 2) if no pose matches within `--threshold` (0.12 m) — out-of-map beams are penalized so partial/degenerate maps can't false-lock (§5b.6 #7) |
| `nav2_params.yaml` | Nav2: `base_link`, odom from `/odom_rf2o`, DWB, footprint 310×245 mm, `deadband_velocity`, `velocity_smoother` |
| `return_home.py` | `--no-amcl` mode (slam_loc): waits for bt_navigator ACTIVE directly (§5b.6 #8) → `goToPose(0,0,0)`; default (legacy AMCL) mode: seeds via dedicated node + TRANSIENT_LOCAL; cancels the goal on any exit |
| `tf_pose.py` | captures `map→base_link` → localization seed file |
| `seed_amcl.py` | standalone AMCL re-seeder (diagnostic) |
| `stop_cmd.py` | publishes zero `/cmd_vel` so the motor latches STOP when nav is killed (deadman is off) |
| `scan_normalize.py` | **LD19 scan resampler** (new, Aug 2026): `/scan_raw` (variable 501–505 beams) → fixed 504-beam `/scan` — kills slam's `expected 504` scan drops (§9); **`body_mask: 0.55`** zeroes the robot's own chassis readings (§5b.6 #18) |
| `l298n_node.py` | + `min_level`/`max_level` power floor/ceiling + `deadman` watchdog + GPIO PWM output (was serial writer-thread, §10) |

### 5b.4 Run it (return-to-origin demo)
```bash
~/rover start          # base: lidar + rf2o + motor + foxglove
~/rover speed          # show power level (default 3)
~/rover map            # slam online_sync — drive the room in Foxglove (walking pace)
~/rover map save       # -> /root/maps/room.{pgm,yaml} + room.seed
~/rover nav            # slam_loc + reseed + Nav2 (no AMCL), returns to origin (0,0,0)
~/rover nav stop       # tear down nav (base stays); reliably stops the motor
~/rover stop           # kill everything
```
Drop the rover at A **before** `map` (so the origin lands on A). Don't maneuver it between `map save` and `nav` (the seed must match the real pose).

### 5b.5 Speed / power model (important)
The SmartElex has 9 discrete duty levels (0–9); the motor node maps `cmd_vel` → level. Key lesson: **turning produces small per-wheel speeds** (track is only 0.15 m) that used to map to level 1 = twitch — not enough torque to turn, in **both** teleop and nav.
- `rover speed N` sets a **single power level** = `min_level = max_level = N` (default **3**). Every motion — forward, reverse, turn, teleop *and* nav — runs at level N.
- Raise it (`rover speed 4`, `5`) for stronger turns / more speed; lower it (`2`) if the wheels slip. The trade-off is inherent to 9-level hardware: more level = more turn power but more slip on each abrupt kick.
- Tuning only the ceiling (`max_level`) does **not** fix weak turns — weak commands sit at the floor. That's why `speed` sets both to one value.

### 5b.6 Bugs hit & fixed during bring-up (the painful list)
1. **AMCL seed rejected — QoS.** `BasicNavigator.setInitialPose()` publishes `/initialpose` with a QoS AMCL rejects (`incompatible QoS … DURABILITY`) → seed never arrived → nav drove blind into a wall. Fix: `return_home.py` publishes `/initialpose` from a **dedicated node** with `TRANSIENT_LOCAL` QoS (Humble imports: `QoSReliabilityPolicy` / `QoSDurabilityPolicy` — *not* `Reliability`/`Durability`, which don't exist).
2. **AMCL left `inactive` — lifecycle race.** Intermittently `lifecycle_manager_localization` logs `amcl/get_state … async_send_request failed` → `Aborting bringup` → AMCL stays inactive → rejects the seed (`Received initial pose request, but AMCL is not yet in the active state`). Fix: `do_nav` → `ensure_amcl_active` polls and force-activates AMCL (`ros2 lifecycle set /amcl activate`); `return_home.py` republishes the seed every 0.4 s for 30 s so it's accepted the moment AMCL goes active.
3. **`return_home.py` crashed** formatting `estimated_time_remaining` (a `Duration`, not a float) → on crash Nav2 kept the goal → runaway. Fix: format `.sec`; `try/finally` cancels the goal on any exit.
4. **`deadman:=0` crashed the motor node** — INTEGER vs DOUBLE param type. Fix: `deadman:=0.0` (float).
5. **`deadman=0` (kept for latched teleop) broke `nav stop`** — killing Nav2 leaves no `/cmd_vel`, so the motor held its last command → runaway. Fix: `pkill_nav` runs `stop_cmd.py` to publish zero `/cmd_vel` *after* killing the nodes, so the motor latches STOP. (Goal cancel/complete is separately safe via the smoother's `velocity_timeout`.)
6. **Turns too weak** — small per-wheel speeds → level 1 → twitch. Fix: `min_level` floor, then the single power level (§5b.5).
7. **Locale spam** (`setlocale: LC_ALL en_US.UTF-8`) — `locale-gen en_US.UTF-8` (host + container) + `export LC_ALL=C` in `~/.bashrc`, `stack.sh`, `~/rover`.
8. **`use_composition:=False`** must be capital F (the launch evaluates it as Python); lowercase `false` → `name 'false' is not defined`. (`use_sim_time:=false` lowercase is fine.)
9. **`map_saver_cli -o`** isn't a valid flag in this build — removed.
10. **`base_frame: base_link`** everywhere (AMCL + both costmaps), not `base_footprint` — same trap as Phase 1.
11. **odom topic is `/odom_rf2o`**, not `/odom` — set on `bt_navigator` + `velocity_smoother`.
12. **⚠️ Lidar mounted 180° backwards** → the map/heading was flipped → nav drove the *wrong way* ("full throttle opposite direction"). **Not a code bug** — remount the LD19 facing the bot's forward and re-map (the old map is stale).
13. **AMCL never converged → RETIRED (Aug 2026).** On this map AMCL wandered / flipped 180°, so `map→odom` disagreed with the raw lidar and nav did "random motion / never reaches goal". Swap: `nav` now runs slam_toolbox **localization mode** — `reseed.py` scan-matches `/scan` against the saved `.pgm` and seeds it via `/initialpose`. This nav2_bringup has **no `use_amcl` arg** (localization_launch.py always launches AMCL), so `do_nav` never runs it: standalone `map_server` (lifecycle-configured manually) + `navigation_launch.py` (nav stack only) + `return_home.py --no-amcl`. Verified parked (Aug 10): slam_loc loads the serialized posegraph, reseed gates honestly, `map→odom` TF publishes, Nav2 nav stack activates with **no** AMCL node.
14. **reseed false-lock on partial maps (fixed Aug 2026).** The original cost averaged only beams whose endpoint fell *inside* the map, so on a degenerate single-wall map a spurious pose (3 m / 284° off the truth) scored 0.046 m and passed the gate → nav would have driven off. Fix: **every** beam counts — out-of-map endpoints cost `OUT_PENALTY` (2.0 m, same as the DT cap), so a correct match must fit the whole scan inside the mapped area. Now a partial map fails the gate honestly ("GATE FAIL", `nav` aborts, exit 2) instead of locking onto a wrong pose. A map must cover the robot's surroundings (grade `READY`) for `nav` to proceed.
15. **`waitUntilNav2Active()` hangs without AMCL.** `nav2_simple_commander` polls `/amcl/get_state` by default — with AMCL retired that service never appears and return_home spins forever (`amcl/get_state service not available, waiting...`). Also `localizer=None` doesn't skip it (`None/get_state` loops too). Fix: `--no-amcl` mode waits on `/bt_navigator/get_state` directly (`wait_for_bt_navigator_active`); this build's lifecycle constant is `State.PRIMARY_STATE_ACTIVE` (not `STATE_ACTIVE`).
16. **Two `/map` publishers = planner chaos (fixed Aug 2026).** slam_toolbox in **localization mode also publishes `/map`** (its posegraph-derived occupancy grid, cropped to the traveled area — the map origin can fall in UNKNOWN space) alongside `map_server`'s full saved pgm. The global costmap then sees two conflicting grids → `GridBased: failed to create plan with tolerance 0.50` for *every* goal, even in 93%-free space. Fix: `do_nav` runs the localization node directly with `-r /map:=/slam_loc_map` (no `localization_launch.py`); **map_server is now the only `/map` owner**, slam_loc only owns `map→odom`. Verify: `ros2 topic info /map -v` → 1 publisher.
17. **CPU starvation wedged the whole stack (fixed Aug 2026).** Under Nav2 + a connected Foxglove client, load hit **8.7 on the Pi 5**: foxglove_bridge alone burned a full core (100%), rf2o ~70%, slam_loc ~46%, +9 Nav2 nodes ~20% each → DDS/TF delivery starved → odom TF stalls, `Message Filter ... queue is full`, plans fail, distances bounce. Fix: global costmap `publish_frequency` 1.0→0.2 + `update_frequency` 1.0→0.5, local costmap publish 2.0→0.5 (visualization-only; the controller reads costmaps internally), foxglove `num_threads` 4→2. Result: container CPU 316%→67%, odom solid 10 Hz. Also: a fresh `docker restart rover_ros2` clears wedged DDS state + zombie processes.
18. **⚠️ The robot's own body is in the scan → baked into every map (fixed Aug 2026).** The LD19 sees two chassis/stand blobs at **0.46–0.50 m** (fixed sectors ≈226°/295°, verified live). Those readings got recorded as obstacles at every spot the robot lingered — at the **map origin** (drop point: showed "0.15 m from a wall") and at the **`map save` seed** (the seed cell was literally OCCUPIED, 0.00 m clearance) — even when the robot was standing in open space with ~1 m of clearance. Downstream this made the return goal (origin) unreachable ("failed to generate a valid path to (0.00,0.00)") and the costmaps showed the robot's own body → `Collision Ahead` → spin/backup recovery loops. Fix: **`body_mask: 0.55` in `scan_normalize.py`** — ranges < 0.55 m are zeroed at the single choke point feeding rf2o/slam/costmaps. Verified: `/scan` now has **zero** readings < 0.6 m (nearest real object 1.04 m), and a parked `map save` shows seed+origin at **0.70 m** clearance (was 0.05/0.15 m). ⚠️ **Re-map the room** (`~/rover map` → sweep → `map save`) — all existing maps (`room`, `livemap`, `livetest`) still contain the body artifacts.

### 5b.7 The 2 s serial floor, revisited — RESOLVED by the PWM migration
Early on, Nav2's 10 Hz `/cmd_vel` had to be filtered through `velocity_smoother.deadband_velocity: [0.05, 0, 0.15]` so micro-jitter didn't hammer the serial 2 s write floor. With the **PWM migration (Aug 2026)** that floor is gone — every command applies within ~2 ms, `deadband_velocity` is now just a small noise floor (see the note in `nav2_params.yaml`), and `xy_goal_tolerance: 0.4` remains only to absorb the bang-bang stop overshoot.

### 5b.8 Limitations (accepted for the demo)
- Single return-to-origin goal (no exploration / multi-goal).
- Stop precision ~0.4 m (by design — "relatively close" is fine).
- 9-level bang-bang motion is jerky and can slip; tune N.
- Assumes a static, feature-rich room; rf2o yaw drift is corrected continuously by AMCL's `map→odom`.
- `left_is_a:=false` channel mapping assumed from Phase-1 wiring — verify teleop L/R first; flip to `true` if it turns the wrong way.
- AMCL can still flip ~180° in a highly symmetric room even when seeded; keep the rover still between `map save` and `nav`.

---

## 6. slam_toolbox params (optional tuning)

The default `online_sync_launch.py` works once odom exists. If mapping is rough, drop a `slam.yaml` (from the reference repo, trimmed) and pass it:
```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
    resolution: 0.05
    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.05
    use_scan_matching: true
    use_scan_barycenter: true
    scan_buffer_maximum_scan_distance: 10.0   # LD19 maxes at 25m; cap for indoor
```
```bash
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=~/ros2_ws/slam.yaml
```

---

## 7. Rollback / safety

- The working container image is saved as `rover_ros2:saved` (Up 2 days, `--privileged --network host -v /dev:/dev`). If you break the writable layer, recommit a clean snapshot first: `docker commit rover_ros2 rover_ros2:saved` (the old `phase2_base` name is moot now that Phase 2 is dropped).
- Phase 1 changes nothing destructive — rf2o is already built; you're just launching nodes.

---

## 8. Cheat sheet — what to run, in order

**Phase 1 (today, no IMU):**
1. `ros2 launch ldlidar_stl_ros2 ld19.launch.py` (patched → publishes `/scan_raw`) + `python3 /root/scan_normalize.py` → fixed-504 `/scan` (10 Hz) + `base_link→base_laser` static TF  — `~/rover start` runs both
2. `ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py`  → owns `odom→base_link`
3. make `/root/slam.yaml` overriding `base_frame: base_link` (one-liner in §4.2)
4. `ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/root/slam.yaml use_sim_time:=false`
5. verify `tf2_echo odom base_link` + `tf2_echo map odom` + slam log shows `Registering sensor` → drive around
6. motor: `python3 /root/l298n_node.py` (node `smart_elex_15d`) → subscribes `/cmd_vel` → **GPIO PWM** (SmartElex 15D independent mode; pins 12/13/23/24). Set power with `~/rover speed N`. Tip: `~/rover start` runs all of the above for you.

**Phase 2 (MPU6050 + EKF):** ❌ **DROPPED** — see STATUS and §5. Do not run these steps; kept for reference only. (Would have required: `i2cdetect -y 1` → `0x68`, an MPU6050 driver, `imu_filter_madgwick`, rf2o `publish_tf:=false`, `ekf_node`, and a `base_link→imu_link` static TF.)

**Phase 3 (autonomous navigation — return-to-origin):** ✅ working.
1. `~/rover speed` → check/set power level (default 3; raise for stronger turns, lower if wheels slip).
2. `~/rover start` → base (lidar + rf2o + motor + foxglove). **Drop the rover at origin point A *before* the next step** (slam starts the robot at map (0,0,0), so A = origin). ⚠️ LD19 must face the bot's forward — it was mounted 180° off; if you remount, re-map.
3. `~/rover map` → slam online_sync; drive the room in Foxglove (walking pace).
4. `~/rover map save` → `/root/maps/room.{pgm,yaml}` + `room.seed` (localization seed).
5. `~/rover nav` → AMCL + Nav2 up (seeded at B), drives back to origin `(0,0,0) = A`.
6. `~/rover nav stop` / `~/rover stop` to tear down.  (Details + the full bug list: §5b.)

---

## 9. Known limitations (be aware for the demo)
- **No absolute yaw reference — accepted.** The IMU/EKF fusion (Phase 2) was tried and **dropped**: the MPU6050 has no magnetometer, so fusing its drifting yaw helped nothing. Yaw drift is instead corrected continuously by SLAM's own `map → odom`. For a demo room this is fine.
- **Laser scan-matching fails in featureless/symmetric spaces** (long empty corridor, circular room). Add visual structure or slow down.
- **rf2o needs the laser to move** — it's a *differential* odometry; it initializes at the origin and tracks motion. Parked-then-go is fine.
- ✅ **LD19 scan-count conformance bug FIXED (Aug 2026)** — it *did* fire (live logs showed `LaserRangeScan contains N range recordings, expected 504`, N=501–505, and slam **dropped** those scans). Now handled by the `scan_normalize` bridge: every frame is resampled onto a fixed 504-beam grid, so slam sees a constant count and zero drops (§STATUS). If it ever recurs, re-check the `ld19.launch.py` `/scan_raw` patch and the normalizer's health (`logs scan`).

---

## 10. Teleop latency — real root cause & fix (`/cmd_vel`)

> **UPDATE (Aug 2026) — RESOLVED FOR GOOD: serial mode RETIRED, PWM migration.** The writer-thread
> fix below reduced latency, but the ~2 s floor is a **firmware property of serial mode**, so the
> final fix was switching the SmartElex to **PWM independent mode** (`l298n_node.py` rewritten again:
> GPIO PWM/DIR writes are non-blocking, ~0.1 ms measured — no floor, no refresh, no writer thread).
> Everything below documents the serial-mode diagnosis + the interim software fix; kept as history.

**Symptom:** clicking forward (Foxglove or keyboard teleop) took seconds before the motors moved — "a millennia."

**Wrong turns (for the record):** first blamed a single-threaded `foxglove_bridge` and set `num_threads:=4` in `stack.sh` — but foxglove_bridge **v3.4.2 doesn't declare/consume that param** (`ros2 param get` → "Parameter not set"), so it was a **no-op**. Then suspected the Docker container. Both disproven by direct measurement. (The `num_threads:=4` line is still in `stack.sh` — harmless; can be reverted.)

**Real root cause (measured, deterministic):** the SmartElex 15D's USB-serial interface accepts ONE write every ~2.0 s. A paced test gave a perfect linear fit — `write()` blocks for exactly `max(0, 2000 − gap_since_last_write)` ms:

| gap since last write | write blocks |
|---|---|
| 0.0 s | 2010 ms |
| 0.5 s | 1511 ms |
| 1.0 s | 1011 ms |
| 1.5 s | 511 ms |
| 2.0 s | 11 ms |
| ≥ 2.5 s | < 1 ms |

Identical on the Pi host and inside the container → it's the **device firmware**, not ROS / Foxglove / Python / Docker / the USB cable. Isolated commands (≥ 2 s apart) are instant; a command sooner than 2 s after the last blocks up to 2 s.

Why Foxglove felt like "millennia": the old motor node (a) **re-sent the current command every 2.5 s** (`REFRESH_PERIOD`) — that kept the device's 2 s window perpetually busy, so an on-change write almost always landed inside it and blocked ~2 s; and (b) it **blocked its 20 Hz control loop on `write()`**, so during that block it couldn't even receive a new `/cmd_vel`.

**Fix applied at the time (Aug 2026) — `l298n_node.py` rewritten (software only; serial mode kept; now superseded by PWM):**
1. **Removed the refresh spam.** The SmartElex latches motor state, so a command persists until changed — no periodic resend needed. Biggest win: the device is now idle between user commands.
2. **All serial I/O moved to a dedicated writer thread.** The 20 Hz control loop only computes the desired packet; the writer thread does the (possibly ~2 s-blocking) `write()`. So `/cmd_vel` is always heard and acted on immediately.
3. Deployed to container `/root/l298n_node.py` (byte-matches the repo); relaunched with `max_level:=5` (50 % safe cap).

**Result:** normal teleop (commands > 2 s apart) is now responsive. The only residual lag is two commands within 2 s (e.g. forward then immediate stop) — the second waits up to 2 s. That ~2 s is the **hardware floor of serial mode**; eliminating it entirely needs switching the SmartElex to **PWM/RC input mode** (DIP change + GPIO PWM wiring + a PWM node) — **done Aug 2026: the rover now runs in PWM independent mode; see the update note at the top of this section.**

**Test harnesses left in place:**
- `~/drive.py` on the Pi host — standalone SmartElex serial WASD tester (no ROS); prints each write's latency. Keys: `w`/`s`/`a`/`d`, `x`/SPACE stop, `q` quit. Run: `python3 ~/drive.py`. **(serial mode only — the board is on PWM now, so it can't drive the motors in the current config; keep the ROS motor node stopped if you use it.)**
- `teleop_twist_keyboard` installed in the container; `~/teleop` wrapper on the host runs it (SSH keyboard teleop, bypasses Foxglove). Keys: `i`/`,`/`j`/`l`, `k` stop.
- Foxglove (`ws://10.7.12.41:8765`) + `~/rover start` both still drive it.

**How to verify the fix:** drive with commands spaced > 2 s — each should be near-instant. Then forward→stop as fast as possible to feel the 2 s hardware floor.

---

## 11. Map-quality debugging session (Aug 10) — why nav spun, + map auto-grader

> **Investigation, not a nav change.** Nav is **not** re-validated end-to-end here. This section records the diagnosis of "nav spins in place / gets lost after one turn" and adds tooling to catch the cause *before* running nav.

### 11.1 Symptoms
- `~/rover nav` runs (AMCL seeds, Nav2 activates, goal sent) but the rover **spins in place / runs recoveries** instead of driving to origin. "One turn and you're doomed" — after any rotation it can't recover.
- On the bench (rover lifted, wheels free) Nav2 commanded `linear.x≈0.10, angular.z≈0.4` (a tight arc) and never converged.

### 11.2 Tests run + findings
1. **Static TF `base_link → base_laser` = identity** (RPY 0, z = 0.180). After the LD19 remount there is **no leftover 180° flip** in TF — the now-forward lidar's scan-0 really is forward.
2. **`/scan` spans `[0, 2π)`** (`angle_min=0`, `angle_max=6.28`, ~500 beams). Full circle; geometrically fine for AMCL/rf2o. (The §9 off-by-one warning did not fire during these runs.)
3. **rf2o odom is healthy**: `/odom_rf2o` at 10 Hz; **stationary drift = 4.5 mm / 1.6° over 20 s (≈0.0002 m/s)** — odom does **not** hallucinate motion while still. Therefore a seed 1.68 m from origin is from **real driving (A→B)**, not drift.
4. **AMCL localizes correctly** — `/amcl_pose` matched the seed within ~2 cm.
5. **NavFn was failing to plan**: nav log spamming `GridBased: failed to create plan with tolerance 0.50`, then `behavior_server` recovery (`DriveOnHeading … backup failed`). **The visible "spinning" is the behavior tree running Spin/BackUp recoveries because the planner can't find a path** — not Nav2 choosing to turn toward the goal.
6. **The saved map was the culprit.** `room.pgm` analysis (Aug 9 map): **27% free / 71% unknown / 3% occupied**; free space shattered into **405 disconnected components**, with **phantom occupied cells scattered *inside* the free space**. ASCII render = one noisy drive-pass, not a filled room. Robot (point B) and goal (origin A) were **not in the same free component** → NavFn correctly gives up.
7. **The Aug 9 map was almost certainly built while the LD19 was still mounted backwards** (the remount happened Aug 10) — a backwards lidar produces a geometrically inconsistent map, a second independent reason it is invalid.

### 11.3 Conclusion (root cause)
The nav stack (rf2o, AMCL, Nav2, motor node) is **sound**. The failures were caused by **map quality**: an under-explored, fragmented map (71% unknown; hundreds of free islands; phantom obstacles from pose jitter during jerky mapping; and built with a backwards lidar). With no connected free path start→goal, NavFn fails → recoveries → spin.

**Proof (controlled experiment, the user's idea):** with the lidar correctly mounted and the rover lifted & stationary, ran a fresh `map` → `map save` → `nav`. seed ≈ `(0,0,0)` and goal = `(0,0,0)` (start == goal). Nav2 printed **`ARRIVED at goal` instantly with an empty `/cmd_vel`** — no spin, no plan failure. I.e. when the robot is genuinely at the goal on a fresh map, nav is trivial and correct. The earlier "spinning" was purely because the robot was 1.68 m from the goal across an unplannable map.

**Frame-semantics reminder:** map-origin `(0,0,0)` = point A (where `map` *started*); the `.seed` = point B (where `map save` ran); `nav` always targets A. "Already at the goal" is only true if the robot is physically at A.

### 11.4 New tooling — `grade_map.py` (auto-grades on `map save`)
- **`/root/grade_map.py [name]`** (new file; source in repo `grade_map.py`) grades a saved map and prints: coverage (free/unknown/occupied %), explored footprint, **# free components + % in the largest**, and whether the **seed ↔ goal (origin)** are connected through free space — then a `READY to nav` / `KEEP MAPPING` verdict (exit 0 / 2) with a one-line fix tip.
- **Wired into `stack.sh` `do_mapsave`:** every `~/rover map save` now auto-prints the grade immediately after the save. No extra command needed.
- **Pass criteria** (deliberately lenient — they flag *unplannable* maps, not aesthetics): free ≥ ~45%, largest free component ≥ ~70% of all free, **and** seed↔goal connected.
- Run by hand any time: `docker exec rover_ros2 python3 /root/grade_map.py room`.

### 11.5 How to make a good map (so nav actually works)
- **Map by coverage, not time.** Stop when Foxglove `/map` shows the whole room floor white with clean black walls and no big gray patches *inside* the room.
- **Drive the middle, not just the edges** — a large explored box with low free % means you looped the perimeter and left the center unknown.
- **Drive slowly & smoothly (`rover speed 2–3`).** The phantom occupied speckle inside free space comes from pose jitter during jerky bang-bang motion → walls recorded at offset positions.
- **Don't spin in place** (rf2o is weakest at pure rotation; yaw drifts ~5°/min). Turn while moving.
- **Close the loop** — return near where `map` started before `map save`; lets slam correct accumulated rf2o yaw drift (the single biggest map-quality win on this rover).
- **Stop in a feature-rich spot** — that pose becomes the AMCL seed; AMCL localizes better with structure around it.
- This room is ~5–7 m; expect ~1–3 min of systematic driving (perimeter loop + a crossing pass through the middle), not ~20 s.

> **State of the map after this session:** the last `map save` graded **READY** (93×93, 98% free, 1 component, seed↔goal connected) — but it was a *smaller* freshly-built map than the earlier 7×7.7 m attempt. End-to-end nav against a full-room READY map has **not** been re-demonstrated since the lidar remount; re-map the full room and confirm a READY grade before relying on `nav`.




BUG FOUNG AND FIXED:

 I found it, and I can prove it without touching anything. It's not the map — it's reseed.py. Every nav run starts the rover believing it is 0.4 m (or more) away from where it actually is.


The bug


reseed.py builds the localization seed by scan-matching the live /scan against the saved .pgm. Its cost function converts a beam endpoint to a pixel row using ymax, the map's top edge:


ymax = oy + h * res                      # line 80  — true top edge


def cost(x, y, yaw):                     # line 112

    ...

    row = int(round((ymax - (y + r*math.sin(th))) / res))   # uses ymax


xmin, xmax = ox + 0.4, ox + w * res - 0.4

ymin, ymax = oy + 0.4, oy + h * res - 0.4    # line 126 — REUSES the name `ymax`

                                             #            as a search bound

for x in np.arange(...):                     # line 128 — every cost() call is AFTER this


Line 126 overwrites ymax with a value 0.4 m smaller, and cost() resolves ymax at call time — so every single cost evaluation in the search maps beam endpoints onto a map shifted 0.4 m in y. Algebraically cost_buggy(x, y, yaw) == cost_true(x, y + 0.4, yaw), so the minimizer returns y − 0.4 m.


Proof, on your actual map


I downloaded /root/maps/room.{pgm,yaml,seed}, raycast an ideal 504-beam scan from the ground-truth pose in room.seed, and ran reseed.py's exact search twice:


┌──────────────────────────┬───────────────────────┬────────┬───────────────┐

│                          │     reported pose     │  cost  │     error     │

├──────────────────────────┼───────────────────────┼────────┼───────────────┤

│ ground truth (room.seed) │ (0.947, −0.401, 1.6°) │ —      │ —             │

├──────────────────────────┼───────────────────────┼────────┼───────────────┤

│ reseed as written        │ (0.950, −0.800, 2.0°) │ 0.0145 │ dy = −0.399 m │

├──────────────────────────┼───────────────────────┼────────┼───────────────┤

│ reseed with ymax fixed   │ (0.950, −0.400, 2.0°) │ 0.0145 │ dy = +0.001 m │

└──────────────────────────┴───────────────────────┴────────┴───────────────┘


Note the costs are identical. The bug is a coordinate shift, not a bad match — so the confidence gate (cost > 0.12 → abort) can never catch it. It reports a beautiful 0.014 m match and hands Nav2 a pose that is 0.4 m wrong. That's why every "the gate passed, localization is confirmed" message you've seen has been lying to you.


It matches your last real run exactly


/tmp/slam_loc.log (Aug 11, 17:24) — reseed seeded slam at (−0.10, −1.02, θ=0.02). room.seed, captured 40 seconds earlier at map save, says the rover was at (0.95, −0.40, θ=0.028). The rover never moved between those two events.


- yaw: identical (0.02 vs 0.028) — the match is angularly perfect

- position: 1.22 m off


Right yaw, wrong position is the signature of a coordinate-indexing bug, not of a genuine ambiguity or a bad map. On real data the 0.4 m shift also pushed the search into a neighbouring basin, which is why the live error is 1.2 m rather than exactly 0.4 m.


Why that makes nav look "random"


1. Localization is wrong by ~1.2 m, and slam_toolbox localization cannot recover it: correlation_search_space_dimension: 0.5 gives it a ±0.25 m search window, and minimum_travel_distance: 0.5 means it only re-matches after half a metre of driving. The wrong map→odom is permanent for the run.

2. Both costmaps have an obstacle_layer fed by /scan. With the pose 1.2 m off, the live scan stamps the room's real walls into the costmap 1.2 m away from where the static map has them — you get a double set of walls, and the corridor between the rover and the origin closes up.

3. NavFn then does what /tmp/nav.log shows: failed to create plan with tolerance 0.50 → Planning algorithm GridBased failed to generate a valid path to (0.00, 0.00) × 14, followed by Running backup, Running spin, Running wait. What you see as "the rover moving randomly" is the behavior tree running recovery behaviours — Nav2 never had a path at all. It never once reached the controller with a valid plan.

4. And the recoveries physically move the rover, which makes the already-wrong pose worse.


At the pose reseed believed (−0.10, −1.02), the nearest mapped wall is 0.15 m — inside the footprint's inscribed radius, with inflation_radius: 0.50. Nav2 thought it was starting inside a wall. At the true pose it had 0.20 m, and the goal (0,0) has 0.50 m clearance and is in the same free component as the seed — the map was plannable. You were right: this was never a map problem.


Also of note: Planner loop missed its desired rate of 5.0 Hz. Current loop rate is 0.52 Hz — NavFn is taking ~2 s per attempt on a 108×87 grid, which should be milliseconds. That's a separate CPU-starvation issue worth looking at after this, but it isn't what's breaking navigation.


The fix is one line


Rename the search bound at reseed.py:126 so it stops clobbering the map's top edge:


ysearch_min, ysearch_max = oy + 0.4, oy + h * res - 0.4

…and use ysearch_min/ysearch_max in the for y in np.arange(...) loop on line 129. cost() and the coverage check then see the true ymax from line 80. 
