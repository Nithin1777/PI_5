# Issue: Nav still failing — AMCL transform disagrees with raw lidar (consistency investigation)

**Date:** 2026-08-10 (evening session)
**Status:** Open — root cause identified; fix not yet verified end-to-end
**Severity:** High (blocks `~/rover nav` return-to-origin)

---

## 1. Symptom

`~/rover nav` keeps failing. Two distinct failure modes were observed this session:

1. **`return_home` ABORT: "AMCL never published /amcl_pose — localization NOT confirmed"** (nav run at ~17:09). AMCL accepted the seed (`Setting pose 1.247 -1.370 -1.274`) but never published a pose → no driving at all.
2. **Instant planner failure** (earlier run): `GridBased: failed to create plan with tolerance 0.50` within ~1 s of the goal being sent, followed by spin/backup recoveries → `Goal failed`.

The user's hypothesis: **"the real values being published [raw lidar distances] may not be similar to what the transform gives."**

---

## 2. Environment / state at test time

- Pi 5 (`10.7.12.41`), container `rover_ros2` (image `rover_ros2:saved`, `--privileged --network host --restart unless-stopped`), ROS 2 Humble.
- **UDP-only DDS active** (`FASTRTPS_DEFAULT_PROFILES_FILE=/root/no_shm.xml`) — fixed the FastDDS-SHM wedges (see §6.3).
- Base up: `LD19 → /scan_raw → scan_normalize (fixed 504-beam /scan) → rf2o + motor + foxglove`. `/scan` 10 Hz, `/odom_rf2o` 10 Hz, `odom → base_link` TF healthy.
- Map under test: `/root/maps/room.pgm` (saved 17:08, 138×127 @ 0.05 m, 6.90 × 6.35 m, free 36 % / unknown 56 % / occ 8 %).
- AMCL brought up via `localization_launch.py` (map_server + AMCL only; no Nav2, **no robot motion**).
- `nav2_params.yaml` AMCL: `set_initial_pose: false`, `max_particles: 500`, `min_particles: 150`, `update_min_d: 0.20`, `update_min_a: 0.15`, `transform_tolerance: 1.0`.
- **No valid seed exists** (stale `room.seed` was deleted earlier — the 17:08 `map save` seed capture had failed).

---

## 3. Tests run + outcomes

### 3.1 Parking-spot check (live `/scan` range profile)
Probe subscribed `/scan` (~4 s) and computed per-45°-sector range statistics.

| Sector (scan angle) | Nearest wall |
|---|---|
| Front-left 45–90° | 0.66 m (closest) |
| Front 0–45° | 0.68 m |
| Left 90–135° | 0.70 m |
| Back 180–225° | 0.86 m |
| Right 270–315° | 1.00 m |
| Front-right 315–360° | 1.40 m |
| Back-right 225–270° | 1.99 m (most open) |

**Outcome:** 0 readings < 0.5 m, 0 < 0.3 m. The rover is parked in **open space**, not against a wall. (Physical surroundings only — no `map → odom` TF up at that point.)

### 3.2 Scan vs transform timestamp offset
Probe measured, over 8 s, the age of the latest `/scan` stamp vs the latest `odom → base_link` transform stamp.

- scan latest stamp age: **0.002 s** (fresh)
- odom_tf latest stamp age: **0.102 s**; `tf_stamp − scan_stamp = −0.100 s` (80 tf messages)

**Outcome:** No systematic offset problem — the transforms lag the scans by ~0.1 s (rf2o stamps odom with the scan time it processed, i.e. sweep-start + pipeline delay). Side effect: AMCL's strict initial-pose transform lookup rejects seeds stamped with `now()` (`Failed to transform initial pose in time ... extrapolation into the future`), but it sets the pose anyway; benign for the scan pipeline (covered by `transform_tolerance: 1.0`).

### 3.3 Scan-to-scan stability of the map match (fixed pose)
At the fixed pose `(1.49, −1.38, yaw 169°)`, computed the mean distance of measured beam endpoints to the nearest map wall (distance-transform likelihood field) over **25 consecutive scans**.

- cost mean **0.161 m**, min 0.152, max 0.169, **std 0.004 m**
- NaN beams 52–68 / 504 (~11–13 %), median range 1.67 m

**Outcome:** The lidar data is **stable scan-to-scan**; the ~10 % NaN beam rate is the known LD19 quirk (non-fatal).

### 3.4 Brute-force scan-to-map match (the key test)
Method: distance-transform (chamfer BFS over the map's occupied cells, clamped 2.0 m) likelihood field; 63-beam subsample of `/scan` (every 8th beam, valid 0.1–2.8 m); coarse search over the map at 0.25 m / 15° yaw, refined to 0.02 m / 1°. Ran 3× (each on a fresh scan).

| Run | Best cost | Best pose | 2nd-best cluster (≥1 m away) |
|---|---|---|---|
| 1 | **0.022 m** | (1.46, −1.38, yaw 167°) | 0.072 m at (0.67, −0.61, yaw 120°) |
| 2 | **0.018 m** | (1.49, −1.38, yaw 169°) | 0.067 m at (0.67, −0.61, yaw 120°) |
| 3 (with direct recheck) | **0.025 m** | (1.46, −1.38, yaw 167°) — recheck of same pose = 0.025 m (search confirmed correct) | 0.071 m at (1.42, 0.14, yaw 30°) |

Also on the same scan as run 3: cost at the near-identical pose `(1.49, −1.38, 169°)` = **0.147 m**.

**Outcome:**
- The raw scan fits the saved map to **~2–3 cm** at **(1.46, −1.38, yaw ≈ 167–169°)** — the map and the sensor are **consistent**.
- The map is **ambiguous**: a second location (0.07 m cost) is a plausible-but-wrong match.
- The match landscape is **razor sharp**: 3 cm / 2° away from the best pose → cost jumps ~6× (0.025 → 0.15). Any small transform error therefore shows up immediately.

### 3.5 AMCL published pose vs the sensor-consistent pose

| When | AMCL pose | Map-match cost at that pose | Δ vs true pose |
|---|---|---|---|
| Broad seed, run A | (2.32, 0.40, yaw −14°) | 0.117 m | ~2 m + **180° flip** |
| Broad seed, run B | (1.39, −1.09, yaw 172°) | 0.090 m | ~0.3 m |
| Later (wandering) | (1.14, −0.75, yaw 150°) | 0.114 m | 0.6 m / 17° |

Convergence quality (broad seed, relaxed update gates): pose std **0.46 m**, covariance x 0.89 / yaw 6.49 rad² → **never converged**. With the stock `update_min_d/a` (0.20 / 0.15) a **stationary** robot gets only 1 update after seeding, then nothing (pose publication gated by motion).

### 3.6 Tight-seed test
Seeded AMCL **exactly** at the scan-matched pose `(1.49, −1.38, yaw 169°)` with cov 0.05, relaxed update gates, 500 particles, waited 20 s.

- Only 3 `/amcl_pose` samples; filter drifted to `(1.33, −1.05, yaw 169°)` — **0.37 m off the seed**, cost 0.078 m → **did NOT hold even a perfect seed**. (Caveat: small sample count.)

### 3.7 (Attempted) 2000-particle test
`nav2_params` variant with `max_particles: 2000, min_particles: 500` + relaxed gates prepared and launched; the verification probe hung (SSH read timeout / orchestration), so **inconclusive — not verified**. Load at the time was low (0.6), so it was not a CPU issue.

---

## 4. What this revealed (conclusion)

1. **The map is fine and the raw sensor is fine** — they agree to ~2–3 cm at one well-defined pose `(1.46, −1.38, yaw ≈ 168°)`.
2. **The published `map → odom` transform (AMCL) does NOT match the raw data.** AMCL either wandered, settled ~0.6 m / 17° away, or flipped ~180°. At AMCL's pose the scan-vs-map consistency is 4–6× worse than at the true pose.
3. **Root cause of the transform mismatch = AMCL localization failure**, not the map and not the sensor:
   - map ambiguity (a competing match cluster at ~0.07 m cost; 56 % unknown cells),
   - low particle count (500) on a sharp likelihood landscape,
   - stationary-robot update gating (`update_min_d/a`) preventing continuous correction,
   - classic symmetric-room 180° flip.
4. **Why nav breaks:** Nav2 plans from AMCL's (wrong) pose, so planner/controller work against a world that contradicts what the lidar sees → `failed to create plan` / wrong-way driving / ABORT.
5. The match landscape being razor-sharp means the localization must be nearly perfect for nav to work — it currently is not.

**User's hypothesis confirmed:** the published transform is not similar to the raw values — but the raw values are the truthful ones, and the map agrees with them.

---

## 5. Recommendations (next steps)

1. **Re-map with better coverage** — drive the middle, close the loop, reduce the 56 % unknown and the competing-match ambiguity. Confirm a `READY` grade with the seed/goal-clearance checks in `grade_map.py` (now gating on seed clearance ≥ 0.30 m).
2. **Raise AMCL particles** (500 → 2000) — params prepared (`nav2_params_loc3.yaml`, deleted after the session; recreate as needed) — **verify properly before relying on it**.
3. **Always nav with a valid seed** — the `map save` seed-capture guard (added this session) now refuses to leave a stale seed; the deleted `room.seed` must be regenerated by a fresh `map save`.
4. Consider lowering `update_min_d/a` or the effect is that a parked robot's AMCL never corrects; alternatively accept that AMCL only re-locks while moving.
5. Cosmetic: zombie `amcl`/`map_server` processes accumulate across `nav stop` → `nav` cycles (SIGKILL'd children of `ros2 launch` are not reaped); a container restart clears them.

---

## 6. Related infrastructure fixed earlier this session (context)

- **6.1 `scan_normalize.py`** — LD19 variable-length scans (501–505) resampled to a fixed 504-beam `/scan`; eliminated slam_toolbox's `expected 504` drops (verified: 0 warnings).
- **6.2 rf2o wedge** — rf2o froze mid-nav-bringup (FastDDS SHM transport failure `Failed init_port fastrtps_port7419: open_and_lock_file failed`), starving AMCL of `odom → base_link`. Fixed with UDP-only DDS profile `no_shm.xml` (verified: 0 RTPS errors, rf2o stable through a full nav bringup).
- **6.3 `stack.sh` guards** — `do_nav` fails fast if `/odom_rf2o` isn't publishing; `map save` aborts and deletes the seed on seed-capture failure; `ensure_amcl_active` now handles the `unconfigured` lifecycle state.

---

## 7. How to reproduce

1. Base up + `localization_launch.py` with `map:=/root/maps/room.yaml params_file:=/root/nav2_params.yaml use_sim_time:=false autostart:=true`.
2. Publish a broad-covariance `/initialpose` (TRANSIENT_LOCAL QoS) and confirm `Setting pose` in the AMCL log.
3. Run the brute-force match (method §3.4) on the live `/scan` vs `room.pgm`; compare the best-matching pose to `/amcl_pose`.
4. Expected: best match ~0.02–0.03 m at `(1.46, −1.38, ~168°)`; `/amcl_pose` elsewhere (0.09–0.15 m cost), typically wandering or flipped.

> All probe scripts were temporary and were removed after the session; the method above is sufficient to reproduce.
