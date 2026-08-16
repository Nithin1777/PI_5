# optimisations.md

Known broken / unoptimised things in this repo, as of 2026-08-16 (commit `858220b`).

Nothing here has been fixed. This is a list, not a changelog. Ordered by what will
hurt first, not by how hard it is.

Context for the whole document: **CPU is not this stack's constraint.** The heavy
lifting (`slam_toolbox`, `nav2`, `rf2o`, `ldlidar_stl_ros2`, `foxglove_bridge`) is
already C++. The Python here is glue plus one control loop plus one scan-match. A
C++ rewrite of this repo would buy approximately nothing — see §2 for the one real
hotspot and why numpy, not C++, is the fix.

---

## 1. SAFETY — Ctrl-C during a heading trim leaves the wheels turning

**Where:** `return_home.py:422-433` (the `finally` block), `return_home.py:212-214`
(the trim pulse loop), `stack.sh:213` and `stack.sh:395` (`deadman:=0.0`).

**What happens:**

The motor node's watchdog is deliberately disabled — the stack launches
`l298n_node.py` with `-p deadman:=0.0`, so a stale `/cmd_vel` **latches forever**
instead of timing out. That is intentional (latched teleop), and every other stop
path in the repo accounts for it:

- normal trim exit insists on a stop 30× (`return_home.py:215-218`)
- `rover nav stop` → `pkill_nav` → `stop_cmd.py` (`stack.sh:101`)
- SIGKILL on the motor node releases the GPIO lines, pins float, driver reads
  floating as off (`l298n_node.py:22`)

The one path that does **not**: `trim_heading()` publishes a rotation `Twist` in a
loop. If a `KeyboardInterrupt` lands inside that loop, it unwinds to
`except KeyboardInterrupt` (`return_home.py:419`), then `finally`, which cancels the
Nav2 goal — and **never publishes a zero Twist**. Nav2's cancel only stops velocities
Nav2 itself is producing; the trim pulse came from this script.

Result: rover keeps rotating at the last commanded duty until something else stops
it. `return_home.py` is run in the foreground by `stack.sh:366`, so Ctrl-C is a
realistic thing to press.

**Fix:** publish a stop `Twist` in the `finally`, using the same insist-30-times
pattern already at `return_home.py:215-218`. `trim_pub` is currently created at
`return_home.py:316` (inside the `try`), so it needs hoisting above the `try` to be
reachable from `finally`. ~5 lines.

---

## 2. PERFORMANCE — `reseed.py`'s scan-match search is the only real hotspot

**Where:** `reseed.py:139-149` (`cost()`), driven by `reseed.py:183-190` (coarse) and
`reseed.py:199-207` (refine).

`cost()` is a **pure-Python loop over beams**, called from inside a triple-nested
grid search. Measured against the actual saved map (`.maps/room.yaml`, 5.4 × 8.2 m):

```
map 108x165 px = 5.4 x 8.2 m
coarse grid: 19x30 pos x 24 yaw = 13,680 cost() calls x 50 beams
                                = 684,000 pure-Python inner iterations
cost() = 16 us
  -> coarse ~0.22 s on an i5-12450HX
  -> ~0.9 s on a Pi 5 (approx 4x scalar penalty)
```

That is the **coarse pass only**, before refine, and before the `pose_free` cull
(`reseed.py:185`) which does reduce the real count.

Two things make this worse than it looks:

- **It scales with map area ÷ step².** The current room is small. A space 3× larger
  per side is ~9× the coarse grid — call it 8 s per reseed.
- **`trim_heading` calls it up to 6 times** (`return_home.py:197-198`,
  `max_pulses=6`), each as a fresh subprocess (see §3). Each trim iteration is
  realistically 5-8 s wall clock, so a non-converging 6-pulse trim is ~40 s.

**Fix — numpy, not C++.** Vectorise `cost()` over beams, and for the coarse pass over
all 24 yaws at once. The per-yaw endpoint offsets `r*cos(yaw+a)` / `r*sin(yaw+a)` do
**not** depend on `x,y`, so they can be precomputed once for the whole coarse grid
instead of recomputed 13,680 times. Expect 10-50×.

**Two traps if anyone does this:**

1. `DT` is `float32` (`reseed.py:61`) but the original accumulates into a Python
   `float` (float64). A naive `np.where(...)` under NumPy 2's weak promotion stays
   float32 and shifts costs by ~1e-7 — enough to flip comparisons. Widen `DT` to
   float64 once, up front.
2. **The refine loop is order-dependent and must not be restructured.**
   `x0, y0, y0a` are mutated *inside* the innermost loop (`reseed.py:207`), while the
   `y` bounds are re-evaluated per `x` iteration and the `yaw` bounds per `(x,y)`
   iteration (`reseed.py:201`, `reseed.py:204`). The search window therefore
   re-centres mid-pass. Hoisting those `np.arange` calls out "for speed" silently
   changes the result. Refine is only ~15% of the cost anyway — vectorise inside it,
   leave its structure alone.

Any change here should be checked for numerical equivalence against the current
implementation on the real map before it goes near the rover.

**Already fixed, for reference:** the distance transform in `load_map` had a 248×
regression of exactly this kind, documented at `reseed.py:50-59` (23.44 s → 0.10 s).

---

## 3. ARCHITECTURE — `subprocess` + stdout scraping as IPC

**Where:** `return_home.py:139-155` (`absolute_pose`), parsing `reseed.py:246`.

`return_home` gets the rover's true pose by shelling out to `reseed.py` and grepping
stdout for a line starting with `POSE `. Every call pays:

- full Python + rclpy interpreter startup
- a fresh `/scan` subscription and wait (`reseed.py:113-117`, up to 10 s)
- a fresh map load + distance transform (`reseed.py:40-79`)
- the search from §2

...and this happens up to 6× per trim, plus once per drive attempt.

This is the single largest piece of duct tape in the repo. `reseed.py` should expose
an importable class holding a persistent node, a cached map/DT, and a
`locate(publish=True)` method; `main()` keeps the CLI exactly as-is so
`stack.sh:341` and `stack.sh:254` are unaffected.

**Non-obvious semantics that must be preserved if this is refactored:**

- `reseed.py` prints `POSE` at line 246 **before** the gates at lines 282-295. So on
  gate failure it exits 2 *having already printed a pose*, and `absolute_pose`
  ignores the exit code and returns that pose — while `/initialpose` was **not**
  published. Callers depend on this split.
- Each subprocess call currently gets a guaranteed-**fresh** scan because the
  subscription is brand new. A persistent subscription holds the *last* scan, which
  would be stale — `trim_heading` measures immediately after a pulse, so it must
  clear and wait for a genuinely new message or the trim reads its own pre-pulse
  pose and never converges.

---

## 4. ARCHITECTURE — no ROS package, no launch files

Everything is loose scripts in `/root`, orchestrated by a 469-line bash script.
Node parameters are inline `-p k:=v` strings in `stack.sh`.

Concrete consequences today:

- **The motor launch is duplicated verbatim** at `stack.sh:210-213` and
  `stack.sh:392-395`. Two places to keep in sync, already.
- **Process management is `pkill -9 -f <unanchored substring>`** plus `/tmp/*.pid`
  (`stack.sh:79-103`). `pkill -9 -f 'l298n_node'` matches any process whose command
  line contains that string.
- Adding a node is an O(n) manual edit: a `BASE_NAMES` entry (`stack.sh:25`), a
  `rosbg.sh` line, a `pkill` line, a `wait_log` call.

Not urgent on its own. It becomes urgent the moment there's a second sensor,
because every one of those edits gets made again by hand.

---

## 5. API FOOTGUN — `l298n_node` silently accepts parameters it ignores

**Where:** `l298n_node.py:178-194` (`_on_set_params`).

The callback returns `SetParametersResult(successful=True)` for **every** parameter,
but only applies three: `min_duty`, `max_duty`, `proportional`. Everything else —
`max_speed`, `max_level`, `min_level`, `deadman`, `track_width`, `left_is_a`,
`invert_left`, `invert_right` — is read once in `__init__` and cached on `self`.

So `ros2 param set /smart_elex_15d max_speed 0.5` prints **"Set parameter
successful"** and changes nothing. `max_speed` is directly in the proportional duty
formula (`l298n_node.py:208`), and the callback's own docstring says the whole point
is retuning live without restarting and losing the GPIO — so it is exactly the knob
someone reaches for mid-calibration.

Silent success is worse than rejection: the rover then contradicts a parameter you
just watched succeed.

**Fix:** return `successful=False` with a `reason` for names that aren't live-tunable,
so the CLI says "restart required" instead of lying.

**Minor, same function:** it mutates `self.*` during validation, before ROS has
committed the batch. If a later parameter in the same call were rejected, the node
keeps a partial change.

---

## 6. Adding a camera changes the Python calculus (but not the answer)

The "Python is fine" conclusion above is scoped to a **lidar-only** stack. 640×480
RGB at 30 fps is ~27 MB/s; any per-pixel work in Python is 100-1000× off, not 4×.

The conclusion is still not "rewrite the repo in C++" — it's **never put the pixel
loop in Python**. Use `cv_bridge` + numpy/OpenCV (C++ underneath) or an existing C++
node, and keep Python as glue. Same rule that already makes the lidar path fine.

Note that §4 is what a camera node lands on, and §2/§3 are already the slowest part
of a `nav` run.

---

## 7. Hardcoded container paths

`reseed.py:36` (`MAPS = "/root/maps"`), `return_home.py:35` (`MAPS_DIR`),
`return_home.py:139` (`/root/reseed.py`), and the `/root/...` paths throughout
`stack.sh`.

Nothing can be run or tested outside the container, including offline verification of
the §2 rewrite. An env-var override with the current value as the default
(`os.environ.get("ROVER_MAPS_DIR", "/root/maps")`) costs nothing and changes no
on-rover behaviour.

---

## 8. Hygiene

- **No `.gitignore`** — `__pycache__/*.pyc` are permanently untracked noise in
  `git status`.
- **~2 MB of PDFs in git history** (`LIDAR.pdf`, `15D_MotorDriver_RP_Manual.pdf`).
- **Stray temp file** committed: `.verify_reseed_tmp.py`.
- No tests. The §2 rewrite in particular has no way to prove it didn't change the
  localization result, which is the main reason it hasn't been done.

---

## 9. Known and accepted — not action items

- **Plaintext SSH password** in `ssh.py:5` and `push.py:17`, present in git history
  (commits `858220b`, `a93b570`), plus `AutoAddPolicy` (no host-key verification).
  Accepted: the rover is on a trusted LAN. Recorded here only so it isn't
  rediscovered and re-flagged later.
- **`speed N` is overridden by a calibrated `max_duty`.** Real, but already detected
  and explicitly warned about at `stack.sh:397-403`.
- **Motor state on `pkill -9`.** Looks unsafe, isn't: lgpio releases the lines when
  the process dies, pins float, and floating reads as off per the driver datasheet
  (`l298n_node.py:22`). `pkill_nodes` also runs `stop_cmd.py` before tearing down the
  base (`stack.sh:101`).
- **`scan_normalize.py:118`'s per-beam loop.** Measured at 100 µs/scan on x86, so
  ~0.4 ms on a Pi 5 at 10 Hz — roughly 0.4% of one core. Not worth touching.

---

## Suggested order

1. §1 — safety, ~5 lines, no behaviour change to a working path.
2. §5 — ~10 lines, prevents a whole class of wasted calibration sessions.
3. §7 — prerequisite for testing §2 offline.
4. §2 — the real speedup; needs an equivalence check against the current output.
5. §3 — the biggest structural win, and the most invasive. Do it after §2, not before.
