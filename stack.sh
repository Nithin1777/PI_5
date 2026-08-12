#!/bin/bash
# /root/stack.sh -- rover stack manager (runs INSIDE rover_ros2 container)
#
#   rover start | stop | restart | status | logs <node>
#   rover map | map save [name] | map stop        (mapping phase)
#   rover nav [name] | nav stop                    (navigation phase)
#
# Lifecycle (return-to-origin demo):
#   1. `start`       -> base only: lidar, scan_normalize, rf2o, motor, foxglove  (no slam)
#   2. `map`         -> slam_toolbox online_sync (drives map->odom); drive in Foxglove
#   3. `map save`    -> save /root/maps/<name>.{pgm,yaml} + capture B-pose seed
#   4. `nav`         -> kill slam; slam_loc (localization) + reseed + map_server + Nav2 (no AMCL); return_home -> origin
#
# (drop the rover at point A BEFORE `map` -- slam_toolbox starts the robot at
#  map (0,0,0), so the origin == your drop point; `nav` returns there.)
export LC_ALL=C                     # avoid setlocale warnings over SSH
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/no_shm.xml   # UDP-only DDS: FastDDS SHM transport wedges nodes under load (froze rf2o mid-nav bringup)
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null

SLAM_YAML=/root/slam.yaml
SLAM_LOC_YAML=/root/slam_loc.yaml
NAV_YAML=/root/nav2_params.yaml
MAPS_DIR=/root/maps
BASE_NAMES=(lidar scan rf2o motor foxglove)
DEFAULT_MAP=room
MAXLEVEL=$(cat /root/.motor_speed 2>/dev/null || echo 3)   # motor POWER level 1..9 (floor=ceiling); set via 'speed <n>'

# ---------------------------------------------------------------- helpers -----
ensure_slam_yaml() {
  if ! grep -q 'base_frame: base_link' "$SLAM_YAML" 2>/dev/null; then
    cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_sync.yaml "$SLAM_YAML"
    sed -i 's/base_frame: base_footprint/base_frame: base_link/' "$SLAM_YAML"
    echo "[stack] created $SLAM_YAML (base_frame=base_link)"
  fi
}

ensure_slam_loc_yaml() {
  # slam_toolbox LOCALIZATION params: Ceres scan-match inside a serialized pose-graph.
  # (repo slam_loc.yaml is deployed to /root; this only recreates it if missing.)
  if ! grep -q 'mode: localization' "$SLAM_LOC_YAML" 2>/dev/null; then
    cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_localization.yaml "$SLAM_LOC_YAML"
    sed -i 's/base_frame: base_footprint/base_frame: base_link/' "$SLAM_LOC_YAML"
    sed -i 's/mode: mapping #localization/mode: localization/' "$SLAM_LOC_YAML"
    echo '    use_sim_time: false' >> "$SLAM_LOC_YAML"
    echo "[stack] created $SLAM_LOC_YAML (localization mode, base_frame=base_link)"
  fi
}

pkill_base()  {
  for n in "${BASE_NAMES[@]}"; do [ -f /tmp/$n.pid ] && kill "$(cat /tmp/$n.pid)" 2>/dev/null; rm -f /tmp/$n.pid; done
  pkill -9 -f 'ldlidar_stl_ros2_node'        2>/dev/null
  pkill -9 -f 'rf2o_laser_odometry_node'     2>/dev/null
  pkill -9 -f 'static_transform_publisher'   2>/dev/null
  pkill -9 -f 'l298n_node'                   2>/dev/null
  pkill -9 -f 'foxglove_bridge'              2>/dev/null
  pkill -9 -f 'scan_normalize'               2>/dev/null    # LD19 resampler -> /scan
  pkill -9 -f 'ros2 launch ldlidar_stl_ros2' 2>/dev/null
  pkill -9 -f 'ros2 launch rf2o_laser_odometry' 2>/dev/null
  pkill -9 -f 'ros2 launch foxglove_bridge'  2>/dev/null
}
pkill_slam()  {
  for n in slam slam_loc; do [ -f /tmp/$n.pid ] && kill "$(cat /tmp/$n.pid)" 2>/dev/null; rm -f /tmp/$n.pid; done
  pkill -9 -f 'sync_slam_toolbox_node|async_slam_toolbox_node|lifelong_slam|localization_slam_toolbox_node' 2>/dev/null
  pkill -9 -f 'ros2 launch slam_toolbox'     2>/dev/null
}
pkill_nav()   {
  [ -f /tmp/nav.pid ] && kill "$(cat /tmp/nav.pid)" 2>/dev/null; rm -f /tmp/nav.pid
  pkill -9 -f 'return_home'                  2>/dev/null
  pkill -9 -f 'ros2 launch nav2_bringup'     2>/dev/null
  pkill -9 -f 'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager|map_server|/amcl|component_container' 2>/dev/null
  sleep 0.3; python3 /root/stop_cmd.py 2>/dev/null   # motor latches STOP (deadman is off)
}
pkill_nodes() { pkill_nav; pkill_slam; pkill_base; }

running() { [ -f /tmp/$1.pid ] && kill -0 "$(cat /tmp/$1.pid 2>/dev/null)" 2>/dev/null; }

wait_nav_up() {   # wait until bt_navigator appears (return_home waits for ACTIVE)
  for i in $(seq 1 40); do
    if timeout 4 ros2 node list 2>/dev/null | grep -q '/bt_navigator'; then return 0; fi
    sleep 1
  done
  return 1
}

wait_tf() {        # wait until TF $1 -> $2 resolves (slam localization publishes map->odom)
  for i in $(seq 1 40); do
    if timeout 4 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -qm1 'Translation'; then return 0; fi
    sleep 1
  done
  return 1
}

ensure_amcl_active() {
  # The nav2 lifecycle_manager_localization sometimes loses a service race on bringup
  # ("amcl/get_state ... async_send_request failed" -> "Aborting bringup") and leaves
  # AMCL inactive OR UNCONFIGURED, so it rejects the /initialpose seed / never
  # subscribes to /map. Detect that and bring it up manually.
  for i in $(seq 1 30); do
    st=$(timeout 4 ros2 lifecycle get /amcl 2>/dev/null | grep -oiE "active|inactive|unconfigured" | head -1)
    case "$st" in
      active)   return 0 ;;
      unconfigured) echo "[nav] AMCL unconfigured (lifecycle bringup race) -> configuring + activating manually ...";
                timeout 6 ros2 lifecycle set /amcl configure >/dev/null 2>&1; sleep 1
                timeout 6 ros2 lifecycle set /amcl activate >/dev/null 2>&1; sleep 1.5 ;;
      inactive) echo "[nav] AMCL inactive (lifecycle bringup race) -> activating manually ...";
                timeout 6 ros2 lifecycle set /amcl activate >/dev/null 2>&1; sleep 1.5 ;;
      *)        sleep 1 ;;   # not up yet
    esac
  done
  st=$(timeout 4 ros2 lifecycle get /amcl 2>/dev/null | grep -oiE "active" | head -1)
  [ "$st" = "active" ] && return 0
  echo "[nav] WARNING: AMCL would not activate -- localization will fail."; return 1
}

# ---------------------------------------------------------------- base --------
do_start() {
  do_stop >/dev/null 2>&1
  sleep 1
  echo "[stack] starting lidar (LD19) ......"; /root/rosbg.sh lidar    ros2 launch ldlidar_stl_ros2 ld19.launch.py
  # NOTE: ld19.launch.py is patched to publish /scan_raw (topic_name: scan_raw).
  # scan_normalize.py (next line) resamples it onto a fixed 504-beam /scan for
  # rf2o/slam/AMCL. If the container is ever rebuilt/re-cloned, re-apply the patch:
  #   sed -i "s/{'topic_name': 'scan'}/{'topic_name': 'scan_raw'}/" \
  #     /root/ros2_ws/src/ldlidar_stl_ros2/launch/ld19.launch.py
  sleep 3
  echo "[stack] starting scan_normalize ....."; /root/rosbg.sh scan     python3 /root/scan_normalize.py
  sleep 2
  echo "[stack] starting rf2o (laser odom) ."; /root/rosbg.sh rf2o     ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
  # NOTE: rf2o_laser_odometry.launch.py is patched 'freq' : 20.0 -> 10.0 (update
  # timer must match the 10 Hz /scan, else rf2o spams "Waiting for laser_scans"
  # on every other tick; odometry itself was unaffected). Re-apply if rebuilt:
  #   sed -i "s/'freq' : 20.0/'freq' : 10.0/" \
  #     /root/ros2_ws/src/rf2o_laser_odometry/launch/rf2o_laser_odometry.launch.py
  sleep 3
  echo "[stack] starting motor (SmartElex) ."
  /root/rosbg.sh motor python3 /root/l298n_node.py \
     --ros-args -p left_is_a:=false -p max_speed:=0.3 -p max_level:=$MAXLEVEL -p min_level:=$MAXLEVEL -p invert_right:=true -p deadman:=0.0
  sleep 1
  echo "[stack] starting foxglove_bridge ..."; /root/rosbg.sh foxglove ros2 launch foxglove_bridge foxglove_bridge_launch.xml num_threads:=2
  sleep 3
  echo
  echo "=== readiness ==="
  grep -m1 "ready" /tmp/motor.log 2>/dev/null && echo "  [motor]  OK (subscribed to /cmd_vel)" || echo "  [motor]  starting..."
  grep -m1 "ready" /tmp/scan.log 2>/dev/null && echo "  [scan]   OK (LD19 -> fixed 504-beam /scan)" || echo "  [scan]   starting..."
  grep -m1 "Listening" /tmp/foxglove.log 2>/dev/null >/dev/null && echo "  [foxglove] bridge up" || tail -1 /tmp/foxglove.log 2>/dev/null
  echo
  echo "[stack] BASE UP (lidar + scan_normalize + rf2o + motor + foxglove)."
  echo "  next: \`map\` to begin mapping, then \`map save\`, then \`nav\`."
  echo "  Foxglove: ws://<pi-ip>:8765"
}

do_stop() {
  pkill_nodes
  sleep 1
  echo "[stack] stopped (all nodes killed)."
}

# ---------------------------------------------------------------- mapping -----
do_map() {
  ensure_slam_yaml
  if ! running rf2o; then echo "[map] base not up -- run 'start' first."; exit 1; fi
  pkill_slam; sleep 1
  echo "[map] starting slam_toolbox online_sync (owns map->odom) ..."
  /root/rosbg.sh slam ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=$SLAM_YAML use_sim_time:=false
  sleep 3
  grep -m1 "Registering sensor" /tmp/slam.log 2>/dev/null && echo "[map] OK -- /map building." \
    || echo "[map] warming up -- check 'logs slam'."
  echo "  drive around in Foxglove, then run:  map save"
}

do_mapsave() {
  name="${1:-$DEFAULT_MAP}"
  mkdir -p "$MAPS_DIR"
  if ! pgrep -f sync_slam_toolbox_node >/dev/null 2>&1; then
    echo "[map save] slam is not running -- run 'map' and drive first."; exit 1
  fi
  echo "[map save] capturing pose (map->base_link) for localization seed ..."
  SEED_FAIL=0
  if ! python3 /root/tf_pose.py map base_link "$MAPS_DIR/$name.seed"; then
    SEED_FAIL=1
    rm -f "$MAPS_DIR/$name.seed"          # NEVER let a stale seed survive a failed capture
    echo "[map save] ERROR: could not capture pose -- deleted any STALE seed so nav won't use it."
    echo "[map save]        (map->odom->base_link must resolve; slam running + rf2o publishing odom. Fix, then re-run 'map save'.)"
  fi
  echo "[map save] saving map to $MAPS_DIR/$name.{pgm,yaml} ..."
  timeout 30 ros2 run nav2_map_server map_saver_cli -f "$MAPS_DIR/$name" --occ 0.65 --free 0.25 2>&1 | tail -3
  if [ -f "$MAPS_DIR/$name.yaml" ]; then
    echo "[map save] serializing slam pose-graph (for slam localization mode) ..."
    if timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
         "{filename: '$MAPS_DIR/$name'}" >/dev/null 2>&1 && \
       [ -f "$MAPS_DIR/$name.posegraph" ] && [ -f "$MAPS_DIR/$name.data" ]; then
      echo "[map save] pose-graph OK: $MAPS_DIR/$name.{posegraph,data}"
    else
      echo "[map save] WARNING: pose-graph serialize FAILED -- slam localization mode needs it; re-run 'map save'."
    fi
    echo "[map save] DONE."
    echo "  eyeball:  $MAPS_DIR/$name.pgm   (or /map in Foxglove)"
    echo "  return:   nav ${name}"
    echo
    echo "[map save] === MAP QUALITY GRADE ==="
    python3 /root/grade_map.py "$name" 2>/dev/null
    rc=$?
    [ "$rc" -ne 0 ] && [ "$rc" -ne 2 ] && echo "[map save] (grader unavailable -- is python3-numpy installed?)"
    if [ "$SEED_FAIL" = 1 ]; then
      echo "[map save] WARNING: map saved but NO seed (seed capture failed) -- nav will use global localization only."
      echo "[map save]          Fix the TF and re-run 'map save' to get a proper seed."
      exit 1
    fi
  else
    echo "[map save] FAILED -- map not written. Is the map non-empty in Foxglove?"; exit 1
  fi
}

do_mapstop() { pkill_slam; sleep 1; echo "[map stop] slam killed (base still up)."; }

# ---------------------------------------------------------------- navigation --
do_nav() {
  name="${1:-$DEFAULT_MAP}"
  if [ ! -f "$NAV_YAML" ]; then echo "[nav] missing $NAV_YAML -- deploy nav2_params.yaml first."; exit 1; fi
  if [ ! -f "$MAPS_DIR/$name.yaml" ]; then
    echo "[nav] no map '$name' ($MAPS_DIR/$name.yaml) -- run 'map save $name' first."; exit 1; fi
  if ! running rf2o; then echo "[nav] base not up (rf2o missing) -- run 'start' first."; exit 1; fi
  if ! timeout 6 ros2 topic hz /odom_rf2o 2>/dev/null | grep -qm1 'average rate'; then
    echo "[nav] rf2o is NOT publishing /odom_rf2o (wedged/dead -- a known FastDDS-SHM failure)."
    echo "[nav] Fix: '~/rover restart' (now uses UDP-only DDS), then verify '~/rover status' shows /odom_rf2o."
    exit 1
  fi

  echo "[nav] map->odom owner = slam_toolbox LOCALIZATION (Ceres scan-match; AMCL retired) ..."
  pkill_nav; pkill_slam; sleep 2          # rf2o keeps odom->base_link throughout
  ensure_slam_loc_yaml
  sed -i "s|map_file_name: .*|map_file_name: $MAPS_DIR/$name|" "$SLAM_LOC_YAML"
  /root/rosbg.sh slam_loc ros2 run slam_toolbox localization_slam_toolbox_node \
      --ros-args --params-file "$SLAM_LOC_YAML" -r /map:=/slam_loc_map
  # NOTE: -r /map:=/slam_loc_map REMAPS slam's posegraph-derived /map away --
  # slam_loc publishes its own occupancy grid (cropped to the traveled area, so the
  # map origin can fall in UNKNOWN space) and map_server publishes the full saved
  # pgm. With both on /map the global costmap saw two conflicting maps and the
  # planner failed to reach (0,0) ("failed to create plan ... tolerance 0.50").
  # map_server is now the ONLY /map owner; slam_loc only owns map->odom.
  sleep 3
  echo "[nav] scan-match re-seeding slam localization (consistency gate) ..."
  if ! python3 /root/reseed.py --map-name "$name"; then
    echo "[nav] ABORT: could not establish a lidar-consistent pose (see reseed output)."
    pkill_slam
    exit 2
  fi
  echo "[nav] waiting for map -> odom from slam localization ..."
  if ! wait_tf map odom; then
    echo "[nav] slam localization never published map->odom -- check 'logs slam_loc'."; pkill_slam; exit 1; fi
  echo "[nav] bringing up map_server (static /map for costmaps) ..."
  /root/rosbg.sh nav2_map ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAPS_DIR/$name.yaml
  sleep 2
  timeout 6 ros2 lifecycle set /map_server configure >/dev/null 2>&1
  timeout 6 ros2 lifecycle set /map_server activate >/dev/null 2>&1
  echo "[nav] bringing up Nav2 navigation stack (no amcl) ..."
  /root/rosbg.sh nav ros2 launch nav2_bringup navigation_launch.py \
      params_file:=$NAV_YAML use_sim_time:=false use_composition:=False autostart:=true
  echo "[nav] waiting for Nav2 lifecycle ..."
  if ! wait_nav_up; then
    echo "[nav] Nav2 did not come up in 40s -- check 'logs nav'."; exit 1; fi
  echo "[nav] Nav2 up -- returning to origin (map 0,0,0) ..."
  echo "          (Ctrl-C here only stops the driver; Nav2 stays up. Use 'nav stop' to end.)"
  python3 /root/return_home.py --map-name "$name" --no-amcl
}

do_navstop() { pkill_nav; pkill_slam; sleep 1; echo "[nav stop] Nav2 + slam localization killed (base still up)."; }

do_speed() {
  lvl="${1:-}"
  if [ -z "$lvl" ]; then
    navvel=$(grep -E '^[[:space:]]*max_velocity:' "$NAV_YAML" 2>/dev/null | head -1 | grep -oE '[0-9.]+' | head -1)
    echo "[speed] motor power level = $MAXLEVEL   (ALL motion -- teleop + nav -- runs at this level; raise for stronger turns / more speed, lower if it slips)"
    echo "[speed] nav  max_velocity = ${navvel:-?} m/s   (nav-only, in $NAV_YAML)"
    echo "[speed] set with:  speed <1..9>   (saved to /root/.motor_speed; persists across restarts)"
    return
  fi
  case "$lvl" in [1-9]) ;; *) echo "[speed] level must be 1..9"; return 1;; esac
  echo "$lvl" > /root/.motor_speed
  MAXLEVEL="$lvl"
  echo "[speed] motor max_level -> $lvl   (saved)"
  if ! running lidar; then echo "[speed] base not up -- applies on next 'start'."; return; fi
  pkill -9 -f l298n_node 2>/dev/null; rm -f /tmp/motor.pid; sleep 1
  /root/rosbg.sh motor python3 /root/l298n_node.py \
     --ros-args -p left_is_a:=false -p max_speed:=0.3 -p max_level:=$lvl -p min_level:=$lvl -p invert_right:=true -p deadman:=0.0
  echo "[speed] motor restarted. Teleop now uses it; nav picks it up on next 'nav'."
}

# ---------------------------------------------------------------- status/logs --
do_status() {
  echo "=== phase ==="
  pgrep -f sync_slam_toolbox_node    >/dev/null && echo "  MAPPING  (slam online_sync)" || true
  pgrep -f '/bt_navigator'           >/dev/null && echo "  NAV ON   (map_server+AMCL+Nav2)" || true
  pgrep -f return_home               >/dev/null && echo "  RETURNING (return_home driving to origin)" || true
  running lidar || running rf2o || running motor || { echo "  (base not running -- run 'start')"; }
  echo
  echo "=== processes ==="
  ps -eo pid,args | grep -iE 'ldlidar_stl_ros2_node|scan_normalize|rf2o_laser_odometry_node|sync_slam_toolbox_node|localization_slam_toolbox_node|l298n_node|foxglove_bridge|controller_server|planner_server|bt_navigator|/amcl|map_server|velocity_smoother' \
                  | grep -v grep | sed 's/ --ros-args.*//' | cut -c1-95
  echo
  echo "=== ros2 nodes ==="
  timeout 8 ros2 node list 2>/dev/null | sort
  echo
  printf "/scan: "; timeout 4 ros2 topic hz /scan 2>&1 | grep -m1 'average rate' || echo "(no rate)"
  printf "/odom_rf2o: "; timeout 4 ros2 topic hz /odom_rf2o 2>&1 | grep -m1 'average rate' || echo "(no rate)"
  echo "TF map -> base_laser:"
  timeout 5 ros2 run tf2_ros tf2_echo map base_laser 2>&1 | grep -E 'Translation|degree' | head -2
}

do_logs() {
  n="${1:-}"
  case "$n" in lidar|scan|rf2o|slam|slam_loc|motor|foxglove|nav) ;; *) echo "usage: stack logs <lidar|scan|rf2o|slam|slam_loc|motor|foxglove|nav>"; return 1;; esac
  [ -f /tmp/$n.log ] || { echo "no log: /tmp/$n.log"; return 1; }
  tail -n 50 -f /tmp/$n.log
}

# ---------------------------------------------------------------- dispatch ----
case "${1:-}" in
  start)     do_start ;;
  stop)      do_stop ;;
  restart)   do_stop; sleep 2; do_start ;;
  status)    do_status ;;
  logs)      shift; do_logs "$@" ;;
  map|mapsave)
    case "${2:-}" in
      save)         shift 2; do_mapsave "$@" ;;
      stop|mapstop) do_mapstop ;;
      "")           do_map ;;
      *)            echo "usage: stack map {save [name] | stop}"; exit 1 ;;
    esac ;;
  nav|navstop)
    if [ "${1:-}" = navstop ] || [ "${2:-}" = stop ]; then do_navstop
    else shift; do_nav "$@"; fi ;;
  speed)       shift; do_speed "$@" ;;
  *) cat <<EOF
usage: stack {start|stop|restart|status|logs <node>}
       stack {map|map save [name]|map stop}
       stack {nav [name]|nav stop}
       stack speed [1..9]

  start         base: lidar + scan_normalize + rf2o + motor + foxglove
  map           begin mapping (slam online_sync); drive in Foxglove
  map save [n]  save map + localization seed  (default name: room)
  map stop      stop mapping (keep base)
  nav [n]       localize (slam_toolbox localization) + navigate back to origin (map 0,0,0)
  nav stop      stop navigation (keep base)
  stop          kill everything
  speed [1..9]  set/show motor POWER level (teleop + nav); default 3; saved
EOF
  exit 1 ;;
esac
