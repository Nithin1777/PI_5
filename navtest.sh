#!/usr/bin/env bash
# navtest.sh [map_name] -- one full displace-then-return-home cycle, with ground truth.
#
# Runs the SAME displacement every time (forward, right turn, forward) so runs are
# comparable, then `stack.sh nav`, then measures where the rover ACTUALLY ended up.
#
# The final measurement deliberately uses reseed.py --no-publish rather than the TF
# map->base_link. Nav2 reporting "Goal succeeded" only means its BELIEF reached the
# goal, and the belief is exactly what has been wrong: one run finished 22 deg off
# tolerance while reporting success. reseed re-localizes against the saved .pgm from
# scratch, so it is independent of the estimate under test. --no-publish because this
# is a measurement, not a correction -- it must not alter what it is measuring.
#
# Run several times to get a pass RATE; a single pass has never been the problem.
# NO `set -u` here. /opt/ros/humble/setup.bash expands unset variables internally, which
# is FATAL under set -u in a non-interactive shell -- the script died on the source line
# and, because that line's stderr was redirected away, left a zero-byte log and no clue.
MAP="${1:-room}"
export LC_ALL=C
source /opt/ros/humble/setup.bash

echo "=================== navtest cycle: map=$MAP ==================="
echo "--- pose before displacement ---"
python3 /root/tf_pose.py map base_link

echo "--- displacing (forward, right turn, forward) ---"
# Durations were 0.6/0.6/1.0 s. Those were sized when the drivetrain ran 2.93x faster
# than commanded; after the duty mapping was corrected the same pulses displaced the
# rover only 0.16 m, so nav started 0.08 m from the goal -- already inside the 0.15 m
# tolerance, and the "test" measured nothing. Lengthened to restore a displacement big
# enough that Nav2 actually has to plan and drive.
python3 /root/pub_cmd.py 2.0 0.10 0.0  >/dev/null; sleep 3
python3 /root/pub_cmd.py 1.5 0.0  -0.5 >/dev/null; sleep 3
python3 /root/pub_cmd.py 2.0 0.10 0.0  >/dev/null; sleep 3

echo "--- pose after displacement (start of the run) ---"
START=$(python3 /root/tf_pose.py map base_link)
echo "$START"

echo "--- navigating home ---"
# Capture to a file and filter AFTERWARDS rather than piping into a live grep.
# stack.sh's pkill_nav runs `pkill -9 -f 'return_home'`, which matches any process whose
# COMMAND LINE contains that string -- including a `grep -E "...|return_home|..."` sitting
# in the pipeline. That killed the filter mid-run and the pipeline reported rc=137 with
# the nav step apparently vanishing. Filtering after the fact also makes NAV_RC the exit
# status of stack.sh itself instead of the last command in a pipe.
/root/stack.sh nav "$MAP" > /tmp/navstep.log 2>&1
NAV_RC=$?
grep -E "reseed|return.home|verify|ABORT|ERROR" /tmp/navstep.log

echo "--- TRUE final pose (independent scan-match x2) ---"
for i in 1 2; do
  timeout 90 python3 /root/reseed.py --map-name "$MAP" --no-publish 2>&1 \
    | grep -E "^POSE|best scan-match"
done

echo "--- nav error counts ---"
for p in "failed to create plan" "Unable to transform robot pose" \
         "patience exceeded" "Collision Ahead"; do
  printf "  %-34s %s\n" "$p" "$(grep -ic "$p" /tmp/nav.log 2>/dev/null)"
done
echo "=================== end cycle (nav rc=$NAV_RC) ==================="
