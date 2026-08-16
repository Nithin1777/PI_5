#!/bin/bash
# /root/rosbg.sh <name> <command...>  -- launch a ROS node detached with logging
#
# (Now versioned in the repo. It lived ONLY in the container before, so a rebuild
#  would have silently lost it even though stack.sh cannot run without it.)
NAME="$1"; shift
if [ -f /tmp/$NAME.pid ]; then
  OLDPID=$(cat /tmp/$NAME.pid 2>/dev/null)
  [ -n "$OLDPID" ] && kill "$OLDPID" 2>/dev/null
  sleep 0.5
fi
nohup setsid bash -c 'source /opt/ros/humble/setup.bash; source /root/ros2_ws/install/setup.bash 2>/dev/null; exec "$@"' rosbg "$@" > /tmp/$NAME.log 2>&1 &
NEWPID=$!
echo "$NEWPID" > /tmp/$NAME.pid
# Was `sleep 2` -- x5 nodes that was 10 s of the ~24 s `start` spent asleep. It
# only ever backed the "did it exec at all" check below; real readiness is now
# polled by stack.sh's wait_log/wait_topic/wait_node, which also detect a node
# that dies later in startup (this check never could).
sleep 0.3
if kill -0 "$NEWPID" 2>/dev/null; then
  echo "[$NAME] ALIVE pid=$NEWPID  log=/tmp/$NAME.log"
else
  echo "[$NAME] EXITED EARLY — check log"
  echo "--- tail ---"
  tail -n 10 /tmp/$NAME.log 2>/dev/null      # only on failure now; the old
fi                                           # unconditional tail printed nothing
                                             # useful and cost a 2 s sleep to fill
