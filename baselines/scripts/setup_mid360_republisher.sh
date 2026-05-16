#!/usr/bin/env bash
# setup_mid360_republisher.sh — helper sourced by run_inside_<algo>.sh when
# DATASET=mid360. Compiles the livox_v2_to_v1_republish package (mounted at
# /republisher_src) inside the currently-loaded catkin workspace, launches
# the republisher node against the roscore the caller has already started,
# and blocks until /livox/mid360/lidar_v1 appears as an advertised topic so
# downstream subscribers don't miss early messages.
#
# Expected env (set by caller):
#   CATKIN_WS            — path to the baked catkin workspace (e.g. /root/catkin_ws)
#   ROS_MASTER_URI       — already exported, roscore already running
#   ROS_HOSTNAME         — already exported
#
# Expected mount (set by run_baseline.sh for DATASET=mid360):
#   /republisher_src     — baselines/tools/livox_v2_to_v1_republish (ro)
#
# Side effects:
#   * Builds /tmp/repub_ws overlay (non-destructive; baked workspace stays
#     clean).  Overlay-sources into the calling shell.
#   * Starts `republish_node` as a background process. REPUB_PID is exported
#     so the caller can kill -INT it on shutdown.
#   * Exits 1 on compile failure or advertise timeout.
set -euo pipefail

if [[ ! -d /republisher_src ]]; then
    echo "[MID360] ERROR: /republisher_src not mounted"; exit 1
fi

echo "[MID360] compiling livox_v2_to_v1_republish overlay workspace"
REPUB_WS="/tmp/repub_ws"
rm -rf "${REPUB_WS}"
mkdir -p "${REPUB_WS}/src"
cp -r /republisher_src "${REPUB_WS}/src/livox_v2_to_v1_republish"

# Chain to the baked devel so livox_ros_driver v1 headers/msgs resolve.
# shellcheck disable=SC1091
source "${CATKIN_WS}/devel/setup.bash"

pushd "${REPUB_WS}" >/dev/null
catkin_make \
    --source src \
    --build build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_DEVEL_PREFIX="${REPUB_WS}/devel" \
    >/tmp/repub_build.log 2>&1 || {
        echo "[MID360] ERROR: republisher compile failed (see /tmp/repub_build.log)"
        tail -20 /tmp/repub_build.log || true
        exit 1
    }
popd >/dev/null
echo "[MID360] republisher built OK"

# Overlay the new pkg so rosrun/roslaunch can find it
# shellcheck disable=SC1091
source "${REPUB_WS}/devel/setup.bash"

# Launch the republisher — background
rosrun livox_v2_to_v1_republish republish_node \
    _input_topic:=/livox/mid360/lidar \
    _output_topic:=/livox/mid360/lidar_v1 \
    _queue_size:=200 \
    >/out/republisher.log 2>&1 &
REPUB_PID=$!
export REPUB_PID
echo "[MID360] republisher started (pid=${REPUB_PID})"

# Wait until the output topic is advertised (publisher lazy-advertises on
# first input msg; it also publishes nothing until rosbag play starts.
# rostopic list doesn't show the topic until a publisher exists — but the
# algorithm subscribes eagerly via roslaunch, and ROS late-binds the
# subscriber to publishers as they appear. Blocking on `rostopic list`
# would deadlock. Instead: verify the node is up via `rosnode ping`.
deadline=$(( $(date +%s) + 15 ))
while (( $(date +%s) < deadline )); do
    if rosnode ping -c1 /livox_v2_to_v1_republish >/dev/null 2>&1; then
        echo "[MID360] republisher node pingable"
        break
    fi
    sleep 0.5
done

if ! rosnode ping -c1 /livox_v2_to_v1_republish >/dev/null 2>&1; then
    echo "[MID360] ERROR: republisher node did not come up within 15 s"
    cat /out/republisher.log 2>/dev/null | tail -20 || true
    kill -INT "${REPUB_PID}" 2>/dev/null || true
    exit 1
fi

echo "[MID360] republisher ready — caller may now launch algorithm and rosbag play"
