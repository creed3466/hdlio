#!/bin/bash
# run_timing_final.sh — Extract timing from baseline internal CSV/logs
# FAST-LIO2/Point-LIO write CSV when runtime_pos_log_enable=true
# iG-LIO Timer::PrintAll() prints to stdout at shutdown (need --screen + roslaunch)
set -e
cd "$(dirname "$0")/.."

RATE="1.0"
SEQ="Dark01"
CPUSET="0-3"
PORT=11311
MEM="4g"

echo "========================================="
echo "  Baseline Timing (final)"
echo "  Started: $(date)"
echo "========================================="

# ===== FAST-LIO2 =====
echo ""
echo "=== FAST-LIO2 ==="
docker rm -f timing_flio 2>/dev/null || true

# Clear old timing log
rm -f baselines/algorithms/fast_lio2/Log/fast_lio_time_log.csv

docker run -d --rm --name timing_flio \
  --network host --cpuset-cpus "$CPUSET" --memory "$MEM" --ipc private \
  -v "$(pwd)/baselines/algorithms/fast_lio2:/root/catkin_ws/src/fast_lio2" \
  -v "/home/euntae/Project/dataset/ros1/surfel_data:/root/catkin_ws/data/m3dgr_surfel:ro" \
  baselines-fast_lio2:ros1 bash -lc "sleep infinity" > /dev/null
sleep 2

docker exec timing_flio bash -c \
  "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3"

docker exec timing_flio bash -c "
  source /opt/ros/noetic/setup.bash
  source /root/catkin_ws/devel/setup.bash
  export ROS_MASTER_URI=http://localhost:${PORT}

  killall -9 rosmaster roscore 2>/dev/null || true; sleep 1
  fuser -k ${PORT}/tcp 2>/dev/null || true; sleep 1

  roscore -p ${PORT} &
  sleep 5
  rosparam set /use_sim_time true

  # Load config with timing enabled
  rosparam load /root/catkin_ws/src/fast_lio2/config/avia.yaml
  rosparam set /runtime_pos_log_enable true
  rosparam set /pcd_save/pcd_save_en false

  # Ensure Log directory exists
  mkdir -p /root/catkin_ws/src/fast_lio2/Log

  rosrun fast_lio fastlio_mapping &
  SLAM_PID=\$!
  sleep 3

  rosbag play /root/catkin_ws/data/m3dgr_surfel/${SEQ}.bag --clock -r ${RATE} --delay 3.0 2>&1 | tail -3
  sleep 5

  kill \$SLAM_PID 2>/dev/null || true
  wait \$SLAM_PID 2>/dev/null || true
  sleep 2
"

docker rm -f timing_flio 2>/dev/null || true

# Extract timing from CSV
FLIO_CSV="baselines/algorithms/fast_lio2/Log/fast_lio_time_log.csv"
if [ -f "$FLIO_CSV" ]; then
  # CSV has columns: total_time, etc. Get mean of first column (total time per scan in ms)
  FLIO_MS=$(python3 -c "
import csv
with open('$FLIO_CSV') as f:
    r = csv.reader(f)
    header = next(r, None)
    vals = []
    for row in r:
        if row and len(row) > 0:
            try: vals.append(float(row[0]))
            except: pass
    if vals:
        print(f'{sum(vals)/len(vals):.1f}')
    else:
        print('EMPTY')
" 2>/dev/null)
  echo "FAST-LIO2: ${FLIO_MS} ms/scan (from CSV, n=$(wc -l < $FLIO_CSV) frames)"
else
  echo "FAST-LIO2: CSV not found at $FLIO_CSV"
  FLIO_MS="FAIL"
fi
echo "FAST-LIO2 done."

# ===== iG-LIO =====
echo ""
echo "=== iG-LIO ==="
docker rm -f timing_iglio 2>/dev/null || true

docker run -d --rm --name timing_iglio \
  --network host --cpuset-cpus "$CPUSET" --memory "$MEM" --ipc private \
  -v "$(pwd)/baselines/algorithms/ig_lio:/root/catkin_ws/src/ig_lio" \
  -v "/home/euntae/Project/dataset/ros1/surfel_data:/root/catkin_ws/data/m3dgr_surfel:ro" \
  baselines-ig_lio:ros1 bash -lc "sleep infinity" > /dev/null
sleep 2

docker exec timing_iglio bash -c \
  "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3"

# iG-LIO: Timer::PrintAll() writes to glog (stderr). Capture everything.
docker exec timing_iglio bash -c "
  source /opt/ros/noetic/setup.bash
  source /root/catkin_ws/devel/setup.bash
  export ROS_MASTER_URI=http://localhost:${PORT}
  export GLOG_logtostderr=1

  killall -9 rosmaster roscore 2>/dev/null || true; sleep 1
  fuser -k ${PORT}/tcp 2>/dev/null || true; sleep 1

  roscore -p ${PORT} &
  sleep 5
  rosparam set /use_sim_time true
  rosparam load /root/catkin_ws/src/ig_lio/config/avia.yaml

  rosrun ig_lio ig_lio_node &
  SLAM_PID=\$!
  sleep 3

  rosbag play /root/catkin_ws/data/m3dgr_surfel/${SEQ}.bag --clock -r ${RATE} --delay 3.0 2>&1 | tail -3
  sleep 5

  kill -INT \$SLAM_PID 2>/dev/null || true
  wait \$SLAM_PID 2>/dev/null || true
  sleep 2

  # Timer prints at exit; also check glog files
  find /tmp -name 'ig_lio_node*' -newer /root/catkin_ws/devel/setup.bash 2>/dev/null | head -5
  for f in /tmp/ig_lio_node.*.log.*; do
    if [ -f \"\$f\" ]; then
      grep 'average time usage' \"\$f\" && cp \"\$f\" /root/catkin_ws/src/ig_lio/timing_log.txt
    fi
  done 2>/dev/null || true
" 2>&1 | tee /tmp/iglio_timing_raw.log

docker rm -f timing_iglio 2>/dev/null || true

# Parse iG-LIO timing from captured output or glog file
IGLIO_MS=$(grep "average time usage" /tmp/iglio_timing_raw.log baselines/algorithms/ig_lio/timing_log.txt 2>/dev/null | \
  awk -F'average time usage: ' '{print $2}' | awk '{print $1}' | sort -rn | head -1 | awk '{printf "%.1f", $1}')
echo "iG-LIO: ${IGLIO_MS:-FAIL} ms/scan"
echo "iG-LIO done."

# ===== Point-LIO =====
echo ""
echo "=== Point-LIO ==="
docker rm -f timing_plio 2>/dev/null || true

# Clear old logs
rm -f baselines/algorithms/point_lio/Log/*.csv

docker run -d --rm --name timing_plio \
  --network host --cpuset-cpus "$CPUSET" --memory "$MEM" --ipc private \
  -v "$(pwd)/baselines/algorithms/point_lio:/root/catkin_ws/src/point_lio" \
  -v "/home/euntae/Project/dataset/ros1/surfel_data:/root/catkin_ws/data/m3dgr_surfel:ro" \
  baselines-point_lio:ros1 bash -lc "sleep infinity" > /dev/null
sleep 2

docker exec timing_plio bash -c \
  "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 2>&1 | tail -3"

docker exec timing_plio bash -c "
  source /opt/ros/noetic/setup.bash
  source /root/catkin_ws/devel/setup.bash
  export ROS_MASTER_URI=http://localhost:${PORT}

  killall -9 rosmaster roscore 2>/dev/null || true; sleep 1
  fuser -k ${PORT}/tcp 2>/dev/null || true; sleep 1

  roscore -p ${PORT} &
  sleep 5
  rosparam set /use_sim_time true

  rosparam load /root/catkin_ws/src/point_lio/config/avia.yaml
  rosparam set /runtime_pos_log_enable true
  rosparam set /pcd_save/pcd_save_en false

  mkdir -p /root/catkin_ws/src/point_lio/Log

  rosrun point_lio pointlio_mapping &
  SLAM_PID=\$!
  sleep 3

  rosbag play /root/catkin_ws/data/m3dgr_surfel/${SEQ}.bag --clock -r ${RATE} --delay 3.0 2>&1 | tail -3
  sleep 5

  kill \$SLAM_PID 2>/dev/null || true
  wait \$SLAM_PID 2>/dev/null || true
  sleep 2
"

docker rm -f timing_plio 2>/dev/null || true

# Extract Point-LIO timing
PLIO_CSV=$(ls baselines/algorithms/point_lio/Log/*time*.csv 2>/dev/null | head -1)
if [ -n "$PLIO_CSV" ] && [ -f "$PLIO_CSV" ]; then
  PLIO_MS=$(python3 -c "
import csv
with open('$PLIO_CSV') as f:
    r = csv.reader(f)
    header = next(r, None)
    vals = []
    for row in r:
        if row and len(row) > 0:
            try: vals.append(float(row[0]))
            except: pass
    if vals:
        print(f'{sum(vals)/len(vals):.1f}')
    else:
        print('EMPTY')
" 2>/dev/null)
  echo "Point-LIO: ${PLIO_MS} ms/scan"
else
  echo "Point-LIO: CSV not found"
  ls baselines/algorithms/point_lio/Log/ 2>/dev/null
  PLIO_MS="FAIL"
fi
echo "Point-LIO done."

echo ""
echo "========================================="
echo "  Per-scan timing (ms), Dark01, i7-12700"
echo "========================================="
printf "  %-12s: %s ms\n" "FAST-LIO2" "${FLIO_MS:-FAIL}"
printf "  %-12s: %s ms\n" "iG-LIO" "${IGLIO_MS:-FAIL}"
printf "  %-12s: %s ms\n" "Point-LIO" "${PLIO_MS:-FAIL}"
printf "  %-12s: %s ms\n" "TofSLAM" "16.9 (internal)"
echo "========================================="
echo "  Done: $(date)"
