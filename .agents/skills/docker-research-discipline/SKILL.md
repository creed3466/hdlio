---
name: docker-research-discipline
description: |
  Docker discipline for ROS1 / ROS2 / ML training research with parallel
  container isolation, determinism guarantees, and lessons from real
  research mistakes (sequence leakage, zombie processes, rosbag rate
  drift, DDS DOMAIN_ID collision, RMW mismatch, QoS drop, ROS1/ROS2
  mix-up).
  TRIGGER when: user runs docker / docker compose / catkin_make /
  colcon build / rosbag play / ros2 bag play / ros2 launch / ns-train /
  torchrun in a research project, or asks "how do I run my SLAM /
  training in docker".
  DO NOT TRIGGER for plain app-dev docker (web/SaaS).
origin: rcc
---

# docker-research-discipline (ROS1 + ROS2)

Research Docker is **not** app-dev Docker. The failure modes are
different. And **ROS1 and ROS2 have different failure modes from each
other** — port-based vs DDS-based isolation, rosmaster vs ros2 daemon,
TCP vs DDS discovery. This skill encodes both.

| App-dev Docker | Research Docker |
|---|---|
| restart policies, healthchecks | inter-sequence state isolation |
| HTTP probes | rosbag `--clock` + sim_time |
| horizontal scaling | CPU pinning, ipc:private |
| log aggregation | per-run determinism gates |

| ROS1 (noetic) | ROS2 (humble / jazzy / rolling) |
|---|---|
| `ROS_MASTER_URI` (port) | `ROS_DOMAIN_ID` (DDS group) |
| `rosmaster` single point | `ros2 daemon` + DDS discovery |
| TCP/UDP transport | DDS middleware (FastDDS / CycloneDDS) |
| QoS: implicit | QoS: explicit (reliability, durability, …) |
| Port isolation: 11311/12/13 | DOMAIN_ID isolation: 0..232 |
| Cleanup: `killall rosmaster` + restart | Cleanup: `ros2 daemon stop` + clear `~/.ros` + restart |
| Bag: `.bag` (`rosbag play`) | Bag: `.db3` / `.mcap` (`ros2 bag play`) |
| Build: `catkin_make` | Build: `colcon build` |

The 14 known traps below list each trap as **ROS1 / ROS2 / Common**.

---

## Trap 1 — Missing `--clock` (sim_time disconnect)

`use_sim_time` must be true AND the bag must publish `/clock`.

### ROS1

```bash
rosbag play <bag> --clock -r 1.0
# Inside the SLAM node launch:
rosparam set /use_sim_time true
```

### ROS2

```bash
ros2 bag play <bag> --clock -r 1.0
# Pass to every node:
ros2 launch my_pkg slam.launch.py use_sim_time:=true
# Or in code: declare_parameter('use_sim_time', True)
```

### Common

Without `/clock`, TF buffers, message filters, and synchronizers
desync silently. **Hook T1 / T7** block this.

---

## Trap 2 — `--wait-for-subscribers` hangs on multi-topic bags

### ROS1

```bash
# DON'T:
rosbag play <bag> --wait-for-subscribers   # hangs forever if multi-topic
```

The flag waits for **every** topic to have a subscriber. Multi-topic
bags include camera / UWB / etc. that your SLAM node doesn't consume.

Fix: use `--topics` to restrict, never use `--wait-for-subscribers`.

### ROS2

The flag **does not exist** in `ros2 bag play`. ROS2's equivalent
problem is publisher QoS not matching subscriber QoS — messages
silently dropped. See **Trap 11**.

### Common

Hook **T2** blocks `--wait-for-subscribers` (ROS1 only).

---

## Trap 3 — Zombie process / discovery cache accumulation

### ROS1

```bash
killall -9 rosmaster roscore roslaunch rosout 2>/dev/null || true
sleep 2
if pgrep -x rosmaster > /dev/null; then
  echo "FATAL: zombie rosmaster"
  exit 1   # parent script triggers docker restart
fi
```

`docker exec`-launched background processes are reparented to
docker-init. `killall` alone is unreliable.

### ROS2

```bash
ros2 daemon stop 2>/dev/null || true
pkill -f ros2 2>/dev/null || true
sleep 2
# Clear DDS discovery cache — critical for ROS2 determinism
rm -rf ~/.ros/log ~/.ros/fastdds* /dev/shm/fastrtps* /tmp/fastdds* 2>/dev/null || true
```

ROS2 has no rosmaster, but the DDS daemon caches discovery state and
each implementation writes to filesystem / shared memory. **Without
clearing these, ghost nodes from previous runs may be visible.**

### Common

`docker restart <container>` is the **only** universally reliable
cleanup. Use it between sequences for both.

---

## Trap 4 — Build cache vs volume mount inode mismatch

### ROS1 (`catkin_make`)

```bash
# Symptom: code edits don't change runtime
find /root/catkin_ws/src/<pkg>/src -name '*.cpp' -exec touch {} \;
catkin_make -j4
# or
catkin_make -j4 --force-cmake
# or hard rebuild
rm -rf /root/catkin_ws/build /root/catkin_ws/devel
catkin_make -DCMAKE_BUILD_TYPE=Release -j4
```

### ROS2 (`colcon`)

```bash
# Symptom: same. CMake cache + colcon symlink-install.
find /root/ros2_ws/src/<pkg>/src -name '*.cpp' -exec touch {} \;
colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release
# or force-cmake
colcon build --packages-select <pkg> --cmake-force-configure
# or hard rebuild
rm -rf /root/ros2_ws/build /root/ros2_ws/install /root/ros2_ws/log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Common

Bind-mount preserves mtime when files come from `git checkout` /
extracts. CMake's stamp file comparison fails. Force a touch or
`--force-cmake` / `--cmake-force-configure`.

---

## Trap 5 — Rate > 1.0 → non-deterministic + pose drop

### ROS1

```bash
rosbag play <bag> --clock -r 1.0   # deterministic
rosbag play <bag> --clock -r 3.0   # SCREENING ONLY — not reproducible
```

### ROS2

```bash
ros2 bag play <bag> --clock -r 1.0   # deterministic
ros2 bag play <bag> --clock -r 3.0   # SCREENING ONLY
```

### Common

Replay > real-time → frame drops + queue watermark race. Determinism
breaks. Declarative enforcement:

```json
{
  "docker_research": {
    "playback_rate": "1.0",
    "playback_rate_warn_if_other": true
  }
}
```

Hook **T3** enforces. T3 applies to both `rosbag play` and `ros2 bag play`.

---

## Trap 6 — Parallel container isolation

### ROS1 — port-based

`network_mode: host` shares ports. Two `roscore` on 11311 conflict.

```yaml
services:
  p1:
    network_mode: host
    ipc: private
    cpuset: "0-3"
    environment:
      ROS_MASTER_URI: "http://localhost:11311"
  p2:
    network_mode: host
    ipc: private
    cpuset: "4-7"
    environment:
      ROS_MASTER_URI: "http://localhost:11312"   # different port
  p3:
    network_mode: host
    ipc: private
    cpuset: "8-11"
    environment:
      ROS_MASTER_URI: "http://localhost:11313"
```

### ROS2 — DDS DOMAIN_ID based

`network_mode: host` is fine — DDS routes by `ROS_DOMAIN_ID`. **Two
containers with the same DOMAIN_ID see each other.**

```yaml
services:
  p1:
    network_mode: host
    ipc: private
    cpuset: "0-3"
    environment:
      ROS_DOMAIN_ID: "100"
      ROS_LOCALHOST_ONLY: "1"           # 멀티캐스트 끔
      RMW_IMPLEMENTATION: "rmw_fastrtps_cpp"   # ALL containers same!
  p2:
    ipc: private
    cpuset: "4-7"
    environment:
      ROS_DOMAIN_ID: "101"              # different domain
      ROS_LOCALHOST_ONLY: "1"
      RMW_IMPLEMENTATION: "rmw_fastrtps_cpp"
  p3:
    ipc: private
    cpuset: "8-11"
    environment:
      ROS_DOMAIN_ID: "102"
      ROS_LOCALHOST_ONLY: "1"
      RMW_IMPLEMENTATION: "rmw_fastrtps_cpp"
```

### Common

CPU pinning + `ipc: private` + memory limit always applies.

Hook **T6** warns ROS1+ROS2 if `--ipc private` missing with
`parallel_containers > 1`. **T8** warns when same `ROS_DOMAIN_ID` used.
**T9** warns when `RMW_IMPLEMENTATION` differs across containers.

---

## Trap 7 — Debug build distorts timing

### ROS1

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -j4   # required for timing experiments
```

### ROS2

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release    # required
```

### Common

Debug-built binary changes per-frame latency enough to push messages
across queue watermarks differently. Determinism dies in Debug.

Hook **T5** (ROS1 catkin) and **T12** (ROS2 colcon) warn.

---

## Trap 8 — Startup races (TCP / DDS)

### ROS1 — Determinism Fix A+B v2 pattern

**H1**: subscriber wall-clock attach time varies → 100-sample
gravity-init IMU window cut at different positions per run.

**H2**: queue watermark too small → IMU vs LiDAR dequeue race on
first frame.

```yaml
# YAML config
deterministic_queue: true
deterministic_queue_delay_ms: 100   # was 5
```

```bash
# Launcher gate
for i in $(seq 1 60); do
  sub=$(rostopic info "$IMU" | awk '/Subscribers:/{f=1;next} f&&/\*/')
  [ -n "$sub" ] && break
  sleep 0.2
done
rosbag play "$BAG" --clock -r 1.0 --delay 3.0   # TCP handshake gate
```

### ROS2 — DDS discovery race

DDS discovery is asynchronous and can take 100ms–10s depending on
implementation + transport. Publisher may start broadcasting before
subscriber finishes discovery.

```bash
# Pre-flight: confirm discovery completed
ros2 topic list | grep -q "<expected_topic>"
ros2 topic info "<expected_topic>" --verbose | grep -q "Publisher count: 1"
ros2 topic info "<expected_topic>" --verbose | grep -q "Subscription count: 1"

# Then: start bag with --delay
sleep 3
ros2 bag play <bag> --clock -r 1.0 --start-offset 0.5
```

For multi-process work: pre-warm DDS by spawning a transient publisher
+ subscriber on the first topic before the real launch.

### Python (PyTorch / sklearn) — RNG race

```python
import random, numpy as np, torch, os
SEED = 42
random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED); torch.cuda.manual_seed_all(SEED)
torch.backends.cudnn.deterministic = True
torch.backends.cudnn.benchmark = False
os.environ['PYTHONHASHSEED'] = str(SEED)

# DataLoader workers need worker_init_fn:
def _worker_init(wid):
    np.random.seed(SEED + wid)
DataLoader(..., num_workers=N, worker_init_fn=_worker_init)
```

### Common

Wall-clock dependencies of any kind threaten determinism. Add a
**startup gate** before the first real message and an **explicit delay**
before bag publish.

---

## Trap 9 — ROS1 / ROS2 image / entrypoint mismatch

### Common

Symptom: container loads ROS2 `setup.bash` but workspace is `catkin_ws`
(ROS1), or `ros_entrypoint.sh` sources `/opt/ros/humble/setup.bash`
while compose mounts ROS1 source paths.

### Fix — separate files

```
docker/
  Dockerfile.ros1      # FROM osrf/ros:noetic-desktop-full + catkin
  Dockerfile.ros2      # FROM osrf/ros:humble-desktop-full + colcon
  ros1_entrypoint.sh   # /opt/ros/noetic/setup.bash + catkin_ws
  ros2_entrypoint.sh   # /opt/ros/humble/setup.bash + ros2_ws
  compose.ros1.yml     # ROS1 services
  compose.ros2.yml     # ROS2 services
```

Never let a single `compose.yml` reference both.

---

## Trap 10 (ROS2-specific) — `ros2 launch` missing `use_sim_time:=true`

When playing back a bag with `--clock`, every node in the launch
must opt in:

```bash
ros2 launch my_pkg slam.launch.py use_sim_time:=true
```

Or set as a parameter inside the launch:

```python
Node(
    package='my_pkg',
    executable='slam_node',
    parameters=[{'use_sim_time': True}],
)
```

Without this, the node uses wall-clock and the `/clock` topic is
ignored. Hook **T10** warns when `ros2 launch` runs alongside
`ros2 bag play`.

---

## Trap 11 (ROS2-specific) — QoS profile mismatch

Publisher and subscriber must agree on Reliability + Durability +
History or messages silently drop.

```python
# Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)
self.create_publisher(MsgType, '/topic', qos)
```

```python
# Subscriber must match (or be more permissive)
self.create_subscription(MsgType, '/topic', cb, qos)
```

Or use a preset:

```python
from rclpy.qos import qos_profile_sensor_data       # best-effort, volatile, depth=5
from rclpy.qos import qos_profile_system_default    # reliable, volatile, depth=10
```

**`research-config.json.docker_research.qos_reliability_required: true`**
makes the hook scan code for `ReliabilityPolicy.BEST_EFFORT` and warn
unless intentionally declared.

---

## Trap 12 (ROS2-specific) — `colcon build` without Release

Already addressed under **Trap 7 / hook T12**.

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# or
colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Trap 13 (ROS2-specific) — Mixed `RMW_IMPLEMENTATION` across containers

`rmw_fastrtps_cpp` and `rmw_cyclonedds_cpp` do **not** interoperate
out of the box. If parallel containers ship different defaults,
nodes can't discover each other → silent failure.

```yaml
environment:
  RMW_IMPLEMENTATION: "rmw_fastrtps_cpp"   # ALL containers identical
```

Hook **T9** verifies via `cfg.docker_research.rmw_implementation`.

---

## Trap 14 (ROS2-specific) — `ROS_DOMAIN_ID` collision

```yaml
# Container 1
environment:
  ROS_DOMAIN_ID: "100"
# Container 2
environment:
  ROS_DOMAIN_ID: "100"   # WRONG — they see each other
```

Allocate distinct DOMAIN_IDs (range 0–232 valid):

```json
{
  "docker_research": {
    "ros_domain_id_base": 100,
    "parallel_containers": 3
    // p1 = 100, p2 = 101, p3 = 102
  }
}
```

Hook **T8** detects same DOMAIN_ID + `parallel_containers > 1`.

---

## Verification checklist (run before every experiment)

```
Common:
[ ] Residual processes = 0 (ROS1: pgrep rosmaster; ROS2: pgrep -f ros2)
[ ] Previous trajectory file deleted
[ ] Bag exists + checksum matches dataset_locks (if pinned)
[ ] Container CPU pinning matches plan
[ ] ipc: private if parallel

ROS1 specific:
[ ] use_sim_time = true (rosparam set)
[ ] rosbag play has --clock
[ ] Multi-topic bag: --topics specified
[ ] Build mtime > source mtime (or --force-cmake done)
[ ] catkin_make Release build

ROS2 specific:
[ ] use_sim_time:=true passed to ros2 launch (or in node parameters)
[ ] ros2 bag play has --clock
[ ] ROS_DOMAIN_ID set per container (distinct)
[ ] RMW_IMPLEMENTATION identical across parallel containers
[ ] ROS_LOCALHOST_ONLY=1 if no multicast needed
[ ] QoS profiles compatible (publisher reliability ≥ subscriber)
[ ] ~/.ros/log + /dev/shm/fastrtps* cleared between sequences
[ ] colcon build Release build
```

---

## Parallel container templates

### ROS1 — 3-way port-based

```bash
# bash docker/run_parallel_ros1.sh <LABEL> <CFG1>:<L1> <CFG2>:<L2> <CFG3>:<L3>
set -e
LABEL="$1"; shift

for i in 1 2 3; do
  cpus="$(( (i-1)*4 ))-$(( i*4-1 ))"
  port=$(( 11310 + i ))
  docker run -d --rm --name "exp_${LABEL}_p$i" \
    --network host --cpuset-cpus "$cpus" --memory 3g --ipc private \
    -e ROS_MASTER_URI="http://localhost:$port" \
    -e ROS_HOSTNAME="localhost" \
    -v "$(pwd)/src:/root/catkin_ws/src:ro" \
    -v "$(pwd)/dump:/root/catkin_ws/dump" \
    tofslam:ros1 bash -lc "sleep infinity"
done
sleep 3

docker exec "exp_${LABEL}_p1" bash -c \
  "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && \
   catkin_make -DCMAKE_BUILD_TYPE=Release -j4"
docker restart "exp_${LABEL}_p2" "exp_${LABEL}_p3"
```

### ROS2 — 3-way DOMAIN_ID-based

```bash
# bash docker/run_parallel_ros2.sh <LABEL> ...
set -e
LABEL="$1"; shift
BASE_DOMAIN=100
RMW="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

for i in 1 2 3; do
  cpus="$(( (i-1)*4 ))-$(( i*4-1 ))"
  domain=$(( BASE_DOMAIN + i - 1 ))
  docker run -d --rm --name "exp_${LABEL}_p$i" \
    --network host --cpuset-cpus "$cpus" --memory 3g --ipc private \
    -e ROS_DOMAIN_ID="$domain" \
    -e ROS_LOCALHOST_ONLY=1 \
    -e RMW_IMPLEMENTATION="$RMW" \
    -v "$(pwd)/src:/root/ros2_ws/src:ro" \
    -v "$(pwd)/dump:/root/ros2_ws/dump" \
    tofslam:ros2 bash -lc "sleep infinity"
done
sleep 3

docker exec "exp_${LABEL}_p1" bash -c \
  "source /opt/ros/humble/setup.bash && cd /root/ros2_ws && \
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
docker restart "exp_${LABEL}_p2" "exp_${LABEL}_p3"
```

---

## Integration with RCC pipeline state

The docker hook can update `research-state.json`:

```json
{
  "docker_runs": [
    {
      "timestamp": "2026-05-11T11:45:00Z",
      "container": "exp_canonical_p1",
      "ros_version": "ros2",
      "ros_distro": "humble",
      "domain_id": 100,
      "rmw": "rmw_fastrtps_cpp",
      "sequence": "Avia/dark01",
      "rate": "1.0",
      "command": "ros2 bag play ... --clock -r 1.0",
      "verdict": "OK"
    }
  ]
}
```

---

## Anti-patterns

- **Don't** mix `--rate 3.0` outputs with `--rate 1.0` outputs in the
  same plot or table (both ROS).
- **Don't** trust `killall` alone — verify (ROS1: pgrep rosmaster;
  ROS2: pgrep -f ros2).
- **Don't** put two parallel ROS1 containers on the same
  `ROS_MASTER_URI` port.
- **Don't** use the same `ROS_DOMAIN_ID` in two parallel ROS2 containers.
- **Don't** mix `RMW_IMPLEMENTATION` across ROS2 parallel containers.
- **Don't** rely on `volume mount + (catkin_make|colcon build)` to
  pick up code changes. Touch or `--force-cmake`.
- **Don't** put `0_docker_rule.md` outside the project — it must
  travel with the source.
- **Don't** mix ROS1 and ROS2 in one `compose.yml`.
- **Don't** use `ros2 bag play` without `--clock` when paired with
  `use_sim_time:=true` (silent desync).
- **Don't** use `--wait-for-subscribers` (ROS1) on multi-topic bags.

---

## Related

- skill: `dataset-versioning` — checksums prevent cross-experiment contamination
- skill: `experiment-tracking` — log container + ros_distro + domain_id to W&B/MLflow tags
- hook: `pre-bash-research-docker-check` — enforces traps T1–T13
- hook: `gpu-profile-snapshot` — nvidia-smi on ml-train docker runs
- schema: `research-config.json.docker_research` — declarative rules (ros_distro switches ROS1/ROS2 branches)
- command: `/reproduce` — verifies dataset + builds before re-running
