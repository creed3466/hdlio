# Server Docker Build and Parallel Operation Plan - 2026-05-16

## Scope

Server-side Docker environment setup and ROS1 Release build test for the HDLIO
reboot repository.

Local development remains limited to research, design, and implementation.
Build and evaluation are performed on:

- Server: `eutae@192.168.0.42`
- Project path: `~/Project/hdlio/`
- Dataset root: `~/Project/data/`

Do not store passwords in repo files, scripts, docs, or memory.

## Verified Repository State

- Remote commit: `945d50d`
- Remote branch: `main`
- Remote git status after build: clean
- Active Docker image: `tofslam:ros1`
- Image id: `d26e03e6c190`
- Image size: `6.08GB`

Dockerfile update in this verification:

- Added `time` for resource-logged builds.
- Added `libspdlog-dev` because `src/tof_slam/CMakeLists.txt` requires
  `find_package(spdlog REQUIRED)`.

## Server Hardware

| Resource | Observed value |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| Kernel | `6.17.0-29-generic` |
| Docker | `29.1.3`, overlayfs, cgroup systemd |
| CPU | Intel 13th Gen Core i7-13700KF |
| CPU layout | 24 logical CPUs, 16 physical cores, 1 socket, 1 NUMA node |
| P-core logical CPUs | `0-15`, paired as `0-1`, `2-3`, ..., `14-15` |
| E-core logical CPUs | `16-23`, single-thread siblings |
| L3 cache | 30 MiB |
| RAM | 31 GiB total |
| Swap | 8 GiB |
| GPU | NVIDIA RTX 4070 Ti, 12 GiB VRAM, driver 580.142 |
| Disk | NVMe root filesystem, 276G total, 68G free after image/build |
| Dataset footprint | `~/Project/data` on same root filesystem, about 171G used |

Docker Compose and Docker Buildx plugins were not installed. Current verified
operation uses direct `docker build` and `docker run`.

## Build Commands

Image build:

```bash
cd ~/Project/hdlio
docker build -t tofslam:ros1 -f baselines/docker/base.Dockerfile baselines
```

Persistent Release build:

```bash
cd ~/Project/hdlio
docker run --name hdlio_build_persist --rm \
  --cpuset-cpus=0-7 \
  --memory=12g \
  --ipc=private \
  --shm-size=2g \
  -v "$PWD:/root/catkin_ws:rw" \
  tofslam:ros1 bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    cd /root/catkin_ws
    /usr/bin/time -v catkin_make -DCMAKE_BUILD_TYPE=Release -j6
  '
```

The persistent build leaves ignored `build/` and `devel/` directories on the
server so later evaluation containers can source:

```bash
source /root/catkin_ws/devel/setup.bash
```

## Build Result

Status: PASS.

Output binary:

```text
~/Project/hdlio/devel/lib/tof_slam/tofslam_node
```

Recorded logs:

- `~/Project/hdlio/dump/server_build_test/catkin_make_release.log`
- `~/Project/hdlio/dump/server_build_test/catkin_make_release_persistent.log`

Persistent build timing:

- Command: `catkin_make -DCMAKE_BUILD_TYPE=Release -j6`
- Wall time: `0:25.05`
- Max RSS: `1035008 KB`
- CPU utilization: `326%`
- Exit status: `0`

Non-blocking warnings observed:

- PCL/VTK optional imported target warnings.
- OpenNI/libusb optional feature disabled warnings.
- `catkin_package()` warning for `Eigen3_INCLUDE_DIRS`.
- C++ warnings for unused locals, ignored `[[unlikely]]` placement, and
  preprocessor directives embedded in macro arguments.

These warnings did not block the ROS1 build.

## Parallel Docker Operation Plan

Default policy:

- Use one Docker image, `tofslam:ros1`, for HDLIO ROS1 build/eval.
- Mount datasets read-only from `~/Project/data`.
- Use one active config per sensor family:
  - Avia: `src/tof_slam/config/avia.yaml`
  - Mid360: `src/tof_slam/config/mid360.yaml`
  - NTU/Ouster: `src/tof_slam/config/ntu.yaml`
- Write every run to a unique output directory under `dump/`.
- Avoid `--ipc=host`; use `--ipc=private --shm-size=2g` unless a specific ROS
  tool requires otherwise.

Recommended build mode:

- Run one build container at a time.
- Use `--cpuset-cpus=0-7 --memory=12g -j6` as the default stable setting.
- For faster one-off rebuilds, `--cpuset-cpus=0-15 -j10` is acceptable, but it
  will take all P-cores and should not run concurrently with evaluations.

Recommended evaluation mode:

- Full-rate or publishable evaluation: run 2 containers max.
- Screening evaluation: run 3 containers max only when memory and disk I/O are
  monitored.
- Avoid 4 concurrent LIO containers on this machine because RAM is 31 GiB and
  datasets share the same NVMe filesystem.

Two-container layout for stable evaluation:

| Container | CPU set | Memory cap | Thread env |
|---|---:|---:|---|
| `hdlio_eval_a` | `0-7` | `8g` | `OMP_NUM_THREADS=4` |
| `hdlio_eval_b` | `8-15` | `8g` | `OMP_NUM_THREADS=4` |
| Host/I/O reserve | `16-23` | remaining | E-cores reserved |

Three-container layout for screening only:

| Container | CPU set | Memory cap | Thread env |
|---|---:|---:|---|
| `hdlio_eval_a` | `0-5` | `7g` | `OMP_NUM_THREADS=3` |
| `hdlio_eval_b` | `6-11` | `7g` | `OMP_NUM_THREADS=3` |
| `hdlio_eval_c` | `12-15,16-19` | `7g` | `OMP_NUM_THREADS=3` |
| Host/I/O reserve | `20-23` | remaining | E-cores reserved |

Evaluation container template:

```bash
docker run --rm --name hdlio_eval_a \
  --cpuset-cpus=0-7 \
  --memory=8g \
  --ipc=private \
  --shm-size=2g \
  -e OMP_NUM_THREADS=4 \
  -e OPENBLAS_NUM_THREADS=1 \
  -e MKL_NUM_THREADS=1 \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 \
  -v "$PWD:/root/catkin_ws:rw" \
  -v "$HOME/Project/data:/data:ro" \
  tofslam:ros1 bash -lc '
    set -e
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash
    cd /root/catkin_ws
    # run sensor-family config evaluation here
  '
```

If host networking is required by a legacy ROS runner, use unique
`ROS_MASTER_URI` ports per container, for example `11311`, `11312`, and
`11313`.

## Open Follow-Ups

- Migrate or replace legacy `docker/run_*.sh` scripts before treating them as
  active evaluation entry points.
- Install Docker Compose/Buildx plugins only if a compose-based runner becomes
  necessary.
- Consider fixing the non-blocking C++ warnings before using warnings as a
  CI-quality gate.
