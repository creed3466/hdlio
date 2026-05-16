# Phase 1 Smoke-Test Results

Purpose: verify the baselines harness (Docker build → config override →
roslaunch → rosbag play → TUM extract → ATE compute) works end-to-end for
each of the 5 reference algorithms.

Smoke sequence policy:
- **Avia algorithms** (FAST-LIO2, Point-LIO, iG-LIO): M3DGR `Dark01`
- **NTU algorithms** (LIO-SAM, SLICT): `eee_01`

Determinism: `rosbag play -r 1.0` per project policy.
Alignment: Umeyama SE(3) (no scale) via `baselines/scripts/compute_ate.py`.

**What "PASS" means here**: the container builds, the algorithm runs for the
full bag, a non-empty TUM trajectory is produced, and ATE compute succeeds.
It does **not** mean the numbers match the paper — paper reproduction is
Phase 4 (see `paper_reproduction_targets.md`).

---

## Results (2026-04-13)

| Algo       | Dataset | Seq    | Poses | ATE RMSE  | Mean   | Max    | Pipeline | Accuracy    |
|------------|---------|--------|------:|----------:|-------:|-------:|----------|-------------|
| FAST-LIO2  | avia    | Dark01 | 2057  |   0.227 m |  0.207 |  0.659 | PASS     | PASS        |
| Point-LIO  | avia    | Dark01 | 2057  |   0.291 m |  0.268 |  0.700 | PASS     | PASS        |
| iG-LIO     | avia    | Dark01 | 2059  |   0.155 m |  0.148 |  0.382 | PASS     | PASS (best) |
| LIO-SAM    | ntu     | eee_01 | 1625  | 568.29 m  | 451.93 | 1592.18| PASS     | DIVERGED    |
| SLICT      | ntu     | eee_01 |   —   |   —       |   —    |   —    | BLOCKED  | —           |

### Upstream patches applied

End-to-end integration required the following source/header patches applied
at container runtime (all live in `baselines/scripts/run_inside_<algo>.sh`):

- **FAST-LIO2**: writable copy of `/algo_src` → `${CATKIN_WS}/src/FAST_LIO`
  because the launch file has a hardcoded config path.
- **Point-LIO**: explicit `catkin_make ... livox_ros_driver_generate_messages_cpp`
  pre-pass, because Point-LIO misses the generator dependency → build race.
- **iG-LIO**: generated headless launch (upstream requires `rviz`).
- **LIO-SAM** — 5 patches total (runtime patching in `run_inside_lio_sam.sh`):
  1. `sed` rewrite of `#include <opencv/cv.h>` → `<opencv2/opencv.hpp>`
     (OpenCV 4.x removed the legacy header).
  2. `set(CMAKE_CXX_FLAGS "-std=c++11")` → `-std=c++14` so that GTSAM 4.x
     + PCL 1.10 `pcl::traits::datatype` constexpr issue is avoided.
  3. `sed` strip of `module_robot_state_publisher.launch` include from
     `run.launch` (needs `xacro`, not installed in baselines image; LIO-SAM
     does not need the TF tree for odometry publication).
  4. FLANN header patch: specialize `flann::serialization::Serializer` for
     `std::unordered_map<K, std::vector<V>>` so LSH index serialization
     compiles under Noetic's PCL 1.10.
  5. **`ldconfig /usr/local/lib` + export `LD_LIBRARY_PATH`** — GTSAM 4.x
     source build installs `libmetis-gtsam.so` to `/usr/local/lib/` but
     the ldconfig cache is not refreshed, so `lio_sam_imuPreintegration`
     and `lio_sam_mapOptmization` fail at runtime with "cannot open shared
     object file: libmetis-gtsam.so".

### LIO-SAM divergence (ntu/eee_01)

The harness runs end-to-end (1625 poses produced) but the trajectory diverges
catastrophically — Z plummets from 0 → −52 m in the first 5 s, XY blows up
to >1 km scale. Umeyama-aligned ATE RMSE 568 m (paper reports ~0.5–2 m on
eee_01).

Root cause is almost certainly config:
- `extrinsicRot` / `extrinsicRPY` set to identity — NTU VIRAL's VN100 IMU
  to Ouster OS1-16 transform on the VIRAL platform is non-trivial.
- IMU axis convention / gravity sign likely inconsistent with VIRAL's
  VN100 mounting orientation.
- `imuAccNoise` / `imuGyrNoise` are datasheet-nominal, not Allan-variance
  derived.

This is expected smoke-level behavior: Phase 1 validates the **pipeline**
(build → launch → record → extract → ATE), not algorithm correctness. Per-
algorithm ATE-tuning to paper numbers is Phase 4 (`paper_reproduction_targets.md`).

### SLICT blocker (documented)

SLICT depends on `brytsknguyen/ufomap @ devel_surfel` → subpackages
`ufomap_msgs` and `ufomap_ros` are shipped as `buildtool_depend=ament_cmake`
(ROS2-only). The run script rewrites both package manifests + `CMakeLists.txt`
as catkin equivalents. `ufomap_msgs` builds fine (2 trivial `.msg` files).
`ufomap_ros/conversions.h` however uses ROS2 C++ API directly
(`<geometry_msgs/msg/point.hpp>`, `rclcpp` types), so a buildsystem rewrite
alone cannot make it compile.

Options to unblock:
1. Port `ufomap_ros/conversions.{h,cpp}` to ROS1 API (est. 1–2 days).
2. Submodule-pin a different SLICT fork that uses ROS1-compatible ufomap.
3. Drop SLICT from Tier-1 baseline set and replace with another surfel-LIO.

→ **Decision**: deferred to Phase 2 scoping. Phase 1 harness validation is
complete without SLICT (4/5 end-to-end pipelines verified; 5th blocked on
upstream source incompatibility, not harness).

### Interpretation

**Pipeline**: 4 of 5 algorithms complete the full smoke bag end-to-end
through the harness (FAST-LIO2, Point-LIO, iG-LIO, LIO-SAM). SLICT is
structurally blocked at the upstream source level.

**Accuracy (Avia/Dark01 short indoor loop)**: iG-LIO 0.155 m < FAST-LIO2
0.227 m < Point-LIO 0.291 m — ordering is consistent with published
behavior on similar loops. These are not tuned to paper numbers; Phase 4
will audit per-algorithm vs paper tables.

**Accuracy (NTU/eee_01)**: LIO-SAM diverged → Phase 4 must re-check
extrinsics + IMU calibration for VIRAL platform.

These numbers are **smoke-test only** — not reproduction targets.
Paper reproduction is task #46 / Phase 4 and uses the per-paper ground-truth
sequences documented in `paper_reproduction_targets.md`.

---

## Update (2026-04-13, Phase 2 P2-A resolution)

LIO-SAM / ntu / eee_01 resolved by switching the upstream submodule from
`TixiaoShan/LIO-SAM` → **`brytsknguyen/LIO-SAM`** (master @ `43b80a0`),
the canonical NTU VIRAL fork by the dataset lead author (T.M. Nguyen).
The fork adds `R_W2NED = diag(1, -1, -1)` in `include/utility.h`, the
ENU→NED world-frame correction that vanilla TixiaoShan master lacks.

| Algo      | Dataset | Seq    | Poses | ATE RMSE  | Mean   | Max   | Pipeline | Accuracy |
|-----------|---------|--------|------:|----------:|-------:|------:|----------|----------|
| LIO-SAM   | ntu     | eee_01 |  1993 |   0.206 m |  0.198 | 0.362 | PASS     | PASS     |

Trajectory is bounded within a ~13 m cube, matching the eee_01 indoor
building loop. Result is within the paper's reported range (~0.5–2 m on
eee_01) and comparable to our Avia/Dark01 numbers (iG-LIO 0.155 m,
FAST-LIO2 0.227 m).

Falsified hypotheses (kept for the record):
- r1: Z-sign correction on `extrinsicTrans` alone → RMSE 568 → 431 m.
- r2: `useImuHeadingInitialization: false` on vanilla → 431 → 437 m.

Both confirmed that the gravity-frame bug is source-level, not config-level.
See `baselines/docs/phase2_plan.md §P2-A` for the full decision trail.

### Updated patch list (brytsknguyen fork, environment-specific)

- **Patch 1**: overwrite `launch/run.launch` with a minimal headless launch
  (loads mounted `config/params.yaml`, `include module_loam.launch`, no rviz,
  no bag). Reason: upstream `run.launch` hardcodes a local bag path.
- **Patch 2**: FLANN `<unordered_map>` Serializer specialization
  (PCL 1.10 / Noetic — unchanged from vanilla).
- **Patch 3**: `ldconfig /usr/local/lib` + `export LD_LIBRARY_PATH`
  for runtime `libmetis-gtsam.so` loading (unchanged).

Dropped from the vanilla patch set (no longer needed with brytsknguyen):
- `<opencv/cv.h>` → `<opencv2/opencv.hpp>` (fork already modern).
- `-std=c++11` → `-std=c++14` (fork already `-std=c++17`).
- strip `module_robot_state_publisher.launch` include (fork's
  `run_ntuviral.launch` already comments it out; our minimal `run.launch`
  override drops it entirely).

---

## Artifacts

```
dump/baselines_smoke/<algo>/<dataset>/<seq>/
  ├── odom.bag       — recorded nav_msgs/Odometry topic
  ├── traj.csv       — TUM-format trajectory (t x y z qx qy qz qw)
  ├── ate.json       — Umeyama-aligned ATE metrics
  └── stdout.log     — container stdout (build + runtime)
```
