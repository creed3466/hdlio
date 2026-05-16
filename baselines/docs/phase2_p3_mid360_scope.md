# Phase 2 P3 — Mid-360 adaptation scoping (2026-04-13)

## Status: **Research blocker identified** — not yet implementable

## The blocker

M3DGR `surfel_data/<seq>.bag` carries **both** LiDAR streams:

| topic                   | msg type                        | driver package      |
|-------------------------|---------------------------------|---------------------|
| `/livox/avia/lidar`     | `livox_ros_driver/CustomMsg`    | livox_ros_driver v1 |
| `/livox/mid360/lidar`   | `livox_ros_driver2/CustomMsg`   | livox_ros_driver2   |
| `/livox/avia/imu`       | `sensor_msgs/Imu`               | —                   |
| `/livox/mid360/imu`     | `sensor_msgs/Imu`               | —                   |

The two `CustomMsg` types are **not wire-compatible**: different ROS package
namespaces, different struct definitions between livox_ros_driver (v1) and
livox_ros_driver2. A v1 subscriber cannot deserialize a v2 message.

### Current state of the 3 baselines

| algo      | subscribes to                       | built drivers in image     |
|-----------|-------------------------------------|----------------------------|
| FAST-LIO2 | `livox_ros_driver::CustomMsg` (v1)  | livox_ros_driver (v1) only |
| Point-LIO | `livox_ros_driver::CustomMsg` (v1)  | livox_ros_driver (v1) only |
| iG-LIO    | `livox_ros_driver::CustomMsg` (v1)  | livox_ros_driver (v1) only |

Dockerfiles do clone `livox_ros_driver2`, but the clone is guarded by
`|| true` so it silently fails (v2 repo's build.sh is ROS2-first and
requires a `sed ROS2 → ROS1` patch that doesn't fully work on Noetic out
of the box). The existing `baselines/configs/<algo>/mid360.yaml` files
reference `/livox/mid360/lidar` correctly, but no algorithm can actually
deserialize the bag's message.

**Confirmation** (this session, 2026-04-13):
```
$ docker run --rm baselines-fast_lio2:ros1 \
    bash -c "ls /root/catkin_ws/devel/lib | grep -i livox"
livox_ros_driver                # only v1
```

## Options

### Option A — Upstream Mid-360 branches (recommended)
Each algo's upstream has a separate branch/commit that adds v2 support:
- **FAST-LIO2**: `hku-mars/FAST_LIO` mainline has a `MID360` preset since
  late 2023; switches include to `livox_ros_driver2` under a compile flag.
  Relevant file: `src/preprocess.h` — add `#ifdef LIVOX_DRIVER_V2`.
- **Point-LIO**: mainline already supports `lidar_type=1` with both driver
  versions when `livox_ros_driver2` is in the workspace and its header is
  included conditionally. Source patch required.
- **iG-LIO**: upstream supports Mid-360 natively if `livox_ros_driver2` is
  built before the algo. Single `#include` swap.

Implementation sketch:
1. Ensure `livox_ros_driver2` builds in the base image (`baselines-base:ros1`):
   requires `sed 's/ROS2_VERSION/ROS1/g' build.sh` + applying the
   Noetic-compat patches from `koide3/livox_ros_driver2-noetic` fork or
   manually dropping `colcon` references.
2. Patch each algo's `preprocess.h/cpp` + `laserMapping.cpp` / node file
   to `#include <livox_ros_driver2/CustomMsg.h>` guarded by a preproc
   define; add a parallel callback + `ros::Subscriber` for the v2 topic.
3. Extend Dockerfile to pass `-DLIVOX_V2=ON` via catkin_make args.
4. Rebuild all 3 images (~12 min each).
5. Sanity on Dark01-mid360 vs paper numbers (if any), then full 9-seq.

Estimated effort: 1–2 days (Research + Architect + Build + Eval).

### Option B — Bag-side republish (fast but fragile)
Write a small ROS node that subscribes to `/livox/mid360/lidar` and
publishes a rewritten `livox_ros_driver::CustomMsg` on a renamed topic
(e.g. `/livox/mid360/lidar_v1`). Point algo configs at that. Struct fields
differ subtly between v1 and v2 (`line`, `tag`, `reflectivity` encoding,
`offset_time` unit) — risk of silent data corruption. Only useful as a
temporary stopgap.

Estimated effort: 0.5 day. Not recommended for publication-grade numbers.

### Option C — Defer P3, jump to P4 (strategically sensible)
Phase 2 goal was "pipeline validation on Tier-1"; that's **met** on Avia
(25/27 accuracy PASS) and NTU (9/9 PASS). Before spending 1–2 days on
Mid-360 source patches, do the Phase 4 paper-reproduction audit on the
existing 36 Avia+NTU runs first — if our numbers already fall inside
published paper ranges, Mid-360 is an orthogonal robustness test that can
land in a follow-up. If our numbers are off, we need parameter tuning
before expanding sensor coverage.

## Recommendation

Go with **Option C**: sequence the work as

1. P4 paper-reproduction audit on Avia+NTU (current results) → task #46.
2. Diagnose iG-LIO Dynamic03/04 divergence as a focused Research task
   (subset of P4).
3. P3 Mid-360 adaptation (Option A) only if paper reproduction passes and
   Mid-360 is needed for the target contribution.

P3 is not on the critical path to next project milestone (paper numbers
reproduction). Deferring it avoids 1–2 days of source patching that may
not yield comparable value.

## Open questions (if P3 is prioritized)

- Does the v2 driver's Noetic-compat fork preserve the full CustomMsg
  struct or introduce subtle field changes?
- Do FAST-LIO2 / Point-LIO / iG-LIO authors publish their own Mid-360
  test numbers? (needed as target for P4 Mid-360 reproduction)
- Is there a known reference trajectory for Mid-360 M3DGR sequences, or
  do we use the same Avia GT (same physical trajectory, different sensor)?
