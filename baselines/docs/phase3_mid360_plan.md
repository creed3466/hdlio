# Phase 3 — Mid-360 baselines plan (2026-04-14)

## Status

**Unblocked** (supersedes `baselines/docs/phase2_p3_mid360_scope.md`).

The previous scope doc stated that `livox_ros_driver/CustomMsg` (v1) and
`livox_ros_driver2/CustomMsg` (v2) are "not wire-compatible" and that Option
B (bag-side republish) risks "silent data corruption" from field-order
drift. That claim is **wrong** — the two messages have identical MD5:

```
$ docker run --rm baselines-fast_lio2:ros1 bash -c \
    "rosmsg md5 livox_ros_driver/CustomMsg"
e4d6829bdfe657cb6c21a746c86b21a6
```

ROS messages with matching MD5 are, by definition, bytewise-compatible on
the wire. Only the ROS package namespace string differs. This makes
**Option B-revised (namespace-only republisher)** strictly safe: a
`topic_tools::ShapeShifter` pass-through that rewrites the datatype
metadata but never touches the payload.

## Bag × topic matrix

All 9 sequences in `/home/euntae/Project/dataset/ros1/surfel_data/` carry
the same topic layout:

| topic                   | msg type                      | rate   | notes                       |
|-------------------------|-------------------------------|--------|-----------------------------|
| `/livox/mid360/lidar`   | `livox_ros_driver2/CustomMsg` | 10 Hz  | Mid-360 non-repetitive scan |
| `/livox/mid360/imu`     | `sensor_msgs/Imu`             | 200 Hz | Mid-360 built-in IMU        |
| `/livox/avia/lidar`     | `livox_ros_driver/CustomMsg`  | 10 Hz  | Avia (parallel stream)      |
| `/livox/avia/imu`       | `sensor_msgs/Imu`             | 200 Hz | Avia IMU                    |
| `/camera/*`             | RGB-D                         | —      | RGB-D camera                |
| `/odom`                 | reference                     | —      | wheel / IMU odometry        |

Ground truth lives at `surfel_data/ground_truth/<seq>.txt`.

## Sequence set

All 9 M3DGR `surfel_data` bags (size-packed for 3-way wave scheduling):

```
Varying-illu03 Varying-illu04 Dark02 Varying-illu05
Occlusion04 Dynamic04 Occlusion03 Dynamic03 Dark01
```

## Republisher design decision

**Decision: source-mount + in-container catkin build (not a sidecar container, not a baked image layer).**

Three architectural options were considered:

| Option                            | Pros                              | Cons                                      |
|-----------------------------------|-----------------------------------|-------------------------------------------|
| A) Bake into each of 3 algo imgs  | Fast startup                      | 3× image rebuilds (~30 min total)         |
| B) Sidecar container, shared net  | Cleanest separation               | Breaks `--ipc private` roscore isolation  |
| **C) Source-mount + catkin_make** | **No rebuilds, isolation intact** | **+~20 s startup per run (amortised)**    |

Option C wins because:

1. The baseline images already contain `livox_ros_driver` v1 with the
   matching MD5. The republisher pkg only adds ~100 lines of C++.
2. Compilation inside the baked devel workspace takes ~15 s (verified on
   `baselines-fast_lio2:ros1`). That's cheaper than a full image rebuild
   cycle for each touch of the republisher source.
3. Each algorithm container keeps its **own private roscore on port 11311
   inside its own `--ipc private` namespace** — identical to the Avia and
   NTU runs. No cross-container ROS_MASTER sharing, no port reassignment
   headaches.
4. The republisher is a throwaway bridge. Baking it into images would
   make the images non-self-documenting (engineers would wonder why
   `baselines-fast_lio2:ros1` ships a livox_v2_to_v1 package).

Implementation:

| File                                                                      | Role                                                                                      |
|---------------------------------------------------------------------------|-------------------------------------------------------------------------------------------|
| `baselines/tools/livox_v2_to_v1_republish/src/republish_node.cpp`         | `ShapeShifter` pass-through; morphs datatype to v1 before `advertise()`.                  |
| `baselines/tools/livox_v2_to_v1_republish/CMakeLists.txt` + `package.xml` | Standard catkin package; depends on `roscpp`, `topic_tools`, `livox_ros_driver` (v1).     |
| `baselines/tools/livox_v2_to_v1_republish/launch/republish.launch`        | Standalone launch (unused by harness but kept for manual debugging).                      |
| `baselines/scripts/setup_mid360_republisher.sh`                           | Sourced by `run_inside_<algo>.sh` when `DATASET=mid360`. Compiles + launches + pings.     |
| `baselines/scripts/run_baseline.sh`                                       | Mounts `baselines/tools/livox_v2_to_v1_republish` at `/republisher_src` for Mid-360 runs. |
| `run_inside_{fast_lio2,ig_lio,point_lio}.sh`                              | Source `setup_mid360_republisher.sh` after `roscore`, kill `REPUB_PID` on shutdown.       |

### Topic wiring

**Decision: edit configs to point at `_v1` suffix** (rather than remap in
each algo's launch file).

- Lower intrusion: launches inside the baked images stay untouched.
- Self-documenting: reading `fast_lio2/mid360.yaml` tells you immediately
  that the topic is bridged.
- One-line YAML change per algo.

Edited configs:

- `baselines/configs/fast_lio2/mid360.yaml` — `lid_topic: /livox/mid360/lidar_v1`
- `baselines/configs/ig_lio/mid360.yaml` — `lidar_topic: /livox/mid360/lidar_v1`
- `baselines/configs/point_lio/mid360.yaml` — `lid_topic: /livox/mid360/lidar_v1`

IMU topic (`/livox/mid360/imu`, `sensor_msgs/Imu`) is native ROS1 — no
bridge needed.

## SLICT + LIO-SAM status

Both require `sensor_msgs/PointCloud2` input, not `CustomMsg`. The
namespace republisher does not help them. They are **deferred**.

| algo     | new config file                             | status                                                               |
|----------|---------------------------------------------|----------------------------------------------------------------------|
| lio_sam  | `baselines/configs/lio_sam/mid360.yaml`     | TEMPLATE — points at `/livox/mid360/points` (not yet produced)       |
| slict    | `baselines/configs/slict/mid360.yaml`       | STUB — points at `/livox/mid360/points_ordered` (not yet produced)   |

Unblocking path (single additional tool covers both):

1. Port `merge_lidar_livox_to_ouster` (from brytsknguyen fork) or
   `slict_livox_to_ouster` into `baselines/tools/livox_to_pointcloud2/`.
2. Chain after the namespace republisher:
   `/livox/mid360/lidar` → `/livox/mid360/lidar_v1` → `/livox/mid360/points[_ordered]`.
3. Validate per-point `offset_time` / `ring` preservation — SLICT's
   surfel deskew depends on it; LIO-SAM's `imageProjection` does too.
4. Flip `pointCloudTopic` / `lidar_topic` in the two configs.
5. Enable in `run_tier1_mid360.sh` by passing `ALGOS="... lio_sam slict"`.

Estimated effort: 0.5–1 day research + 0.5 day implementation + 0.5 day
validation. Scheduled as follow-up **after the 3-algo baseline lands**.

## Paper reference numbers

M3DGR paper Mid-360 per-algo ATE RMSE: **TBD**. A search through the
repo's existing `docs/` and `baselines/docs/paper_reproduction_targets.md`
turned up Avia + NTU numbers only — Mid-360 targets were not extracted
(the Phase 2 scope doc listed it as an open question and it was never
closed). This must be resolved before Go/No-Go.

Action: extract the relevant table from the M3DGR paper PDF (or dataset
README) and append a per-algo × 9-seq target table here. If the paper
does not report Mid-360 numbers, document that and fall back to "no
regression vs. Avia on the same sequence" as the acceptance gate.

## Go / no-go gate

Before launching the full 3-algo × 9-seq matrix, verify:

1. ☐ Smoke test (Dark01, fast_lio2) produces a non-empty `traj.csv` with
   sensible trajectory length (≥ ~10 m given the 3-min bag).
2. ☐ Republisher forward count (`/out/republisher.log`) > 0 and matches
   the bag's Mid-360 frame count to within 5 %.
3. ☐ No MD5 mismatch warnings in `stdout.log` from ROS.
4. ☐ Paper reference numbers extracted (or explicit fall-back rule stated).

If all ☐ → run full matrix. If any fails → diagnose before expanding.

## Harness usage

```bash
# Full 3-algo × 9-seq run (default)
bash baselines/scripts/run_tier1_mid360.sh phase3_mid360_20260414

# Smoke: fast_lio2 only, single seq — use run_baseline.sh directly
BASELINE_CPUSET=8-11 \
  bash baselines/scripts/run_baseline.sh fast_lio2 mid360 Dark01 smoke_mid360_$(date +%Y%m%d_%H%M)

# Override algorithm set (e.g. when PC2 adapter lands, add lio_sam)
ALGOS="fast_lio2 point_lio ig_lio lio_sam" \
  bash baselines/scripts/run_tier1_mid360.sh phase3_mid360_full
```

## Open questions

Inherited from prior scope doc; still open unless noted otherwise.

1. **GT alignment for Mid-360**: Does `ground_truth/<seq>.txt` correspond
   to the Avia body frame, the Mid-360 body frame, or a platform-level
   frame? If Avia, the extrinsics in `mid360.yaml` (lever arm from IMU
   body to Mid-360) must be composed with Avia→platform during ATE
   eval. **Not yet resolved** — treat first-run trajectory lengths as
   the sanity check, then revisit.
2. **SLICT / LIO-SAM adapter**: Deferred — see §SLICT + LIO-SAM status.
3. **Sequence-scope decision**: Should Mid-360 matrix match Avia's 9-seq
   Tier-1 set 1:1, or should "baseline failure on Avia Dynamic03/04
   already validated, skip for Mid-360" be applied? **Current default:
   run all 9**, because the whole point of the Mid-360 dimension is
   sensor-robustness comparison.
4. **Paper reference numbers**: See §Paper reference numbers.
5. **Mid-360 extrinsic sign convention**: `extrinsic_T = [-0.011, -0.02329, 0.04412]`
   is copied from the M3DGR calibration; confirm it's IMU→Lidar (not
   Lidar→IMU) by cross-checking against one of the three algo configs'
   accepted sign in an Avia baseline that is already passing ATE.

## Supersession

This document supersedes `baselines/docs/phase2_p3_mid360_scope.md` in
the following respects:

- §"The blocker" — v1 and v2 CustomMsg **are** wire-compatible (identical
  MD5); the prior "not wire-compatible" claim is retracted.
- §"Option B" — namespace-only republish is **strictly safe** (no field
  interpretation), not "fast but fragile".
- §"Recommendation" — Option C (defer) is no longer preferred. Option
  B-revised (this document) unblocks Mid-360 at ~0.1 day cost.
- Option A (upstream branch switch) remains an option for a future
  cleanup pass, but is no longer on the critical path.
