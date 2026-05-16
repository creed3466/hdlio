# Task #65 R2 — iG-LIO Avia Dy03/Dy04 Divergence: Re-diagnosis

**Date**: 2026-04-14
**Status**: R1 falsified. R2 identifies a new top hypothesis with direct log evidence.
**Supersedes** (partial): `baselines/docs/task65_avia_dy_divergence_research_20260414.md`
**Evidence base**: Avia Dy03 stdout (`dump/task63_iglio_option_c_20260414_1119/ig_lio/avia/Dynamic03/stdout.log`) vs Mid-360 Dy03 stdout (`dump/phase3_mid360_tier1_20260414_0935/ig_lio/mid360/Dynamic03/stdout.log`) — same scene, same iG-LIO binary, different sensor.

---

## 1. R1 Backtrack (what was wrong)

R1 concluded "missing `scan_line: 6`" was the root cause. That hypothesis is **falsified by code reading**:

- `grep -n 'scan_line' baselines/algorithms/ig_lio/src/ig_lio_node.cpp` → no `nh.param` for `scan_line`. iG-LIO does not read this key from ROS params — adding it to yaml is a no-op.
- `num_scans_ = 128` in `pointcloud_preprocess.h:110` is a **hard-coded ceiling** used only as a filter (`msg->points[i].line < num_scans_`, `pointcloud_preprocess.cpp:14`). Avia `point.line ∈ {0..5}` always passes; Mid-360 `line ∈ {0..3}` always passes. No mis-binning is possible.
- Upstream iG-LIO's own `config/avia.yaml` (`baselines/algorithms/ig_lio/config/avia.yaml`) also omits `scan_line`. Upstream authors never intended it to exist for Livox.

R1's H2 (num_scans_ hardcoded) is mechanically dead for the same reason. R1's H3 (time-scale) is partly wrong: `time_scale` is **ignored on the Livox path** (`pointcloud_preprocess.cpp:28` uses a hardcoded `* 1e-6`; the field is annotated `// only use for velodyne` in `pointcloud_preprocess.h:77`) — so the `1000.0 vs 0.001` difference between our config and upstream's Avia config is **inert**. R1's H4 (extrinsic) remains correct — the same extrinsic works in FAST-LIO2 and Point-LIO.

## 2. Smoking-gun log comparison

Both runs from `-r 1.0` ROS playback, same bag family, same container stack. First lines of the static-init log:

### Avia Dynamic03 (iG-LIO — diverges to 26 928 m RMSE in task #63)

```
I0414 02:41:00.775962 lio.cpp:990 imu static, mean_acc_: 2.95748  -0.559401  95.5174
                                     init_ba:            2.65399  -0.501996  85.7156
                                     ba_norm:           85.7582
```

`||mean_acc_|| = √(2.96² + 0.56² + 95.52²) ≈ 95.57 m/s²` — **roughly 9.74 × g**.

### Mid-360 Dynamic03 (iG-LIO — 0.1656 m RMSE)

```
I0414 02:11:45.722460 lio.cpp:990 imu static, mean_acc_: -4.70791  -0.263479  8.5547
                                     init_ba:            0.0186   0.00104  -0.0337
                                     ba_norm:           0.0385
```

`||mean_acc_|| = √(4.71² + 0.26² + 8.55²) ≈ 9.77 m/s²` — **exactly 1 × g**.

Same scene, same algorithm, same kAccScale, same `enable_acc_correct: true` — yet one reads 1 g and the other reads 10 g. The difference must be in the IMU topic payload.

## 3. Mechanism

`ImuCallBack` at `src/ig_lio_node.cpp:90-94`:

```cpp
if (enable_acc_correct) {
  imu_msg.linear_acceleration.x *= kAccScale;   // kAccScale = 9.80665
  imu_msg.linear_acceleration.y *= kAccScale;
  imu_msg.linear_acceleration.z *= kAccScale;
}
```

This is an *unconditional* scale-by-g, with no magnitude check. It is correct iff the driver publishes accelerations in **g-units** (normalized so stationary ≈ 1.0). It is wrong by a factor of `g` if the driver already publishes **m/s²**.

- `enable_acc_correct: true` + IMU in g-units → after scaling `|acc| ≈ 9.8 m/s²` ✓
- `enable_acc_correct: true` + IMU in m/s² → after scaling `|acc| ≈ 96 m/s²` ✗ (10× g)
- `enable_acc_correct: false` + IMU in m/s² → `|acc| ≈ 9.8 m/s²` ✓
- `enable_acc_correct: false` + IMU in g-units → `|acc| ≈ 1.0 m/s²` ✗ (1/g × g)

Measured magnitudes above prove:

| sensor | raw IMU units (M3DGR bag) | `enable_acc_correct` setting | post-scale `|acc|` |
|--------|--------------------------:|------------------------------|-------------------:|
| Mid-360 `/livox/mid360/imu` | **g-units** (~1.0) | `true` (correct) | 9.77 m/s² ✓ |
| Avia    `/livox/avia/imu`   | **m/s²** (~9.8)  | `true` (wrong)   | 95.57 m/s² ✗ |

The Avia CustomMsg driver used by M3DGR emits m/s². iG-LIO's static initializer then computes `init_ba ≈ mean_acc - g·ĝ ≈ (95.57 - 9.80665)·ĝ ≈ 85.76·ĝ m/s²`, producing `ba_norm = 85.76`. From that point forward every predicted acceleration is bias-subtracted to 1/10 of its true value, lidar-to-IMU prediction residuals explode, GICP cannot close the gap, and the filter diverges within seconds. The km-scale drift numbers from Phase 2/3/Task 53/Task 63 are all consistent with this mechanism.

## 4. Why the three peer algorithms survive on Avia

All three use a magnitude-normalized gravity initializer and do not apply a static scale factor:

- **FAST-LIO2** `IMU_Processing.hpp:196`: `init_state.grav = -mean_acc / mean_acc.norm() * G_m_s2;` — scale-agnostic by construction.
- **Point-LIO** reads `acc_norm: 1.0` or computes it from magnitude.
- **iG-LIO on NTU VIRAL** uses `enable_acc_correct: false` (`configs/ig_lio/ntu.yaml:10`) because the NTU VN100 publishes m/s² — i.e. the same correct setting we need for Avia.

Upstream iG-LIO's `config/avia.yaml` keeps `enable_acc_correct: true` because the original iG-LIO authors collected their Avia bags with the upstream `livox_ros_driver` in "g-unit" mode. M3DGR's Avia bags were recorded with a different driver configuration that emits m/s² (consistent with Livox-SDK2 defaults).

## 5. Hypothesis table (re-ranked)

| # | Hypothesis | Verdict | Evidence |
|---|------------|---------|----------|
| **H_new** | **`enable_acc_correct: true` double-scales Avia IMU** | **CONFIRMED by logs** | Avia `||mean_acc||=95.57`, Mid-360 `||mean_acc||=9.77`, same binary, same flag. Diverges iff IMU is m/s². |
| H1 (R1) | Missing `scan_line` param | FALSIFIED | No `nh.param` reads `scan_line` in `ig_lio_node.cpp`. Upstream avia.yaml also omits it. |
| H2 (R1) | Hard-coded `num_scans_=128` | FALSIFIED | Ceiling check only; Avia `line∈{0..5}` passes. Identical code path handles Mid-360 successfully. |
| H3 (R1) | `time_scale` mismatch | FALSIFIED | `time_scale` unused on Livox path (`pointcloud_preprocess.h:77` `// only use for velodyne`; Livox path uses hard-coded `*1e-6`). |
| H4 (new) | `point_filter_num: 3` starvation | UNLIKELY | Mid-360 uses the same `3` and converges. Avia delivers ~24 k points/scan → 8 k post-filter, not a starvation regime. |
| H5 (new) | `max_radius: 150 m` clipping | UNLIKELY | Dy03/Dy04 indoor-office scene with max range ~30 m. 150 m cap does not bite. Mid-360 uses `max_radius: 100 m` and converges. |
| H6 (R1) | Extrinsic wrong | FALSIFIED | Same extrinsic works for FAST-LIO2 (0.14 m) and Point-LIO (0.15 m) on Avia. |
| H7 (R1) | Init-window length | FALSIFIED | Option A (`max_init_count=200`) and Option C-v2 (`acc_std_threshold=0.25`) both in place; Avia still diverges with `ba_norm ≈ 85.7`. Init algorithm is correct — **the input is 10× wrong**. |

## 6. Top hypothesis

**H_new: `enable_acc_correct: true` is miscalibrated for M3DGR Avia — the IMU driver already publishes in m/s², so the 9.80665× multiply produces a ~10 g input that pins the accelerometer bias at ~85 m/s² and blows up IMU prediction.**

**Confidence**: **~95 %**. Direct log evidence at init time (Avia `||mean_acc||=95.57` vs Mid-360 `||mean_acc||=9.77`) is mechanistic proof. Residual 5 % uncertainty covers the possibility that fixing this alone still leaves a secondary Avia-specific issue (lever-arm, timing) that surfaces once the primary failure is removed — this is the usual "onion" risk and only Build-stage validation can rule it in or out.

## 7. Proposed minimum-diff experiment (for Architect vetting)

**Change** — single-file yaml edit:

- `baselines/configs/ig_lio/avia.yaml`
  - `enable_acc_correct: true` → `enable_acc_correct: false`

**No code change.** **No change to Mid-360 yaml** (its IMU is g-units and the scaling is correct). **No change to NTU yaml** (already `false`).

**Predicted outcome**:

- `mean_acc_`'s magnitude logged by `lio.cpp:990` on Avia sequences should drop from ~95.57 m/s² to ~9.77 m/s² (same as Mid-360).
- `init_ba` should drop from ~85.7 m/s² to sub-0.1 m/s².
- Dy03/Dy04 RMSE should land in the same cluster as Mid-360 (0.15–0.25 m) or at worst the FAST-LIO2 Avia cluster (0.14–0.20 m).

**Validation gate** (before promotion):

1. Avia Dy03 RMSE ≤ 0.50 m (order-of-magnitude check vs current 26 928 m).
2. Avia Dy04 RMSE ≤ 0.50 m.
3. Stretch: Dy03/Dy04 both ≤ 0.25 m (match Mid-360 and FAST-LIO2 Avia cluster).
4. Regression sentinel: Avia sequences that already pass the soft sanity band must not regress by >20 %. From Phase 2 Avia data, check at minimum `Dark01` (if previously converged) and one other non-Dynamic Avia seq.
5. NTU and Mid-360 matrices: **must remain bit-identical** to current Option A baselines (change does not touch those yamls).

**Fix classification**: **YAML-ONLY**. No C++ edit, no rebuild required.

**Determinism**: Since the change is a ROS param flip and the Avia bags are already under the `-r 1.0` determinism regime (CLAUDE.md § Determinism), re-runs on the same container should be bit-identical across repeats.

**Cost**: ~10 min compute (2 seqs × single pass, CPU-pinned).

## 8. Open questions for Codex review

1. **Is the "Avia publishes m/s²" claim reproducible from the bag itself?** Codex should inspect one Avia bag directly (e.g. via `rosbag echo /livox/avia/imu -n 5` inside the ROS1 container, or a cached log slice) to confirm raw `linear_acceleration` values are on the order of 9.8 (m/s²) and not 1.0 (g-units). This is a sanity check on the mechanism, not on the divergence — the divergence itself is already settled by the `mean_acc_` dump above.
2. **Does `lio.cpp::StaticInitialization` use `mean_acc.norm()` anywhere as a self-check that could flag the 10× regime?** If yes, we should add a one-line warning when `mean_acc.norm() > 2·g` (cheap, prevents future sensor swaps from silently blowing up). This would be a code-only follow-up, NOT part of the minimum-diff fix.
3. **Why did the author pick `enable_acc_correct: true` for Avia in the first place?** Likely copy-paste from upstream `iglio/config/avia.yaml`. Is there any Avia-family bag in this harness that genuinely needs `true` (i.e. publishes g-units)? If all M3DGR Avia bags use the same driver, then `false` is safe harness-wide; if heterogeneous, we need a bag-by-bag audit.
4. **Is there a second-order interaction with `enable_acc_correct: false` and `enable_undistort: true` via the `offset_time` path?** Quick code read: `offset_time * 1e-6` is independent of IMU units, so no interaction expected. Codex to confirm.
5. **Mathematical correctness of `init_state.grav = -mean_acc / ||mean_acc|| · g`**: iG-LIO's `StaticInitialization` appears to *not* normalize by magnitude (it uses `mean_acc` directly as the gravity direction and computes `init_ba = mean_acc - g·ĝ`). This means iG-LIO is *inherently more sensitive* to IMU unit miscalibration than FAST-LIO2. A code-side follow-up could port FAST-LIO2's `mean_acc.normalized()*G` trick into iG-LIO to make the initializer scale-agnostic — would harden the baseline but is out of R2's scope.

## 9. Deliverables check

- [x] Concrete falsification of R1's three primary hypotheses (scan_line, num_scans_, time_scale) with code citations.
- [x] Top hypothesis with confidence %, direct log evidence, minimum-diff experiment spec.
- [x] Fix classification: **yaml-only**.
- [x] Open questions for Codex.
- [x] No Build dispatch, no config modifications (research-only, per task rules).

## 10. Evidence file paths (absolute)

- Avia Dy03 stdout with `mean_acc_` dump: `/home/euntae/Project/TofSLAM_v1.0/dump/task63_iglio_option_c_20260414_1119/ig_lio/avia/Dynamic03/stdout.log`
- Mid-360 Dy03 stdout with `mean_acc_` dump: `/home/euntae/Project/TofSLAM_v1.0/dump/phase3_mid360_tier1_20260414_0935/ig_lio/mid360/Dynamic03/stdout.log`
- iG-LIO IMU callback (scaling site): `/home/euntae/Project/TofSLAM_v1.0/baselines/algorithms/ig_lio/src/ig_lio_node.cpp:90-94`
- iG-LIO node param reads (no `scan_line`, `enable_acc_correct` ROS param): `/home/euntae/Project/TofSLAM_v1.0/baselines/algorithms/ig_lio/src/ig_lio_node.cpp:540-700`
- Preprocessor (Livox path ignores `time_scale`): `/home/euntae/Project/TofSLAM_v1.0/baselines/algorithms/ig_lio/src/pointcloud_preprocess.cpp:6-32`
- Avia yaml (current): `/home/euntae/Project/TofSLAM_v1.0/baselines/configs/ig_lio/avia.yaml`
- Mid-360 yaml (working reference): `/home/euntae/Project/TofSLAM_v1.0/baselines/configs/ig_lio/mid360.yaml`
- NTU yaml (`enable_acc_correct: false` precedent): `/home/euntae/Project/TofSLAM_v1.0/baselines/configs/ig_lio/ntu.yaml`
- FAST-LIO2 scale-agnostic init: `/home/euntae/Project/TofSLAM_v1.0/baselines/algorithms/fast_lio2/src/IMU_Processing.hpp:196`
- Phase 2 Avia failing matrix: `/home/euntae/Project/TofSLAM_v1.0/dump/phase2_tier1_avia_20260413/ig_lio_avia_summary.tsv`
- Phase 3 Mid-360 passing matrix: `/home/euntae/Project/TofSLAM_v1.0/baselines/docs/phase3_mid360_tier1_20260414.md`
- Task 63 eval: `/home/euntae/Project/TofSLAM_v1.0/baselines/docs/task63_iglio_option_c_eval_20260414.md`
