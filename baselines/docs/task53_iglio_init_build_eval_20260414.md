# Task #53 — iG-LIO Initialization Fix: Build + Eval Report

**Date**: 2026-04-14
**Author**: Research Harness (automated eval)
**Scope**: Option A only — raise `max_init_count` 20 → 200 (static init window)
**Status**: **GATE FAIL — DO NOT COMMIT.** Option A is insufficient.
**Label**: `task53_iglio_init_fix_20260414_0158`
**Research basis**: `baselines/docs/task53_iglio_init_research_20260414.md`

---

## 1. Summary

| Item | Result |
|------|--------|
| Code change | `max_init_count` exposed as ROS param; default 20, NTU/Avia YAMLs set 200 |
| Build | `baselines-ig_lio:ros1` rebuilt OK (only pre-existing upstream warnings) |
| eee_01 gate | **FAIL** — 0.267 m observed (gate ≤ 0.20 m, was 1.011 m pre-fix) |
| sbs_03 gate | **FAIL** — 0.789 m observed (gate ≤ 0.20 m, was 0.803 m pre-fix) |
| NTU regression check (7 previously-good seqs) | **PASS** — max regression +12% (nya_03) < 20% cap |
| Avia Dynamic03/04 | **DIVERGED** (RMSE ≈ 8.7 km / 276 km; report-only, not a gate) |
| Commit | **SKIPPED** (per abort protocol: eee_01 missed gate) |

---

## 2. Eval Table — 11 rows (Option A: `max_init_count = 200`)

All runs: `rosbag play -r 1.0`, 3-way container concurrency, CPU pools p1=0-3 / p2=4-7 / p3=8-11, `baselines-ig_lio:ros1`.
Baseline values from `baselines/docs/phase4_p4_gapfix_ate_rmse.json` (iG-LIO row, pre-fix).

| seq         | pre-fix RMSE (m) | post-fix RMSE (m) | Δ (%)    | gate | notes |
|-------------|-----------------:|------------------:|---------:|:----:|:------|
| eee_01      | 1.0113           | **0.2667**        | −73.6%   | **FAIL** (>0.20 m) | large improvement but still above gate |
| eee_02      | 0.1379           | 0.1263            | −8.4%    | OK   | |
| eee_03      | 0.1662           | 0.1680            | +1.1%    | OK   | |
| nya_01      | 0.1386           | 0.1290            | −6.9%    | OK   | |
| nya_02      | 0.2192           | 0.1718            | −21.6%   | OK   | (improvement, not regression) |
| nya_03      | 0.1529           | 0.1674            | +9.5%    | OK   | <20% regression cap |
| sbs_01      | 0.1409           | 0.1424            | +1.1%    | OK   | |
| sbs_02      | 0.1455           | 0.1459            | +0.3%    | OK   | |
| sbs_03      | 0.8028           | **0.7890**        | −1.7%    | **FAIL** (>0.20 m) | essentially no improvement |
| Avia Dy03   | n/a              | **8650.67**       | —        | report-only | catastrophic divergence (terminal ≈16 km) |
| Avia Dy04   | n/a              | **275980.68**     | —        | report-only | catastrophic divergence |

Source trajectories and per-run ATE JSONs:
- NTU: `dump/task53_iglio_init_fix_20260414_0158/ig_lio/ntu/<seq>/{traj.csv,ate.json}` and `ig_lio_ntu_summary.tsv`
- Avia: `dump/task53_iglio_init_fix_20260414_0158/ig_lio/avia/Dynamic0{3,4}/{traj.csv,ate.json}`

---

## 3. Code Change (minimal, back-compat)

- `baselines/algorithms/ig_lio/include/ig_lio/lio.h`
  - Added `size_t max_init_count{20};` to `Config` struct.
  - Constructor now copies `max_init_count_ = config_.max_init_count;` so existing member default is overridden from YAML.
- `baselines/algorithms/ig_lio/src/ig_lio_node.cpp`
  - `nh.param<int>("max_init_count", max_init_count, 20);` (clamped ≥1)
  - Threaded into `lio_config.max_init_count`; logged via `LOG(INFO)`.
- `baselines/configs/ig_lio/ntu.yaml`
  - `max_init_count: 200` (≈0.5 s @ VN100 400 Hz)
- `baselines/configs/ig_lio/avia.yaml`
  - `max_init_count: 200` (≈1.0 s @ Livox Avia 200 Hz)

Back-compat preserved — when the param is absent, default 20 matches upstream.

---

## 4. Build

`bash baselines/scripts/build_algo.sh ig_lio` → image `baselines-ig_lio:ros1` rebuilt.
Only pre-existing upstream warnings observed (PCL `uint*` deprecation, unused `yaw_end` in `pointcloud_preprocess.cpp`, unused `w_x` in `lio.cpp`). No new warnings from our edits.

---

## 5. Why Option A Was Insufficient (hypothesis)

### 5.1 eee_01 — partial improvement, attitude bias still present
- Pre-fix quat (20 samples / 50 ms): `q ≈ (0.999, 0, 0.044, 0)`
- Post-fix quat (200 samples / 500 ms): `q ≈ (0.999055, 0.000038, 0.043463, -0.000250)`
- Pitch component essentially unchanged (~5°). Averaging 200 IMU samples at 400 Hz does **not** wash out the motion acceleration when the bag starts mid-motion — the operator has already begun walking/turning before the first IMU sample we process. A longer arithmetic mean of a biased signal remains biased.
- Max error dropped 4.36 m → 1.37 m (good), RMSE 1.011 → 0.267 m (−74%), but still 1.3× the 0.20 m gate.

### 5.2 sbs_03 — no meaningful change (1.7% improvement only)
- The sbs_03 failure mode appears **not** to be static-init attitude: the pre-fix and post-fix RMSEs are within 2%. This sequence likely diverges mid-run (e.g., degenerate geometry, loop closure absence, surfel-map drift), not at initialization.
- Max error dropped slightly (4.36 m → 4.29 m post-fix; essentially identical), further supporting a non-init failure mode.

### 5.3 Avia Dynamic03 / Dynamic04 — catastrophic divergence
- RMSE 8.7 km (Dy03) and 276 km (Dy04). Terminal positions at the end of a 284 s bag are ~16 km from origin — this is runaway divergence, not attitude bias.
- Pre-fix iG-LIO numbers on Avia Dynamic03/04 are not in our `phase4_p4_gapfix` JSON (Avia row absent). The Phase-2 smoke that led us here had iG-LIO already failing these seqs; Option A does not fix them. Root cause is **not** static-init attitude on Avia — likely IMU bias / noise settings, LiDAR-IMU extrinsic mismatch, or stream-start conditions different from NTU.
- For the iG-LIO harness, Avia Dynamic remains an **open item** independent of Task #53.

---

## 6. Recommended Next Steps (from Research doc §Options)

Per `task53_iglio_init_research_20260414.md`, the recommended escalation path is:

1. **Option C — Motion-aware static init (priority 1)**:
   - Reject the init window if `mean_gyr.norm() > 0.01 rad/s` OR
     `std(acc) > 0.05 m/s²` during the window.
   - If rejected, defer init until a valid stationary window (or a fixed timeout).
   - This addresses both eee_01 (init'd mid-motion) and is defensible theoretically — static init assumes stationarity. Upstream fastlio2 / point-lio use the same premise.

2. **Option B — AHRS-based init (priority 2)**:
   - Use VN100 firmware orientation (NTU provides it) as the init attitude.
   - Requires NTU bag to include `/imu/data` with non-identity orientation. Needs inspection.
   - Not applicable to Avia unless Livox Avia publishes AHRS (it does not on these bags).

3. **sbs_03**: investigate mid-run divergence independently — **not** an init problem.
   - Candidates: map-drift under long straight corridors, loop-closure absence in iG-LIO, point-plane gain imbalance.

4. **Avia Dynamic03/04**: file as **separate task** (not Task #53 scope).
   - IMU noise params (`acc_cov=0.1`, `gyr_cov=0.1`) might be the wrong order of magnitude for Livox Avia built-in IMU.
   - Revisit extrinsic `t_imu_lidar` and `time_scale: 1000.0` assumption.

---

## 7. Decision

- **Do NOT commit the Option A code change.** The param plumbing is correct and back-compat, but the YAML change (20 → 200) alone does not meet the gate.
- **Keep the harness edits in the working tree** so a follow-up Option C / B PR can build on the exposed `max_init_count` param.
- **Escalate to Research → Codex review** with Option C motion-aware rejection as the next hypothesis.

---

## 8. Artifacts

- Eval root: `dump/task53_iglio_init_fix_20260414_0158/`
- NTU summary: `dump/task53_iglio_init_fix_20260414_0158/ig_lio_ntu_summary.tsv`
- Per-seq ATE JSON: `dump/task53_iglio_init_fix_20260414_0158/ig_lio/{ntu,avia}/<seq>/ate.json`
- Dispatcher log (Avia): `dump/task53_iglio_init_fix_20260414_0158/avia_dispatch.log`
- Research doc: `baselines/docs/task53_iglio_init_research_20260414.md`
- Baseline reference: `baselines/docs/phase4_p4_gapfix_ate_rmse.json`
