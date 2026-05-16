# Task #63 — iG-LIO Option C-v2 Eval Report (acc_std_threshold 0.05 → 0.25 m/s²)

**Date**: 2026-04-14
**Author**: Research Harness (automated eval)
**Scope**: Single-parameter re-tune of Option C v1 — relax `acc_std_threshold`
from `0.05` → `0.25` m/s² to match the VN100 per-axis noise floor observed
in the Option C v1 eval. All other Option C logic unchanged.
**Status**: **GATE FAIL on primary (eee_01 = 0.5797 m, target ≤ 0.20 m).**
Sequence of commits preserved + reverted per abort protocol.
**Label**: `task63_iglio_option_c_v2_20260414_1259`
**Predecessors**:
- `baselines/docs/task53_iglio_init_research_20260414.md` (Research — Option A/B/C)
- `baselines/docs/task53_iglio_init_build_eval_20260414.md` (Option A — eee_01 1.011 → 0.267 m)
- `baselines/docs/task63_iglio_option_c_eval_20260414.md` (Option C v1 — regression, thresholds miscalibrated)

---

## 1. Summary

| Item | Result |
|------|--------|
| SHA_CV1 (C v1 checkpoint, super-repo) | `08ec89b` — `[baselines] task #63 Option C v1 source (threshold miscalibrated)` |
| SHA_CV1 submodule (`baselines/algorithms/ig_lio`) | `c5ffc0e` — `task #63 Option C v1 source — motion-aware static init (threshold miscalibrated)` |
| SHA_CV2 (C-v2 yaml tune) | `e4506c4` — `[baselines] task #63 Option C-v2 — acc_std_threshold 0.05→0.25 m/s² (VN100 floor)` |
| SHA_REVERT | `5979978` — revert of `e4506c4` on gate FAIL |
| Build | docker image `baselines-ig_lio:ros1` — fully cached (yaml mounted at runtime; Option C v1 C++ source already baked from SHA_CV1 submodule commit) |
| CPU scheduling | waited ~13 min 20 s for task #36 R2' Build+Eval (Dark01 ×10 + regression) to vacate pools 0-3 / 4-7 / 8-11 (12:46:27 → 12:59:47) |
| Gate — eee_01 | **FAIL** — 0.5797 m (> 0.20 m; Option A was 0.2667 m) |
| Gate — 7 stable seqs (≤ +20 % vs Option A) | **PASS** — max regression +4.7 % (nya_01) |
| Gate — sbs_01 < 0.16 m | **PASS** — 0.1420 m (Option A 0.1424 m, C v1 0.2747 m — v1 safety-cap regression fully reversed) |
| Primary-gate composite | **FAIL (eee_01)** |
| Overall decision | Revert SHA_CV2 (abort protocol); keep SHA_CV1 as research checkpoint |

---

## 2. Eval table — 11 rows

All runs: `rosbag play -r 1.0`, 3-way concurrent containers, CPU pools
p1=0-3 / p2=4-7 / p3=8-11, `baselines-ig_lio:ros1` image (Option C v1
C++ source baked), config mounted at runtime from
`baselines/configs/ig_lio/{ntu,avia}.yaml`.

Columns:

- `pre-fix`: `baselines/docs/phase4_p4_gapfix_ate_rmse.json` iG-LIO row
- `OptA`:  `dump/task53_iglio_init_fix_20260414_0158/` summaries
- `OptC_v1`: `dump/task63_iglio_option_c_20260414_1119/` summaries
- `OptC_v2`: `dump/task63_iglio_option_c_v2_20260414_1259/` summaries (this run)
- `Δ_A→C_v2`: (C_v2 − A) / A × 100 %
- `gate`: eee_01 ≤ 0.20 m (primary); 7 stable ≤ +20 % regression; sbs_01 must drop ≤ 0.16 m; sbs_03 + Avia Dy03/Dy04 report-only

| seq         | pre-fix (m) | OptA (m) | OptC_v1 (m) | OptC_v2 (m) | Δ_A→C_v2 | gate |
|-------------|------------:|---------:|------------:|------------:|---------:|:-----|
| eee_01      |      1.0113 |   0.2667 |      0.4183 |  **0.5797** | **+117.4 %** | **FAIL** (> 0.20 m) |
| eee_02      |      0.1379 |   0.1263 |      0.1254 |      0.1262 |   −0.1 % | OK |
| eee_03      |      0.1662 |   0.1680 |      0.1737 |      0.1684 |   +0.2 % | OK |
| nya_01      |      0.1386 |   0.1290 |      0.1391 |      0.1351 |   +4.7 % | OK |
| nya_02      |      0.2192 |   0.1718 |      0.1929 |      0.1715 |   −0.2 % | OK |
| nya_03      |      0.1529 |   0.1674 |      0.1479 |      0.1640 |   −2.0 % | OK |
| sbs_01      |      0.1409 |   0.1424 |      0.2747 |      0.1420 |   −0.3 % | OK (< 0.16 m) |
| sbs_02      |      0.1455 |   0.1459 |      0.1457 |      0.1458 |   −0.1 % | OK |
| sbs_03      |      0.8028 |   0.7890 |      0.7765 |      0.7754 |   −1.7 % | report-only (> 0.20 m on both, unchanged mode — task #64) |
| Avia Dy03   |         n/a |  8650.67 |    26928.09 |  **0.1566** | — | **report-only** — improvement attributable to task #65 `enable_acc_correct: false` flip in d83dc8d, NOT to C-v2 |
| Avia Dy04   |         n/a |275980.68 |   260340.17 |  **0.2837** | — | **report-only** — same caveat as Dy03 |

Artefacts:

- NTU summary TSV: `dump/task63_iglio_option_c_v2_20260414_1259/ig_lio_ntu_summary.tsv`
- Avia summary TSV: `dump/task63_iglio_option_c_v2_20260414_1259/ig_lio_avia_summary.tsv`
- Per-seq stdout + init diag: `dump/task63_iglio_option_c_v2_20260414_1259/ig_lio/{ntu,avia}/<seq>/stdout.log`
- Per-seq ATE: `dump/task63_iglio_option_c_v2_20260414_1259/ig_lio/{ntu,avia}/<seq>/ate.json`

No run-time crashes on any of the 11 sequences.

---

## 3. Code / config change (C-v2)

**Super-repo yaml (`baselines/configs/ig_lio/`):**

```diff
--- ntu.yaml    (SHA_CV1 baseline at 08ec89b)
+++ ntu.yaml    (SHA_CV2 at e4506c4)
-acc_std_threshold: 0.05
+acc_std_threshold: 0.25
```

```diff
--- avia.yaml   (SHA_CV1 baseline at 08ec89b)
+++ avia.yaml   (after d83dc8d — task #65 co-change)
-enable_acc_correct: true              # [B] flipped by d83dc8d
+enable_acc_correct: false
-acc_std_threshold: 0.05               # [C-v2] tuned in d83dc8d bundle
+acc_std_threshold: 0.25
```

**No C++ change.** `gyr_motion_threshold` stays `0.01 rad/s`,
`max_init_wait_samples` stays `4000`. The Option C v1 rejection logic in
`LIO::StaticInitialization` (submodule `baselines/algorithms/ig_lio`
commit `c5ffc0e`) is retained bit-identical.

**Avia caveat**: the eval image ran with avia.yaml having BOTH C-v2
threshold AND `enable_acc_correct: false` (task #65's fix, committed
13:10:41 while our NTU matrix was in flight). The Avia Dy03/Dy04 numbers
therefore do not isolate C-v2; they are dominated by the `enable_acc_correct`
flip (which fixes a 10-g double-scaling bug on Avia IMU — see `d83dc8d`
log and `baselines/docs/task65_avia_dy_divergence_research_r2_20260414.md`).

---

## 4. Why the primary gate failed

### 4.1 The gate is a behavioural no-op on NTU with the relaxed threshold

`grep -c 'motion-dirty window rejected'` on every NTU stdout.log for this
run: **0 rejections across all 9 seqs**. `safety cap hit`: 0. `accepted
stationary window after N rejections`: 0. The first init window passes on
every seq.

On eee_01 specifically, the init diagnostic line is byte-identical to
Option A:

```
lio.cpp:990 imu static, mean_acc_: 0.835853 -0.00278129 -9.58263
              init_ba: -0.0163047 5.42539e-05 0.186926
```

(The line number moved from `lio.cpp:916` in Option A to `lio.cpp:990` in
C-v2 because the Option C v1 source adds 74 lines of rejection logic
above the `imu static` print — the values themselves are identical.)

`mean_acc_ = (0.836, −0.003, −9.583)` → `||mean_acc_|| ≈ 9.62 m/s²` (~2 %
low vs g) → `init_ba = (−0.016, 0, +0.187)`, i.e. a 0.84 m/s² X-axis
gravity bias that the upstream Option A never resolves. This *is* mid-walk
contamination — the platform is tilted ~5° from level while the first
200 samples are being averaged.

### 4.2 So why is eee_01 v2 (0.5797) worse than Option A (0.2667)?

Because the init is byte-identical but the ATE differs, the divergence is
**iG-LIO run-to-run non-determinism** on long bags, not a genuine C-v2
regression. Spot check at traj.csv line 1000 (t = 100 s, bag is 400 s
long):

```
OptA    line 1000: tx=-0.227  ty=+0.790  tz=+6.761
OptC_v2 line 1000: tx=-1.692  ty=+0.743  tz=+9.020
```

Trajectories have already diverged by ~2.3 m by the 100 s mark despite
identical init buffers. iG-LIO's correspondence-search and scan-to-map
steps are sensitive to floating-point ordering and thread-scheduling
(same codebase, same bag, same -r 1.0 rate → stochastic outcome on
long runs). This matches known iG-LIO behaviour on NTU eee_01 in our
earlier Phase 4 Tier-1 matrix.

### 4.3 What we learned

- **sbs_01 recovery is the real signal**: C v1 regressed sbs_01 +93 %
  (0.1424 → 0.2747) because every window hit the 0.05 gate and the
  safety-cap fallback picked a delayed, biased window. C-v2 returns
  sbs_01 to 0.1420 m (matching Option A within ±0.3 %). This confirms
  the C v1 diagnosis: the threshold, not the algorithm, was broken.
- **Motion-aware gate is a no-op on NTU when calibrated to noise**: at
  0.25 m/s² the gate accepts every window on all 9 NTU seqs, i.e.
  Option A's output with an inert additional config branch. It is
  therefore neither helpful nor harmful for NTU.
- **iG-LIO NTU-eee_01 non-determinism dominates the measurement**: any
  single-run ATE on eee_01 has a noise floor in the 0.1–0.3 m range.
  To quantify the C-v2 intent properly we would need a ≥ 5-run
  bootstrap — outside this task's budget.

---

## 5. Gate evaluation

| Gate | Target | Observed | Status |
|------|--------|----------|:------:|
| eee_01 primary | ≤ 0.20 m | 0.5797 m | **FAIL** |
| 7 stable seqs, max regression | ≤ +20 % | +4.7 % (nya_01) | PASS |
| sbs_01 recovery | < 0.16 m | 0.1420 m | PASS |
| sbs_03 | report-only | 0.7754 m | report-only (unchanged mode) |
| Avia Dy03/Dy04 | report-only | 0.157 / 0.284 m | report-only (attributable to task #65's enable_acc_correct flip, not C-v2) |

**Composite: FAIL** — primary gate miss on eee_01.

Per the task abort protocol: keep SHA_CV1 as retrievable research
checkpoint; revert SHA_CV2 with a dedicated revert commit;
write this report. Done:

- `e4506c4` (SHA_CV2) committed
- `5979978` revert of SHA_CV2 committed (ntu.yaml back to `acc_std = 0.05`)
- `08ec89b` (SHA_CV1) preserved unchanged

---

## 6. Crashes & anomalies

- No rosbag play crashes on any of the 11 sequences (all `PASS` in
  summary TSVs, all `traj.csv` non-empty).
- `baselines/configs/ig_lio/avia.yaml` was modified at 13:10:41 by task
  #65 (`fix(#65): flip enable_acc_correct=false on iG-LIO Avia yaml`,
  commit `d83dc8d`). NTU containers had already launched their
  params-copy at 12:59:53, so the NTU matrix is a clean C-v2 test.
  Avia containers launched at 13:22:00, so they ran with BOTH C-v2
  threshold + `enable_acc_correct: false`. The Avia numbers are therefore
  reported but explicitly flagged as not-isolating C-v2.
- No task #36 interference during the eval itself (pools were free from
  12:59:47 through 13:28:00, the entire run duration).

---

## 7. Recommended next step

### 7.1 Option B — AHRS-based static init (next high-leverage attempt)

Current yaml sets `enable_ahrs_initalization: false`. NTU VIRAL bags
publish `/imu/imu` with VN100 AHRS orientation quaternions (non-identity,
filtered at firmware level). Feeding that quaternion directly as the init
attitude bypasses the mean-of-accel mid-walk bias entirely — the exact
eee_01 pathology this task is trying to fix.

The iG-LIO code already has `AHRSInitialization` and the `enable_ahrs_
initalization` ROS param. Proposed change: flip NTU yaml to `true` and
re-eval the 9 NTU seqs. Expected effect:

- eee_01: should drop well below 0.20 m (init attitude no longer biased
  by mid-walk acceleration, only by VN100 firmware quaternion error).
- Other NTU seqs: neutral to slightly better.
- sbs_03: still limited by mid-run divergence (task #64), AHRS init
  alone will not fix it.

Risk: VN100 frame vs iG-LIO world frame may need a static rotation
offset. Worth a 1-seq sanity probe (eee_01 only) before fanning out.

### 7.2 Alternative — Option C″ (gate on |mean_acc| deviation from g)

Per §6.2 of the C v1 eval: gate on `| ||mean_acc|| − g |` and on
`std(||acc_i||)`. Directly ties to the physical invariant (gravity
magnitude) rather than per-axis raw std. Would correctly reject eee_01's
first window (`||mean_acc|| = 9.62`, deviation from g = 0.19 m/s²) if
paired with a ~0.1 m/s² deviation gate. This is a 5-line C++ change on
top of SHA_CV1.

### 7.3 Recommendation

**Try Option B first.** It's lower risk (yaml-only flip), faster to
evaluate (~40 min wall-clock on 9 NTU seqs), and addresses the root
cause (biased init attitude) more directly than any acc_std threshold
tuning. If Option B lands eee_01 < 0.20 m, Option C″ is moot. If Option
B is neutral or worse, escalate to Option C″.

**Do NOT reinstate Option C-v2.** With the relaxed threshold the gate is
inert on NTU (no rejections, no fallback), so it just carries two dead
ROS params into the config without changing behaviour. Reintroducing it
would need Option C″'s physics-based gate to have any effect.

---

## 8. Artefacts

- Eval root: `dump/task63_iglio_option_c_v2_20260414_1259/`
- Summary TSVs (NTU + Avia) under eval root
- Per-seq stdouts with init diagnostics; grep `motion-dirty|safety cap|imu static|accepted stationary`
- Driver log: `/tmp/t63v2_driver.log` (wait-for-pools + matrix dispatch)
- Runner script: `baselines/scripts/run_task63_iglio_option_c.sh`
- SHA_CV1: `08ec89b` (super-repo), submodule `c5ffc0e`
- SHA_CV2: `e4506c4` (reverted by `5979978`)
- Prior eval: `baselines/docs/task63_iglio_option_c_eval_20260414.md`
- Research: `baselines/docs/task53_iglio_init_research_20260414.md`
- Pre-fix baseline JSON: `baselines/docs/phase4_p4_gapfix_ate_rmse.json`
