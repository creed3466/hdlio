# Task #36 R2' Build + Eval — Dark01 LiDAR-Anchored Deterministic Release

**Date:** 2026-04-14  
**Branch:** `fix/task36-r2-lidar-anchor`  
**Architect contract:** `docs/reports/task36_r2_architect_20260414.md` (`7bc2ff7`)  
**Baseline commit:** `f6538d4`  
**Checkpoint commit:** `ed0413d` (empty, pre-R2')  
**Outcome:** **FAIL at P0** — bifurcation persists, 2 ATE classes, CV = 0.56 %. **Rolled back** (branch kept for post-mortem per CLAUDE.md §8; main untouched).

---

## 1. Open-Question Confirmations (pre-build, per architect §Open Questions)

| # | Question | Answer | Evidence |
|---|----------|--------|----------|
| OQ-1 | Does Fix A's 100 ms watermark protect gravity-init, or is gravity-init ordering-only? | **Ordering-only.** Gravity init triggers purely on IMU count (`init_samples=100`) inside `imu_adapter_.process()`; the watermark only gates *when* IMUs leave the queue, not *whether* init succeeds. The `queue_last_lidar_ts_ == 0.0 → release` fallback preserves init determinism. | `slam_node.cpp` 807-850 (`if (!estimator_->initialized() && imu_adapter_.initialized())` branch fires unconditionally per IMU) |
| OQ-2 | Does `queue_last_lidar_ts_` anchor on *post-dedup* LiDAR ts? | **Yes.** `lidar_callback` (lines 704-721) takes `queue_mutex_`, performs the 50 ms dedup, and only calls `enqueue_event` on passing messages. `queue_last_lidar_ts_` is updated *inside* `enqueue_event`, so it sees only post-dedup timestamps. | `slam_node.cpp` 710-720 dedup-before-enqueue chain |
| OQ-3 | Is VI04's 200 ms watermark a distinct root cause or just a wider watermark? | **Watermark width.** Config comment (`varying_illu04.yaml:45`) and the referenced `determinism_fix_architect_20260411.md` describe it as a Phase 1‴ bracket answer (threshold ∈ (100, 150] ms → 200 ms with margin). No distinct path. R2' replaces the watermark mechanism entirely. | `varying_illu04.yaml:45-49` comment block |

All three answered favorably — proceeded to build.

---

## 2. Build — 5 commits on `fix/task36-r2-lidar-anchor`

| Step | SHA | Purpose |
|------|-----|---------|
| checkpoint | `ed0413d` | Empty commit, pre-R2' rollback target |
| (a) | `0378725` | `slam_node.hpp` — add `queue_last_lidar_ts_` state + `deterministic_queue_lidar_anchor_` toggle |
| (b) | `c001e84` | `slam_node.cpp:enqueue_event` — update `queue_last_lidar_ts_` on LiDAR insert |
| (c) | `3ba2ec1` | `slam_node.cpp:processing_loop` — LiDAR-anchored readiness predicate + config param load + INFO log mode label |
| (d) | `acc10ec` | 9× `config/avia_seq/*.yaml` — `deterministic_queue_lidar_anchor: true` |

**Compile status:** `PASS` on `tofslam:ros1` image with `catkin_make -DCMAKE_BUILD_TYPE=Release -j4`. Only pre-existing warnings (unrelated `#ifdef` inside macro args at lines 635/637/639). Smoke build on cpuset 0-3 verified before running P0.

---

## 3. P0 — Dark01 ×10 with boundary-hash instrumentation (primary gate)

**Command:**
```
TOFSLAM_DEBUG_BOUNDARY_HASH=1 bash docker/run_avia_debug_dark01.sh r2_v1
python3 docker/analyze_debug_multirun.py dump/r2_v1/
```

**Result: FAIL** — 2 ATE classes, CV 0.56 %.

### Per-run ATE RMSE

| run | ATE RMSE | Class |
|-----|---------:|:------|
| 01 | 0.14429536234731874 m | A |
| 02 | 0.14429536234731874 m | A |
| 03 | 0.14429536234731874 m | A |
| 04 | 0.14599701865829343 m | B |
| 05 | 0.14599701865829343 m | B |
| 06 | 0.14599701865829343 m | B |
| 07 | 0.14599701865829343 m | B |
| 08 | 0.14599701865829343 m | B |
| 09 | 0.14599701865829343 m | B |
| 10 | 0.14599701865829343 m | B |

- n=10, 3-way parallel (p1/p2/p3 cpusets 0-3 / 4-7 / 8-11, ROS ports 11311/12/13).
- Δ = 1.702 mm, CV = 0.56 %.
- Intra-class B0–B10 boundary hashes bit-identical (confirmed by `diff -q` across run1↔run2↔run3 and run4↔run5↔run10).
- All 10 runs produced identical trajectory length (2057 poses).

### Boundary-hash first-divergence localization

| Stage | First diverging frame |
|-------|----------------------:|
| B0 (RawScan) | — (identical through end) |
| **B1 (Preprocessed)** | **842** |
| B2 (Predicted) | 842 |
| B3 (Corrs) | 842 |
| B4 (IekfIter) | 842 |
| B5 (Output) | 842 |
| B6 (ScanForCorr) | 842 |
| B7 (L1CountOrder) | 842 |
| B8 (ShareCountOrder) | 842 |
| B9 (AffectedOrder) | 842 |
| B10 (SurfelDirtyOrder) | 843 |

- B0@842 **identical** across classes → raw LiDAR input for frame 842 is bit-identical.
- B5@841 **identical** → previous-frame output state matches.
- B1@842 **differs** → preprocessing/undistortion of a bit-identical scan with a bit-identical prior state produced different outputs.

### Comparison vs baseline (`f6538d4`, Research v2 §10)

| Metric | Baseline `f6538d4` | R2' `acc10ec` | Δ |
|--------|-------------------:|--------------:|---|
| Classes | 2 | **2** | **unchanged** |
| Class A runs | 2 | 3 | ±1 |
| Class B runs | 8 | 7 | ±1 |
| Class A ATE | 0.14603393329227404 | **0.14429536234731874** | **shifted** |
| Class B ATE | 0.14599701865829343 | 0.14599701865829343 | **identical** |
| First divergence | B1@frame 1703 | **B1@frame 842** | **moved earlier** |

R2' changed the bifurcation values and moved the first-divergent frame, proving the code path is active and the predicate fires. But it did **not** collapse the classes.

### Root-cause hypothesis (post-mortem)

The observation set `{B0@842 = identical, B5@841 = identical, B1@842 = divergent}` forces the conclusion that the divergence enters between the frame-841 IEKF output and the frame-842 preprocessing step. The only state mutation between those two points is the stream of IMU samples fed to `estimator_->feed_imu()` in the 841→842 bag window (~100 ms, ~20 IMUs). If the *set or order* of IMUs delivered to `feed_imu` differs, the propagated `current_state_` entering frame 842 preprocessing differs, and `B1` (which depends on undistortion using `current_state_`) bifurcates.

**Residual race under R2':** R2' is an *ordering* invariant — it guarantees an IMU with ts ∈ (LiDAR@n−1.ts, LiDAR@n.ts) is released *before* LiDAR@n **if it is already in the queue when LiDAR@n enters**. It does **not** wait for an IMU that arrives *after* LiDAR@n has already been enqueued and popped. Concretely:

1. LiDAR@n arrives at the queue; `queue_last_lidar_ts_ = LiDAR@n.ts`.
2. Predicate is immediately satisfied for LiDAR@n (self-ready: `LiDAR.ts ≤ queue_last_lidar_ts_`).
3. Processing loop pops LiDAR@n and starts `feed_lidar(...)` for frame n.
4. Subsequently an IMU@t with `t < LiDAR@n.ts` arrives via the callback (wall-clock jitter); sort-insert places it at the front of the queue.
5. That IMU is now eligible (`t < queue_last_lidar_ts_`), but frame n has already begun — the IMU is fed *after* frame n's preprocessing.

Under the old 100 ms watermark this was prevented by the wall-clock wait (`queue_newest_ts_ − 100 ms` kept LiDAR@n waiting long enough for all peer IMUs to arrive). R2' removed that safety without replacing it.

This matches the architect's §5 risk row **"M2 worker-wake-up race (Research §12 rank 2) — Possible residual after R2'"**. Architect's projected escalation path: R3.

---

## 4. P1 / P2 / P3 — not executed

Per architect contract §7 and task spec abort conditions: **any gate fail → rollback**. P0 failed, so P1 regression (Dynamic03, VI03, VI04, Dark02, Occlusion03), P2 fingerprint class count, and P3 performance were not run. No resources were spent running them. Main-branch secured results remain unmodified.

---

## 5. Rollback

- Code changes **remain on branch `fix/task36-r2-lidar-anchor`** (not merged).
- Main branch untouched — `git log main` unchanged, no deployed regression.
- Checkpoint `ed0413d` is the recorded rollback target; `git reset --hard ed0413d` on the branch would leave only the empty checkpoint if a clean rollback on the branch itself is preferred (not executed here — branch retained verbatim for post-mortem per spec).
- Six unrelated commits (`08ec89b`, `cd1df0f`, `085ce63`, `33f5277`, `d83dc8d`, `aea6f1f`) landed on the same branch name from a concurrent push during the P0 run — they touch iG-LIO and research docs only, do not modify `slam_node.{cpp,hpp}`, and do not affect the P0 evaluation.

---

## 6. Deliverables checklist

| Item | Status |
|------|--------|
| Branch + all commit SHAs | `fix/task36-r2-lidar-anchor` / `ed0413d`, `0378725`, `c001e84`, `3ba2ec1`, `acc10ec` |
| Compile status | PASS |
| 3 open-question confirmations | All YES |
| P0 result | 2 classes, ATE {0.14429536..., 0.14599702...}, B1@frame 842 first divergence, intra-class bit-identity confirmed |
| P1 per-seq result | N/A (gated by P0) |
| P2 fingerprint count | 2 (still failing) |
| P3 `total_ms` regression % | N/A (gated by P0). Frame-1 `total_ms` = 2.9709 ms on R2' run1 for reference. |
| Merge SHA | — (not merged) |
| Rollback SHA | `ed0413d` (checkpoint retained; branch not hard-reset) |
| Blockers | R2' predicate addresses the ordering contract but not the timing contract. Residual late-IMU-arrival race (architect §5 M2) persists. Need R3. |

---

## 7. Recommendations for Research / Architect

1. **R3 should combine ordering + timing.** R2' + a narrow late-arrival wait: when popping a LiDAR@t_L, require `wall_clock_now() − enqueue_time(LiDAR@t_L) ≥ ε` where ε is the maximum expected IMU arrival jitter (empirically 10–100 ms on M3DGR Avia per Fix A history). Alternatively, keep the 100 ms watermark strictly for wall-clock safety and add the LiDAR anchor on top as an ordering guarantee.
2. **Instrument IMU enqueue wall-clock offsets.** Log `(IMU.ts, enqueue_wall_clock_ns)` and `(LiDAR@n.ts, pop_wall_clock_ns)` for frames 840–843 across classes to confirm the late-arrival hypothesis directly.
3. **Consider a batched-pop variant** that pops all IMUs with `ts < LiDAR@n.ts` before starting `feed_lidar(n)`, inside one mutex-held section — but only after a short wait to let late IMUs arrive. This was the architect-rejected "pull-everything-now" formulation; combined with a wait it becomes deterministic.

---

## 8. Evidence artifacts

- `dump/r2_v1/Dark01/run{1..10}/` — traj, diagnostics, debug_state, debug_imu, boundary_run{N}.csv
- `logs/task36_r2/p0_dark01.log` — full P0 run log
- This report: `baselines/docs/task36_r2_build_eval_20260414.md`
