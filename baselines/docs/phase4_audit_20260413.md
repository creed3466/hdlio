# Phase 4 Audit — Tier-1 vs Paper-Reported Numbers (2026-04-13)

**Input**: 36 Tier-1 baseline runs (27 Avia × 3 algos + 9 NTU × LIO-SAM)
from commit `22f5ee1`, all at `rosbag play -r 1.0`.

**Criterion** (per `baselines/docs/paper_reproduction_targets.md`):
- PASS if |Δ| ≤ 10 %
- WARN if |Δ| ≤ 20 %
- FAIL if |Δ| > 20 %

---

## Scope of this audit

This is **P4.1 — first-pass audit on existing runs**. P4.2 (expand
cross-check runs) and P4.3 (native-paper datasets) are defined at the
end but not executed yet.

We can only compare where a public paper table supplies per-sequence
numbers. That gives us two concrete audits:

| algo × dataset          | cross-reference source          | auditable? |
|-------------------------|---------------------------------|:----------:|
| LIO-SAM × NTU VIRAL     | SLICT Table IV (RA-L 2023)      | ✓          |
| FAST-LIO2 × NTU VIRAL   | SLICT Table IV (RA-L 2023)      | **need to run** |
| Point-LIO × NTU VIRAL   | none published                  | qualitative only |
| iG-LIO × NTU VIRAL      | none published                  | qualitative only |
| * × M3DGR Avia          | M3DGR is a new dataset (2024)   | qualitative only |

---

## LIO-SAM × NTU — ATE RMSE vs two authoritative references

**P4.2 step 4 complete (2026-04-13 update)**: retrieved the NTU VIRAL
IJRR 2022 paper itself (Nguyen et al., arXiv 2202.00379, IJRR 41(3),
Table 4). The NTU VIRAL authors ran LIO-SAM themselves using their
own plug-and-play launcher. Their numbers are **in the same band as
SLICT Table IV**, not in the 0.2 m band we originally hypothesised.
Hypothesis A (our integration matches the canonical NTU VIRAL
publication) is therefore **falsified**.

| seq    | NTU VIRAL ref (IJRR 2022 Table 4) | SLICT ref (RA-L 2023 Table IV) | our Tier-1 | Δ vs NTU VIRAL | verdict |
|--------|-----------------------------------:|-------------------------------:|-----------:|---------------:|:-------:|
| eee_01 |                            0.075 m |                        0.200 m |    0.206 m |      +174.7 % | FAIL |
| eee_02 |                            0.069 m |                        0.140 m |    0.209 m |      +202.9 % | FAIL |
| eee_03 |                            0.101 m |                        0.100 m |    0.210 m |      +108.9 % | FAIL |
| nya_01 |                            0.076 m |                        0.090 m |    0.210 m |      +176.3 % | FAIL |
| nya_02 |                            0.090 m |                        0.080 m |    0.199 m |      +121.1 % | FAIL |
| nya_03 |                            0.137 m |                        0.080 m |    0.197 m |       +43.8 % | FAIL |
| sbs_01 |                            0.089 m |                        0.050 m |    0.218 m |      +144.9 % | FAIL |
| sbs_02 |                            0.083 m |                        0.050 m |    0.245 m |      +195.2 % | FAIL |
| sbs_03 |                            0.140 m |                        0.040 m |    0.457 m |      +226.4 % | FAIL |

**Result vs authoritative NTU VIRAL reference: 0/9 PASS.**
eee_01's earlier apparent PASS vs SLICT Table IV was coincidental —
it matched the *highest* value in SLICT's row, not the dataset
authors' 0.075 m canonical number.

### Diagnostic observations (updated 2026-04-13 PM)

1. **Our numbers are ~2× the authoritative reference, clustered, not
   scattered.** 8/9 Tier-1 values sit in 0.197–0.245 m (1σ ≈ 0.015 m);
   NTU VIRAL authors' numbers sit in 0.069–0.140 m (mean ≈ 0.096 m).
   A ~2× clustered gap against a benchmark run by the dataset authors
   themselves points to a **systematic configuration gap**, not a
   random per-seq bug.
2. **SLICT Table IV is validated.** SLICT's 0.04–0.20 m band is in
   the same order of magnitude as the NTU VIRAL authors' band. SLICT
   is not a re-tuned outlier — it is consistent with the dataset's own
   published benchmark.
3. **sbs_03 is a genuine 2nd-order problem.** Even accepting the
   clustered ~0.2 m drift, sbs_03 is ~2× higher than our own cluster
   (0.457 m vs 0.218 m mean). Likely a scene-specific LIO-SAM failure
   mode on top of the baseline config gap.
4. **No free parameter tuning to "fix" this.** Per CLAUDE.md
   §Research, we do not close this by parameter hunting — we go
   to Research on the brytsknguyen fork's NTU VIRAL launch config
   first.

### Verdict on LIO-SAM × NTU audit

**FAIL** — integration gap confirmed. Our brytsknguyen/LIO-SAM NTU
pipeline systematically reports ~2× the ATE RMSE of both the NTU
VIRAL authors' own Table 4 (IJRR 2022) and SLICT Table IV (RA-L 2023).
The ~0.1 m gap is unlikely to be a per-seq tuning artifact —
candidates include:

- Launch file / param divergence from the brytsknguyen fork's NTU
  example launcher (`run_ntuviral.launch` or similar).
- IMU extrinsic mis-specification (R_B_L, t_B_L) or IMU topic
  (`/imu/imu` vs `/imu/imu_raw` etc.).
- Ouster OS1-16 LiDAR type / ring / timestamp unit mismatch.
- Ground-truth alignment methodology (we use Umeyama SE(3) no-scale
  vs the authors' loop-closed full-SE(3) with scale).
- R_W2NED world-frame rotation applied on top of an already-NED
  trajectory (double-transform).

**Next stage: Research on brytsknguyen/LIO-SAM's canonical NTU VIRAL
launcher + our runner's invocation** (P4.2+).

---

## M3DGR Avia × 3 algos — qualitative check only

No published paper evaluates any of these 3 algos on M3DGR (the dataset
was released 2024). Absolute cross-reference impossible. We audit only:

1. **Inter-algorithm ranking consistency** (vs Phase 1 smoke Dark01):
   Phase 1 ordering: iG-LIO (0.155) < FAST-LIO2 (0.227) < Point-LIO (0.291).
   Tier-1 mean (ex ⚠) across 9 seqs:
   - iG-LIO 0.383 < FAST-LIO2 0.397 < Point-LIO 0.593.
   Same ordering — pipeline is consistent.
2. **No divergence on static scenes** — confirmed 22/27, all 3 algos fine
   on Dark, Occlusion, Varying-illu seqs.
3. **Expected behavioural failure on Dynamic seqs** — iG-LIO diverges
   on Dynamic03/04 (413 m / 18 km); FAST-LIO2 and Point-LIO fine.
   Open diagnosis task #53.

**Verdict on Avia audit**: **qualitative PASS** (ranking + static-scene
behaviour consistent with expectations). No quantitative paper-number
comparison is possible here.

---

## Summary — P4.1 audit verdict

| dimension                             | status |
|---------------------------------------|:------:|
| Pipeline completion (36 runs)         | ✅ 36/36 |
| Determinism (Dark01 bit-identical ref)| ✅ 3/3 |
| Inter-algorithm ranking consistency   | ✅      |
| Static-scene robustness               | ✅ 33/34 |
| Dynamic-scene robustness (iG-LIO)     | ❌ 0/2 |
| Quantitative paper-number match (LIO-SAM NTU vs SLICT Table IV)     | ❌ 1/9 |
| Quantitative paper-number match (LIO-SAM NTU vs NTU VIRAL Table 4)  | ❌ 0/9 |
| Quantitative paper-number match (3 Avia algos NTU) | ⏳ running (bv7dp80zw) |
| Quantitative paper-number match (native paper datasets) | ⏳ not yet run |

Phase 1 pipeline goal is met. **LIO-SAM × NTU integration gap
confirmed against two independent authoritative references (NTU VIRAL
IJRR 2022 Table 4 and SLICT RA-L 2023 Table IV). Both papers cluster
LIO-SAM on NTU in the 0.04–0.20 m band; ours clusters at 0.20–0.25 m
(~2× the dataset authors' own numbers). Phase 4 must enter Research
before publication-ready claims are made.**

---

## P4 open work (not yet executed)

### P4.2 — Cross-check runs (Research-enabling)

Goal: add data points that let us diagnose LIO-SAM NTU mismatch and
audit the 3 Avia algos' NTU behaviour.

1. **Run FAST-LIO2 × NTU 9-seq** — direct SLICT Table IV comparison
   (SLICT reports FAST-LIO2 on NTU: eee_01 0.22 / eee_02 0.14 / …).
   Reuses existing `run_tier1_algo_dataset.sh fast_lio2 ntu <label>`.
   Blocker: needs NTU-compatible FAST-LIO2 config (Ouster OS1-16 +
   VectorNav VN100, 9-axis). Config template at
   `baselines/configs/fast_lio2/ntu.yaml` — verify contents before
   running.
2. **Run Point-LIO × NTU 9-seq** — no direct paper reference, but lets
   us sanity-check trajectory convergence and compare to FAST-LIO2.
3. **Run iG-LIO × NTU 9-seq** — same rationale; also probes whether
   iG-LIO divergence is Dynamic-specific or has other failure modes.
4. **Revisit published LIO-SAM numbers.** ✅ **DONE (2026-04-13 PM).**
   - (a) TixiaoShan/LIO-SAM IROS 2020: no NTU VIRAL numbers (dataset
     released after the paper).
   - (b) Nguyen et al. IJRR 2022 (arXiv 2202.00379) Table 4: LIO-SAM
     on NTU at 0.069–0.140 m (mean ≈ 0.096 m). Authors ran LIO-SAM
     themselves using their own plug-and-play launcher.
   - (c) SLICT RA-L 2023 Table IV: 0.04–0.20 m, same band as (b).
   - Conclusion: our ~0.2 m cluster is NOT validated. Both
     authoritative references put LIO-SAM below 0.15 m on NTU.

Effort spent: ~1 hr web research (completed). Cross-check runs still
in background dispatch (bv7dp80zw).

### P4.3 — Native-paper dataset runs

Each algo ran on its own paper's validation sequence(s) at ±10 % target:
- FAST-LIO2 × HKU Avia main_building (sub-metre closed-loop)
- Point-LIO × racing_drone (convergence check)
- iG-LIO × Newer College short (0.08–0.10 m target)
- LIO-SAM × park_dataset (< 1 m closed-loop drift)
- SLICT × NTU eee_01 (0.07 m target) — **blocked on SLICT build** (P1 known)

Effort: depends on bag availability — most are public but not yet
downloaded. Could be several days of data prep.

### P4.4 — Root cause + remediation (**now unblocked**)

P4.2 step 4 (paper reference verification) confirmed the integration
gap is real. Enter the Research → Codex → Architect → Build → Eval
pipeline per CLAUDE.md. Research stage targets:

1. **brytsknguyen/LIO-SAM fork: canonical NTU VIRAL launcher.**
   Inspect the fork's NTU example launch file (e.g.
   `LIO-SAM/launch/run_ntuviral.launch` or equivalent) against our
   `baselines/scripts/run_baseline.sh` LIO-SAM NTU invocation. Diff
   every: topic name, frame_id, TF tree, IMU sensor model, feature
   extraction thresholds, mapping downsample leaf sizes,
   ISAM2 / GTSAM optimiser config, loop-closure enable flag.
2. **IMU / LiDAR extrinsic.** Verify R_B_L, t_B_L, gravity convention,
   and the interpretation direction (some NTU launchers publish T_L_B,
   others T_B_L). A ~0.1 m drift is consistent with ~cm-level extrinsic
   offset amplified over 300 s trajectory.
3. **R_W2NED application.** We apply `R_W2NED = diag(1,-1,-1)` somewhere
   in our runner (commit e56d335). If the fork already transforms to
   NED internally, our layer is a double transform.
4. **GT alignment methodology.** Our Umeyama SE(3) no-scale vs the
   NTU VIRAL authors' likely loop-closed evaluation — different
   alignment can account for a fixed per-seq bias but not a clustered
   2× gap. Still worth confirming.
5. **sbs_03 2nd-order outlier.** Only after the systematic gap is
   resolved, diagnose the additional sbs_03 failure mode.

---

## Recommendation (updated 2026-04-13 PM)

P4.2 step 4 is **done**; the research question is answered: our
LIO-SAM NTU integration has a systematic gap. Next immediate actions:

1. **Wait for background NTU cross-check (bv7dp80zw) to finish.**
   FAST-LIO2 × NTU also has SLICT Table IV reference
   (0.07–0.22 m band) — if our FAST-LIO2 also comes in 2× higher,
   the gap is cross-algo (likely a shared NTU integration layer:
   topic / extrinsic / rate / TF). If FAST-LIO2 matches published,
   the gap is LIO-SAM-specific (fork-local config issue).
2. **Enter Research on LIO-SAM NTU launcher diff** (P4.4 item 1).
   Delegate to Explore / docs-lookup for repo inspection. Do NOT
   touch params before diffs are in hand.
3. **Do NOT start P4.3 (native-paper datasets) yet.** Publication-
   grade paper reproduction is blocked until the canonical NTU VIRAL
   reproduction is within ±20 %.
