---
name: paper-review-iterate
description: >-
  Iterative paper review-and-revise workflow with strict context isolation.
  Compiles the paper PDF, dispatches parallel adversarial reviewers
  (Claude Opus 4.7 + Codex GPT-5.5), cross-validates findings, applies
  fixes with user approval, recompiles, and loops until no BLOCKING
  issues remain or the user stops. The reviewers receive ONLY the PDF
  and target venue — no project-internal context. Supports two modes:
  standard (2 parallel reviewers) and deep (up to 10 reviewer×role
  combinations in parallel).
  TRIGGER when: user requests adversarial paper review, pre-submission
  audit, or revision loop for a paper PDF targeting a specific venue.
  DO NOT TRIGGER when: paper is not yet drafted, or user wants
  project-aware feedback (use a different skill/agent for that).
origin: euntae
---

# Paper Review Iterate — Multi-Model Adversarial Review Loop

A workflow that drives a paper through repeated adversarial review by
Claude Opus 4.7 and Codex GPT-5.5 in parallel, cross-validates findings,
applies fixes, recompiles, and loops to convergence.

## When to Use

- A paper PDF is ready for adversarial review before submission.
- You want **model-independent** cross-validation (Opus + GPT-5.5).
- Project-internal context must NOT leak into the review.
- Multiple iteration cycles are expected (revise → recompile → re-review).

## When NOT to Use

- The paper is not yet drafted (use writing assistance instead).
- The user wants project-aware feedback (e.g., "is this consistent with
  our other papers?") — use a project-aware skill instead.
- A single review without iteration suffices — invoke the
  `paper-reviewer` agent directly.

## Required Inputs

- `pdf_path`: absolute path to the compiled PDF
- `venue`: e.g., "IEEE RA-L", "ICRA 2026", "IROS 2026", "CVPR 2026", "TRO"
- `compile_command` (optional): how to recompile after edits (e.g.,
  `cd paper/latex && /tmp/tectonic main.tex`)
- `mode` (optional): `standard` (default, 2 parallel reviewers) or
  `deep` (up to 10 reviewer×role combinations)
- `max_iterations` (optional): default 5

## Workflow

### Stage 0: Setup

1. Verify `pdf_path` exists and has a positive page count via `pdfinfo`.
2. If `compile_command` is unknown, ask user.
3. Create a working dir: `docs/reports/paper-review-<YYYYMMDD>/`.
4. Initialize iteration counter `i=1`.

### Stage 1: Compile (skip on iteration 1 if PDF already exists)

Run `compile_command`. Verify:
- Exit code 0
- PDF written
- Page count within venue limit (warn if exceeded)

### Stage 2: Parallel Review

#### Standard mode (2 reviewers)

Spawn TWO `paper-reviewer` agents in a **single message** (parallel):

```
Agent 1: paper-reviewer
  prompt:
    pdf_path: <absolute>
    venue: <user-stated>
    reviewer: claude-opus
    role: general

Agent 2: paper-reviewer
  prompt:
    pdf_path: <absolute>
    venue: <user-stated>
    reviewer: codex-gpt55
    role: general
```

**CRITICAL:** prompts contain ONLY the four fields above. No additional
context. No "by the way", no "the authors mention", no project paths.
The agent's input contract refuses anything else.

#### Deep mode (up to 10 reviewers)

Spawn up to 10 agents in a single message:

```
claude-opus  × {general, math-rigor, empirical-fairness, reproducibility, novelty}
codex-gpt55  × {general, math-rigor, empirical-fairness, reproducibility, novelty}
```

Use deep mode when the paper is in late-stage pre-submission and a
thorough audit justifies the cost (~$5-15 in tokens depending on length).

### Stage 3: Cross-Validate

Build a finding-matrix:

| Issue (location + brief) | Opus general | Codex general | Opus math | Codex math | ... | Confidence |
|--------------------------|:------------:|:-------------:|:---------:|:----------:|-----|:----------:|
| Eq. 5 derivation incomplete | BLOCK | BLOCK |   |   |   | HIGH |
| LIO-SAM missing Table III | — | BLOCK |   |   |   | MED |
| Eq. 7 vs Eq. 1 α_min mismatch | — | BLOCK |   |   |   | MED |
| ... | | | | | | |

Confidence rules:
- **HIGH**: ≥2 reviewers agree on severity (BLOCK/MAJOR/MINOR) with
  matching location.
- **MED**: 1 reviewer flagged. Flag for human triage.
- **LOW**: only mentioned in passing or strengths-section. Defer.

Independent findings (one model only) are valuable — record them. Do not
silently drop.

### Stage 4: Triage with User

Present a single consolidated table. For each issue, the user decides:

| Action | Meaning |
|--------|---------|
| **Accept** | Apply the suggested fix (or a refined version) |
| **Rebut** | Write a rebuttal note for the venue's response phase |
| **Defer** | Record but do not act this iteration |
| **Reject** | Reviewer is wrong; document why |

Aim to complete all HIGH-confidence issues per iteration. MED and LOW
can carry over.

### Stage 5: Apply Fixes

For each Accepted item:
1. Read affected section.
2. Apply minimal edit (Edit tool).
3. Note the change in a per-iteration changelog at
   `docs/reports/paper-review-<YYYYMMDD>/iter-<N>-changes.md`.

### Stage 6: Recompile + Page-Count Gate

Run `compile_command`. Verify:
- Build success
- Page count ≤ venue limit
- No new compile warnings introduced (compare warning count before/after)

If the page count exceeds the venue limit, **rollback this iteration's
edits** and ask the user to choose between:
- Tighter wording
- Drop one fix
- Move content to supplementary

### Stage 7: Re-review (Optional)

If user requests, re-spawn `paper-reviewer` agents on the updated PDF.

Compute monotonicity check:
- BLOCKING count must monotonically decrease, OR
- Newly introduced BLOCKING items must be explicitly justified
  (e.g., a fix to one issue exposed a deeper one)

If a NEW BLOCKING is introduced and not justified → **rollback** and
re-triage.

### Stage 8: Commit

```bash
git add -A paper/
git commit -m "rev(paper): address <issue list> (iter <N>)"
```

Commit message lists the resolved issues by location.

### Stage 9: Loop or Terminate

Continue to next iteration if:
- BLOCKING count > 0
- iteration < max_iterations
- user has not stopped

Terminate if:
- BLOCKING count = 0 (success)
- user stops
- iteration ≥ max_iterations
- page overflow that cannot be resolved
- monotonicity violated and rollback chosen

## Termination Report

Always end with a report at `docs/reports/paper-review-<YYYYMMDD>/final.md`:

```markdown
## Final Status
- Iterations: N
- Final verdict (consensus): ACCEPT | MINOR_REVISION | MAJOR_REVISION | REJECT
- Blocking remaining: K
- Major remaining: K
- Minor remaining: K
- Final page count: P / venue-limit
- Final commit hash: <sha>

## Cross-Validation Matrix (Final)
<table>

## Issues Resolved Across Iterations
<list>

## Issues Deferred / Rebutted
<list with rationale>

## Reviewer Disagreements
<cases where Opus and Codex disagreed — useful for the rebuttal>
```

## Cost Awareness

| Mode | Calls per iteration | Approximate cost |
|------|---------------------|------------------|
| standard | 2 (Opus + Codex) | $1–3 |
| deep | up to 10 (5 roles × 2 engines) | $5–15 |

For typical 9-page IEEE papers expect 3-5 iterations to reach a clean
review. Plan token budget accordingly.

## Anti-patterns

- FAIL: Including any project context in the agent prompt — defeats the
  isolation guarantee.
- FAIL: Skipping the page-count gate — late-stage page overflow is painful.
- FAIL: Accepting all reviewer suggestions blindly — disagreements between
  Opus and Codex are signal, not noise.
- FAIL: Editing across iterations without git commits — review history is
  lost.
- FAIL: Running deep mode on every iteration — reserve for final pre-submit
  check.

## Example Invocation

User: "Review my paper at paper/latex/main.pdf for IEEE RA-L. Iterate
until clean."

Skill flow:
1. Compile (cd paper/latex && tectonic main.tex)
2. Verify 9 pages
3. Spawn Opus + Codex (standard mode) in parallel
4. Cross-validate → consolidated table
5. Ask user to triage HIGH-confidence findings
6. Apply accepted fixes
7. Recompile, verify page count
8. Loop steps 3–7 until BLOCKING = 0 or iteration cap
9. Write final.md and commit
