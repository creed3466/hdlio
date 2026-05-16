# RCC Command Prompt: /rcc-state

Source: commands/rcc-state.md

Use this prompt to run the RCC `rcc-state` workflow.


# /rcc-state

Inspect the current research pipeline state. Read-only operation.

## Workflow

1. **Read** `.claude/research-state.json`. If absent, print:
   ```
   No RCC pipeline state in this project.
   Initialize by invoking the `researcher` agent (Task → researcher).
   ```

2. **Display sections**:

**A) Header**
   ```
   RCC pipeline state — <project>
   Current stage: <stage>
   Started:       <iso>
   Baseline:      <active_baseline_ref>  sha256: <12-char-prefix>
   Experiment:    <active_experiment.name>  (tracker: <tracker>, run: <run_id>)
   ```

**B) Stage history (last 5)**
   ```
   Stage           Agent             Verdict  Output ref
   research        researcher        PASS     docs/research/note-2026-05-11.md
   codex-review    codex-reviewer    PASS     —
   architect       architect         PENDING  docs/specs/design.md
   ```

**C) Pipeline locks**
   ```
   codex-review    ← research                 PASS: unlocked
   architect       ← research, codex-review   PASS: unlocked
   build           ← architect                 blocked by: architect
   eval            ← build                     blocked by: build
   ```

**D) Regression history (last 5)**
   ```
   Metric  Value   Baseline  Δ%      Verdict
   ATE     0.295   0.293     +0.68   OK
   ATE     0.301   0.293     +2.73   WARN
   ATE     0.420   0.293    +43.34   REGRESSION
   ```

**E) Dataset locks**
   ```
   Path              SHA256 (prefix)  Size       Recorded
   data/raw/train.h5 abc12345…        134 MB     2026-05-11
   ```

**F) GPU history (last 3)**
   ```
   Timestamp            VRAM%   Util%   Command
   2026-05-11T11:45     94%     97%     python train.py
   ```

3. **Suggested next action** at bottom — one of:
   - "Run `/research` to start a new pipeline"
   - "Invoke `<agent>` for stage `<stage>`"
   - "Backtrack: `<stage>` is REVISE — re-run with feedback"
   - "Pipeline converged. Consider invoking the `paper-review-iterate` skill for submission."

## Implementation

Read `.claude/research-state.json` directly (no agent invocation needed).
Use the `Read` tool. Format with the layout above.

## Amending a wrong verdict

When `stage-record`'s auto-detect inferred PENDING (or guessed wrong), use
the amend CLI to correct the most recent `stage_history` entry. The amend
recomputes `passed_stages` + `current_stage` as if the entry had originally
carried the new verdict.

```
node scripts/rcc-state-amend.js PASS "researcher used READY_FOR_BUILD"
node scripts/rcc-state-amend.js BLOCK "rerun: empirically falsified"
node scripts/rcc-state-amend.js --cwd <other-project> PASS
```

The amend preserves the original verdict as `entry.amended_from` and tags
`amended_at` / `amended_reason` / `amended_by` for the audit trail.

A non-empty `entry.detection_source = "none"` in the latest history entry
is the canonical signal that an amend is needed: the auto-detector saw no
verdict marker in the sub-agent response.

## Related

- schema: `schemas/research-state.schema.json`
- lib: `scripts/lib/research-state.js`
- amend CLI: `scripts/rcc-state-amend.js`
- hooks: `stage-gate`, `stage-record`, `precompact-save-state`
