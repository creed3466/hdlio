# RCC Command Prompt: /reproduce

Source: commands/reproduce.md

Use this prompt to run the RCC `reproduce` workflow.


# /reproduce

Reproduce the active experiment from `research-state.json`. Verifies
dataset checksums first to guarantee reproducibility.

## Workflow

1. **Locate active experiment**
   - Read `research-state.json.active_experiment`.
   - If absent → fail with hint to run `/experiment` first.

2. **Verify dataset locks**
   - For each entry in `dataset_locks[]`, compute current `sha256`
     and compare to recorded value.
   - **If mismatch** → **BLOCK** the reproduce attempt:
     ```
     Dataset drift detected:
       <path>  expected <sha-1234> got <sha-9876>
     ```
   - Suggest either `git restore` the data file or update lock via
     `dataset-versioning` skill.

3. **Verify baseline lock**
   - Compute `sha256` of `active_baseline_ref`; compare to
     `active_baseline_sha256`.
   - If mismatch → warn (don't block; baseline drift is a research
     decision, dataset drift is a reproducibility error).

4. **Reconstruct command**
   - Skill: `experiment-tracking` — given the recorded `run_id`, fetch
     the original launch command from the tracker (W&B/MLflow API)
     or from `experiments/<name>/launch.sh` if filesystem-only.

5. **Execute**
   ```bash
   # in background to free the session
   ```
   Run via `Bash` with `run_in_background: true`. Capture the new
   `run_id`.

6. **Schedule completion poll**
   - `ScheduleWakeup(delaySeconds=1200, reason="checking /reproduce")`
   - On wakeup: check tracker for run status; if complete, invoke
     `eval-regression-check` against `active_baseline_ref`.

7. **Report verdict**
   - `OK` — within tolerance of original run
   - `DRIFT` — within tolerance of *baseline* but not original run
   - `REGRESSION` — outside tolerance vs baseline

## Failure cases

- Dataset drift → BLOCK
- No tracker run_id and no `launch.sh` → ask user to provide command
- Background run failure → surface stderr + leave research-state
  untouched

## Related

- `/baseline` — verify baseline pin first
- `/experiment` — register the run being reproduced
- skill: `experiment-tracking`
- skill: `dataset-versioning`
- hook: `eval-regression-check`
