# RCC Command Prompt: /baseline

Source: commands/baseline.md

Use this prompt to run the RCC `baseline` workflow.


# /baseline

Manage the project's metric baseline pin used by eval-regression-check
and `/reproduce`.

## Workflow

1. **Read config** — `research-config.json` for `metrics[].baseline_path`.
2. **Read state** — `research-state.json.active_baseline_ref` and
   `active_baseline_sha256`.
3. **Compare** — compute `sha256` of the on-disk baseline; if it differs
   from the pinned hash, baseline is *dirty*.

4. **If clean**: pretty-print metrics table:

   ```
   Metric        Baseline   Direction          Tolerance
   ATE           0.293      lower_is_better    ±5%
   PSNR          28.5       higher_is_better   ±5%
   ```

5. **If dirty**: `AskUserQuestion`:
   - **Re-pin** — replace `active_baseline_sha256` with new value (one-line
     confirmation of new metric values shown)
   - **Revert file** — `git checkout <baseline_path>` to restore pinned
   - **Cancel** — leave state inconsistent (user takes responsibility)

6. **Pin operation**:
   ```bash
   sha256sum <baseline_path>
   # update research-state.json:
   #   active_baseline_ref + active_baseline_sha256
   ```

7. **Mirror to memory**: when re-pinning, append to
   `~/.claude/memory/<project>_baseline_log.md` so the lineage is durable
   across sessions.

## Output contract

- Table of metrics with current pinned values
- Status: `CLEAN` / `DIRTY` / `MISSING`
- Recommended next action

## Related

- `/experiment` — register new run linked to current baseline
- `/reproduce` — re-run with this baseline as comparison target
- `scripts/hooks/eval-regression-check.js` — reads same baseline
