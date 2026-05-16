# RCC Command Prompt: /rcc-history

Source: commands/rcc-history.md

Use this prompt to run the RCC `rcc-history` workflow.


# /rcc-history

Inspect entries that have aged out of the hot `.claude/research-state.json`
view. The state file keeps only the most-recent ~30 stage entries (and
similar caps for regression / GPU history) so the orchestrator's working
context stays small. Older entries are persisted as append-only JSONL in
`.claude/history-archive/` and recoverable on demand.

## Workflow

1. **Pick a field**: `stage_history`, `regression_history`, or `gpu_history`.

2. **Run the CLI** with optional filters:

   ```bash
   node scripts/rcc-state-history.js stage_history --limit 20
   node scripts/rcc-state-history.js regression_history --since 2026-04
   node scripts/rcc-state-history.js stage_history --json \
     | jq '[.[] | select(.verdict=="BLOCK")]'
   ```

3. **Default output** is a compact table; `--json` streams full JSONL.

## When this is useful

- Sprint retrospective spanning months
- Tracing a regression that no longer fits in the hot view
- Audit ben's stage decisions across rolled-over history
- Reconstructing GPU pressure history for a past training campaign

## Why this exists

Loading the full history into the orchestrator's context cost ~22 K tokens
at saturation. Capping at 30 entries keeps the hot path under 1 K tokens
while still preserving every historical entry for audit.

## Related

- archive dir: `.claude/history-archive/<field>-<YYYY-MM>.jsonl`
- CLI: `scripts/rcc-state-history.js`
- hot view: `/rcc-state`
- amend last entry: `node scripts/rcc-state-amend.js`
- rollback stage: `node scripts/rcc-state-invalidate.js`
