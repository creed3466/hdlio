# RCC Command Prompt: /experiment

Source: commands/experiment.md

Use this prompt to run the RCC `experiment` workflow.


# /experiment

Register a new experiment in RCC pipeline state and link it to an
experiment tracker (W&B / MLflow / TensorBoard) and a CC Task.

## Workflow

1. **Read pipeline state**
   - `Read .claude/research-state.json`. If absent, initialize via the
     stage-record hook (auto-created on first agent invocation).

2. **Gate check**
   - The experiment stage belongs to **Build** in the 5-stage pipeline.
     Verify `current_stage` is one of `build` or `eval`. If earlier,
     warn that Research/Codex Review/Architect should complete first.

3. **Gather metadata** via `AskUserQuestion`:
   - `experiment_name` (3-50 chars, hyphen-separated)
   - `hypothesis` (one sentence)
   - `expected_direction` (improves|matches|degrades)
   - `tracker` (wandb | mlflow | tensorboard | none)

4. **Invoke the `experiment-tracking` skill**
   ```
   Skill: experiment-tracking
   args: "register name=<name> hypothesis='...' tracker=<tracker>"
   ```
   The skill returns:
   - recommended `init` code snippet for the chosen tracker
   - reproducibility cell (seeds, env capture)
   - link to `active_baseline_ref` from research-state.json

5. **Mirror to CC Task**
   - `TaskCreate({ subject: "Run experiment: <name>", description: <hypothesis> })`
   - Record returned task id.

6. **Update research-state.json**
   ```json
   active_experiment: {
     name, hypothesis, started_at, tracker, run_id, task_id
   }
   ```

7. **Return** a one-screen summary to the user:
   - experiment name, tracker, baseline pinned, CC task id
   - the init snippet to paste into their training script

## Failure cases

- Out-of-stage (research/codex-review/architect not done) → warn but allow
  (researcher may want to log preparatory runs).
- No baseline pinned → suggest `/baseline` first.
- Tracker MCP not available → fall back to filesystem-only logging.

## Related

- `/baseline` — pin or inspect baseline
- `/reproduce` — re-run latest experiment with locked seed
- `/sweep` — launch HPO sweep
- skill: `experiment-tracking`
