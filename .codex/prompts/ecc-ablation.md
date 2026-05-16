# RCC Command Prompt: /ablation

Source: commands/ablation.md

Use this prompt to run the RCC `ablation` workflow.


# /ablation

Design and queue a systematic ablation study. Operates on the
hypothesis recorded by the most recent `researcher` agent run.

## Workflow

1. **Read research artifact**
   - Locate the most recent `output_ref` in
     `research-state.json.stage_history` with `stage = "research"`.
   - `Read` that file. Extract:
     - Components mentioned in `## Mathematical Analysis` / `## Hypothesis`
     - Each will become an ablation knob.

2. **Component selection** via `AskUserQuestion` (multiSelect):
   - List up to 6 detected components, each with a one-line description.
   - User picks which to ablate.

3. **Matrix generation**
   - For each selected component, generate two configs: with / without
     (or two parameter levels).
   - Compose Cartesian product (up to 16 cells; warn if larger).

4. **Preview** — use `AskUserQuestion.preview` to show the ablation
   table:

   ```
   id    component_A     component_B     component_C
   abl1  off             on              high
   abl2  off             on              low
   abl3  on              on              high
   ...
   ```

5. **Generate config**
   - Write `experiments/ablation_<timestamp>.yaml` listing each cell
     with run name + config diff.
   - Update `research-state.json.experiments[]` with one
     `PENDING` entry per cell.

6. **CC Task fanout**
   - `TaskCreate` per cell with `addBlockedBy` linking to a single
     "ablation parent" task.
   - Each cell task: "Ablation #N: <component diff>"

7. **Suggest `/sweep`** when matrix > 10 cells (suggests Optuna /
   Ray Tune for efficiency).

## Failure cases

- No prior researcher output → suggest `/research` first.
- Hypothesis has no decomposable components → ask user to specify
  manually via free-text input.

## Related

- `/experiment` — register single run
- `/sweep` — HPO-style search instead of full grid
- `/reproduce` — re-run specific ablation cell

## Output contract

- Path to generated `experiments/ablation_*.yaml`
- Number of cells queued
- Parent task id + child task ids
