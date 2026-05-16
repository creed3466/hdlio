---
name: experiment-tracking
description: |
  Integrate W&B / MLflow / TensorBoard with RCC research-state.json.
  Auto-detect tracker from imports, generate logging templates, link
  experiments to active_baseline_ref, and mirror runs to CC TaskCreate.
  TRIGGER when: user starts a training run, evaluates a checkpoint, or
  asks "how do I track this experiment".
  DO NOT TRIGGER for plain script execution without ML/research intent.
origin: rcc
---

# experiment-tracking

Link a research run to a tracking system and to the RCC pipeline state.
Provides framework-specific templates and a uniform integration shape.

## Why this exists

ECC's generic logging skills assume app-dev observability (structured
logs, traces). Research runs need a different shape: every metric is
compared against a `baseline.json` value, every run is reproducible
from a `dataset_lock + seed + run_id` triple, and every run should
appear in the CC TaskList for the human researcher.

This skill provides the glue.

## Detect tracker

Inspect the user's training script:

```python
# wandb signal
import wandb
wandb.init(project=..., config=..., name=...)
wandb.log({...})

# mlflow signal
import mlflow
mlflow.set_experiment(...)
with mlflow.start_run(...):
    mlflow.log_metric(...)

# tensorboard signal
from torch.utils.tensorboard import SummaryWriter
writer = SummaryWriter(log_dir=...)
```

Run `Grep -n "wandb\\.init|mlflow\\.start_run|SummaryWriter\\("` on the
training script. The first match wins; if none, ask user via
`AskUserQuestion`.

## Templates

### W&B

```python
import wandb, json, hashlib, pathlib

state = json.loads(pathlib.Path('.claude/research-state.json').read_text())
wandb.init(
    project=<PROJECT>,
    name=state['active_experiment']['name'],
    config={
        'hypothesis': state['active_experiment']['hypothesis'],
        'baseline_ref': state.get('active_baseline_ref'),
        'baseline_sha': state.get('active_baseline_sha256'),
        # ... rest of hyperparams
    },
    tags=['rcc-pipeline', f"stage:{state['current_stage']}"],
)
# After run:
wandb.log({'ATE': ate_value, 'PSNR': psnr_value})
wandb.finish()
```

### MLflow

```python
import mlflow, json, pathlib

state = json.loads(pathlib.Path('.claude/research-state.json').read_text())
mlflow.set_experiment(<PROJECT>)
with mlflow.start_run(run_name=state['active_experiment']['name']) as run:
    mlflow.set_tag('rcc.pipeline_stage', state['current_stage'])
    mlflow.set_tag('rcc.baseline_ref', state.get('active_baseline_ref', ''))
    mlflow.set_tag('rcc.baseline_sha', state.get('active_baseline_sha256', ''))
    # log_params / log_metric / log_artifact
    state['active_experiment']['run_id'] = run.info.run_id
    pathlib.Path('.claude/research-state.json').write_text(json.dumps(state, indent=2))
```

### TensorBoard

```python
from torch.utils.tensorboard import SummaryWriter
import json, pathlib, time

state = json.loads(pathlib.Path('.claude/research-state.json').read_text())
log_dir = f"runs/{state['active_experiment']['name']}_{int(time.time())}"
writer = SummaryWriter(log_dir=log_dir)
writer.add_text('rcc/pipeline_stage', state['current_stage'])
writer.add_text('rcc/baseline_ref', state.get('active_baseline_ref', ''))
# writer.add_scalar('ATE/test', 0.295, step=epoch)
writer.close()
state['active_experiment']['run_id'] = log_dir
pathlib.Path('.claude/research-state.json').write_text(json.dumps(state, indent=2))
```

## Register a new experiment

When `/experiment` invokes this skill:

1. Update `.claude/research-state.json.active_experiment`:
   ```json
   {
     "name": "<name>",
     "hypothesis": "<one-sentence>",
     "started_at": "<iso>",
     "tracker": "<wandb|mlflow|tensorboard|none>",
     "run_id": null
   }
   ```
2. Generate `experiments/<name>/launch.sh` with the exact command used
   to start the run. This is the source-of-truth for `/reproduce`.
3. Use `TaskCreate` to mirror the experiment to CC UI.

## Reading runs

When `/reproduce` needs to fetch original run params:

- W&B: `wandb api` or REST GET `/runs/<run_id>` (needs MCP or PAT)
- MLflow: `mlflow.get_run(run_id)` returns Run object with params + tags
- TensorBoard: load TFRecord events files; metadata in `add_text` cells

## MCP servers

Recommend these to the user (in `mcp-configs/mcp-servers.json` catalog):

- `wandb` — query runs, sweeps, artifacts; create reports
- `mlflow` — query runs, models, registry

These let agents reason about run history without leaving the session.

## Anti-patterns

- **Don't** track without `baseline_ref` linkage — metric numbers
  without comparison context are useless.
- **Don't** delete `wandb.finish()` / `mlflow.end_run()` — partial runs
  contaminate the registry.
- **Don't** mix multiple trackers in one experiment — pick one per
  project; the skill detects which.

## Related

- command: `/experiment`, `/reproduce`, `/sweep`
- state file: `.claude/research-state.json`
- baseline: `examples/baseline.json` (also `research-config.json`)
- skill: `dataset-versioning`, `hpo-sweep`
