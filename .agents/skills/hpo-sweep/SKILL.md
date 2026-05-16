---
name: hpo-sweep
description: |
  Launch and manage hyperparameter sweeps (Optuna, Ray Tune, W&B
  Sweeps) with auto-polling via ScheduleWakeup. Each trial mirrors to a
  CC Task. Convergence-aware stopping.
  TRIGGER when: user wants to "sweep <param>", "tune <hyperparams>", or
  invokes `/sweep`.
  DO NOT TRIGGER for single experiments — use `/experiment` instead.
origin: rcc
---

# hpo-sweep

Hyperparameter search is long-running and easy to lose context on.
This skill encodes RCC's auto-polling pattern: launch in background,
schedule wakeup checks, stop on convergence.

## Choose the right tool

| Framework | When |
|---|---|
| **Optuna** | Bayesian (TPE) search, no infra required, in-process. Default for SLAM/3DGS. |
| **Ray Tune** | distributed search, native ASHA / PBT, needs Ray cluster |
| **W&B Sweeps** | server-side coordination, multiple agents on different machines, needs W&B |

## Optuna template

```python
# experiments/sweep_<timestamp>.py
import optuna, json, pathlib

def objective(trial):
    lr = trial.suggest_float('lr', 1e-5, 1e-2, log=True)
    batch = trial.suggest_categorical('batch_size', [16, 32, 64])
    # ... train and return metric
    return metric_value

study = optuna.create_study(
    study_name='ablation-feat-X',
    direction='minimize',          # or maximize
    storage='sqlite:///experiments/sweep.db',
    load_if_exists=True,
)
study.optimize(objective, n_trials=50, n_jobs=1)

# Persist best to research-state.json
state = json.loads(pathlib.Path('.claude/research-state.json').read_text())
state['active_experiment']['best_trial'] = {
    'params': study.best_params,
    'value': study.best_value,
}
pathlib.Path('.claude/research-state.json').write_text(json.dumps(state, indent=2))
```

## Ray Tune ASHA

```python
from ray import tune
from ray.tune.schedulers import ASHAScheduler

config = {
    'lr': tune.loguniform(1e-5, 1e-2),
    'batch_size': tune.choice([16, 32, 64]),
}
scheduler = ASHAScheduler(metric='loss', mode='min', max_t=100, grace_period=10)

tuner = tune.Tuner(
    train_fn,
    param_space=config,
    tune_config=tune.TuneConfig(scheduler=scheduler, num_samples=50),
)
results = tuner.fit()
```

## W&B Sweeps

```yaml
# sweep.yaml
method: bayes
metric:
  name: ATE
  goal: minimize
parameters:
  lr:
    distribution: log_uniform_values
    min: 0.00001
    max: 0.01
  batch_size:
    values: [16, 32, 64]
program: train.py
```

```bash
wandb sweep sweep.yaml
wandb agent <ENTITY/PROJECT/SWEEP_ID>
```

## Auto-polling pattern (CC native)

This is the key research advantage. Long sweeps shouldn't lose context:

1. Launch sweep via `Bash` with `run_in_background: true`.
2. Record sweep id + tracker URL in `research-state.json.active_experiment`.
3. Call `ScheduleWakeup(delaySeconds=1800, reason="poll <sweep-id>")`.
4. On wakeup:
   - Query tracker for trial count + best metric.
   - Convergence check (see below).
   - If converged → stop; else re-schedule.

The `sweep-launch-detect` hook automates step 3 — when it sees an
optuna / wandb sweep / ray tune launch, it schedules the wakeup.

## Convergence detection

Stop polling when **any** of:

- All `n_trials` complete.
- Top-3 trials' metric variance < `convergence_threshold`
  (default 1% of metric range).
- User explicitly stops (`/loop --stop sweep` or kill).
- Time budget exceeded (default 12h).

## Mirroring trials to CC Tasks

Each trial gets a CC task via `TaskCreate` when it completes (lazy —
creating 50 tasks upfront clutters the UI). Format:

```
"Sweep trial #<n>: ATE=0.291 lr=3.2e-4 batch=32"
```

## Anti-patterns

- **Don't** run sweeps inside an interactive Claude Code session
  blocking the conversation. Always background + poll.
- **Don't** sweep without first verifying baseline reproduces in a
  single run — sweeps over a buggy training loop produce nonsense.
- **Don't** sweep more than 4 hyperparameters at once without ASHA-style
  early stopping. The search space blows up.

## Related

- command: `/sweep`, `/ablation` (full-grid alternative)
- hook: `sweep-launch-detect`
- state: `active_experiment.tracker` + `run_id`
- skill: `experiment-tracking`
