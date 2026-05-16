# RCC Command Prompt: /sweep

Source: commands/sweep.md

Use this prompt to run the RCC `sweep` workflow.


# /sweep

Launch a hyperparameter sweep and auto-track progress via
`ScheduleWakeup`. The session is freed after launch and re-enters
when polling intervals elapse.

## Workflow

1. **Skill: hpo-sweep**
   - Read project to detect existing framework (Optuna study script,
     `wandb sweep` yaml, Ray Tune script).
   - If none detected, ask user which to set up.

2. **Search strategy** via `AskUserQuestion`:
   - Optuna Bayesian (TPE) — most general
   - Ray Tune ASHA — early-stopping bandit
   - W&B Sweeps — grid / random / bayes (uses W&B server)

3. **Generate config**
   - `experiments/sweep_<timestamp>.yaml` (or `optuna_*.py` etc).
   - Number of trials (`AskUserQuestion`).
   - Metric direction + early-stopping criteria.

4. **Pre-flight**
   - GPU snapshot (`gpu-profile-snapshot` hook reads current VRAM).
     Warn if existing usage > 50%.
   - Verify dataset locks (same as `/reproduce`).

5. **Launch**
   ```bash
   # Bash with run_in_background: true
   #   optuna create-study ... && optuna study optimize ...
   #   or: wandb sweep config.yaml && wandb agent <sweep-id>
   #   or: python ray_tune_search.py
   ```

6. **Schedule polling loop**
   - `ScheduleWakeup(delaySeconds=1800, reason="poll <sweep-id>")`
   - On wakeup: check tracker for trial count + best metric.
     If best metric improves past tolerance OR all trials complete →
     stop polling. Else re-schedule.

7. **TaskCreate** per trial (lazy — only when a trial completes).

## Convergence detection

Stop polling when one of:
- All trials complete.
- Top-K trials' variance < `convergence_threshold` (default 1% of metric range).
- User has explicitly stopped (`/loop --stop sweep`).

## Failure cases

- No tracker MCP and no local sweep tool → suggest `pip install optuna`.
- GPU OOM during launch → surface error, attempt smaller batch suggestion.

## Related

- skill: `hpo-sweep`
- hook: `sweep-launch-detect` — auto-schedules polling
- `/experiment` — single run instead of sweep
- `/ablation` — full-grid alternative
