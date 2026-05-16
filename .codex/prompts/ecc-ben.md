# RCC Command Prompt: /ben

Source: commands/ben.md

Use this prompt to run the RCC `ben` workflow.


# Ben Command

Delegate to the **ben** agent, the default orchestrator for research-driven work.

## What This Command Does

1. Identify the current stage (Research, Architect, Build, or Eval)
2. Route to the appropriate specialist agent
3. Enforce stage gates (no Build without Architect; no Architect without Research)
4. Bridge context between stages with a compact Goal / Baseline / Proposal / Evidence / Unresolved summary
5. Run adversarial review before major research or architecture conclusions are finalized

## When to Use

- Non-trivial research-driven work (algorithms, models, architecture, theoretical claims)
- When you are unsure which stage you are in
- When you need stage gating enforced rather than self-managed

## Arguments

`$ARGUMENTS` — free-form task description. Ben will infer the stage.

## Delegation

Invoke the `ben` agent. Pass the arguments as the task brief. Ben will:
- report the identified stage
- state what has been established
- state what remains uncertain
- delegate the next step to the appropriate specialist agent

## Related

- `/research`, `/architect`, `/build` — stage-specific entry points
- `/eval` — eval-harness skill (formal evaluation runner)
