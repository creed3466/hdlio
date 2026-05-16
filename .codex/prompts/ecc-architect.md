# RCC Command Prompt: /architect

Source: commands/architect.md

Use this prompt to run the RCC `architect` workflow.


# Architect Command

Enter **Stage 2: Architect** of the Research → Architect → Build → Eval pipeline.

## What This Command Does

1. Turn the research output into an implementation-ready design
2. Define objective, baseline, proposal, interfaces, risks, and validation plan
3. Run adversarial review before the design is treated as stable input to Build

## When to Use

- Research stage has produced a stable proposal with evidence
- Before writing non-trivial implementation code
- When refactoring large systems or making structural decisions

## Arguments

`$ARGUMENTS` — research result summary or design question.

## Delegation

Invoke the `ben` agent with an Architect-stage brief. Ben will prefer these specialists:
- `architect` — system design, trade-off analysis, interface design
- `planner` — phased implementation plan
- `code-architect` — concrete implementation blueprint grounded in current codebase

## Expected Output

- Objective
- Baseline (what we compare against)
- Proposal (chosen approach)
- Architecture / interfaces (concrete contracts)
- Risks and mitigations
- Validation plan (how Eval will judge success)

Result is saved to `docs/specs/` as a durable spec document.

## Gate to Build

Do not start Build until:
- objective, baseline, proposal are explicit
- validation plan defines what success and failure look like
- adversarial review items have been classified

## Critical Rule

**No non-trivial implementation without passing through Architect.** If Build is about to start without a spec, stop and return to Architect first.
