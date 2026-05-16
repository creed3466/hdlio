# RCC Command Prompt: /research

Source: commands/research.md

Use this prompt to run the RCC `research` workflow.


# Research Command

Enter **Stage 1: Research** of the Research → Architect → Build → Eval pipeline.

## What This Command Does

1. Clarify the problem statement
2. Inspect papers, official docs, prior methods, baselines, and constraints
3. Extract the strongest candidate direction
4. Run a mandatory review pass before the research result is treated as stable input to Architect

## When to Use

- Starting any non-trivial algorithmic, modeling, architectural, or theoretical work
- When the problem statement or baseline is unclear
- When you need to compare against prior approaches

## Arguments

`$ARGUMENTS` — topic, paper, problem, or research question.

## Delegation

Invoke the `ben` agent with a Research-stage brief. Ben will prefer these specialists:
- `docs-lookup` — official library / API / framework references
- `deep-research` skill — multi-source web + code research via Exa / Firecrawl
- `search-first` skill — existing-solution discovery before writing anything new
- `planner` — problem decomposition
- `architect` — if the research reveals a structural question

## Expected Output

- Problem statement
- Baseline or prior approach
- Key evidence (citations, data, measurements)
- Candidate proposal
- Open questions

Result is saved to `docs/research/` when durable.

## Gate to Architect

Do not promote a research result into Architect until:
- the review pass has been processed
- every major critique is classified as Accept / Rebut / Defer
- no unresolved Defer items block implementation
