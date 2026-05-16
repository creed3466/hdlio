# RCC Command Prompt: /build

Source: commands/build.md

Use this prompt to run the RCC `build` workflow.


# Build Command

Enter **Stage 3: Build** of the Research → Architect → Build → Eval pipeline.

## What This Command Does

1. Implement against the design produced in Architect — not against vague discussion
2. Surface design drift explicitly rather than silently diverging
3. Route to language-specific reviewers / build resolvers as needed
4. Emit implementation notes and test / execution observations

## When to Use

- After Architect stage has produced a spec under `docs/specs/`
- When executing a planned implementation phase

## Arguments

`$ARGUMENTS` — build task, usually a reference to the spec section being implemented.

## Delegation

Invoke the `ben` agent with a Build-stage brief. Ben will prefer:
- language-specific reviewers: `python-reviewer`, `typescript-reviewer`, `cpp-reviewer`, `rust-reviewer`, `go-reviewer`, `java-reviewer`, `kotlin-reviewer`, `csharp-reviewer`, `flutter-reviewer`
- build resolvers: `build-error-resolver`, `cpp-build-resolver`, `go-build-resolver`, `rust-build-resolver`, `java-build-resolver`, `kotlin-build-resolver`, `dart-build-resolver`, `pytorch-build-resolver`
- `code-reviewer` for implementation quality checks
- `tdd-guide` when TDD discipline is required

## Expected Output

- Code changes
- Implementation notes (what was implemented, what was deferred)
- Explicit list of deviations from architecture
- Execution / test notes

Implementation notes go to `docs/experiments/` when they capture experimental runs.

## Drift Policy

If the implementation needs to diverge from the spec:
1. State the drift explicitly
2. Update `docs/specs/` to reflect the new design
3. Do not let the spec and the code silently disagree
