'use strict';

/**
 * Single source of truth for sub-agent → pipeline-stage mapping.
 *
 * Two consumers historically diverged:
 *   - scripts/hooks/stage-record.js — wide map (record EVERY sub-agent
 *     dispatch into stage_history; ~20 agents)
 *   - scripts/hooks/stage-gate.js   — narrow map (only gate sub-agents
 *     with hard prerequisite chains; ~6 agents)
 *
 * The split is intentional (reviewers should be tracked post-hoc but
 * not gated against build/eval prereqs because they are often invoked
 * ad-hoc), BUT the two maps repeatedly drifted out of sync. This file
 * is the canonical definition both hooks import.
 *
 * Audit (2026-05-14): the pre-consolidation state had
 *   stage-record  → paper-reviewer: 'research'
 *   stage-gate    → paper-reviewer: 'paper-review'
 * which meant gate validated paper-reviewer against eval-PASS prereq
 * but record entered it as a research-stage entry — a hidden asymmetry
 * that surfaced during the S2 audit.
 */

// Wide map: every sub-agent whose dispatch should appear in
// stage_history. Reviewers go to codex-review (adversarial gate);
// build-resolvers and refactor-cleaner to build; e2e/tdd to eval.
// general-purpose intentionally excluded (too generic).
const STAGE_FOR_SUBAGENT = Object.freeze({
  // research stage
  researcher: 'research',
  'paper-reviewer': 'research',

  // codex-review stage (adversarial / model-independent)
  'codex-reviewer': 'codex-review',
  'code-reviewer': 'codex-review',
  'security-reviewer': 'codex-review',
  'cpp-reviewer': 'codex-review',
  'python-reviewer': 'codex-review',
  'rust-reviewer': 'codex-review',
  'go-reviewer': 'codex-review',
  'typescript-reviewer': 'codex-review',
  'java-reviewer': 'codex-review',
  'kotlin-reviewer': 'codex-review',
  'csharp-reviewer': 'codex-review',
  'flutter-reviewer': 'codex-review',

  // architect stage
  architect: 'architect',
  'code-architect': 'architect',
  planner: 'architect',

  // build stage (implementation / fix)
  'build-error-resolver': 'build',
  'cpp-build-resolver': 'build',
  'rust-build-resolver': 'build',
  'go-build-resolver': 'build',
  'java-build-resolver': 'build',
  'kotlin-build-resolver': 'build',
  'dart-build-resolver': 'build',
  'pytorch-build-resolver': 'build',
  'refactor-cleaner': 'build',

  // eval stage (test / measurement)
  'e2e-runner': 'eval',
  'tdd-guide': 'eval',
});

// Narrow map: only sub-agents whose dispatch must be GATED on
// prerequisite chains. Reviewers + build-resolvers are commonly
// invoked ad-hoc and gating them blocks legitimate work. They are
// still tracked by STAGE_FOR_SUBAGENT (stage-record) above.
//
// Note: paper-reviewer maps to 'paper-review' (a virtual stage with
// EXTRA_GATES pointing to eval). This differs from STAGE_FOR_SUBAGENT
// where paper-reviewer is 'research' — deliberate. The two maps
// answer different questions:
//   - record: "what stage's history bucket does this entry belong to?"
//   - gate:   "what prereq chain must be satisfied before this dispatch?"
const GATE_STAGES = Object.freeze({
  researcher: 'research',
  'codex-reviewer': 'codex-review',
  architect: 'architect',
  'code-architect': 'architect',
  planner: 'architect',
  'paper-reviewer': 'paper-review',
});

// Independent gate: paper-reviewer requires eval to have PASSed.
const EXTRA_GATES = Object.freeze({
  'paper-review': Object.freeze(['eval']),
});

// Set of agents whose response output should be checked for
// review_integrity (Defect B3-recurrence safeguard). Adversarial
// stages MUST forward to an independent model — self-inspection is
// a violation of the stage's purpose.
const ADVERSARIAL_AGENTS = Object.freeze(new Set([
  'codex-reviewer',
  'paper-reviewer',
]));

module.exports = {
  STAGE_FOR_SUBAGENT,
  GATE_STAGES,
  EXTRA_GATES,
  ADVERSARIAL_AGENTS,
};
