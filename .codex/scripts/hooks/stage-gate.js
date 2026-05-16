#!/usr/bin/env node
'use strict';

/**
 * pre:agent:stage-gate (RCC v2.1 Phase α hook)
 *
 * Trigger: PreToolUse on `Task` tool.
 *
 * When ben invokes a subagent corresponding to a downstream pipeline
 * stage (architect, code-reviewer, paper-reviewer), verify that all
 * prerequisite stages in research-state.json are PASS. If not:
 *   profile=minimal  → no-op
 *   profile=standard → warn (exit 0)
 *   profile=strict   → block (exit 2)
 *
 * Stage → required prereqs map (locks):
 *   architect       ← research, codex-review
 *   code-reviewer   ← architect (only during 5-stage build, not for ad-hoc reviews)
 *   paper-reviewer  ← eval
 *
 * No .claude/research-state.json present → graceful no-op.
 */

const fs = require('fs');
const path = require('path');
const rs = require('../lib/research-state');
// S1 (2026-05-14): single source of truth for sub-agent → stage map.
// GATE_STAGES is the narrow map (only agents with hard prereq chains
// get gated; reviewers + build-resolvers stay ungated for ad-hoc use).
const {
  GATE_STAGES: STAGE_FOR_SUBAGENT,
  EXTRA_GATES,
} = require('../lib/subagent-stage-map');

const MAX_STDIN = 1024 * 1024;

function readStdin() {
  return new Promise(resolve => {
    let raw = '';
    process.stdin.setEncoding('utf8');
    process.stdin.on('data', chunk => {
      if (raw.length < MAX_STDIN) {
        const remaining = MAX_STDIN - raw.length;
        raw += chunk.substring(0, remaining);
      }
    });
    process.stdin.on('end', () => resolve(raw));
    process.stdin.on('error', () => resolve(raw));
  });
}

function getProfile() {
  const raw = String(
    process.env.RCC_HOOK_PROFILE || process.env.ECC_HOOK_PROFILE || 'standard'
  ).trim().toLowerCase();
  return ['minimal', 'standard', 'strict'].includes(raw) ? raw : 'standard';
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  // CC has emitted both "Task" and "Agent" for sub-agent dispatch
  // across versions. Accept both.
  const toolName = input.tool_name || input.toolName || '';
  if (toolName !== 'Task' && toolName !== 'Agent') { passthrough(); return; }

  const subagentType = input.tool_input?.subagent_type || '';
  const cwd = input.cwd || process.cwd();
  if (!rs.isValidProjectCwd(cwd)) {
    // Fail open quietly: gate cannot enforce against an unknown project.
    // Logging is informational only — never block on invalid input alone.
    process.stderr.write(
      `[rcc:stage-gate] WARNING: cwd is not a valid project root: ${cwd}\n`,
    );
    passthrough();
    return;
  }

  const state = rs.readState(cwd);
  if (!state) {
    // Disambiguate "no state yet" from "state exists but unreadable".
    // Under strict the latter must NOT fail open — that would let a
    // corrupt state silently bypass every prerequisite lock.
    const statePath = rs.resolveStatePath(cwd);
    if (fs.existsSync(statePath)) {
      const profile = getProfile();
      const header = profile === 'strict'
        ? '[stage-gate] BLOCKED: state.json present but unreadable (corrupt/unknown schema)'
        : '[stage-gate] WARNING: state.json present but unreadable (corrupt/unknown schema)';
      process.stderr.write(`${header}\n`);
      process.stderr.write(`  path: ${statePath}\n`);
      process.stderr.write(`  subagent: ${subagentType}\n`);
      process.stderr.write('  Inspect the file or remove it to recover. Refusing to fail open under strict.\n');
      passthrough();
      process.exit(profile === 'strict' ? 2 : 0);
    }
    passthrough();
    return;   // no state file at all → no enforcement (fresh project)
  }

  // Determine effective stage
  let requestedStage = STAGE_FOR_SUBAGENT[subagentType];
  if (!requestedStage) { passthrough(); return; }  // unknown subagent → no gate

  // Determine prerequisites
  let missing;
  if (EXTRA_GATES[requestedStage]) {
    const passed = new Set(
      (state.stage_history || [])
        .filter(h => h.verdict === 'PASS')
        .map(h => h.stage)
    );
    missing = EXTRA_GATES[requestedStage].filter(s => !passed.has(s));
  } else {
    missing = rs.blockedPrerequisites(state, requestedStage);
  }

  if (missing.length === 0) { passthrough(); return; }

  const profile = getProfile();

  // Defect B2 (2026-05-13): differentiate PENDING (auto-detect missed)
  // from explicit BLOCK (sub-agent issued BLOCK verdict). In standard
  // profile, PENDING is a soft warning (user should amend); BLOCK is a
  // hard block (user must explicitly override or backtrack). In strict,
  // both block.
  //
  // Walk stage_history backward for each missing prereq stage to find
  // the latest verdict on it.
  const latestVerdictPerStage = {};
  for (const h of (state.stage_history || []).slice().reverse()) {
    if (!h || !h.stage) continue;
    if (latestVerdictPerStage[h.stage] === undefined) {
      latestVerdictPerStage[h.stage] = h.verdict || 'PENDING';
    }
  }

  const blockingPrereqs = missing.filter(
    s => latestVerdictPerStage[s] === 'BLOCK'
  );
  const pendingPrereqs = missing.filter(
    s => latestVerdictPerStage[s] === 'PENDING' || latestVerdictPerStage[s] === undefined
  );
  const revisePrereqs = missing.filter(
    s => latestVerdictPerStage[s] === 'REVISE'
  );

  // RCC_FORCE_ADVANCE=1 env override: one-shot bypass for current
  // sub-agent dispatch. Recorded in stage_history as override on next
  // record. Use sparingly — bypasses BOTH block and warn.
  const forceAdvance = String(process.env.RCC_FORCE_ADVANCE || '').trim() === '1';

  let isBlocking;
  if (forceAdvance) {
    process.stderr.write('[stage-gate] FORCE_ADVANCE=1 — bypassing gate (one-shot)\n');
    isBlocking = false;
  } else if (profile === 'strict') {
    isBlocking = true;
  } else if (profile === 'minimal') {
    isBlocking = false;
  } else {
    // standard: BLOCK or REVISE on prereq is hard-block; PENDING is warn-only
    isBlocking = blockingPrereqs.length > 0 || revisePrereqs.length > 0;
  }

  const header = isBlocking
    ? '[stage-gate] BLOCKED: out-of-order pipeline stage'
    : '[stage-gate] WARNING: out-of-order pipeline stage';
  process.stderr.write(`${header}\n`);
  process.stderr.write(`  subagent: ${subagentType} (stage=${requestedStage})\n`);
  process.stderr.write(`  unmet prerequisites: ${missing.join(', ')}\n`);
  process.stderr.write(`  current_stage in state: ${state.current_stage}\n`);
  if (blockingPrereqs.length > 0) {
    process.stderr.write(`  BLOCK verdict on: ${blockingPrereqs.join(', ')}\n`);
    process.stderr.write(
      `  → user-explicit override required: \`node scripts/rcc-state-override.js BLOCK "<rationale ≥20 chars>"\`\n`,
    );
  }
  if (revisePrereqs.length > 0) {
    process.stderr.write(`  REVISE verdict on: ${revisePrereqs.join(', ')}\n`);
    process.stderr.write(`  → return to prereq stage and re-submit\n`);
  }
  if (pendingPrereqs.length > 0) {
    process.stderr.write(`  PENDING (auto-detect missed) on: ${pendingPrereqs.join(', ')}\n`);
    process.stderr.write(
      `  → amend via \`node scripts/rcc-state-amend.js <PASS|REVISE|BLOCK> [reason]\`\n`,
    );
  }
  process.stderr.write('  Override options: RCC_FORCE_ADVANCE=1 (one-shot), RCC_HOOK_PROFILE=minimal\n');

  passthrough();
  process.exit(isBlocking ? 2 : 0);
}

main().catch(() => {
  // Never crash hard — passthrough on any error
  process.exit(0);
});
