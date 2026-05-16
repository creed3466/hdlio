#!/usr/bin/env node
'use strict';

/**
 * post:agent:stage-record (RCC v2.1 Phase α hook)
 *
 * Trigger: PostToolUse on `Task` tool.
 *
 * After ben invokes a subagent, append the resulting stage outcome to
 * research-state.json. Verdict is auto-detected from the subagent
 * response text:
 *   "VERDICT: PASS" / "PASS" header / no critique → PASS
 *   "VERDICT: REVISE" / "REVISE" / "needs revision" → REVISE
 *   "VERDICT: BLOCK" / "BLOCK" / "critical issue" → BLOCK
 *   otherwise → PENDING
 *
 * Also: ensure .claude/research-state.json exists (initialize on first
 * subagent invocation).
 *
 * Profile gating:
 *   minimal  → no-op (don't track state)
 *   standard → record (always exit 0)
 *   strict   → record (always exit 0; gate runs separately)
 */

const fs = require('fs');
const path = require('path');
const rs = require('../lib/research-state');
// S1 (2026-05-14): single source of truth for sub-agent → stage
// mapping. Previously stage-record and stage-gate each defined their
// own map and they drifted (paper-reviewer asymmetry, missing entries
// added in one but not the other). The shared lib eliminates drift.
const {
  STAGE_FOR_SUBAGENT,
  ADVERSARIAL_AGENTS,
} = require('../lib/subagent-stage-map');
const { classifyReviewEvidence } = require('../lib/codex-invocation-evidence');

const MAX_STDIN = 4 * 1024 * 1024; // larger to capture subagent output

// Defect G4 (2026-05-13): `general-purpose` is intentionally NOT in
// STAGE_FOR_SUBAGENT. ben sets RCC_STAGE_HINT=research env to tag a
// general-purpose run as research when applicable.

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

// Length of the trailing slice scanned first. Verdicts are conventionally
// emitted at the end of sub-agent responses, so a tail-first pass avoids
// false matches in earlier prose / quoted examples and runs cheaply on
// large outputs.
const TAIL_SCAN_BYTES = 8 * 1024;

// Defect B1 (2026-05-13) — extract sub-agent response across multiple
// CC payload shapes. Modern CC Task/Agent returns content as array of
// blocks; older builds use string output. Exported for unit testing.
function extractAgentOutput(input) {
  const tr = (input && input.tool_response) || (input && input.tool_output) || {};

  // 1. Plain string output (older CC, non-Task tools)
  if (typeof tr.output === 'string' && tr.output.length > 0) return tr.output;

  // 2. Plain string content (older CC variant)
  if (typeof tr.content === 'string' && tr.content.length > 0) return tr.content;

  // 3. Array of content blocks (modern CC Task/Agent tool)
  if (Array.isArray(tr.content)) {
    const parts = [];
    for (const block of tr.content) {
      if (typeof block === 'string') {
        parts.push(block);
      } else if (block && typeof block.text === 'string') {
        parts.push(block.text);
      } else if (block && typeof block.content === 'string') {
        parts.push(block.content);
      } else if (block && Array.isArray(block.content)) {
        // Nested content (rare; defensive)
        for (const inner of block.content) {
          if (typeof inner === 'string') parts.push(inner);
          else if (inner && typeof inner.text === 'string') parts.push(inner.text);
        }
      }
    }
    return parts.join('\n');
  }

  // 4. Direct text field (some non-Task tools)
  if (typeof tr.text === 'string' && tr.text.length > 0) return tr.text;

  // 5. Legacy tool_output.output (already merged into tr above via fallback)
  return '';
}

// Export for unit tests; module.exports works under CommonJS even when
// the rest of the file runs as a script.
if (typeof module !== 'undefined' && module.exports) {
  module.exports.extractAgentOutput = extractAgentOutput;
}

// Strip fenced code blocks so a `VERDICT: PASS` inside an example block
// (sub-agent showing what a passing response would look like) does not
// poison verdict detection. Both ``` and ~~~ fences are handled.
function stripFencedBlocks(text) {
  return text
    .replace(/```[\s\S]*?```/g, ' ')
    .replace(/~~~[\s\S]*?~~~/g, ' ');
}

function scanVerdict(text) {
  // Explicit "VERDICT: X" wins (canonical form documented in agents/ben.md)
  const explicit = text.match(/\bVERDICT:\s*(PASS|REVISE|BLOCK)\b/i);
  if (explicit) return explicit[1].toUpperCase();

  // RCC convention forms used in ben/researcher/architect agents.
  // BLOCK takes precedence over REVISE/PASS so a "ABORT_RECOMMENDED but
  // here is what would pass" response is recorded as BLOCK.
  if (/\bABORT[_\s]RECOMMENDED\b/i.test(text)) return 'BLOCK';
  if (/\bRULE\s*16\b/i.test(text)) return 'BLOCK';
  if (/\bEMPIRICALLY[_\s]FALSIFIED\b/i.test(text)) return 'BLOCK';
  if (/\bcatastrophic\s+FAIL\b/i.test(text)) return 'BLOCK';
  if (/\bNEEDS[_\s]FURTHER[_\s]REVISION\b/i.test(text)) return 'REVISE';
  if (/\bNEEDS[_\s]REVISION\b/i.test(text)) return 'REVISE';
  if (/\bPASS[_\s]WITH[_\s]REVISIONS?\b/i.test(text)) return 'REVISE';
  if (/\bCONDITIONAL[_\s]APPROVE\b/i.test(text)) return 'REVISE';
  if (/\bREADY[_\s]FOR[_\s](?:CODEX[_\s]REVIEW|ARCHITECT|BUILD|EVAL)\b/i.test(text)) return 'PASS';

  // Look for stand-alone PASS/REVISE/BLOCK in headers (## PASS, # BLOCK)
  if (/^\s*#{1,3}\s*BLOCK/im.test(text)) return 'BLOCK';
  if (/^\s*#{1,3}\s*(?:ABORT|FAIL)\b/im.test(text)) return 'BLOCK';
  if (/^\s*#{1,3}\s*REVISE/im.test(text)) return 'REVISE';
  if (/^\s*#{1,3}\s*(PASS|APPROVED|OK)\b/im.test(text)) return 'PASS';

  // Heuristics
  if (/\bcritical issue\b|\bblock(?:ing|ed)?\b/i.test(text)) return 'BLOCK';
  if (/\bneeds revision\b|\brevise\b/i.test(text)) return 'REVISE';

  return 'PENDING';
}

function detectVerdict(output) {
  return detectVerdictDetailed(output).verdict;
}

// Same logic as detectVerdict but also reports which window matched.
// Used so the entry can carry detection_source: "tail"|"body"|"none",
// letting ben/user diagnose why a PENDING was inferred and reach for
// the amend CLI when the auto-detection guessed wrong.
function detectVerdictDetailed(output) {
  const raw = String(output || '');
  const stripped = stripFencedBlocks(raw);

  if (stripped.length > TAIL_SCAN_BYTES) {
    const tail = stripped.slice(-TAIL_SCAN_BYTES);
    const v = scanVerdict(tail);
    if (v !== 'PENDING') return { verdict: v, source: 'tail' };
  }
  const verdict = scanVerdict(stripped);
  return { verdict, source: verdict === 'PENDING' ? 'none' : 'body' };
}

function extractOutputRef(output) {
  // Look for a path-like artifact pointer (e.g. "Wrote docs/research/...")
  const m = String(output || '').match(/(?:docs\/[^\s]+\.(md|tex)|paper\/[^\s]+\.(md|tex))/);
  return m ? m[0] : '';
}

// Detect whether an adversarial reviewer (codex-reviewer / paper-reviewer)
// performed its own review instead of forwarding to the independent model.
// Sprint 13 R0.6→R0.8 had 2/4 such Defect B3 false-fallbacks; R0.9 R2
// recurrence (2026-05-14) prompted this hook-side detection.
//
// Returns "weakened" | null. When "weakened", ben.md case 0 weakened-PASS
// guard takes over (reject the PASS, require redo or fallback).
function detectReviewIntegrity(output) {
  const text = String(output || '');
  // These phrases appear ONLY when the reviewer is admitting it did
  // not invoke the external model. They are the literal signals seen
  // in the R0.9 R2 false-fallback report.
  const patterns = [
    /\bmodel[\s-]independence\s+(?:partially\s+)?(?:weakened|lost|broken|compromised)\b/i,
    /\bartifact\s+inspection\b/i,
    /\bself[\s-]inspection\b/i,
    /\bself[\s-]review(?:ed)?\b/i,
    /\bsandbox\s+prevented\b/i,
    /\bcodex-cli-aware\s+(?:artifact\s+)?inspection\b/i,
    /\breviewed\s+by\s+(?:claude|the\s+orchestrator)\s+instead\b/i,
  ];
  for (const re of patterns) {
    if (re.test(text)) return 'weakened';
  }
  return null;
}

function countCitations(output) {
  const text = String(output || '');
  const verified = (text.match(/\[verified[^\]]*\]/gi) || []).length;
  const unverified = (text.match(/\[unverified[^\]]*\]/gi) || []).length;
  return { verified, unverified };
}

function extractBottleneck(output) {
  const text = String(output || '');
  const m = text.match(/bottleneck[:\s]+([^\n.]+)/i);
  return m ? m[1].trim().slice(0, 140) : '';
}

function wasTruncated() {
  return (
    process.env.RCC_HOOK_INPUT_TRUNCATED === '1' ||
    process.env.ECC_HOOK_INPUT_TRUNCATED === '1'
  );
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  if (getProfile() === 'minimal') { passthrough(); return; }

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  // CC has emitted both "Task" (early versions) and "Agent" (current)
  // for sub-agent dispatch. Accept both so the hook fires regardless of
  // which CC build the user is on.
  const toolName = input.tool_name || input.toolName || '';
  if (toolName !== 'Task' && toolName !== 'Agent') { passthrough(); return; }

  const subagentType = input.tool_input?.subagent_type || '';
  let stage = STAGE_FOR_SUBAGENT[subagentType];

  // Defect G4 (2026-05-13): for general-purpose agent, only treat as a
  // pipeline stage if ben explicitly set RCC_STAGE_HINT. Otherwise the
  // general-purpose call is bookkeeping/exploration and should not
  // pollute stage_history.
  if (!stage && subagentType === 'general-purpose') {
    const hint = String(process.env.RCC_STAGE_HINT || '').trim().toLowerCase();
    const validStages = ['research', 'codex-review', 'architect', 'build', 'eval'];
    if (validStages.includes(hint)) {
      stage = hint;
    }
  }

  if (!stage) { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  if (!rs.isValidProjectCwd(cwd)) {
    process.stderr.write(
      `[rcc:stage-record] WARNING: refusing to write state — invalid cwd: ${cwd}\n`,
    );
    passthrough();
    return;
  }

  // Read tool_response or tool_output (multiple shapes seen across CC builds).
  //
  // Defect B1 (2026-05-13): modern CC Task/Agent tool returns
  // tool_response.content as an array of content blocks
  // ([{type:"text", text:"..."}, ...]). The prior implementation cast that
  // array with `String([...])` which yields "[object Object]" and broke
  // verdict regex, output-ref extraction, citation counting, and bottleneck
  // extraction (all returned empty / PENDING). Evidence: Sprint 13 R0.6→R0.8
  // session — 7/7 sub-agent invocations recorded PENDING with
  // detection_source="none" despite each agent emitting "VERDICT: PASS"
  // explicitly.
  //
  // extractAgentOutput() handles all observed shapes:
  //   1. tool_response.output (string)           — older CC builds
  //   2. tool_response.content (string)          — older CC builds
  //   3. tool_response.content (array of blocks) — modern CC Task/Agent
  //   4. tool_response.text (string)             — some non-Task tools
  //   5. tool_output.output (string)             — legacy fallback
  const output = extractAgentOutput(input);

  const truncated = wasTruncated();
  const { verdict, source: detectionSource } = detectVerdictDetailed(output);
  const outputRef = extractOutputRef(output);
  const citations = countCitations(output);
  const bottleneck = stage === 'research' ? extractBottleneck(output) : '';
  // Defect B3-recurrence (2026-05-14): detect when an adversarial
  // reviewer self-inspected instead of forwarding to the independent
  // model. Two complementary checks:
  //   1. Text heuristic — sub-agent admits weakening in its response
  //      ("model independence weakened", "artifact inspection", etc.)
  //   2. Evidence check — `.claude/codex-invocations.jsonl` audit log
  //      (written by scripts/run-codex.sh) confirms a real codex CLI
  //      call happened within the past 30 min. If no audit entry, the
  //      reviewer almost certainly bypassed codex.
  // Heuristic wins over evidence (text "weakened" stays "weakened" even
  // if audit log has an entry from an unrelated dispatch).
  let reviewIntegrity = null;
  if (ADVERSARIAL_AGENTS.has(subagentType)) {
    reviewIntegrity = detectReviewIntegrity(output);
    // codex-reviewer specifically REQUIRES audit-log evidence.
    // paper-reviewer doesn't use run-codex.sh, so evidence check
    // doesn't apply (its independence comes from paper-only context
    // isolation, not Codex invocation).
    if (!reviewIntegrity && subagentType === 'codex-reviewer') {
      reviewIntegrity = classifyReviewEvidence(cwd);
    }
  }

  if (truncated) {
    process.stderr.write(
      `[stage-record] WARNING: hook input was truncated; verdict may be ` +
      `incomplete. Detected=${verdict}. Consider raising the stage-record ` +
      `maxStdin argument in hooks.json.\n`,
    );
  }

  // Defect B4 (2026-05-13): extract CC's tool_use_id so the
  // stage_history entry can be cross-linked back to the CC dispatch
  // (session logs, parent message, etc.). Sprint 13 R0.6→R0.8 evidence:
  // 7 dispatches → 7 stage_history entries → 0 cc_tool_use_id set, so
  // there was no way to correlate state with CC's dispatch records.
  // CC PostToolUse passes the dispatch identifier at the top level
  // (typical: `tool_use_id`), sometimes under different keys across
  // builds.
  const ccToolUseId =
    String(input.tool_use_id || '') ||
    String(input.toolUseId || '') ||
    String(input.tool_input?.tool_use_id || '') ||
    String(input.tool_response?.tool_use_id || '');

  try {
    rs.withStateLock(cwd, () => {
      const state = rs.ensureState(cwd);
      const entry = {
        stage,
        agent: subagentType,
        verdict,
        detection_source: detectionSource,
      };
      if (outputRef) entry.output_ref = outputRef;
      if (citations.verified > 0) entry.citations_verified = citations.verified;
      if (citations.unverified > 0) entry.citations_unverified = citations.unverified;
      if (bottleneck) entry.bottleneck = bottleneck;
      if (truncated) entry.input_truncated = true;
      if (ccToolUseId) entry.cc_tool_use_id = ccToolUseId;
      if (reviewIntegrity) entry.review_integrity = reviewIntegrity;

      rs.recordStage(state, entry);
      rs.writeState(cwd, state);
    });

    // Single-line structured trace for grep-ability across CC log files.
    // Includes the detection source so ben/user knows whether the verdict
    // came from the tail (preferred) or the full body, and can reach for
    // `rcc-state-amend` when the auto-detect missed.
    const refPart = outputRef ? ` ref=${outputRef}` : '';
    const idPart = ccToolUseId ? ` cc_tool_use_id=${ccToolUseId.slice(0, 16)}` : '';
    const amendHint = verdict === 'PENDING'
      ? ' — amend via `node scripts/rcc-state-amend.js <PASS|REVISE|BLOCK> [reason]`'
      : '';
    // Weakened-PASS surface: print a loud second line so ben's
    // weakened-PASS guard (ben.md case 0) cannot miss the signal.
    let integrityWarn = '';
    if (reviewIntegrity === 'weakened') {
      integrityWarn =
        `\n[rcc:stage-record] WEAKENED-PASS DETECTED on ${stage}: ` +
        `review_integrity=weakened (text heuristic). ` +
        `ben.md case 0 weakened-PASS guard requires invalidate + redo.`;
    } else if (reviewIntegrity === 'unverified') {
      integrityWarn =
        `\n[rcc:stage-record] UNVERIFIED REVIEW on ${stage}: ` +
        `review_integrity=unverified (no .claude/codex-invocations.jsonl ` +
        `entry in last 30 min — codex CLI likely was NOT invoked). ` +
        `ben.md case 0 weakened-PASS guard requires invalidate + redo via run-codex.sh.`;
    }
    process.stderr.write(
      `[rcc:stage-record] stage=${stage} agent=${subagentType} ` +
      `verdict=${verdict} source=${detectionSource}${refPart}${idPart}${amendHint}${integrityWarn}\n`,
    );
  } catch (e) {
    process.stderr.write(`[rcc:stage-record] error: ${e.message}\n`);
  }

  passthrough();
}

main().catch(() => process.exit(0));
