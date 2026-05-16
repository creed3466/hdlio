'use strict';

/**
 * S2 (2026-05-14): Evidence-based verification that codex-reviewer
 * actually invoked the Codex CLI (gpt-5.5) instead of self-inspecting.
 *
 * Defect B3 + B3-recurrence: codex-reviewer sub-agent has self-
 * inspected on multiple occasions despite its agent prompt explicitly
 * forbidding it. Text-based heuristic (detectReviewIntegrity) catches
 * the cases where the sub-agent admits weakening; this lib catches the
 * cases where the sub-agent silently fakes by checking the audit log
 * `run-codex.sh` writes on every successful invocation.
 *
 * Source of truth: `.claude/codex-invocations.jsonl` — append-only
 * JSONL written by scripts/run-codex.sh after every exit-0 codex exec.
 * Each line:
 *   {
 *     "timestamp": "2026-05-14T...",
 *     "codex_version": "codex-cli 0.130.0",
 *     "effort": "xhigh",
 *     "prompt_path": "/tmp/codex_<round>/prompt.txt",
 *     "prompt_bytes": 1234,
 *     "output_path": "/tmp/codex_<round>/review.md",
 *     "output_bytes": 5678,
 *     "output_sha256": "...",
 *     "exit_code": 0
 *   }
 *
 * Note: this lib reads the JSONL but never trusts it for correctness
 * of the review itself — it only verifies the FACT of invocation.
 * Combined with detectReviewIntegrity's text patterns, it makes
 * self-inspection silently-faked responses very hard to slip through.
 */

const fs = require('fs');
const path = require('path');

const AUDIT_LOG_RELPATH = path.join('.claude', 'codex-invocations.jsonl');

// Default window: 30 minutes is roughly the longest a codex review
// dispatch takes (large prompt + xhigh + sandbox startup). Most
// reviews complete in seconds. 30m is a generous outer bound that
// still excludes "I ran codex last sprint" cases.
const DEFAULT_WINDOW_MS = 30 * 60 * 1000;

function resolveAuditLog(cwd) {
  return path.join(cwd || process.cwd(), AUDIT_LOG_RELPATH);
}

// Tail the last N bytes of the audit log and parse JSONL.
// JSONL is small (~200 bytes per entry, ≤2 codex invocations per
// codex-review round, ~10 rounds per sprint → ~4 KB per sprint).
function readRecentEntries(cwd, maxBytes = 256 * 1024) {
  const p = resolveAuditLog(cwd);
  if (!fs.existsSync(p)) return [];
  let body;
  try {
    const stat = fs.statSync(p);
    const fd = fs.openSync(p, 'r');
    try {
      const readSize = Math.min(stat.size, maxBytes);
      const buf = Buffer.alloc(readSize);
      fs.readSync(fd, buf, 0, readSize, Math.max(0, stat.size - readSize));
      body = buf.toString('utf8');
      if (stat.size > maxBytes) {
        // drop possibly-partial first line
        const nl = body.indexOf('\n');
        if (nl !== -1) body = body.slice(nl + 1);
      }
    } finally {
      try { fs.closeSync(fd); } catch { /* ignore */ }
    }
  } catch {
    return [];
  }
  const out = [];
  for (const line of body.split('\n')) {
    if (!line.trim()) continue;
    try {
      out.push(JSON.parse(line));
    } catch {
      // malformed line — skip
    }
  }
  return out;
}

// Find the most recent successful codex invocation within `windowMs`
// of now. Returns null if none found.
function findRecentCodexInvocation(cwd, options = {}) {
  const windowMs = options.windowMs || DEFAULT_WINDOW_MS;
  const entries = readRecentEntries(cwd);
  if (entries.length === 0) return null;
  const cutoff = Date.now() - windowMs;
  let best = null;
  for (const e of entries) {
    if (e.exit_code !== 0) continue;
    const t = Date.parse(e.timestamp || '');
    if (!Number.isFinite(t) || t < cutoff) continue;
    if (!best || t > Date.parse(best.timestamp)) best = e;
  }
  return best;
}

// Decide whether a codex-reviewer response should be flagged as
// "unverified". Returns:
//   - null              → has evidence, OK
//   - "unverified"      → no recent codex invocation in audit log
//
// Used by stage-record.js after detectReviewIntegrity. If text
// heuristic already returned "weakened", that wins (more specific).
// If text heuristic returned null, evidence check decides.
function classifyReviewEvidence(cwd, options = {}) {
  const recent = findRecentCodexInvocation(cwd, options);
  return recent ? null : 'unverified';
}

module.exports = {
  AUDIT_LOG_RELPATH,
  DEFAULT_WINDOW_MS,
  resolveAuditLog,
  readRecentEntries,
  findRecentCodexInvocation,
  classifyReviewEvidence,
};
