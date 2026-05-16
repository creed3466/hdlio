#!/usr/bin/env node
'use strict';

/**
 * pre:write:architect-scope (Defect G1, 2026-05-13)
 *
 * Trigger: PreToolUse on Write or Edit, when the invoking sub-agent is
 * `architect` (detected via env hint or stdin payload).
 *
 * Purpose: architect agent has Write+Edit capability (granted per G1)
 * but writes are restricted to architecture artifact paths. Anything
 * outside the allow-list is blocked (strict) or warned (standard).
 *
 * Allowed paths (glob-equivalent):
 *   - docs/specs/**\/*.md
 *   - docs/research/sprint*_architecture*.md
 *   - docs/research/*architecture*.md
 *   - docs/research/*architect*.md
 *   - docs/research/*design*.md
 *   - docs/adr/**\/*.md
 *
 * Profile gating:
 *   minimal  → no-op
 *   standard → warn (exit 0)
 *   strict   → block (exit 2)
 */

const fs = require('fs');
const path = require('path');

const MAX_STDIN = 256 * 1024;

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

// Path patterns where architect IS allowed to write
const ALLOWED_PATTERNS = [
  /^docs\/specs\/.+\.md$/i,
  /^docs\/research\/sprint[^/]*architect[^/]*\.md$/i,
  /^docs\/research\/[^/]*architecture[^/]*\.md$/i,
  /^docs\/research\/[^/]*architect[^/]*\.md$/i,
  /^docs\/research\/[^/]*design[^/]*\.md$/i,
  /^docs\/adr\/.+\.md$/i,
];

function isAllowedPath(filePath) {
  // Normalize: strip leading ./ and any absolute-path prefix beyond cwd
  let p = String(filePath || '').replace(/^\.\//, '');
  // If absolute, take only the tail relative to project root (best-effort)
  const cwd = process.env.CLAUDE_PROJECT_DIR || process.cwd();
  if (p.startsWith(cwd)) {
    p = p.slice(cwd.length).replace(/^\/+/, '');
  }
  return ALLOWED_PATTERNS.some(pat => pat.test(p));
}

function isArchitectInvocation(input) {
  // Architect can be detected several ways:
  // 1. Explicit env hint set by Ben before dispatching
  if (process.env.RCC_INVOKING_AGENT === 'architect') return true;
  // 2. CC may pass agent type in metadata (defensive — schema varies)
  if (input && input.agent_type === 'architect') return true;
  if (input && input.subagent_type === 'architect') return true;
  // 3. Falls through — assume NOT architect (no enforcement on non-arch)
  return false;
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  if (getProfile() === 'minimal') { passthrough(); return; }

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  // Only gate Write/Edit tools
  const toolName = input.tool_name || input.toolName || '';
  if (toolName !== 'Write' && toolName !== 'Edit' && toolName !== 'MultiEdit') {
    passthrough();
    return;
  }

  // Only applies when architect is the invoking agent
  if (!isArchitectInvocation(input)) {
    passthrough();
    return;
  }

  const filePath =
    input.tool_input?.file_path ||
    input.tool_input?.path ||
    input.toolInput?.file_path ||
    '';

  if (!filePath) {
    passthrough();
    return;
  }

  if (isAllowedPath(filePath)) {
    passthrough();
    return;
  }

  // Path is outside architect's allowed scope
  const profile = getProfile();
  const msg =
    `[architect-write-scope] ${profile === 'strict' ? 'BLOCKED' : 'WARN'}: ` +
    `architect agent attempted to write outside allowed paths.\n` +
    `  path: ${filePath}\n` +
    `  allowed: docs/specs/, docs/research/*architect*.md, ` +
    `docs/research/*design*.md, docs/adr/\n`;

  if (profile === 'strict') {
    process.stderr.write(msg);
    process.exit(2);
  } else {
    process.stderr.write(msg);
    passthrough();
  }
}

if (require.main === module) {
  main().catch(err => {
    process.stderr.write(`[architect-write-scope] ERROR: ${err.message}\n`);
    process.exit(0); // fail-open
  });
}

// Export for unit tests
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { isAllowedPath, ALLOWED_PATTERNS };
}
