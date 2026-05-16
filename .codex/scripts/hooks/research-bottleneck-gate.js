#!/usr/bin/env node
'use strict';

/**
 * pre:write:research-bottleneck-gate (RCC research hook, STRICT ONLY)
 *
 * PreToolUse Write hook on docs/research/**.md files. Verifies the artifact
 * follows the researcher.md output contract:
 *   - Has "## Mathematical Analysis" section
 *   - Has "## Hypothesis" section
 *   - Has "## Theoretical Justification" section
 *   - Mathematical Analysis mentions a *named* bottleneck
 *
 * Behaviour:
 *   - profile=strict only → warn (does NOT block, even strict)
 *   - profile=standard or minimal → no-op
 *   - No `.claude/research-config.json` or no research_paths match → no-op
 *
 * Rationale: research drafts are iterative. We don't want to block early
 * drafts that are still WIP. We just remind that the artifact is missing
 * structure when published as a research output.
 */

const fs = require('fs');
const path = require('path');
const { matchesGlob } = require('../lib/research-glob.js');

const MAX_STDIN = 1024 * 1024;
const REQUIRED_SECTIONS = [
  /^##\s+Mathematical Analysis\b/im,
  /^##\s+Hypothesis\b/im,
  /^##\s+Theoretical Justification\b/im,
];
const BOTTLENECK_KEYWORDS = [
  /\bbottleneck\b/i,
  /\brank[- ]?defic/i,
  /\beigenvalue\b/i,
  /\bcondition number\b/i,
  /\bperturbation\b/i,
  /\binformation[- ](loss|theoretic)/i,
  /\bdegenerac/i,
];

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

function loadConfig(projectRoot) {
  const candidates = [
    path.join(projectRoot, '.claude', 'research-config.json'),
    path.join(projectRoot, 'research-config.json'),
  ];
  for (const p of candidates) {
    if (fs.existsSync(p)) {
      try { return JSON.parse(fs.readFileSync(p, 'utf8')); } catch { return null; }
    }
  }
  return null;
}


function getProfile() {
  const raw = String(process.env.RCC_HOOK_PROFILE || process.env.ECC_HOOK_PROFILE || 'standard').trim().toLowerCase();
  return ['minimal', 'standard', 'strict'].includes(raw) ? raw : 'standard';
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  // Strict-only hook
  if (getProfile() !== 'strict') { passthrough(); return; }

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  if ((input.tool_name || input.toolName) !== 'Write') { passthrough(); return; }

  const filePath = input.tool_input?.file_path || input.tool_input?.path;
  if (!filePath || !filePath.endsWith('.md')) { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  const config = loadConfig(cwd);
  if (!config) { passthrough(); return; }

  const researchPaths = config.research_paths || [];
  // Narrow to research artifacts only
  let relPath = filePath;
  if (path.isAbsolute(filePath)) {
    const rel = path.relative(cwd, filePath);
    if (!rel.startsWith('..')) relPath = rel;
  }
  if (!researchPaths.some(g => matchesGlob(relPath, g))) { passthrough(); return; }
  // Further narrow: only docs/research/**/*.md
  if (!/(^|\/)docs\/research\//.test(relPath)) { passthrough(); return; }

  const content = input.tool_input?.content || '';

  const missingSections = REQUIRED_SECTIONS
    .filter(re => !re.test(content))
    .map(re => re.source.replace(/^\^\\#\\#\\s\+/, '## ').replace(/\\b$/, ''));

  const hasBottleneck = BOTTLENECK_KEYWORDS.some(re => re.test(content));

  if (missingSections.length === 0 && hasBottleneck) { passthrough(); return; }

  process.stderr.write('[research-bottleneck-gate] WARNING: research artifact incomplete\n');
  process.stderr.write(`  file: ${relPath}\n`);
  if (missingSections.length > 0) {
    process.stderr.write(`  missing sections:\n`);
    for (const s of missingSections) process.stderr.write(`    - ${s}\n`);
  }
  if (!hasBottleneck) {
    process.stderr.write(`  no named bottleneck keyword found\n`);
    process.stderr.write(`  expected one of: bottleneck, rank-deficien*, eigenvalue,\n`);
    process.stderr.write(`                   condition number, perturbation, information-loss, degeneracy\n`);
  }
  process.stderr.write('  See researcher.md output contract.\n');

  // Strict-mode: warn only (exit 0). Do NOT block — research drafts are iterative.
  passthrough();
}

main().catch(() => {
  process.stdout.write('');
  process.exit(0);
});
