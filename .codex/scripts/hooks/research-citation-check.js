#!/usr/bin/env node
'use strict';

/**
 * pre:write:research-citation-check (RCC research hook)
 *
 * PreToolUse hook on Write/Edit. When the target file matches any of the
 * project's `research_paths`, scans the new content for citation patterns
 * (e.g. "Smith et al.", "[Author 2024]", "arXiv:1234.5678", "30%
 * improvement") and verifies that each citation is followed within ±3 lines
 * by a `[verified: ...]` or `[unverified]` tag.
 *
 * Behaviour:
 *   - Project has no `.claude/research-config.json` → no-op (graceful)
 *   - Target file does not match research_paths → no-op
 *   - Profile=standard → warn (exit 0, stderr message)
 *   - Profile=strict   → block (exit 2)
 *
 * Reads from stdin (Claude Code hook input). Echoes stdin back to stdout
 * unchanged so downstream hooks see the original event.
 */

const fs = require('fs');
const path = require('path');
const { matchesGlob } = require('../lib/research-glob.js');

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

function loadResearchConfig(projectRoot) {
  const candidates = [
    path.join(projectRoot, '.claude', 'research-config.json'),
    path.join(projectRoot, 'research-config.json'),
  ];
  for (const p of candidates) {
    if (fs.existsSync(p)) {
      try {
        return JSON.parse(fs.readFileSync(p, 'utf8'));
      } catch {
        return null;
      }
    }
  }
  return null;
}


function targetMatchesAny(relPath, patterns) {
  if (!Array.isArray(patterns)) return false;
  return patterns.some(p => matchesGlob(relPath, p));
}

// Citation patterns (each indicates a verifiable external claim)
const CITATION_PATTERNS = [
  /\b[A-Z][a-z]+(?:\s+(?:and|&)\s+[A-Z][a-z]+)?\s+et\s+al\.?\b/,    // Smith et al.
  /\[(?:[A-Z][a-zA-Z]+,?\s*)+(?:19|20)\d{2}[a-z]?\]/,                // [Author 2024]
  /\barXiv:\s*\d{4}\.\d{4,5}\b/i,                                    // arXiv:1234.5678
  /\bdoi:\s*10\.\d{4,9}\/[-._;()/:A-Z0-9]+/i,                        // DOI
  /\b\d+(?:\.\d+)?\s*%\s+(?:improvement|reduction|gain|increase|decrease|better|worse)/i, // "30% improvement"
];

const VERIFICATION_TAG = /\[(verified(?::[^\]]*)?|unverified(?::[^\]]*)?)\]/i;

function findUntaggedCitations(content) {
  const lines = content.split('\n');
  const findings = [];

  for (let i = 0; i < lines.length; i++) {
    for (const pat of CITATION_PATTERNS) {
      const m = lines[i].match(pat);
      if (!m) continue;

      // Look for [verified|unverified] within ±3 lines (inclusive)
      let tagged = false;
      for (let j = Math.max(0, i - 3); j <= Math.min(lines.length - 1, i + 3); j++) {
        if (VERIFICATION_TAG.test(lines[j])) { tagged = true; break; }
      }

      if (!tagged) {
        findings.push({
          line: i + 1,
          excerpt: lines[i].trim().substring(0, 100),
          match: m[0],
        });
      }
      break; // one citation per line is enough
    }
  }
  return findings;
}

function getProfile() {
  const raw = String(process.env.RCC_HOOK_PROFILE || process.env.ECC_HOOK_PROFILE || 'standard').trim().toLowerCase();
  return ['minimal', 'standard', 'strict'].includes(raw) ? raw : 'standard';
}

async function main() {
  const raw = await readStdin();

  // Always pass through stdin
  const passthrough = () => process.stdout.write(raw);

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  const toolName = input.tool_name || input.toolName || '';
  if (!['Write', 'Edit', 'MultiEdit'].includes(toolName)) { passthrough(); return; }

  const filePath = input.tool_input?.file_path || input.tool_input?.path;
  if (!filePath) { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  const config = loadResearchConfig(cwd);
  if (!config) { passthrough(); return; }

  const researchPaths = config.research_paths || [];
  if (researchPaths.length === 0) { passthrough(); return; }

  // Convert to project-relative
  let relPath = filePath;
  if (path.isAbsolute(filePath)) {
    const rel = path.relative(cwd, filePath);
    if (!rel.startsWith('..')) relPath = rel;
  }

  if (!targetMatchesAny(relPath, researchPaths)) { passthrough(); return; }

  // Gather content to scan
  let content = '';
  if (toolName === 'Write') {
    content = input.tool_input?.content || '';
  } else if (toolName === 'Edit') {
    content = input.tool_input?.new_string || '';
  } else if (toolName === 'MultiEdit') {
    const edits = input.tool_input?.edits || [];
    content = edits.map(e => e.new_string || '').join('\n');
  }

  const findings = findUntaggedCitations(content);
  if (findings.length === 0) { passthrough(); return; }

  // Emit findings to stderr
  const profile = getProfile();
  const header = profile === 'strict'
    ? '[research-citation-check] BLOCKED: untagged citations'
    : '[research-citation-check] WARNING: untagged citations';
  process.stderr.write(`${header}\n`);
  process.stderr.write(`  file: ${relPath}\n`);
  for (const f of findings.slice(0, 10)) {
    process.stderr.write(`  line ${f.line}: "${f.excerpt}" → match="${f.match}"\n`);
  }
  if (findings.length > 10) {
    process.stderr.write(`  ... +${findings.length - 10} more\n`);
  }
  process.stderr.write('  Add [verified: <source>] or [unverified] within ±3 lines.\n');

  passthrough();
  process.exit(profile === 'strict' ? 2 : 0);
}

main().catch(() => {
  process.stdout.write(raw);
  process.exit(0);
});
