#!/usr/bin/env node
'use strict';

/**
 * post:bash:sweep-launch-detect (RCC v2.1 Phase α hook)
 *
 * Trigger: PostToolUse on Bash.
 *
 * Detect HPO sweep launches (Optuna, Ray Tune, W&B Sweeps) and emit a
 * structured stderr hint that ben/Claude should use ScheduleWakeup to
 * auto-poll. We don't (and can't) invoke ScheduleWakeup ourselves — that
 * is a tool the LLM uses inside the session. This hook surfaces the
 * intent so the assistant knows to schedule.
 *
 * Profile gating:
 *   minimal  → no-op
 *   standard → emit hint to stderr (assistant decides)
 *   strict   → emit hint + write `.claude/.rcc-sweep-pending.json` so
 *              `/loop` can pick it up
 */

const fs = require('fs');
const path = require('path');
const rs = require('../lib/research-state.js');

const MAX_STDIN = 1024 * 1024;

const SWEEP_PATTERNS = [
  { framework: 'optuna',   re: /\boptuna\s+(create-study|study\s+optimize)\b/ },
  { framework: 'optuna',   re: /\bpython\s+.*study\.optimize\(/ },
  { framework: 'wandb',    re: /\bwandb\s+sweep\b/ },
  { framework: 'wandb',    re: /\bwandb\s+agent\b/ },
  { framework: 'ray',      re: /\bray\.tune\.run\b/ },
  { framework: 'ray',      re: /\bpython\s+.*Tuner\(.*\)\.fit\(\)/ },
];

const SUGGESTED_POLL_SECONDS = 1800;

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

function detectSweep(cmd) {
  for (const { framework, re } of SWEEP_PATTERNS) {
    if (re.test(cmd)) return framework;
  }
  return null;
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  const profile = getProfile();
  if (profile === 'minimal') { passthrough(); return; }

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  const toolName = input.tool_name || input.toolName || '';
  if (toolName !== 'Bash') { passthrough(); return; }

  const cmd = String(input.tool_input?.command || '');
  if (!cmd) { passthrough(); return; }

  const framework = detectSweep(cmd);
  if (!framework) { passthrough(); return; }

  const cwd = input.cwd || process.cwd();

  process.stderr.write(
    `[sweep-launch-detect] ${framework} sweep launch detected — ` +
    `schedule a wakeup poll: ScheduleWakeup(delaySeconds=${SUGGESTED_POLL_SECONDS}, ` +
    `reason="poll ${framework} sweep")\n`
  );

  if (profile === 'strict' && fs.existsSync(path.join(cwd, '.claude'))) {
    // Persist the pending sweep hint so other tooling can act
    try {
      const pending = {
        timestamp: rs.nowIso(),
        framework,
        command: cmd.slice(0, 300),
        suggested_poll_seconds: SUGGESTED_POLL_SECONDS,
      };
      fs.writeFileSync(
        path.join(cwd, '.claude', '.rcc-sweep-pending.json'),
        JSON.stringify(pending, null, 2)
      );
    } catch { /* never crash */ }
  }

  passthrough();
}

main().catch(() => process.exit(0));
