#!/usr/bin/env node
'use strict';

/**
 * post:bash:gpu-profile-snapshot (RCC v2.1 Phase α hook)
 *
 * Trigger: PostToolUse on Bash.
 *
 * When a Bash command appears to launch ML training (matches the same
 * `ml-train` detection patterns as post-bash-build-complete), capture
 * a one-line `nvidia-smi` snapshot to .claude/gpu-snapshots/<ts>.json
 * and append a summary entry to research-state.json.gpu_history.
 *
 * Profile gating:
 *   minimal  → no-op
 *   standard → snapshot + warn if VRAM > 95% (still exit 0)
 *   strict   → snapshot + block follow-up Bash if VRAM > 95% (exit 0
 *              here; eval-regression-check will see the regression)
 *
 * Graceful degradation: no nvidia-smi → no-op. No .claude/ → no-op.
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');
const rs = require('../lib/research-state.js');

const MAX_STDIN = 1024 * 1024;

// Same patterns as post-bash-build-complete ml-train family
const ML_TRAIN_PATTERNS = [
  /\bns-train\b/,
  /\bgaussian-splatting\b/,
  /\bpython\s+(\.\/)?train(_\w+)?\.py\b/,
  /\baccelerate\s+launch\b/,
  /\btorchrun\b/,
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

function getProfile() {
  const raw = String(
    process.env.RCC_HOOK_PROFILE || process.env.ECC_HOOK_PROFILE || 'standard'
  ).trim().toLowerCase();
  return ['minimal', 'standard', 'strict'].includes(raw) ? raw : 'standard';
}

function isMlTrain(cmd) {
  return ML_TRAIN_PATTERNS.some(re => re.test(cmd));
}

function tryNvidiaSmi() {
  try {
    const out = execSync(
      'nvidia-smi --query-gpu=index,utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits',
      { encoding: 'utf8', timeout: 4000 }
    );
    return out.trim().split('\n').map(line => {
      const [index, util, used, total, temp] = line.split(',').map(s => s.trim());
      return {
        index: Number(index),
        gpu_util_pct: Number(util),
        vram_used_mib: Number(used),
        vram_total_mib: Number(total),
        vram_used_pct: total ? (Number(used) / Number(total)) * 100 : null,
        temperature_c: Number(temp),
      };
    });
  } catch {
    return null;
  }
}

async function main() {
  const raw = await readStdin();
  const passthrough = () => process.stdout.write(raw);

  if (getProfile() === 'minimal') { passthrough(); return; }

  let input;
  try { input = JSON.parse(raw); } catch { passthrough(); return; }

  const toolName = input.tool_name || input.toolName || '';
  if (toolName !== 'Bash') { passthrough(); return; }

  const cmd = String(input.tool_input?.command || '');
  if (!isMlTrain(cmd)) { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  if (!rs.isValidProjectCwd(cwd)) {
    process.stderr.write(
      `[rcc:gpu-profile-snapshot] WARNING: invalid project cwd: ${cwd}\n`,
    );
    passthrough();
    return;
  }
  if (!fs.existsSync(path.join(cwd, '.claude'))) { passthrough(); return; }

  const gpus = tryNvidiaSmi();
  if (!gpus) {
    process.stderr.write('[gpu-profile-snapshot] nvidia-smi unavailable; skipping snapshot\n');
    passthrough();
    return;
  }

  // Write snapshot
  try {
    const snapDir = path.join(cwd, '.claude', 'gpu-snapshots');
    fs.mkdirSync(snapDir, { recursive: true });
    const ts = new Date().toISOString().replace(/[:.]/g, '-');
    const snapPath = path.join(snapDir, `${ts}.json`);
    fs.writeFileSync(snapPath, JSON.stringify({ timestamp: rs.nowIso(), command: cmd.slice(0, 200), gpus }, null, 2));

    const maxVramPct = Math.max(...gpus.map(g => g.vram_used_pct || 0));
    const maxUtil = Math.max(...gpus.map(g => g.gpu_util_pct || 0));

    // Append to state under exclusive lock so concurrent stage-record /
    // gpu-snapshot fires don't clobber each other's history.
    try {
      rs.withStateLock(cwd, () => {
        const state = rs.ensureState(cwd);
        rs.appendGpuSnapshot(state, {
          command: cmd.slice(0, 200),
          snapshot_path: path.relative(cwd, snapPath),
          vram_used_pct_max: Number(maxVramPct.toFixed(1)),
          gpu_util_pct_max: maxUtil,
        });
        rs.writeState(cwd, state);
      });
    } catch { /* never crash */ }

    process.stderr.write(
      `[gpu-profile-snapshot] ${gpus.length} GPU(s), max VRAM=${maxVramPct.toFixed(1)}%, max util=${maxUtil}%\n`
    );

    if (maxVramPct > 95) {
      process.stderr.write(`[gpu-profile-snapshot] WARNING: VRAM ${maxVramPct.toFixed(1)}% — OOM risk imminent\n`);
    }
  } catch (e) {
    process.stderr.write(`[gpu-profile-snapshot] error: ${e.message}\n`);
  }

  passthrough();
}

main().catch(() => process.exit(0));
