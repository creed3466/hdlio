#!/usr/bin/env node
/**
 * Executes a hook script only when enabled by ECC hook profile flags.
 *
 * Usage:
 *   node run-with-flags.js <hookId> <scriptRelativePath> [profilesCsv]
 */

'use strict';

const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');
const { isHookEnabled } = require('../lib/hook-flags');

const DEFAULT_MAX_STDIN = 1024 * 1024;
// Absolute ceiling — prevents a misconfigured hook entry from pinning
// arbitrary memory. Tunable upward when a future hook genuinely needs more.
const HARD_MAX_STDIN = 16 * 1024 * 1024;

function parseMaxStdin(rawValue) {
  if (rawValue == null || rawValue === '') return DEFAULT_MAX_STDIN;
  const n = Number(rawValue);
  if (!Number.isFinite(n) || n <= 0) return DEFAULT_MAX_STDIN;
  return Math.min(Math.floor(n), HARD_MAX_STDIN);
}

function readStdinRaw(maxBytes) {
  return new Promise(resolve => {
    let raw = '';
    let truncated = false;
    process.stdin.setEncoding('utf8');
    process.stdin.on('data', chunk => {
      if (raw.length < maxBytes) {
        const remaining = maxBytes - raw.length;
        raw += chunk.substring(0, remaining);
        if (chunk.length > remaining) {
          truncated = true;
        }
      } else {
        truncated = true;
      }
    });
    process.stdin.on('end', () => resolve({ raw, truncated }));
    process.stdin.on('error', () => resolve({ raw, truncated }));
  });
}

function writeStderr(stderr) {
  if (typeof stderr !== 'string' || stderr.length === 0) {
    return;
  }

  process.stderr.write(stderr.endsWith('\n') ? stderr : `${stderr}\n`);
}

function emitHookResult(raw, output) {
  if (typeof output === 'string' || Buffer.isBuffer(output)) {
    process.stdout.write(String(output));
    return 0;
  }

  if (output && typeof output === 'object') {
    writeStderr(output.stderr);

    if (Object.prototype.hasOwnProperty.call(output, 'stdout')) {
      process.stdout.write(String(output.stdout ?? ''));
    } else if (!Number.isInteger(output.exitCode) || output.exitCode === 0) {
      process.stdout.write(raw);
    }

    return Number.isInteger(output.exitCode) ? output.exitCode : 0;
  }

  process.stdout.write(raw);
  return 0;
}

function writeLegacySpawnOutput(raw, result) {
  const stdout = typeof result.stdout === 'string' ? result.stdout : '';
  if (stdout) {
    process.stdout.write(stdout);
    return;
  }

  if (Number.isInteger(result.status) && result.status === 0) {
    process.stdout.write(raw);
  }
}

function getPluginRoot() {
  if (process.env.CLAUDE_PLUGIN_ROOT && process.env.CLAUDE_PLUGIN_ROOT.trim()) {
    return process.env.CLAUDE_PLUGIN_ROOT;
  }
  return path.resolve(__dirname, '..', '..');
}

async function main() {
  const [, , hookId, relScriptPath, profilesCsv, maxStdinArg] = process.argv;
  const maxBytes = parseMaxStdin(maxStdinArg);
  const { raw, truncated } = await readStdinRaw(maxBytes);

  if (!hookId || !relScriptPath) {
    process.stdout.write(raw);
    process.exit(0);
  }

  if (!isHookEnabled(hookId, { profiles: profilesCsv })) {
    process.stdout.write(raw);
    process.exit(0);
  }

  const pluginRoot = getPluginRoot();
  const resolvedRoot = path.resolve(pluginRoot);
  const scriptPath = path.resolve(pluginRoot, relScriptPath);

  // Prevent path traversal outside the plugin root
  if (!scriptPath.startsWith(resolvedRoot + path.sep)) {
    process.stderr.write(`[Hook] Path traversal rejected for ${hookId}: ${scriptPath}\n`);
    process.stdout.write(raw);
    process.exit(0);
  }

  if (!fs.existsSync(scriptPath)) {
    process.stderr.write(`[Hook] Script not found for ${hookId}: ${scriptPath}\n`);
    process.stdout.write(raw);
    process.exit(0);
  }

  // Prefer direct require() when the hook exports a run(rawInput) function.
  // This eliminates one Node.js process spawn (~50-100ms savings per hook).
  //
  // SAFETY: Only require() hooks that export run(). Legacy hooks execute
  // side effects at module scope (stdin listeners, process.exit, main() calls)
  // which would interfere with the parent process or cause double execution.
  let hookModule;
  const src = fs.readFileSync(scriptPath, 'utf8');
  const hasRunExport = /\bmodule\.exports\b/.test(src) && /\brun\b/.test(src);

  if (hasRunExport) {
    try {
      hookModule = require(scriptPath);
    } catch (requireErr) {
      process.stderr.write(`[Hook] require() failed for ${hookId}: ${requireErr.message}\n`);
      // Fall through to legacy spawnSync path
    }
  }

  if (hookModule && typeof hookModule.run === 'function') {
    try {
      const output = hookModule.run(raw, { truncated, maxStdin: maxBytes });
      process.exit(emitHookResult(raw, output));
    } catch (runErr) {
      process.stderr.write(`[Hook] run() error for ${hookId}: ${runErr.message}\n`);
      process.stdout.write(raw);
    }
    process.exit(0);
  }

  // Legacy path: spawn a child Node process for hooks without run() export
  const result = spawnSync(process.execPath, [scriptPath], {
    input: raw,
    encoding: 'utf8',
    env: {
      ...process.env,
      ECC_HOOK_INPUT_TRUNCATED: truncated ? '1' : '0',
      ECC_HOOK_INPUT_MAX_BYTES: String(maxBytes),
      RCC_HOOK_INPUT_TRUNCATED: truncated ? '1' : '0',
      RCC_HOOK_INPUT_MAX_BYTES: String(maxBytes)
    },
    cwd: process.cwd(),
    timeout: 30000
  });

  writeLegacySpawnOutput(raw, result);
  if (result.stderr) process.stderr.write(result.stderr);

  if (result.error || result.signal || result.status === null) {
    const failureDetail = result.error
      ? result.error.message
      : result.signal
        ? `terminated by signal ${result.signal}`
        : 'missing exit status';
    writeStderr(`[Hook] legacy hook execution failed for ${hookId}: ${failureDetail}`);
    process.exit(1);
  }

  process.exit(Number.isInteger(result.status) ? result.status : 0);
}

main().catch(err => {
  process.stderr.write(`[Hook] run-with-flags error: ${err.message}\n`);
  process.exit(0);
});
