#!/usr/bin/env node
'use strict';

const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');
const { writeArtifactManifest } = require('./artifact-manifest');
const { runSsh, validateHostEntry } = require('./probe-resource');

function shellQuote(value) {
  return `'${String(value).replace(/'/g, `'\\''`)}'`;
}

function parseArgs(argv) {
  const options = {
    host: '',
    ssh: '',
    remotePath: '',
    outDir: '',
    timeoutSec: 60,
    json: false,
  };
  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === '--host') {
      options.host = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--ssh') {
      options.ssh = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--remote-path') {
      options.remotePath = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--out') {
      options.outDir = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--timeout') {
      options.timeoutSec = Number(argv[index + 1] || 60);
      index += 1;
    } else if (arg === '--json') {
      options.json = true;
    } else if (arg === '--help' || arg === '-h') {
      options.help = true;
    } else {
      throw new Error(`Unknown argument: ${arg}`);
    }
  }
  return options;
}

function ensureValidOptions(options) {
  if (!options.host || !options.ssh) throw new Error('--host and --ssh are required');
  if (!options.remotePath) throw new Error('--remote-path is required');
  if (!options.outDir) throw new Error('--out is required');
  if (!Number.isFinite(options.timeoutSec) || options.timeoutSec <= 0) {
    throw new Error('--timeout must be a positive number of seconds');
  }
}

function collectArtifacts(options) {
  ensureValidOptions(options);
  const host = validateHostEntry({
    id: options.host,
    ssh: options.ssh,
    workdir: '~/rcc-runs',
  });
  const outDir = path.resolve(options.outDir);
  fs.mkdirSync(outDir, { recursive: true });

  const remoteCommand = [
    'set -eu',
    `if [ -d ${shellQuote(options.remotePath)} ]; then`,
    `  tar -cf - -C ${shellQuote(options.remotePath)} .`,
    'else',
    '  tar -cf - -T /dev/null',
    'fi',
  ].join('\n');

  const remote = runSsh(host, remoteCommand, {
    timeoutSec: options.timeoutSec,
    encoding: 'buffer',
  });
  if (remote.status !== 0) {
    throw new Error(`remote artifact collection failed: ${String(remote.stderr || remote.stdout || '').trim()}`);
  }

  const extract = spawnSync('tar', ['-xf', '-', '-C', outDir], {
    input: remote.stdout,
    encoding: 'buffer',
    stdio: ['pipe', 'pipe', 'pipe'],
    timeout: options.timeoutSec * 1000,
  });
  if (extract.status !== 0) {
    throw new Error(`local artifact extraction failed: ${String(extract.stderr || '').trim()}`);
  }

  const { manifest, outPath } = writeArtifactManifest(outDir);
  return {
    out_dir: outDir,
    manifest_path: outPath,
    file_count: manifest.file_count,
  };
}

function usage() {
  console.log(`Usage:
  collect-artifacts --host <id> --ssh <alias> --remote-path <path> --out <dir> [--json]
`);
}

function main(argv = process.argv.slice(2)) {
  const options = parseArgs(argv);
  if (options.help) {
    usage();
    return 0;
  }
  const result = collectArtifacts(options);
  if (options.json) {
    console.log(JSON.stringify(result, null, 2));
  } else {
    console.log(`Collected ${result.file_count} artifact(s) into ${result.out_dir}`);
  }
  return 0;
}

if (require.main === module) {
  try {
    process.exit(main());
  } catch (error) {
    console.error(`collect-artifacts: ${error.message}`);
    process.exit(1);
  }
}

module.exports = {
  collectArtifacts,
  main,
  parseArgs,
  shellQuote,
};
