#!/usr/bin/env node
'use strict';

const path = require('path');
const { runRemoteEval, shellQuote } = require('./remote-eval-run');
const { splitCsv } = require('./probe-resource');

function parseArgs(argv) {
  const options = {
    cwd: process.cwd(),
    configPath: '',
    registryPath: '',
    stateDir: '',
    host: 'auto',
    label: '',
    image: '',
    configure: '',
    build: '',
    test: '',
    eval: '',
    timeoutSec: null,
    requires: ['docker'],
    json: false,
    dryRun: false,
    noCache: false,
    keepRemote: false,
  };

  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === '--cwd') {
      options.cwd = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--config') {
      options.configPath = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--registry') {
      options.registryPath = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--state-dir') {
      options.stateDir = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--host') {
      options.host = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--label') {
      options.label = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--image') {
      options.image = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--configure') {
      options.configure = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--build') {
      options.build = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--test') {
      options.test = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--eval') {
      options.eval = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--timeout') {
      options.timeoutSec = Number(argv[index + 1] || 0);
      index += 1;
    } else if (arg === '--requires') {
      options.requires = splitCsv(argv[index + 1] || '');
      index += 1;
    } else if (arg === '--json') {
      options.json = true;
    } else if (arg === '--dry-run') {
      options.dryRun = true;
    } else if (arg === '--no-cache') {
      options.noCache = true;
    } else if (arg === '--keep-remote') {
      options.keepRemote = true;
    } else if (arg === '--help' || arg === '-h') {
      options.help = true;
    } else {
      throw new Error(`Unknown argument: ${arg}`);
    }
  }
  return options;
}

function usage() {
  console.log(`Usage:
  remote-cpp-eval --label <label> --image <image>
                  [--configure <cmd>] [--build <cmd>] [--test <cmd>] [--eval <cmd>]
                  [--host auto|<id>] [--requires docker,gpu] [--timeout <sec>]
                  [--registry <path>] [--state-dir <path>]
                  [--json] [--dry-run] [--no-cache]
`);
}

function validateCppOptions(options) {
  if (!options.configure && !options.build && !options.test && !options.eval) {
    throw new Error('At least one of --configure, --build, --test, or --eval is required');
  }
}

function appendStep(lines, outVar, name, command) {
  if (!command) return;
  const logFile = `${name}.log`;
  lines.push(`printf "%s\\n" ${shellQuote(`$ ${command}`)} > "$${outVar}/${logFile}"`);
  lines.push(`sh -lc ${shellQuote(command)} >> "$${outVar}/${logFile}" 2>&1`);
}

function buildCppCommand(options) {
  const outVar = 'rcc_out';
  const label = options.label;
  const lines = [
    'set -eu',
    `${outVar}=${shellQuote(path.posix.join('dump', label))}`,
    `mkdir -p "$${outVar}"`,
  ];
  appendStep(lines, outVar, 'configure', options.configure);
  appendStep(lines, outVar, 'build', options.build);
  appendStep(lines, outVar, 'test', options.test);
  appendStep(lines, outVar, 'eval', options.eval);
  return lines.join('\n');
}

function runCppEval(options) {
  validateCppOptions(options);
  const command = buildCppCommand(options);
  return runRemoteEval({
    cwd: options.cwd,
    configPath: options.configPath,
    registryPath: options.registryPath,
    stateDir: options.stateDir,
    host: options.host,
    label: options.label,
    image: options.image,
    command,
    timeoutSec: options.timeoutSec,
    requires: options.requires,
    json: options.json,
    dryRun: options.dryRun,
    noCache: options.noCache,
    keepRemote: options.keepRemote,
  });
}

function main(argv = process.argv.slice(2)) {
  const options = parseArgs(argv);
  if (options.help) {
    usage();
    return 0;
  }
  const result = runCppEval(options);
  if (options.json) {
    console.log(JSON.stringify(result, null, 2));
  } else if (result.status === 'unavailable') {
    console.log('remote-cpp-eval: no matching remote host available');
  } else if (result.dry_run) {
    console.log(`remote-cpp-eval dry-run: ${result.selected_host} ${result.run_id}`);
  } else {
    console.log(`remote-cpp-eval ${result.status}: ${result.run.run_id}`);
  }
  if (result.status === 'unavailable') return 1;
  if (result.run && result.run.exit_code !== 0) return result.run.exit_code || 1;
  return 0;
}

if (require.main === module) {
  try {
    process.exit(main());
  } catch (error) {
    console.error(`remote-cpp-eval: ${error.message}`);
    process.exit(1);
  }
}

module.exports = {
  appendStep,
  buildCppCommand,
  main,
  parseArgs,
  runCppEval,
  validateCppOptions,
};
