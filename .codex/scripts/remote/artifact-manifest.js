#!/usr/bin/env node
'use strict';

const crypto = require('crypto');
const fs = require('fs');
const path = require('path');

function normalizeRelativePath(value) {
  return String(value || '').replace(/\\/g, '/').replace(/^\.\/+/, '');
}

function sha256File(filePath) {
  const hash = crypto.createHash('sha256');
  hash.update(fs.readFileSync(filePath));
  return hash.digest('hex');
}

function listFiles(root, prefix = '') {
  if (!fs.existsSync(root)) return [];
  const entries = fs.readdirSync(root, { withFileTypes: true })
    .sort((left, right) => left.name.localeCompare(right.name));
  const files = [];
  for (const entry of entries) {
    if (entry.name === '.git' || entry.name === 'node_modules') continue;
    const relative = prefix ? path.join(prefix, entry.name) : entry.name;
    const absolute = path.join(root, entry.name);
    if (entry.isDirectory()) {
      files.push(...listFiles(absolute, relative));
    } else if (entry.isFile()) {
      files.push(normalizeRelativePath(relative));
    }
  }
  return files;
}

function buildArtifactManifest(dirPath, options = {}) {
  const root = path.resolve(dirPath);
  const skip = new Set(['artifact-manifest.json']);
  const files = listFiles(root)
    .filter(file => !skip.has(file))
    .map(file => {
      const absolute = path.join(root, file);
      const stat = fs.statSync(absolute);
      return {
        path: file,
        size_bytes: stat.size,
        sha256: sha256File(absolute),
      };
    });

  return {
    version: 1,
    generated_at: options.generatedAt || new Date().toISOString(),
    root: root,
    file_count: files.length,
    files,
  };
}

function writeArtifactManifest(dirPath, options = {}) {
  fs.mkdirSync(dirPath, { recursive: true });
  const manifest = buildArtifactManifest(dirPath, options);
  const outPath = path.join(dirPath, 'artifact-manifest.json');
  fs.writeFileSync(outPath, `${JSON.stringify(manifest, null, 2)}\n`);
  return { manifest, outPath };
}

function parseArgs(argv) {
  const options = { dir: '', json: false };
  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === '--dir') {
      options.dir = argv[index + 1] || '';
      index += 1;
    } else if (arg === '--json') {
      options.json = true;
    } else if (arg === '--help' || arg === '-h') {
      options.help = true;
    } else if (!options.dir) {
      options.dir = arg;
    } else {
      throw new Error(`Unknown argument: ${arg}`);
    }
  }
  return options;
}

function usage() {
  console.log(`Usage:
  artifact-manifest --dir <artifact_dir> [--json]
`);
}

function main(argv = process.argv.slice(2)) {
  const options = parseArgs(argv);
  if (options.help || !options.dir) {
    usage();
    return options.help ? 0 : 1;
  }
  const result = writeArtifactManifest(options.dir);
  if (options.json) {
    console.log(JSON.stringify(result.manifest, null, 2));
  } else {
    console.log(`Wrote ${result.outPath}`);
  }
  return 0;
}

if (require.main === module) {
  try {
    process.exit(main());
  } catch (error) {
    console.error(`artifact-manifest: ${error.message}`);
    process.exit(1);
  }
}

module.exports = {
  buildArtifactManifest,
  listFiles,
  main,
  normalizeRelativePath,
  sha256File,
  writeArtifactManifest,
};
