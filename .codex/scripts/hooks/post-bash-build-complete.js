#!/usr/bin/env node
'use strict';

/**
 * post-bash-build-complete (RCC v2)
 *
 * Scoped to RCC-supported research languages (Python + C++):
 *   - native (cmake / make)
 *   - ROS (catkin / colcon)
 *   - Python install (pip / uv / setup.py)
 *   - Docker build
 *   - ML training (nerfstudio / gaussian-splatting / train.py / torchrun / accelerate)
 *
 * Web/Rust/Go families intentionally removed — RCC v2 platform scope is
 * Python and C++ only. Add additional families only if research projects
 * begin to require them.
 *
 * Emits one structured stderr line per detected build family.
 */

const MAX_STDIN = 1024 * 1024;
let raw = '';

process.stdin.setEncoding('utf8');
process.stdin.on('data', chunk => {
  if (raw.length < MAX_STDIN) {
    const remaining = MAX_STDIN - raw.length;
    raw += chunk.substring(0, remaining);
  }
});

// Build-family detection patterns. Order matters only for label preference;
// the first match wins. Each pattern targets a *command invocation*, not arbitrary
// text, to keep false positives low.
const BUILD_PATTERNS = [
  { label: 'cmake',   re: /\bcmake\s+(--build|-B|-S)\b/ },
  { label: 'make',    re: /(^|[\s;&|`(])make(?:\s|$)(?!.*--help)/ },
  { label: 'ros1',    re: /\bcatkin_make\b/ },
  { label: 'ros1',    re: /\bcatkin\s+build\b/ },
  { label: 'ros2',    re: /\bcolcon\s+build\b/ },
  { label: 'python-install', re: /\bpip\s+install\s+(-e\s+\.|--editable\s+\.|-r\s|-U)/ },
  { label: 'python-install', re: /\buv\s+(pip\s+)?install\b/ },
  { label: 'python-install', re: /\bpython\s+setup\.py\s+(install|develop|build)\b/ },
  { label: 'docker-build',   re: /\bdocker\s+build\b/ },
  { label: 'docker-build',   re: /\bdocker-compose\s+build\b/ },
  { label: 'ml-train',       re: /\bns-train\b/ },                         // nerfstudio
  { label: 'ml-train',       re: /\bgaussian-splatting\b/ },                // 3DGS pipeline
  { label: 'ml-train',       re: /\bpython\s+(\.\/)?train(_\w+)?\.py\b/ },  // train.py / train_3dgs.py
  { label: 'ml-train',       re: /\baccelerate\s+launch\b/ },
  { label: 'ml-train',       re: /\btorchrun\b/ },
];

function detectBuildFamily(cmd) {
  for (const { label, re } of BUILD_PATTERNS) {
    if (re.test(cmd)) return label;
  }
  return null;
}

process.stdin.on('end', () => {
  try {
    const input = JSON.parse(raw);
    const cmd = String(input.tool_input?.command || '');
    const family = detectBuildFamily(cmd);
    if (family) {
      console.error(`[Hook] Build completed (family=${family}) — async analysis running in background`);
    }
  } catch {
    // ignore parse errors and pass through
  }

  process.stdout.write(raw);
});
