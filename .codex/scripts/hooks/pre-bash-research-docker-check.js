#!/usr/bin/env node
'use strict';

/**
 * pre:bash:research-docker-check (RCC v2.3 hook)
 *
 * Trigger: PreToolUse on Bash.
 *
 * Validates research-Docker / rosbag / ros2-bag / catkin_make /
 * colcon build / ros2 launch commands against
 * `research-config.json.docker_research` rules and 14 known traps
 * encoded in skills/docker-research-discipline.
 *
 * 14 traps (per skill):
 *   T1  — rosbag/ros2-bag play missing --clock         (sim_time)
 *   T2  — rosbag play --wait-for-subscribers           (ROS1, hangs)
 *   T3  — bag play -r N where N > playback_rate        (non-determinism)
 *   T4  — multi-topic bag without --topics             (heuristic)
 *   T5  — catkin_make without Release                  (ROS1 build)
 *   T6  — docker run without --ipc private (parallel)
 *   T7  — ros2 bag play missing --clock                (alias of T1 — handled together)
 *   T8  — same ROS_DOMAIN_ID across parallel containers (ROS2)
 *   T9  — RMW_IMPLEMENTATION mismatch / unset          (ROS2)
 *   T10 — ros2 launch without use_sim_time:=true       (ROS2)
 *   T11 — QoS BEST_EFFORT in source without opt-in     (informational; out-of-scope here)
 *   T12 — colcon build without Release                 (ROS2 build)
 *   T13 — ROS_DOMAIN_ID outside 0..232                 (ROS2)
 *
 * Profile gating:
 *   minimal  → no-op
 *   standard → warn
 *   strict   → block T1/T7/T2 (hard fails), warn rest
 */

const fs = require('fs');
const path = require('path');

const MAX_STDIN = 1024 * 1024;

const ROS2_DISTROS = ['humble', 'jazzy', 'iron', 'rolling'];
const ROS1_DISTROS = ['noetic', 'melodic', 'kinetic'];

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

function loadConfig(cwd) {
  for (const p of [
    path.join(cwd, '.claude', 'research-config.json'),
    path.join(cwd, 'research-config.json'),
  ]) {
    if (fs.existsSync(p)) {
      try { return JSON.parse(fs.readFileSync(p, 'utf8')); } catch { return null; }
    }
  }
  return null;
}

// ── Command detection ────────────────────────────────────────────────
function isRos1BagPlay(cmd)  { return /\brosbag\s+play\b/.test(cmd); }
function isRos2BagPlay(cmd)  { return /\bros2\s+bag\s+play\b/.test(cmd); }
function isBagPlay(cmd)      { return isRos1BagPlay(cmd) || isRos2BagPlay(cmd); }

function isCatkinMake(cmd)   { return /\bcatkin_make\b/.test(cmd); }
function isColconBuild(cmd)  { return /\bcolcon\s+build\b/.test(cmd); }
function isBuildCmd(cmd)     { return isCatkinMake(cmd) || isColconBuild(cmd); }

function isRos2Launch(cmd)   { return /\bros2\s+launch\b/.test(cmd); }
function isDockerRun(cmd)    { return /\bdocker\s+run\b/.test(cmd); }

function extractBagRate(cmd) {
  // Both rosbag play and ros2 bag play use -r / --rate
  const m = cmd.match(/(?:^|\s)(?:-r|--rate)[=\s]+([0-9.]+)/);
  return m ? Number(m[1]) : null;
}

function extractDomainId(cmd) {
  // docker run ... -e ROS_DOMAIN_ID=NNN
  const m = cmd.match(/ROS_DOMAIN_ID[=\s]+["']?([0-9]+)["']?/);
  return m ? Number(m[1]) : null;
}

function extractRmw(cmd) {
  const m = cmd.match(/RMW_IMPLEMENTATION[=\s]+["']?(rmw_[a-z_]+)["']?/);
  return m ? m[1] : null;
}

function hasFlag(cmd, flag) {
  const re = new RegExp(`(^|\\s)${flag.replace(/[-/\\^$*+?.()|[\]{}]/g, '\\$&')}(\\s|=|$)`);
  return re.test(cmd);
}

function hasUseSimTimeArg(cmd) {
  return /\buse_sim_time\s*:=\s*(true|True|1)\b/.test(cmd);
}

// ── Finding generator ────────────────────────────────────────────────

function findings(cmd, cfg, profile) {
  const out = [];
  const docker = cfg?.docker_research || {};
  const distro = String(docker.ros_distro || '').toLowerCase();
  const isRos2 = ROS2_DISTROS.includes(distro);
  const isRos1 = ROS1_DISTROS.includes(distro);
  // If distro is unset, treat both checks as informational

  // ── T1 / T7 — Missing --clock (both ROS1 + ROS2) ──
  if (isBagPlay(cmd) && !hasFlag(cmd, '--clock')) {
    const which = isRos2BagPlay(cmd) ? 'T7' : 'T1';
    const cmdName = isRos2BagPlay(cmd) ? 'ros2 bag play' : 'rosbag play';
    out.push({
      code: which,
      severity: 'BLOCK',
      msg: `${cmdName} missing --clock (sim_time will be inconsistent)`,
    });
  }

  // ── T2 — --wait-for-subscribers (ROS1 only; flag doesn't exist in ROS2) ──
  const forbidden = docker.forbidden_flags || ['--wait-for-subscribers'];
  if (isRos1BagPlay(cmd)) {
    for (const f of forbidden) {
      if (hasFlag(cmd, f)) {
        out.push({
          code: 'T2',
          severity: 'BLOCK',
          msg: `rosbag play uses forbidden flag ${f} (hangs on multi-topic bags)`,
        });
      }
    }
  }

  // ── T3 — rate > expected playback_rate ──
  if (isBagPlay(cmd)) {
    const r = extractBagRate(cmd);
    const expected = docker.playback_rate != null ? Number(docker.playback_rate) : 1.0;
    if (r !== null && r > expected && docker.playback_rate_warn_if_other !== false) {
      out.push({
        code: 'T3',
        severity: 'WARN',
        msg: `bag play -r ${r} > expected ${expected} (non-deterministic; screening only)`,
      });
    }
  }

  // ── T4 — multi-topic heuristic (filename hints) ──
  if (isBagPlay(cmd) && !hasFlag(cmd, '--topics')) {
    const hints = (docker.multi_topic_bag_hints || ['ntu_viral', 'm3dgr', 'multi_topic']);
    for (const hint of hints) {
      const safe = hint.replace(/[-/\\^$*+?.()|[\]{}]/g, '\\$&');
      if (new RegExp(safe, 'i').test(cmd)) {
        out.push({
          code: 'T4',
          severity: 'WARN',
          msg: `multi-topic bag matched hint "${hint}" but no --topics flag — may include unwanted topics`,
        });
        break;
      }
    }
  }

  // ── T5 — catkin_make without Release ──
  if (isCatkinMake(cmd) && !/CMAKE_BUILD_TYPE=Release/.test(cmd)) {
    if (docker.release_build_required !== false) {
      out.push({
        code: 'T5',
        severity: 'WARN',
        msg: 'catkin_make without -DCMAKE_BUILD_TYPE=Release (Debug distorts timing)',
      });
    }
  }

  // ── T12 — colcon build without Release ──
  if (isColconBuild(cmd) && !/CMAKE_BUILD_TYPE=Release/.test(cmd)) {
    if (docker.release_build_required !== false) {
      out.push({
        code: 'T12',
        severity: 'WARN',
        msg: 'colcon build without -DCMAKE_BUILD_TYPE=Release in --cmake-args (Debug distorts timing)',
      });
    }
  }

  // ── T6 — docker run without --ipc private when parallel declared ──
  if (isDockerRun(cmd) && docker.parallel_containers && Number(docker.parallel_containers) > 1) {
    if (!/--ipc[\s=]+private/.test(cmd)) {
      out.push({
        code: 'T6',
        severity: 'WARN',
        msg: 'docker run without --ipc private in a parallel setup',
      });
    }
  }

  // ── T8 — same ROS_DOMAIN_ID across parallel containers (ROS2) ──
  if (isDockerRun(cmd) && isRos2 && docker.parallel_containers > 1) {
    const did = extractDomainId(cmd);
    const base = docker.ros_domain_id_base;
    if (did !== null && base !== undefined) {
      const allowed = [];
      for (let i = 0; i < docker.parallel_containers; i++) allowed.push(base + i);
      if (!allowed.includes(did)) {
        out.push({
          code: 'T8',
          severity: 'WARN',
          msg: `ROS_DOMAIN_ID=${did} not in expected range [${base}..${base + docker.parallel_containers - 1}] for parallel containers`,
        });
      }
    } else if (did === null) {
      out.push({
        code: 'T8',
        severity: 'WARN',
        msg: 'ROS2 parallel docker run missing ROS_DOMAIN_ID — containers will collide on default domain 0',
      });
    }
    // T13 — domain id range check
    if (did !== null && (did < 0 || did > 232)) {
      out.push({
        code: 'T13',
        severity: 'BLOCK',
        msg: `ROS_DOMAIN_ID=${did} outside valid range 0..232`,
      });
    }
  }

  // ── T9 — RMW mismatch (ROS2) ──
  if (isDockerRun(cmd) && isRos2 && docker.rmw_must_match_across_containers !== false) {
    const rmw = extractRmw(cmd);
    const expectedRmw = docker.rmw_implementation;
    if (expectedRmw && rmw && rmw !== expectedRmw) {
      out.push({
        code: 'T9',
        severity: 'WARN',
        msg: `RMW_IMPLEMENTATION=${rmw} differs from project default ${expectedRmw} (DDS interop fails silently)`,
      });
    } else if (expectedRmw && !rmw && docker.parallel_containers > 1) {
      out.push({
        code: 'T9',
        severity: 'WARN',
        msg: `ROS2 parallel container missing RMW_IMPLEMENTATION env (declare project default ${expectedRmw})`,
      });
    }
  }

  // ── T10 — ros2 launch without use_sim_time:=true (when sim_time required) ──
  if (isRos2Launch(cmd) && docker.use_sim_time_required && !hasUseSimTimeArg(cmd)) {
    // Don't fire if user explicitly disables (use_sim_time:=false)
    if (!/\buse_sim_time\s*:=\s*(false|False|0)\b/.test(cmd)) {
      out.push({
        code: 'T10',
        severity: 'WARN',
        msg: 'ros2 launch missing use_sim_time:=true (bag --clock will be ignored by the node)',
      });
    }
  }

  return out;
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
  if (!cmd) { passthrough(); return; }

  const cwd = input.cwd || process.cwd();
  const cfg = loadConfig(cwd);
  const profile = getProfile();

  const issues = findings(cmd, cfg, profile);
  if (issues.length === 0) { passthrough(); return; }

  const hasBlock = issues.some(i => i.severity === 'BLOCK');
  const header = (profile === 'strict' && hasBlock)
    ? '[research-docker-check] BLOCKED: research-Docker discipline violation'
    : '[research-docker-check] WARNING: research-Docker discipline violation';

  process.stderr.write(`${header}\n`);
  process.stderr.write(`  command: ${cmd.slice(0, 160)}\n`);
  for (const i of issues) {
    process.stderr.write(`  • ${i.code} (${i.severity}): ${i.msg}\n`);
  }
  process.stderr.write('  Reference: skills/docker-research-discipline/SKILL.md\n');

  passthrough();
  process.exit(profile === 'strict' && hasBlock ? 2 : 0);
}

main().catch(() => process.exit(0));
