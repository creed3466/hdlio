'use strict';

/**
 * Read/write helpers for .claude/research-state.json
 *
 * Single source of truth for the RCC 5-stage pipeline.
 *
 * Used by:
 *   scripts/hooks/stage-gate.js       (PreToolUse on Agent/Task — verify locks)
 *   scripts/hooks/stage-record.js     (PostToolUse on Agent/Task — append history)
 *   scripts/hooks/canonical-eval-detect.js (PostToolUse on Bash — append history + regression)
 *   scripts/hooks/gpu-profile-snapshot.js (PostToolUse on Bash — append gpu_history)
 *   scripts/hooks/precompact-save-state.js (PreCompact — flush to memory)
 *   scripts/hooks/eval-regression-check.js (alert-only; does NOT write state)
 *
 * Format: schemas/research-state.schema.json
 */

const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

const STATE_RELPATH = path.join('.claude', 'research-state.json');
const ARCHIVE_RELDIR = path.join('.claude', 'history-archive');
const SCHEMA_VERSION = '2.1';
// Accepted on read so existing v1 / v2.0 state files continue to load
// without being overwritten. Why: TofSLAM and other live projects already
// carry hand-maintained "2.1" state.json that the v1 schema would silently
// erase on the next ensureState() call.
const ACCEPTED_SCHEMA_VERSIONS = Object.freeze(['2.1', '2.0', 1]);

const DEFAULT_LOCKS = Object.freeze({
  'codex-review': ['research'],
  architect: ['research', 'codex-review'],
  build: ['architect'],
  eval: ['build'],
});

const VALID_STAGES = Object.freeze(['research', 'codex-review', 'architect', 'build', 'eval']);

// Hot-view caps. Each writer keeps only the most-recent N entries in the
// "working" state.json so ben's read-into-context cost stays bounded
// (~1 KB per entry × these caps ≈ 3 KB total at steady state). Trimmed
// entries are persisted to .claude/history-archive/ as append-only JSONL
// so audit queries (rcc-state-history CLI) can reconstruct anything older.
//
// Rationale: 95% of orchestrator decisions only need the last few entries
// plus passed_stages summary. The previous 500/200/100 caps were
// "survives 50 sprints" but cost ~75 KB / ~22 K tokens at saturation —
// see 20260513_hook_issue follow-up "history bloat" analysis.
const STAGE_HISTORY_MAX = 30;
const REGRESSION_HISTORY_MAX = 30;
const GPU_HISTORY_MAX = 20;
const NOTES_MAX = 10;

function resolveStatePath(cwd) {
  const root = cwd || process.cwd();
  return path.join(root, STATE_RELPATH);
}

function resolveArchiveDir(cwd) {
  return path.join(cwd || process.cwd(), ARCHIVE_RELDIR);
}

// Append `entries` to .claude/history-archive/<field>-<YYYY-MM>.jsonl.
// Best-effort: archive failures must never break the hot write path.
// Each entry becomes a single JSON line; readers must tolerate
// concurrent partial-line writes by skipping malformed lines.
function appendToArchive(cwd, field, entries) {
  if (!Array.isArray(entries) || entries.length === 0) return;
  try {
    const dir = resolveArchiveDir(cwd);
    fs.mkdirSync(dir, { recursive: true });
    const ym = new Date().toISOString().slice(0, 7);   // YYYY-MM
    const file = path.join(dir, `${field}-${ym}.jsonl`);
    const body = entries.map(e => JSON.stringify(e)).join('\n') + '\n';
    fs.appendFileSync(file, body);
  } catch (e) {
    process.stderr.write(
      `[research-state] archive append failed (${field}): ${e.message}\n`,
    );
  }
}

// Iterate archive lines for a given field. Yields parsed entries
// across all months matching the optional `since` (YYYY-MM, inclusive).
// Tolerant of partial / corrupt lines.
function readArchive(cwd, field, options = {}) {
  const dir = resolveArchiveDir(cwd);
  if (!fs.existsSync(dir)) return [];
  const since = options.since || null;     // 'YYYY-MM'
  const limit = options.limit || Infinity;
  let files;
  try {
    files = fs.readdirSync(dir)
      .filter(f => f.startsWith(`${field}-`) && f.endsWith('.jsonl'))
      .filter(f => !since || f.slice(field.length + 1, field.length + 8) >= since)
      .sort();   // ascending YYYY-MM
  } catch {
    return [];
  }
  const out = [];
  for (const file of files) {
    let body;
    try { body = fs.readFileSync(path.join(dir, file), 'utf8'); } catch { continue; }
    for (const line of body.split('\n')) {
      if (!line.trim()) continue;
      try {
        out.push(JSON.parse(line));
        if (out.length >= limit) return out;
      } catch { /* skip malformed line */ }
    }
  }
  return out;
}

function nowIso() {
  return new Date().toISOString();
}

function emptyState() {
  return {
    version: SCHEMA_VERSION,
    session_id: '',
    current_stage: 'research',
    started_at: nowIso(),
    stage_history: [],
    locks: { ...DEFAULT_LOCKS },
  };
}

function readState(cwd) {
  const p = resolveStatePath(cwd);
  if (!fs.existsSync(p)) return null;
  try {
    const data = JSON.parse(fs.readFileSync(p, 'utf8'));
    if (typeof data !== 'object' || data === null) return null;
    if (!ACCEPTED_SCHEMA_VERSIONS.includes(data.version)) return null;
    return data;
  } catch {
    return null;
  }
}

// How many .bak.<timestamp> files to retain after a migration backup.
// Older ones are pruned by cleanupOldBackups so .claude/ doesn't grow
// indefinitely. 5 covers a handful of failed migrations without losing
// recovery options.
const BACKUP_RETENTION = 5;

function cleanupOldBackups(cwd, keepCount = BACKUP_RETENTION) {
  try {
    const p = resolveStatePath(cwd);
    const dir = path.dirname(p);
    if (!fs.existsSync(dir)) return;
    const base = path.basename(p);
    const prefix = `${base}.bak.`;
    const baks = fs.readdirSync(dir)
      .filter(f => f.startsWith(prefix))
      .map(f => {
        const full = path.join(dir, f);
        let mtime = 0;
        try { mtime = fs.statSync(full).mtimeMs; } catch {}
        return { full, mtime };
      })
      .sort((a, b) => b.mtime - a.mtime);   // newest first
    for (let i = keepCount; i < baks.length; i++) {
      try { fs.unlinkSync(baks[i].full); } catch {}
    }
  } catch {
    // best-effort cleanup; never throw from a write path
  }
}

function writeState(cwd, state, options = {}) {
  const p = resolveStatePath(cwd);
  fs.mkdirSync(path.dirname(p), { recursive: true });

  // Write-time migration. Legacy versions remain accepted on read for
  // backward compat, but every write normalizes to the current schema so
  // a project that bounces back-and-forth between tool versions stops
  // drifting. The originating version is preserved exactly once.
  if (options.migrate !== false
      && state
      && typeof state === 'object'
      && state.version !== SCHEMA_VERSION) {
    if (state.version != null && state._migrated_from === undefined) {
      state._migrated_from = state.version;
    }
    state.version = SCHEMA_VERSION;
  }

  // Flush any pending archive entries before serializing. The
  // __pending_archive bag is set by trimWithArchive when caps overflow.
  // We drain it and ship dropped entries to .claude/history-archive/
  // as append-only JSONL — never lost, never in-prompt.
  const pending = state.__pending_archive;
  if (pending && typeof pending === 'object') {
    for (const [field, entries] of Object.entries(pending)) {
      if (Array.isArray(entries) && entries.length > 0) {
        appendToArchive(cwd, field, entries);
      }
    }
    delete state.__pending_archive;
  }

  const tmp = `${p}.tmp`;
  fs.writeFileSync(tmp, JSON.stringify(state, null, 2));
  fs.renameSync(tmp, p);
}

// Cross-process exclusive lock around a read-modify-write closure on
// research-state.json. Required because multiple Task tool calls can fire
// post:agent:stage-record in parallel, and without a lock the second hook's
// write clobbers the first's appended entry.
//
// Lock is implemented as an exclusive-create lock file
// (.claude/research-state.json.lock). Stale locks are reclaimed after
// LOCK_STALE_MS — the only legitimate holder is a hook script which
// finishes in <1s, so a 30s staleness threshold is generous.
const LOCK_STALE_MS = 30 * 1000;
const LOCK_MAX_RETRIES = 100;
const LOCK_BASE_DELAY_MS = 5;

function sleepBusy(ms) {
  const end = Date.now() + ms;
  // Synchronous wait — hooks run as short-lived processes, an async
  // sleep would race against process.exit handlers in some callers.
  // Atomics.wait on a SharedArrayBuffer is the clean alternative.
  const sab = new SharedArrayBuffer(4);
  const view = new Int32Array(sab);
  Atomics.wait(view, 0, 0, Math.max(1, end - Date.now()));
}

function withStateLock(cwd, fn, options = {}) {
  const p = resolveStatePath(cwd);
  fs.mkdirSync(path.dirname(p), { recursive: true });
  const lockPath = `${p}.lock`;
  const maxRetries = options.maxRetries || LOCK_MAX_RETRIES;
  const baseDelayMs = options.baseDelayMs || LOCK_BASE_DELAY_MS;
  const staleMs = options.staleMs || LOCK_STALE_MS;

  for (let attempt = 0; attempt < maxRetries; attempt++) {
    let fd;
    try {
      fd = fs.openSync(lockPath, 'wx');
    } catch (e) {
      if (e.code !== 'EEXIST') throw e;
      try {
        const st = fs.statSync(lockPath);
        if (Date.now() - st.mtimeMs > staleMs) {
          try { fs.unlinkSync(lockPath); } catch {}
          continue;
        }
      } catch {
        // lock disappeared between EEXIST and stat — retry immediately
        continue;
      }
      const delay = Math.min(
        baseDelayMs * Math.pow(1.5, Math.min(attempt, 12)),
        500,
      );
      sleepBusy(delay);
      continue;
    }

    try {
      // Tag the lock with our pid so a debugger can identify who is holding it
      try { fs.writeSync(fd, String(process.pid)); } catch {}
      return fn();
    } finally {
      try { fs.closeSync(fd); } catch {}
      try { fs.unlinkSync(lockPath); } catch {}
    }
  }

  throw new Error(
    `withStateLock: failed to acquire ${lockPath} after ${maxRetries} retries`,
  );
}

function ensureState(cwd, options = {}) {
  let state = readState(cwd);
  if (!state) {
    const p = resolveStatePath(cwd);
    if (fs.existsSync(p)) {
      // Existing file failed readState — schema mismatch, JSON parse error,
      // or non-object root. Back it up before overwrite so the user can
      // recover hand-maintained state instead of silently losing it.
      const bak = `${p}.bak.${Date.now()}`;
      try {
        fs.copyFileSync(p, bak);
        process.stderr.write(
          `[research-state] unreadable state.json — backed up to ${bak}\n`,
        );
        cleanupOldBackups(cwd);
      } catch (e) {
        process.stderr.write(
          `[research-state] backup failed (${e.message}); refusing to overwrite\n`,
        );
        return null;
      }
    }
    state = emptyState();
    if (options.sessionId) state.session_id = options.sessionId;
    if (options.persist !== false) writeState(cwd, state);
  }
  return state;
}

// Stage prerequisites — returns array of unmet prerequisite stage names,
// or [] when the requested stage is allowed. Uses passed_stages summary
// when present (survives history truncation), falls back to stage_history
// scan, and respects invalidated_stages so a rollback (invalidateStage)
// shadows PASSes recorded BEFORE the invalidation timestamp.
function blockedPrerequisites(state, requestedStage) {
  if (!state || !state.locks || !state.locks[requestedStage]) return [];
  const required = state.locks[requestedStage];
  const invalidatedAt = state.invalidated_stages || {};
  const passed = new Set();

  if (state.passed_stages && typeof state.passed_stages === 'object') {
    for (const stage of Object.keys(state.passed_stages)) {
      if (state.passed_stages[stage]) passed.add(stage);
    }
  }
  for (const h of state.stage_history || []) {
    if (!h || h.verdict !== 'PASS' || !h.stage) continue;
    const inv = invalidatedAt[h.stage];
    if (inv) {
      const ts = h.ended_at || h.started_at || '';
      if (ts && ts < inv) continue;   // shadowed by later invalidation
    }
    passed.add(h.stage);
  }
  return required.filter(stage => !passed.has(stage));
}

function isStageAllowed(state, requestedStage) {
  return blockedPrerequisites(state, requestedStage).length === 0;
}

function recordStage(state, entry, options = {}) {
  if (!state.stage_history) state.stage_history = [];
  if (!state.passed_stages) state.passed_stages = {};
  const filled = {
    started_at: nowIso(),
    ended_at: nowIso(),
    verdict: 'PENDING',
    ...entry,
  };
  state.stage_history.push(filled);

  // Maintain a permanent summary of first PASS per stage so prerequisite
  // checks remain correct after history truncation.
  if (
    filled.verdict === 'PASS'
    && VALID_STAGES.includes(filled.stage)
    && !state.passed_stages[filled.stage]
  ) {
    state.passed_stages[filled.stage] = filled.ended_at || nowIso();
  }

  if (filled.verdict === 'PASS' && VALID_STAGES.includes(filled.stage)) {
    state.current_stage = nextStage(filled.stage);
  }

  // Cap stage_history. The passed_stages summary already captured each
  // first PASS, so prerequisite checks survive truncation. Trimmed
  // entries are exposed via options.onTrim so the caller (writeState /
  // hook) can archive them.
  trimWithArchive(state, 'stage_history', STAGE_HISTORY_MAX, options);

  return filled;
}

function nextStage(stage) {
  const order = ['research', 'codex-review', 'architect', 'build', 'eval'];
  const idx = order.indexOf(stage);
  if (idx < 0 || idx === order.length - 1) return 'converged';
  return order[idx + 1];
}

// Amend the verdict of the most recent stage_history entry. Used when
// stage-record auto-detection got it wrong (e.g. recorded PENDING because
// the sub-agent used an unfamiliar verdict phrase) and ben/user wants
// to correct without manually editing state.json.
//
// On amend to PASS, both passed_stages and current_stage are recomputed
// as if the entry had originally PASSed.
function amendLastVerdict(state, verdict, options = {}) {
  if (!state || !Array.isArray(state.stage_history) || state.stage_history.length === 0) {
    throw new Error('amendLastVerdict: stage_history is empty');
  }
  const allowed = new Set(['PASS', 'REVISE', 'BLOCK', 'PENDING']);
  if (!allowed.has(verdict)) {
    throw new Error(`amendLastVerdict: invalid verdict "${verdict}"`);
  }
  const last = state.stage_history[state.stage_history.length - 1];
  const previous = last.verdict;
  last.verdict = verdict;
  last.amended_at = nowIso();
  if (options.reason) last.amended_reason = String(options.reason);
  if (options.amendedBy) last.amended_by = String(options.amendedBy);
  if (previous !== verdict) {
    last.amended_from = previous;
  }

  if (verdict === 'PASS' && VALID_STAGES.includes(last.stage)) {
    if (!state.passed_stages) state.passed_stages = {};
    if (!state.passed_stages[last.stage]) {
      state.passed_stages[last.stage] = last.amended_at;
    }
    state.current_stage = nextStage(last.stage);
  }

  return { previous, current: verdict, entry: last };
}

// Invalidate a previously-PASSed stage and (by default) cascade-clear all
// downstream stages. Used when the orchestrator decides to backtrack —
// e.g. S13-R3 BLOCK forces a re-research, so research must un-PASS so the
// downstream prereq chain stops claiming "research already done".
//
// Without this, passed_stages.research remains forever and
// blockedPrerequisites would leave architect/build unlocked across a
// rollback.
//
// Records an audit entry in stage_history with verdict=BLOCK and
// notes describing the invalidation reason.
const PIPELINE_ORDER = Object.freeze(['research', 'codex-review', 'architect', 'build', 'eval']);

function invalidateStage(state, stage, options = {}) {
  if (!VALID_STAGES.includes(stage)) {
    throw new Error(`invalidateStage: unknown stage "${stage}"`);
  }
  if (!state) throw new Error('invalidateStage: state required');
  if (!state.passed_stages) state.passed_stages = {};

  const reason = String(options.reason || 'manual invalidation');
  const cascade = options.cascade !== false;
  const invalidatedBy = options.invalidatedBy ? String(options.invalidatedBy) : '';

  const idx = PIPELINE_ORDER.indexOf(stage);
  const stagesToInvalidate = cascade
    ? PIPELINE_ORDER.slice(idx)        // stage + everything after
    : [stage];

  if (!state.invalidated_stages) state.invalidated_stages = {};

  // Pick a stamp that is STRICTLY later than every existing PASS entry
  // for any stage we are about to invalidate. ISO strings have ms
  // resolution; back-to-back invalidate+recordStage calls in the same
  // tick would otherwise produce stamp === ended_at and the `<`
  // shadowing check in blockedPrerequisites would let the prior PASS
  // win.
  let stampMs = Date.now();
  for (const h of state.stage_history || []) {
    if (!h || h.verdict !== 'PASS' || !stagesToInvalidate.includes(h.stage)) continue;
    const ts = h.ended_at || h.started_at;
    if (!ts) continue;
    const t = Date.parse(ts);
    if (Number.isFinite(t) && t >= stampMs) stampMs = t + 1;
  }
  const stamp = new Date(stampMs).toISOString();

  const cleared = [];
  for (const s of stagesToInvalidate) {
    if (state.passed_stages[s]) {
      cleared.push(s);
      delete state.passed_stages[s];
    }
    // Always record the invalidation timestamp so blockedPrerequisites
    // shadows pre-invalidation PASS entries in stage_history, even if
    // the stage was never in passed_stages (e.g. legacy v1 state).
    state.invalidated_stages[s] = stamp;
  }

  // Reset current_stage to the earliest invalidated stage so the
  // orchestrator routes back there.
  state.current_stage = stage;

  // Audit log entry so the rollback is visible in stage_history.
  if (!state.stage_history) state.stage_history = [];
  const note = `invalidated ${cleared.length > 0 ? cleared.join(', ') : '(none cleared)'}: ${reason}`;
  recordStage(state, {
    stage,
    agent: 'invalidate-cli',
    verdict: 'BLOCK',
    notes: note,
    ...(invalidatedBy ? { amended_by: invalidatedBy } : {}),
  });

  return { stage, cleared, reason };
}

// Cross-process / cross-project safety: reject obviously dangerous cwd
// values before the hook writes anything. Prevents a stray CC payload
// with cwd="/" or cwd=process.env.HOME from polluting global directories.
function isValidProjectCwd(cwd) {
  if (!cwd || typeof cwd !== 'string') return false;
  if (!path.isAbsolute(cwd)) return false;
  let resolved;
  try {
    resolved = path.resolve(cwd);
    if (!fs.existsSync(resolved)) return false;
    if (!fs.statSync(resolved).isDirectory()) return false;
  } catch {
    return false;
  }
  const os = require('os');
  const forbidden = new Set([
    '/',
    '/etc',
    '/usr',
    '/var',
    '/tmp',
    '/root',
    '/home',
    os.homedir(),
    path.join(os.homedir(), '.claude'),
    path.join(os.homedir(), '.codex'),
    path.join(os.homedir(), '.cursor'),
  ]);
  if (forbidden.has(resolved)) return false;
  return true;
}

function appendRegression(state, entry, options = {}) {
  if (!state.regression_history) state.regression_history = [];
  state.regression_history.push({
    timestamp: nowIso(),
    ...entry,
  });
  trimWithArchive(state, 'regression_history', REGRESSION_HISTORY_MAX, options);
}

function appendGpuSnapshot(state, snap, options = {}) {
  if (!state.gpu_history) state.gpu_history = [];
  state.gpu_history.push({
    timestamp: nowIso(),
    ...snap,
  });
  trimWithArchive(state, 'gpu_history', GPU_HISTORY_MAX, options);
}

function appendNote(state, note) {
  if (!Array.isArray(state.notes)) state.notes = [];
  state.notes.push(String(note));
  if (state.notes.length > NOTES_MAX) {
    state.notes = state.notes.slice(-NOTES_MAX);
  }
}

// Generic in-state trim helper. Drops entries beyond `cap` keeping the
// newest, increments `<field>_truncated_count` for audit, and stashes the
// dropped entries on `state.__pending_archive[<field>]` so writeState (or
// a hook) can flush them to disk after the state mutation completes.
// `__pending_archive` is intentionally `__`-prefixed so it does not get
// serialised to state.json (writeState strips it before write).
function trimWithArchive(state, field, cap, options = {}) {
  const arr = state[field];
  if (!Array.isArray(arr) || arr.length <= cap) return [];
  const drop = arr.length - cap;
  const dropped = arr.slice(0, drop);
  state[field] = arr.slice(-cap);
  const counterField = `${field}_truncated_count`;
  state[counterField] = (state[counterField] || 0) + drop;

  if (!state.__pending_archive) state.__pending_archive = {};
  if (!state.__pending_archive[field]) state.__pending_archive[field] = [];
  for (const entry of dropped) state.__pending_archive[field].push(entry);

  if (typeof options.onTrim === 'function') {
    try { options.onTrim(field, dropped); } catch { /* never throw from trim */ }
  }
  return dropped;
}

function pinBaseline(state, relPath, content) {
  state.active_baseline_ref = relPath;
  state.active_baseline_sha256 = sha256(content);
}

function pinDataset(state, relPath, content) {
  if (!state.dataset_locks) state.dataset_locks = [];
  const existing = state.dataset_locks.find(d => d.path === relPath);
  const entry = {
    path: relPath,
    sha256: sha256(content),
    size_bytes: typeof content === 'string' ? Buffer.byteLength(content) : content.length,
    recorded_at: nowIso(),
  };
  if (existing) {
    Object.assign(existing, entry);
  } else {
    state.dataset_locks.push(entry);
  }
  return entry;
}

function sha256(buf) {
  const h = crypto.createHash('sha256');
  h.update(typeof buf === 'string' ? Buffer.from(buf) : buf);
  return h.digest('hex');
}

module.exports = {
  STATE_RELPATH,
  ARCHIVE_RELDIR,
  SCHEMA_VERSION,
  ACCEPTED_SCHEMA_VERSIONS,
  DEFAULT_LOCKS,
  VALID_STAGES,
  STAGE_HISTORY_MAX,
  REGRESSION_HISTORY_MAX,
  GPU_HISTORY_MAX,
  NOTES_MAX,
  BACKUP_RETENTION,
  resolveStatePath,
  resolveArchiveDir,
  appendToArchive,
  readArchive,
  nowIso,
  emptyState,
  readState,
  writeState,
  withStateLock,
  ensureState,
  cleanupOldBackups,
  blockedPrerequisites,
  isStageAllowed,
  recordStage,
  amendLastVerdict,
  invalidateStage,
  isValidProjectCwd,
  nextStage,
  appendRegression,
  appendGpuSnapshot,
  appendNote,
  trimWithArchive,
  pinBaseline,
  pinDataset,
  sha256,
};
