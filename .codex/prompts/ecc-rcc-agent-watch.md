# RCC Command Prompt: /rcc-agent-watch

Source: commands/rcc-agent-watch.md

Use this prompt to run the RCC `rcc-agent-watch` workflow.


# rcc-agent-watch

A standalone Python script that renders a Layout-A compact dashboard of
Claude Code sub-agent activity by reading the active session jsonl plus
`.claude/research-state.json`. **No model API calls** — everything is
parsed client-side, so the script is free to run continuously.

This complements (does not replace) the CC's built-in `agents` slash and
`claude agents` overlay:

| Tool | Token cost | Refresh | Where it runs | Remote-Control friendly |
|---|---|---|---|---|
| `agents` (CC built-in slash) | 0 (UI only) | on-demand | inside CC session | ❌ (Remote Control limitation) |
| `claude agents` (overlay) | Haiku per row, ~15s polling | live | separate terminal | depends |
| **`rcc-agent-watch.py`** | **0** | snapshot or `--watch` | any shell | **✓** (just a Python script) |

## Quick start

```bash
# Snapshot (one-shot)
python3 ~/.claude/scripts/rcc-agent-watch.py --project /path/to/your-project

# Live refresh, 5s interval (Ctrl-C to exit)
python3 ~/.claude/scripts/rcc-agent-watch.py --project /path/to/your-project --watch

# Show last 12 completed instead of default 8
python3 ~/.claude/scripts/rcc-agent-watch.py --last 12

# Disable color (for redirecting to a log)
python3 ~/.claude/scripts/rcc-agent-watch.py --no-color >> watch.log
```

## Remote use (copy + run on the server)

When CC runs on a remote dev server and the built-in `agents` slash is unavailable over
Remote Control, copy the script to the remote and run it there. No SSH
wiring is needed — the script just reads local files.

```bash
# from your workstation
scp ~/Project/claude/rcc/scripts/rcc-agent-watch.py \
    user@research-server:~/rcc-agent-watch.py

# then on the remote
ssh user@research-server
python3 ~/rcc-agent-watch.py --project /home/user/Project/TofSLAM_v1.0 --watch
```

You can also edit the `CONFIG` block at the top of the file directly so
`--project` is no longer needed:

```python
# ============================================================
# CONFIG — edit these directly when copying to a fresh machine
# ============================================================
PROJECT_ROOT = "/home/user/Project/your-project"
WATCH_INTERVAL = 5
SHOW_LAST_N    = 8
USE_COLOR_DEFAULT = True
# ============================================================
```

## Output (Layout A)

```
═══════════════════════════════════════════════════════════
 RCC Agent Watch · TofSLAM_v1.0  ·  Sprint 13 / S13-B.B.1
 14:10:43  ·  session: 1a549c0…cccb.jsonl
═══════════════════════════════════════════════════════════
 Pipeline: research ✓  codex-review ✓  architect ✓  build ◐  eval —

 Active sub-agents (1):
   ● code-reviewer           elapsed 01:43

 Recent completed (last 5):
   [✗] BLOCK   codex-reviewer          02:30
   [✓] PASS    researcher              02:15
   [↻] REVISE  researcher              01:48
   [✓] PASS    architect               01:08
   [⊘] PEND    general-purpose         00:34

 Totals: 47 sub-agent dispatch · avg 00:56 · tokens in=820.3K out=412.7K
 (cold archive: 14 stage_history entries trimmed — see rcc-state-history.js)
```

Symbol legend:

| Glyph | Meaning |
|---|---|
| `●` cyan | active (in-flight) sub-agent |
| `✓` green | PASS verdict (cross-referenced from research-state.json) |
| `✗` red | BLOCK / tool error |
| `↻` yellow | REVISE |
| `⊘` gray | PENDING (verdict undetected by stage-record hook) |
| `·` gray | no verdict info (state.json out of window, or hook didn't fire) |
| `—` gray | stage not yet entered |
| `◐` cyan | `current_stage` in state.json |

## How it figures things out

1. **Session jsonl auto-discovery** — looks for `~/.claude/projects/-<slug>/*.jsonl`
   where `<slug>` is the project root with `/`, `_`, `.` all replaced by `-`.
   Picks the most-recently-modified file.

2. **Active vs completed** — every `tool_use` with `name="Agent"` (or
   legacy `name="Task"`) is an active sub-agent until a matching
   `tool_result` with the same `tool_use_id` appears in the jsonl.

3. **Verdict** — best-effort lookup against `stage_history` in
   `.claude/research-state.json` matching by `agent` field and an
   `ended_at` within a 5-minute window.

4. **Pipeline chips** — derived from `passed_stages` (C5),
   `invalidated_stages` (I3), and `current_stage` in research-state.json.

5. **Token totals** — accumulated from every `message.usage` block in the
   jsonl tail window (session-level, not per-agent — Claude Code's jsonl
   doesn't attribute tokens to individual sub-agent dispatches).

## CLI flags

| Flag | Default | Notes |
|---|---|---|
| `--project PATH` | from CONFIG | absolute project root |
| `--claude-home PATH` | `~/.claude` | for non-standard CC homes |
| `--watch` | off | refresh loop, Ctrl-C exits |
| `--interval N` | 5 | seconds between refreshes |
| `--last N` | 8 | recent completions to show |
| `--no-color` | off | disable ANSI (useful for logs/pipes) |
| `--tail-bytes N` | 4 MB | jsonl bytes to scan from EOF |
| `--session PATH` | auto | override session jsonl auto-discovery |

## Limitations

- **Tail-window only**: defaults to last 4 MB of the jsonl. Sub-agents
  fired earlier in a very long session are not shown unless you bump
  `--tail-bytes`. (RCC's typical sub-agent dispatch density: ~4 MB ≈ a
  few hundred recent events.)
- **Verdict cross-reference is best-effort**: if the
  `post:agent:stage-record` hook hasn't fired yet (race window) or the
  agent isn't in `STAGE_FOR_SUBAGENT`, the verdict column shows `·`.
- **Per-sub-agent token attribution not possible** from the jsonl alone;
  the totals row is session-level.
- **`--watch` clears the screen** between refreshes (`os.system('clear')`).
  Use `--no-color` and pipe stdout if you need scrollable history.

## Related

- CC built-in slash: `agents` (in-session UI)
- CC built-in CLI: `claude agents` (live overlay, Haiku per row)
- archive query: `node scripts/rcc-state-history.js`
- amend verdict: `node scripts/rcc-state-amend.js`
- rollback stage: `node scripts/rcc-state-invalidate.js`
