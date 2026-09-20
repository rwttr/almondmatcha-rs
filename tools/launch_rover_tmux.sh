#!/bin/bash
# tools/launch_rover_tmux.sh -- RPi tmux launch script (almondmatcha-rs).
#
# Replaces `ws_rpi/launch_rover_tmux.sh` from the ROS 2 tree. That script
# started nine ROS 2 nodes over a 3x3 pane grid on ROS_DOMAIN_ID=5; this one
# starts three prebuilt Rust binaries over a 2x2 grid, because the rewrite
# collapsed nine nodes into three processes (see `docs/RUST_REWRITE_PLAN.md`
# for the node-to-crate mapping). Same session name ("rover"), same
# pane-titles-and-colour-banner style, same run-directory-per-launch idea,
# carried over deliberately because the user liked it.
#
# Why exactly three processes, not one: design defect D1
# (`docs/RUST_REWRITE_PLAN.md` sec 13.3b). The RPi runs `rover-control`,
# `rover-navigation` and `rover-telemetry` as three separate OS processes,
# each binding its own UDP port -- `[services]` in `config/rover.toml` maps
# a *process* (not a host) to a `host:port`, exactly to let three listeners
# coexist on one machine without an `EADDRINUSE` collision. Do not "simplify"
# this back down to one pane per host; that is the bug D1 fixed.
#
# Session: "rover", one window, single 2x2 pane grid:
#
#   col->  LEFT                RIGHT
#   row0   0: rover-control    1: rover-navigation
#   row1   2: rover-telemetry  3: (spare shell)
#
#   [0] rover-control     -- EKF + guidance + actuation, positional config arg
#   [1] rover-navigation  -- dual GNSS + mission state machine, --config
#   [2] rover-telemetry   -- per-topic CSV logging + 5 Hz Telemetry feed
#   [3] spare shell        -- for `rover-tap`, `ls runs/`, etc.
#
# Detach:   Ctrl+b d   (session keeps running)
# Reattach: tmux attach -t rover
# Kill:     tmux kill-session -t rover   (see CONTROLS block at EOF too)
#
# SKIP_ATTACH=1 ./tools/launch_rover_tmux.sh   builds the session and returns
# without attaching -- used by CI/smoke checks and by hand when you only want
# to confirm the session came up clean.

set -euo pipefail

SESSION_NAME="rover"

# ---------------------------------------------------------------------------
# Find the repo root from this script's own location, not a hardcoded home
# directory path. The ROS 2 originals assumed `~/almondmatcha`; this repo was
# renamed to `almondmatcha-rs` and a fresh clone on the RPi may sit anywhere,
# so derive ROOT instead of guessing. Everything below hangs off it.
# ---------------------------------------------------------------------------
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CONFIG_PATH="$ROOT/config/rover.toml"
BIN_DIR="$ROOT/target/release"
RUNS_ROOT="$ROOT/runs"

# ---------------------------------------------------------------------------
# Preflight: fail fast and clearly, before creating any tmux pane.
# ---------------------------------------------------------------------------

if ! command -v tmux >/dev/null 2>&1; then
    echo "launch_rover_tmux: tmux is not installed" >&2
    exit 1
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
    echo "launch_rover_tmux: config not found at $CONFIG_PATH" >&2
    exit 1
fi

# Prebuilt release binaries only -- never `cargo run`. A field launch is not
# the moment to discover a fresh clone needs a multi-minute compile; if a
# binary is missing, say exactly which one and exactly how to build it, and
# stop, rather than silently kicking off a build the operator didn't ask for.
REQUIRED_BINS=(rover-control rover-navigation rover-telemetry)
missing=()
for bin in "${REQUIRED_BINS[@]}"; do
    if [[ ! -x "$BIN_DIR/$bin" ]]; then
        missing+=("$bin")
    fi
done
if (( ${#missing[@]} > 0 )); then
    echo "launch_rover_tmux: missing release binaries:" >&2
    for bin in "${missing[@]}"; do
        echo "  - $BIN_DIR/$bin" >&2
    done
    echo "Build them first:" >&2
    echo "  cd \"$ROOT\" && cargo build --release -p rover-control -p rover-navigation -p rover-telemetry" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# RUST_LOG must be set, or every pane looks dead.
#
# All three binaries call `env_logger::init()`, and env_logger defaults to
# error-only output when RUST_LOG is unset -- so `log::info!("rover-telemetry
# ready, binding ...")` and every other status line silently prints nothing.
# A pane sitting there with no output looks exactly like a hung process in
# the field. Default to `info`; override with e.g. `RUST_LOG=debug
# ./tools/launch_rover_tmux.sh` for more.
# ---------------------------------------------------------------------------
export RUST_LOG="${RUST_LOG:-info}"

# rover-navigation's serial ports/bauds, overridable per the same reasoning
# its own --help gives: a bench setup or a second board on the same host
# legitimately needs a different path than the real rover's udev-enumerated
# defaults. Left unset, rover-navigation applies its own defaults
# (/dev/ttyACM0 @ 460800, /dev/ttyUSB0 @ 115200) -- these are only passed
# through when the operator overrides them.
ROVER_UBLOX_PORT="${ROVER_UBLOX_PORT:-}"
ROVER_UBLOX_BAUD="${ROVER_UBLOX_BAUD:-}"
ROVER_SPRESENSE_PORT="${ROVER_SPRESENSE_PORT:-}"
ROVER_SPRESENSE_BAUD="${ROVER_SPRESENSE_BAUD:-}"

nav_args=(--config "'$CONFIG_PATH'")
[[ -n "$ROVER_UBLOX_PORT" ]] && nav_args+=(--ublox-port "$ROVER_UBLOX_PORT")
[[ -n "$ROVER_UBLOX_BAUD" ]] && nav_args+=(--ublox-baud "$ROVER_UBLOX_BAUD")
[[ -n "$ROVER_SPRESENSE_PORT" ]] && nav_args+=(--spresense-port "$ROVER_SPRESENSE_PORT")
[[ -n "$ROVER_SPRESENSE_BAUD" ]] && nav_args+=(--spresense-baud "$ROVER_SPRESENSE_BAUD")
NAV_CMD="$BIN_DIR/rover-navigation ${nav_args[*]}"

CONTROL_CMD="$BIN_DIR/rover-control '$CONFIG_PATH'"
TELEMETRY_CMD="$BIN_DIR/rover-telemetry --config '$CONFIG_PATH' --runs-dir '$RUNS_ROOT'"

# Kill existing session if one is already running.
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# ---------------------------------------------------------------------------
# ROVER_RUN_DIR allocation.
#
# `crates/rover-runs/src/lib.rs`'s `RunDir::resolve` is the actual contract:
# it prefers `$ROVER_RUN_DIR` when set, and only falls back to scanning
# `runs_root` itself and computing its own `run_NNN_<stamp>` when the
# variable is absent. `rover-telemetry` is the only one of the three RPi
# binaries that uses it (via `--runs-dir`), but every process still gets it
# exported below so a launch never scatters output across processes that
# started at slightly different wall-clock seconds.
#
# The number and name are computed here with the *same* scheme as
# `rover-runs::next_run_number` / `run_dir_name` -- not a reimplementation
# that happens to look similar. `next_run_number` strips the "run_" prefix,
# takes exactly the next 3 characters (not "up to the next underscore"),
# and parses them as the run number, ignoring anything that doesn't parse;
# `format_run_timestamp` builds "YYYYMMDD_HHMMSS" from a UTC unix timestamp
# (its own test cross-checks this against `date -u`), not local time -- so
# this allocator uses `date -u`, not plain `date`, deliberately.
mkdir -p "$RUNS_ROOT"
_max_run=0
for _entry in "$RUNS_ROOT"/run_*; do
    [[ -d "$_entry" ]] || continue
    _name="$(basename "$_entry")"
    _rest="${_name#run_}"
    _digits="${_rest:0:3}"
    if [[ "$_digits" =~ ^[0-9]{3}$ ]]; then
        _n=$((10#$_digits))
        (( _n > _max_run )) && _max_run=$_n
    fi
done
_next_run=$(( _max_run + 1 ))
_stamp="$(date -u +%Y%m%d_%H%M%S)"
# Assigned separately from `export` (shellcheck SC2155): `export X="$(...)"`
# takes the exit status of `export`, not of the substitution, so a failing
# `date`/`printf` would sail straight past `set -e` and leave a malformed run
# directory name.
_run_dir_name="$(printf 'run_%03d_%s' "$_next_run" "$_stamp")"
ROVER_RUN_DIR="$RUNS_ROOT/$_run_dir_name"
export ROVER_RUN_DIR
echo "[run] output directory for this launch: $ROVER_RUN_DIR"

# ============================================================================
# Per-process console logs
# ============================================================================
# Every process's stdout+stderr is teed to $ROVER_RUN_DIR/<name>.log
# alongside the CSVs, so a run is one self-contained folder and a process
# that dies at startup leaves its reason on disk. Without this the only copy
# of a startup error -- e.g. a config load failure, or "no [services] entry
# for `navigation`" -- lives in a tmux scrollback that is gone once the
# session is killed, which is exactly when it is needed.
#
# stdbuf -oL -eL is required, not cosmetic: stdout becomes block-buffered
# the moment it is a pipe rather than a tty, so without it the log lags by
# kilobytes and a crash discards the buffered tail -- losing precisely the
# lines that explain the crash. (There is no PYTHONUNBUFFERED here: every
# RPi process is a Rust binary, not Python.)
#
# `tee` needs $ROVER_RUN_DIR to already exist -- unlike `RunDir` inside the
# binaries, which creates it lazily on the first CSV write, this launcher
# creates it eagerly up front, immediately after allocating it above.
mkdir -p "$ROVER_RUN_DIR"

# ---------------------------------------------------------------------------
# Pane environment -- load-bearing, and not obvious.
#
# `export ROVER_RUN_DIR=...` above affects THIS script, and tmux only copies
# the calling environment into a new session when it also has to start a new
# server. If a tmux server is already running on this machine -- which on the
# RPi it usually is, because that is how you keep things alive across an SSH
# drop -- the new session attaches to that existing server and its panes
# inherit the SERVER's environment instead. `ROVER_RUN_DIR` is not in tmux's
# default `update-environment` list, so the panes would see it unset.
#
# That failure is silent and nasty: `RunDir::resolve` would stop seeing the
# override and each process would allocate its OWN `run_NNN_<stamp>/`, so the
# CSVs would scatter across several directories while the tee'd logs below
# still went to the one this script allocated -- precisely the scattering the
# shared run directory exists to prevent. Verified against tmux 3.7: with a
# pre-existing server, a new session's pane reports ROVER_RUN_DIR as empty.
#
# So: export it inside every pane explicitly, rather than trusting inheritance.
PANE_ENV="export ROVER_RUN_DIR='$ROVER_RUN_DIR' RUST_LOG='$RUST_LOG'"

# run_logged <pane> <logname> <command>
# The command is passed single-quoted by every caller so that any $(...) or
# quoting inside it reaches the pane's shell intact and is evaluated there,
# not here.
run_logged() {
    local pane="$1" name="$2" cmd="$3"
    tmux send-keys -t "$SESSION_NAME:$pane" \
        "stdbuf -oL -eL $cmd 2>&1 | tee -a '$ROVER_RUN_DIR/$name.log'" C-m
}

# ---------------------------------------------------------------------------
# Build the 2x2 grid.
# ---------------------------------------------------------------------------
tmux new-session -d -s "$SESSION_NAME" -n "rover"

tmux split-window -h -t "$SESSION_NAME:0"        # 0 (left) | 1 (right)
tmux select-pane  -t "$SESSION_NAME:0.0"
tmux split-window -v -t "$SESSION_NAME:0.0"      # 0 (top-left) / 2 (bot-left)
tmux select-pane  -t "$SESSION_NAME:0.1"
tmux split-window -v -t "$SESSION_NAME:0.1"      # 1 (top-right) / 3 (bot-right)

sleep 0.5

# Enable pane titles and colorize borders. Scoped to THIS session's window
# (`-w -t`), not `-g` as the ROS 2 original did: these are window options, and
# setting them globally reaches into whatever other tmux sessions the operator
# already has open on this machine. A launch script has no business restyling
# someone's unrelated shells.
WINDOW="$SESSION_NAME:0"
tmux set-option -w -t "$WINDOW" pane-border-status top
# `#{@name}`, not `#{pane_title}`. A pane's title is whatever the program
# inside it last set via an OSC escape -- and an interactive zsh/bash prompt
# sets it on every command, so a title assigned with `select-pane -T` here is
# overwritten the instant the pane's shell draws its first prompt. (Observed:
# the borders read "yupi@host:~/almondmatcha" instead of the process names.)
# `@name` is a tmux *user option* on the pane, which nothing inside the pane
# can reach, so the label survives. `-T` is still set alongside it, purely so
# `tmux list-panes` shows something meaningful too.
tmux set-option -w -t "$WINDOW" pane-border-format " [#{pane_index}] #{@name} "
tmux set-option -w -t "$WINDOW" pane-border-style fg=colour240
tmux set-option -w -t "$WINDOW" pane-active-border-style fg=colour51

# Pane 0 (top-left): rover-control
tmux set-option -p -t "$SESSION_NAME:0.0" @name "Rover_Control"
tmux select-pane -t "$SESSION_NAME:0.0" -T "Rover_Control"
tmux send-keys   -t "$SESSION_NAME:0.0" "cd '$ROOT'" C-m
tmux send-keys   -t "$SESSION_NAME:0.0" "$PANE_ENV" C-m
tmux send-keys   -t "$SESSION_NAME:0.0" "clear && echo -e '\\e[1;36m>>> [1/3] ROVER CONTROL <<<\\e[0m' && sleep 1" C-m
run_logged 0.0 rover_control "$CONTROL_CMD"

# Pane 1 (top-right): rover-navigation
tmux set-option -p -t "$SESSION_NAME:0.1" @name "Rover_Navigation"
tmux select-pane -t "$SESSION_NAME:0.1" -T "Rover_Navigation"
tmux send-keys   -t "$SESSION_NAME:0.1" "cd '$ROOT'" C-m
tmux send-keys   -t "$SESSION_NAME:0.1" "$PANE_ENV" C-m
tmux send-keys   -t "$SESSION_NAME:0.1" "clear && echo -e '\\e[1;32m>>> [2/3] ROVER NAVIGATION <<<\\e[0m' && sleep 1" C-m
run_logged 0.1 rover_navigation "$NAV_CMD"

# Pane 2 (bottom-left): rover-telemetry
tmux set-option -p -t "$SESSION_NAME:0.2" @name "Rover_Telemetry"
tmux select-pane -t "$SESSION_NAME:0.2" -T "Rover_Telemetry"
tmux send-keys   -t "$SESSION_NAME:0.2" "cd '$ROOT'" C-m
tmux send-keys   -t "$SESSION_NAME:0.2" "$PANE_ENV" C-m
tmux send-keys   -t "$SESSION_NAME:0.2" "clear && echo -e '\\e[1;33m>>> [3/3] ROVER TELEMETRY <<<\\e[0m' && sleep 1" C-m
run_logged 0.2 rover_telemetry "$TELEMETRY_CMD"

# Pane 3 (bottom-right): spare shell
tmux set-option -p -t "$SESSION_NAME:0.3" @name "Spare_Shell"
tmux select-pane -t "$SESSION_NAME:0.3" -T "Spare_Shell"
tmux send-keys   -t "$SESSION_NAME:0.3" "cd '$ROOT'" C-m
tmux send-keys   -t "$SESSION_NAME:0.3" "$PANE_ENV" C-m
tmux send-keys   -t "$SESSION_NAME:0.3" "clear && echo -e '\\e[1;90m>>> [SPARE] shell ready <<<\\e[0m'" C-m

# Focus the telemetry pane (it prints "logging to ..." and is where a run's
# health is most visible) and attach.
tmux select-pane -t "$SESSION_NAME:0.2"

# An `[[ ... ]] && tmux attach` one-liner here would make the script exit 1
# whenever SKIP_ATTACH=1, because the failed test becomes the script's final
# exit status -- and SKIP_ATTACH is exactly the path a smoke check uses, so it
# would report failure on success. Spell it out instead.
if [[ "${SKIP_ATTACH:-0}" == "1" ]]; then
    echo "[tmux] session '$SESSION_NAME' is up; not attaching (SKIP_ATTACH=1)."
    echo "[tmux] attach with: tmux attach -t $SESSION_NAME"
else
    tmux attach-session -t "$SESSION_NAME"
fi

# CONTROLS:
# Ctrl+b arrow keys : Navigate between panes
# Ctrl+b z          : Zoom current pane (toggle fullscreen)
# Ctrl+b [          : Scroll mode (press q to exit)
# Ctrl+d or 'exit'  : Close current pane
#
# SHUTDOWN:
# tmux kill-session -t rover
# OR: Ctrl+b, then type ':kill-session' and press Enter
# OR: Close all panes individually with Ctrl+d
