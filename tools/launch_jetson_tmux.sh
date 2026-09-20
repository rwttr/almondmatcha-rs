#!/bin/bash
# tools/launch_jetson_tmux.sh -- Jetson tmux launch script (almondmatcha-rs).
#
# Replaces `ws_jetson/launch_jetson_tmux.sh` from the ROS 2 tree. That script
# started five ROS 2 nodes across two DDS domains (camera_stream,
# lane_detection, rover_kinematic_control, local monitoring, camera recorder);
# this one starts ONE process, because the rewrite collapsed the whole vision
# chain into a single Python process and moved kinematic control to the RPi.
# See `docs/RUST_REWRITE_PLAN.md` sec 1.2 for the node-to-process mapping.
#
# Session: "jetson", one window, two panes side by side:
#
#   [0] rover-perception  -- D415 capture + lane detection + LaneMeasurement
#   [1] spare shell        -- for `rover-tap`, `ls runs/`, etc.
#
# Detach:   Ctrl+b d   (session keeps running)
# Reattach: tmux attach -t jetson
# Kill:     tmux kill-session -t jetson
#
# SKIP_ATTACH=1 ./tools/launch_jetson_tmux.sh   builds the session and returns
# without attaching.

set -euo pipefail

SESSION_NAME="jetson"

# Repo root from this script's own location -- see the matching comment in
# launch_rover_tmux.sh for why this is derived rather than hardcoded.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CONFIG_PATH="$ROOT/config/rover.toml"
VENV="$ROOT/perception/.venv"
PERCEPTION_BIN="$VENV/bin/rover-perception"
RUNS_ROOT="$ROOT/runs"

# ---------------------------------------------------------------------------
# Preflight: fail fast and clearly, before creating any tmux pane.
# ---------------------------------------------------------------------------

if ! command -v tmux >/dev/null 2>&1; then
    echo "launch_jetson_tmux: tmux is not installed" >&2
    exit 1
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
    echo "launch_jetson_tmux: config not found at $CONFIG_PATH" >&2
    exit 1
fi

# The console script is declared by `perception/pyproject.toml`
# (`rover-perception = "rover_perception.main:main"`), so it only exists once
# the package has been installed into the venv. Same reasoning as the RPi
# script's release-binary check: say exactly what is missing and exactly how
# to fix it, rather than starting an install nobody asked for.
if [[ ! -x "$PERCEPTION_BIN" ]]; then
    echo "launch_jetson_tmux: rover-perception not found at $PERCEPTION_BIN" >&2
    echo "Set the venv up first:" >&2
    echo "  cd \"$ROOT/perception\" && python3 -m venv .venv" >&2
    echo "  .venv/bin/pip install -e ." >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Unbuffered output.
#
# PYTHONUNBUFFERED is the Python half of the same argument `stdbuf -oL -eL`
# makes below: stdout goes block-buffered the moment it is a pipe rather than
# a tty, so without it the log lags by kilobytes and a crash discards the
# buffered tail -- losing precisely the lines that explain the crash. The
# canonical example on this machine is the D415 failing to initialise and
# printing its "Connected devices: [...]" diagnostic on the way down.
# ---------------------------------------------------------------------------
export PYTHONUNBUFFERED=1

# ---------------------------------------------------------------------------
# Camera / detector options, all overridable from the environment.
#
# Field defaults: CSV logging ON (a run you cannot analyse afterwards was a
# wasted drive), no preview window (nothing is attached to the Jetson's HDMI
# in a field, and cv2.imshow on a headless box fails), and the throttled
# heartbeat rather than --verbose (the ROS 2 original logged every frame at
# ~30 FPS, which is noise).
#
# Note `--csv` takes a PATH, not a boolean -- checked against
# `perception/rover_perception/main.py`'s argparse. There is no established
# filename for it in `docs/CSV_LOGGING.md` (the Jetson's Tier 2 log never
# existed in this repo), so the launcher picks one and puts it in the shared
# run directory with everything else.
# ---------------------------------------------------------------------------
PERCEPTION_CSV="${PERCEPTION_CSV:-}"     # set below, once ROVER_RUN_DIR exists
PERCEPTION_PREVIEW="${PERCEPTION_PREVIEW:-0}"
PERCEPTION_VERBOSE="${PERCEPTION_VERBOSE:-0}"
PERCEPTION_SERIAL="${PERCEPTION_SERIAL:-}"
PERCEPTION_VIDEO="${PERCEPTION_VIDEO:-}"
PERCEPTION_WIDTH="${PERCEPTION_WIDTH:-}"
PERCEPTION_HEIGHT="${PERCEPTION_HEIGHT:-}"
PERCEPTION_FPS="${PERCEPTION_FPS:-}"

# Kill existing session if one is already running.
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# ---------------------------------------------------------------------------
# ROVER_RUN_DIR allocation -- identical scheme to launch_rover_tmux.sh, and
# for the same reason: `crates/rover-runs/src/lib.rs`'s `RunDir::resolve`
# prefers this variable over allocating its own `run_NNN_<stamp>/`. The two
# machines each allocate their own (they have separate filesystems), but both
# lay a run out the same way, which is what makes the two halves of a run
# recognisable as one after they are rsync'd together.
#
# `next_run_number` strips the "run_" prefix, takes exactly the next three
# characters and parses them; `format_run_timestamp` builds "YYYYMMDD_HHMMSS"
# from a UTC unix timestamp -- hence `date -u`, not plain `date`.
# ---------------------------------------------------------------------------
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
# Assigned separately from `export` (shellcheck SC2155): see the RPi script.
_run_dir_name="$(printf 'run_%03d_%s' "$_next_run" "$_stamp")"
ROVER_RUN_DIR="$RUNS_ROOT/$_run_dir_name"
export ROVER_RUN_DIR
echo "[run] output directory for this launch: $ROVER_RUN_DIR"

# `tee` needs the directory to exist. Unlike the loggers inside the processes,
# which create it lazily on their first write, this launcher creates it
# eagerly -- there is always at least a console log to write.
mkdir -p "$ROVER_RUN_DIR"

# Now that the run directory is known, default the CSV path into it.
PERCEPTION_CSV="${PERCEPTION_CSV:-$ROVER_RUN_DIR/lane_detection.csv}"

perception_args=(--config "'$CONFIG_PATH'" --csv "'$PERCEPTION_CSV'")
[[ "$PERCEPTION_PREVIEW" == "1" ]] && perception_args+=(--preview)
[[ "$PERCEPTION_VERBOSE" == "1" ]] && perception_args+=(--verbose)
[[ -n "$PERCEPTION_SERIAL" ]] && perception_args+=(--serial "$PERCEPTION_SERIAL")
[[ -n "$PERCEPTION_VIDEO" ]] && perception_args+=(--video "'$PERCEPTION_VIDEO'")
[[ -n "$PERCEPTION_WIDTH" ]] && perception_args+=(--width "$PERCEPTION_WIDTH")
[[ -n "$PERCEPTION_HEIGHT" ]] && perception_args+=(--height "$PERCEPTION_HEIGHT")
[[ -n "$PERCEPTION_FPS" ]] && perception_args+=(--fps "$PERCEPTION_FPS")
PERCEPTION_CMD="$PERCEPTION_BIN ${perception_args[*]}"

# ---------------------------------------------------------------------------
# Pane environment -- see launch_rover_tmux.sh's long comment. Short version:
# tmux only copies this script's environment into a new session when it also
# has to start a new server, so with a server already running the panes would
# see ROVER_RUN_DIR unset and the run would scatter. Export explicitly.
# ---------------------------------------------------------------------------
PANE_ENV="export ROVER_RUN_DIR='$ROVER_RUN_DIR' PYTHONUNBUFFERED=1"

# run_logged <pane> <logname> <command>
# The command is passed single-quoted by every caller so that any $(...) or
# quoting inside it reaches the pane's shell intact and is evaluated there.
run_logged() {
    local pane="$1" name="$2" cmd="$3"
    tmux send-keys -t "$SESSION_NAME:$pane" \
        "stdbuf -oL -eL $cmd 2>&1 | tee -a '$ROVER_RUN_DIR/$name.log'" C-m
}

# ---------------------------------------------------------------------------
# Build the session: two panes side by side.
# ---------------------------------------------------------------------------
tmux new-session -d -s "$SESSION_NAME" -n "jetson"
tmux split-window -h -t "$SESSION_NAME:0"

sleep 0.5

WINDOW="$SESSION_NAME:0"
tmux set-option -w -t "$WINDOW" pane-border-status top
# `#{@name}` rather than `#{pane_title}`: an interactive shell rewrites its
# pane title on every prompt via an OSC escape, which clobbers anything set
# with `select-pane -T`. A tmux user option cannot be reached from inside the
# pane. See launch_rover_tmux.sh for the full note.
tmux set-option -w -t "$WINDOW" pane-border-format " [#{pane_index}] #{@name} "
tmux set-option -w -t "$WINDOW" pane-border-style fg=colour240
tmux set-option -w -t "$WINDOW" pane-active-border-style fg=colour51

# Pane 0: perception
tmux set-option -p -t "$SESSION_NAME:0.0" @name "Perception"
tmux select-pane -t "$SESSION_NAME:0.0" -T "Perception"
tmux send-keys   -t "$SESSION_NAME:0.0" "cd '$ROOT'" C-m
tmux send-keys   -t "$SESSION_NAME:0.0" "$PANE_ENV" C-m
tmux send-keys   -t "$SESSION_NAME:0.0" "clear && echo -e '\\e[1;36m>>> [1/1] PERCEPTION (D415 + lane detection) <<<\\e[0m' && sleep 1" C-m
run_logged 0.0 perception "$PERCEPTION_CMD"

# Pane 1: spare shell
tmux set-option -p -t "$SESSION_NAME:0.1" @name "Spare_Shell"
tmux select-pane -t "$SESSION_NAME:0.1" -T "Spare_Shell"
tmux send-keys   -t "$SESSION_NAME:0.1" "cd '$ROOT'" C-m
tmux send-keys   -t "$SESSION_NAME:0.1" "$PANE_ENV" C-m
tmux send-keys   -t "$SESSION_NAME:0.1" "clear && echo -e '\\e[1;90m>>> [SPARE] shell ready <<<\\e[0m'" C-m

tmux select-pane -t "$SESSION_NAME:0.0"

# Spelled out rather than `[[ ... ]] && tmux attach`, which would make the
# script exit 1 whenever SKIP_ATTACH=1 -- the failed test would become the
# script's final exit status, so a smoke check would report failure on success.
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
# tmux kill-session -t jetson
