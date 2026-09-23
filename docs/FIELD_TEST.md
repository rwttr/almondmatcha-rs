# Running a field test

How to take the rover from a cold bench to a logged run on the field, and how
to get the data back afterwards.

This is an operator's document. It assumes the hardware described in
`docs/HARDWARE.md` §1 and the software on branch `rs`. Where a step can only
be done once and can ruin the run if skipped, it says so.

**The rover must be able to complete a mission with the base station powered
off.** The base is observability and optional override, never a dependency.
If a procedure here reads as though the base is load-bearing, that is a bug in
the procedure.

---

## Build everything first

There are **four separate build products**, and `cargo build` at the repo root
produces only the first of them. Nothing in §0 works until all four exist on
the machine that needs them.

| # | Product | Built where | Command | Artifacts |
|---|---|---|---|---|
| 1 | Host binaries | RPi and base station | `cargo build --release` | `target/release/<name>` |
| 2 | Chassis firmware | any machine with the ARM target | `cd firmware/chassis && cargo build --release` | `firmware/chassis/target/thumbv7em-none-eabihf/release/chassis-fw` |
| 3 | Sensors firmware | any machine with the ARM target | `cd firmware/sensors && cargo build --release` | `firmware/sensors/target/thumbv7em-none-eabihf/release/sensors-fw` |
| 4 | Perception package | Jetson | `cd perception && python3 -m venv .venv && .venv/bin/pip install -e .` | `perception/.venv/bin/rover-perception` |

⚠️ **The two firmware crates are deliberately not workspace members.** Each has
an empty `[workspace]` table in its own `Cargo.toml`, because a Cargo workspace
cannot hold two different default targets and these build for
`thumbv7em-none-eabihf` while everything else builds for the host. **`cargo
build --release` at the repo root does not build them, and never will** — you
must `cd` into each crate. This catches people every time.

`rust-toolchain.toml` already pins the toolchain and lists
`thumbv7em-none-eabihf` under `targets`, so rustup installs the cross target
on its own the first time you build inside the repo.

### Which binaries each machine actually needs

| Machine | Binaries |
|---|---|
| RPi `192.168.1.1` | `rover-control`, `rover-navigation`, `rover-telemetry` |
| Base `192.168.1.10` | `ground-station`, `rover-doctor`, `rover-tap` |
| Jetson `192.168.1.5` | the `rover-perception` console script (product 4) |

`cargo build --release` builds all twelve crates; if you would rather not,
`cargo build --release -p rover-control -p rover-navigation -p rover-telemetry`
is what the RPi launch script tells you to run when it finds a binary missing.

### Getting the code onto the machines

**There is no established deployment story in this repo, and this document is
not going to invent one.** What is true today:

- The three rover machines are on a closed `192.168.1.0/24` with **no internet**
  (`docs/HARDWARE.md` §1), so `cargo build` on those machines only works if the
  Cargo registry cache is already warm or a crates mirror is reachable. All
  three `Cargo.lock` files are committed, so the dependency set is at least
  pinned and reproducible.
- **Binaries cannot simply be copied from a development Mac.** The RPi and the
  Jetson are `aarch64-unknown-linux-gnu`; an Apple-silicon laptop builds
  `aarch64-apple-darwin`. Same word size, different platform — the binary will
  not run. Cross-compiling to the Pi is possible but is not set up here.
- The realistic path today is **a git clone on each machine, built there**,
  with the network brought up long enough to fetch crates, or a pre-warmed
  `~/.cargo` copied across.

If you establish a better answer during a real bring-up, record it here — this
section is a description of an unsolved problem, not a procedure.

---

## 0. Before you leave the bench

These are not warm-up steps. Each one is a blocker: if it is not cleared, the
run either cannot produce usable data or cannot safely happen at all. The
current status of all of them is `docs/STATUS_OPEN.md` §1.

### 0.1 Drivetrain calibration — blocks everything metric

`ticks_per_rev`, `metres_per_tick` and `track_width_m` in `config/rover.toml`
are all `0.0`. Until they are measured there is **no metric speed anywhere in
the system**: the estimator disables odometry rather than dividing by zero,
and every `speed_mps` in every log is meaningless.

Run both procedures in `docs/CALIBRATION.md` — about twenty minutes with a
tape measure (`docs/RUST_REWRITE_PLAN.md` §2.6 is the reasoning behind them,
not the steps). Procedure A gives ticks per revolution; Procedure B gives
metres per tick on the actual field surface, and B is the one that goes into
production.

> ⚠️ **Calibrate against the decoding mode the firmware actually uses.** The
> `sensors` firmware does 4× decoding. The old mbed firmware did 2×. A
> constant measured against the wrong one makes the rover run at half or
> double the commanded speed. Record which mode the number belongs to in
> `config/rover.toml` next to the value.

**Expect roughly 15,000 counts for ten hand turns** (~1500 ticks/rev at this
firmware's 4× decoding). That expectation comes from a recollection of the
ROS 2 system, not a measurement — `docs/HARDWARE.md` §3 records where it came
from and how much to trust it. Use it as a check, not as a value to type in:

- ~15,000 → decoding and wiring confirmed, take the measured number.
- ~7,500 → the board is still decoding at 2×. Stop and fix that first; this
  is exactly the failure the warning above describes, and you have caught it
  in one minute instead of after a run at double the intended speed.
- anything else → find out why before driving.

`rover-doctor` (§3) fails loudly while this is outstanding. That failure is
correct. Do not skip it.

### 0.2 Servo direction, and the D5 re-tune — 60 seconds on blocks

**The D5 sign defect is fixed** (`docs/RUST_REWRITE_PLAN.md` §13.3b): the lane
detector's `theta` was the negative of `d(cross_track)/d(distance)` while both
consumers assumed positive, and `LaneDetector.detect` now negates before
publishing. That is settled in software and regression-tested.

Two things still need you, on blocks, with the wheels clear of the ground:

**Check the servo direction.** This is the one link in the steering chain with
no test that has never run — the firmware's `steer > 0` → physically right
mapping.

1. Bring up the rover normally (§2) with the drive wheels jacked up.
2. Show the camera a line angled clearly **to the right**.
3. Watch the steering servo. **It should turn right.**

If it turns left, stop. That is a firmware-level inversion in
`motor.rs`'s `steer_dir`/`SERVO_CENTER_DEG` mapping, not a repeat of D5, and
nothing downstream will be trustworthy until it is understood.

**Then expect more steering than the ROS 2 rover gave you.** The D5 fix is a
**3.21× increase in straight-line steering authority** (1.834 → 5.882 deg/deg)
against gains that have never run with a correctly-signed heading term.

- Start slow — `speed 20` — and stay on blocks for the first pass.
- If it oscillates, scale **both** `k_lat` and `k_head` down together.
  `config/rover.toml` carries the arithmetic; `[0.312×, 1.0×]` is the bracket
  to search, not a recommendation to jump to either end.
- Expect the largest change where `cross_track` is small and heading error is
  not — at line crossings. That band used to steer the wrong way.

### 0.3 Flashing

Nothing in `firmware/` has ever met silicon. First flash is a bench activity,
not a field one — do it before you pack. `docs/RUST_REWRITE_PLAN.md` §13.4 has
the full hardware-verification debt list in risk order; the Ethernet PHY is
the hard gate, and §13.4a explains what the firmware can and cannot tell you
about it.

`docs/CALIBRATION.md` §§1–3 is the full procedure — probe-rs install, the udev
rule, identifying the two boards by ST-Link serial, and the exact flash
commands. Do not duplicate it from memory; this section covers only what is
specific to a **field departure** rather than a bench calibration session.

**Both ST-Links are on USB at once, so always pass `--probe`.** Without it
`probe-rs` picks whichever board it enumerates first, and that ordering is not
stable across replugs — you will eventually flash sensors firmware onto the
chassis board and spend an hour on it.

| Board | IP | ST-Link serial |
|---|---|---|
| Sensors | `192.168.1.6` | `066DFF3932504E3043014542` |
| Chassis | `192.168.1.2` | `066AFF3932504E3043101915` |

The chassis serial comes from the old OpenOCD flashing aliases, not from a
`probe-rs list` capture — `docs/CALIBRATION.md` §1 has the provenance. If a
flash lands on the wrong board, re-capture both with `probe-rs list` and
correct §1's table.

```sh
cd firmware/sensors && cargo build --release
probe-rs run --chip STM32F767ZITx --probe 0483:374b:066DFF3932504E3043014542 \
    target/thumbv7em-none-eabihf/release/sensors-fw

cd ../chassis && cargo build --release
probe-rs run --chip STM32F767ZITx --probe 0483:374b:066AFF3932504E3043101915 \
    target/thumbv7em-none-eabihf/release/chassis-fw
```

⚠️ **Flash the sensors board WITHOUT `--features calibration`.** That feature
adds a 1 Hz `defmt` tick readout which exists for `docs/CALIBRATION.md`'s
Procedure A and is pure noise in a field run. If you have just come from a
calibration session the board is still carrying the calibration image —
reflash it with the plain `cargo build --release` above. `docs/CALIBRATION.md`
§3.2 covers this.

**Watch each board's `defmt` output before you unplug the cable.** The boot
sequence tells you three things worth reading:

- **The POST result** — a board that fails its power-on self-test still boots
  and still publishes, degraded and saying so.
- **The reset cause** — an unexpected watchdog or brown-out reset here is a
  finding, not noise.
- ⚠️ **The PHY strap warning.** Both boards log `ANAR` and the decoded
  `MODE[2:0]` strap once, and warn if 100BASE-TX full duplex is not being
  advertised. **If that warning fires, stop and read
  `docs/RUST_REWRITE_PLAN.md` §13.4a** — the link will come up and `poll_link`
  will return true regardless, so this is the only moment you get told. It is
  a decision point about whether to write `ANAR` before auto-negotiation, not
  a nuisance message to scroll past.

Both boards should reach "link up" and start publishing. A board that fails
its power-on self-test still boots and still publishes — degraded, and saying
so. That is by design: a rover that refuses to start because its power monitor
died is worse than one that drives without power telemetry. `rover-doctor`
and `board_diagnostics.csv` are where you find out.

### 0.4 Pack list

Tape measure, chalk or marking tape, wheel chocks or blocks, the switch and
its power, laptop with the base station build, and a way to retrieve SD/eMMC
data in the field if the network is the thing that fails.

---

## 1. Physical setup and power order

All five machines sit on one gigabit switch, `192.168.1.0/24`, static
addresses, no DHCP. See `docs/HARDWARE.md` §1 for the table.

Power in this order:

1. **Switch** — first, so every host finds its link on boot.
2. **RPi** (192.168.1.1) and **Jetson** (192.168.1.5).
3. **Both NUCLEOs** (chassis 192.168.1.2, sensors 192.168.1.6).
4. **Base station** (192.168.1.10) — last, and optional.

**Keep the drive wheels off the ground until §4.** The chassis firmware starts
with the command watchdog already tripped, so an unattended board sits at zero
throttle rather than whatever the pins powered up as — but that is a safety
net, not a substitute for blocks.

Check the LAN before anything else:

```sh
for ip in 192.168.1.1 192.168.1.2 192.168.1.5 192.168.1.6 192.168.1.10; do
  ping -c1 -W1 $ip >/dev/null && echo "$ip up" || echo "$ip DOWN"
done
```

The two NUCLEOs negotiate 100 Mbit/s full duplex. If one comes up at 10 Mbit/s
half duplex, everything will still appear to work and the fault will not show
until it degrades into packet loss — this is the specific silent failure the
board diagnostics exist to catch, and `rover-doctor` checks for it explicitly.

---

## 2. Launch order

Rover first, base last. Each command below assumes you are in the repository
root on that machine; all of them default to `config/rover.toml`.

### On the RPi (192.168.1.1) — one tmux script

```sh
./tools/launch_rover_tmux.sh
```

That is the normal path. It replaces `ws_rpi/launch_rover_tmux.sh` from the
ROS 2 tree and behaves the same way: one tmux session named `rover`, a titled
pane per process, and one run directory for the whole launch.

| Pane | Process |
|---|---|
| 0 | `rover-control` — estimate → guide → actuate |
| 1 | `rover-navigation` — GNSS ×2, mission state machine |
| 2 | `rover-telemetry` — CSV logging + 5 Hz `Telemetry` feed |
| 3 | spare shell — for `rover-tap`, `ls runs/` |

The RPi runs three processes that each bind their own UDP port. That is design
defect D1, and it is why `[services]` maps a *process* to a `host:port` rather
than a machine to a port. Do not collapse them back into one.

What the script does that matters:

- **One shared run directory.** It allocates `runs/run_NNN_<stamp>/` once and
  exports it as `ROVER_RUN_DIR`, which `rover-runs`' `RunDir::resolve` already
  prefers over allocating its own. Without this the three processes each pick
  their own directory and a single launch scatters across three. It exports
  the variable *into each pane explicitly* rather than relying on inheritance,
  because tmux only passes the caller's environment through when it also has
  to start a new server — with a server already running, the panes would see
  it unset.
- **Every pane is teed to `$ROVER_RUN_DIR/<name>.log`**, so a process that
  dies at startup leaves its reason on disk instead of only in a scrollback
  you are about to kill.
- ⚠️ **It sets `RUST_LOG=info`.** These binaries use `env_logger`, which is
  **error-only when `RUST_LOG` is unset** — run them bare and the panes print
  nothing and look hung. Override with `RUST_LOG=debug ./tools/launch_rover_tmux.sh`.
- **It refuses to start if the release binaries are missing**, naming the
  build command, rather than beginning a multi-minute compile in a field.

```text
Ctrl+b d                     detach, leaving everything running
tmux attach -t rover         reattach
tmux kill-session -t rover   stop the run
SKIP_ATTACH=1 ./tools/...    build the session without attaching
```

Serial port overrides go through the environment:
`ROVER_UBLOX_PORT`, `ROVER_UBLOX_BAUD`, `ROVER_SPRESENSE_PORT`,
`ROVER_SPRESENSE_BAUD`. Check before you assume — `/dev/ttyACM0` is an
enumeration order, not a stable identity.

**Running a process by hand** (for debugging one of them in isolation):

```sh
./target/release/rover-control config/rover.toml          # ⚠️ positional
./target/release/rover-navigation --config config/rover.toml
./target/release/rover-telemetry  --config config/rover.toml --runs-dir runs
```

⚠️ **`rover-control` takes its config as a positional argument**, not
`--config`, unlike the other two. It reads `std::env::args().nth(1)`. Passing
it `--config config/rover.toml` makes it try to load a file literally named
`--config` and exit. This inconsistency is real; the launch script handles it,
a hand-typed command will not.

Set `RUST_LOG=info` yourself if you run one by hand, for the reason above.

### On the Jetson (192.168.1.5) — one tmux script

```sh
./tools/launch_jetson_tmux.sh
```

Session `jetson`: perception in pane 0, a spare shell in pane 1. Same run
directory convention, same tee'd logs, same `SKIP_ATTACH=1`. It allocates its
own `run_NNN_<stamp>/` on the Jetson's own filesystem — the two machines have
separate disks, and §5 is where the halves get reassembled.

Field defaults are CSV logging **on** (a run you cannot analyse afterwards was
a wasted drive), no preview window, and the throttled heartbeat rather than
per-frame logging. Overrides, all environment variables:

| Variable | Effect |
|---|---|
| `PERCEPTION_PREVIEW=1` | Debug preview window. Bench only — it costs frame rate and fails on a headless box. |
| `PERCEPTION_VIDEO=PATH` | Replay a video file instead of the D415. How you test without a camera. |
| `PERCEPTION_SERIAL=...` | Pick a specific D415 by serial. |
| `PERCEPTION_WIDTH` / `_HEIGHT` / `_FPS` | Capture overrides. |
| `PERCEPTION_CSV=PATH` | Move the frame log; defaults to `$ROVER_RUN_DIR/lane_detection.csv`. |
| `PERCEPTION_VERBOSE=1` | Log every frame rather than a heartbeat. |

By hand, if you need it:

```sh
perception/.venv/bin/rover-perception --config config/rover.toml --csv runs/lane.csv
```

The console script exists only after `pip install -e .` (build product 4). If
the package is merely importable, `perception/.venv/bin/python -m
rover_perception.main` takes the same flags.

### On the base station (192.168.1.10) — optional

```sh
cargo run --release -p ground-station
```

Operator commands are whole words followed by Enter, not single keystrokes:

| Command | Alias | Effect |
|---|---|---|
| `estop` | `e` | E-stop. Retransmitted every second until acknowledged. |
| `clearestop` | `ce` | Explicitly release the E-stop latch. |
| `cancel` | `c` | Cancel the mission. **Does not touch the E-stop latch.** |
| `goal <lat> <lon>` | `g` | Set a goal. |
| `speed <pct>` | `s` | Speed limit, 0–100. |
| `nop` | | Keep-alive. |

`clearestop` is deliberately a separate word from `cancel`. Cancelling a
mission is not obviously "please let me drive again", and it no longer means
that.

> There is no single-keypress E-stop: `estop` needs an Enter press. The real
> guarantee is not typing speed — it is that the command retransmits every
> second until the rover acknowledges it.
>
> **There is also no documented physical E-stop on this rover.** The two
> automatic layers in `docs/HARDWARE.md` §6 — the 200 ms command watchdog and
> the 500 ms IWDG — catch a dead peer and dead firmware respectively, and
> neither is a substitute for a human being able to stop a moving vehicle.
> Cutting motor power is the fallback. Know where that switch is before you
> drive, and keep someone on it.

---

## 3. Preflight — `rover-doctor`

Run this on the base PC after everything is up and before the wheels touch the
ground.

```sh
cargo run --release -p rover-doctor
```

Binds as `base`, listens ten seconds (`--listen-secs` — long enough for more
than one period of every feed, including the boards' 1 Hz heartbeat), prints
one line per check, exits non-zero on NO-GO so it can gate a launch script.

Checks: every service reachable; both boards passed POST; no abnormal reset
causes; both links at 100 Mbit/s full duplex with no symbol errors; no health
bits; **drivetrain calibrated**; RTK fix quality; lane detection live;
estimator converged.

Reading the output:

- **`NOT SEEN` ≠ `NO-GO`.** "Never heard from" usually means a process is down
  or a cable is out; "heard, and bad" means a real fault. Different actions.
  Both exit non-zero.
- **It never reports GO on missing data.** Absence is not evidence of health.

A NO-GO on drivetrain calibration is expected until §0.1 is done. No flag
skips it.

---

## 4. The drive

Escalate. Do not start with the full circuit — this is step 13 of the work
breakdown, and the point is to be able to bisect a regression, which you
cannot do if everything changes at once.

1. **On blocks, wheels clear.** Confirm the servo turns the right way (§0.2)
   and follows the lane. Confirm the command watchdog trips when you kill
   `ground-station`, and that the motors ramp to zero rather than stepping.
2. **Straight line, walking pace.** `speed 20`. Watch `cross_track_m` in the
   base display, not the rover.
3. **Gentle curve.** The first place the heading term does real work.
4. **Full circuit.**

Abort at the first surprise and look at the logs before repeating. A run you
do not understand is worth less than no run.

Stop the run cleanly — `cancel`, then stop `rover-telemetry` — rather than
cutting power. Files are flushed on close.

---

## 5. Where the data goes

Two independent captures, same `run_NNN_<stamp>/` convention on both machines,
so a pair from one session matches up by eye. The run-directory logic lives in
the `rover-runs` crate and is shared, not duplicated.

**On the rover** (`rover-telemetry`, `--runs-dir`, default `runs`):

```
runs/run_001_20260920_143052/
    rover_state.csv     chassis_status.csv    board_diagnostics.csv
    mission_status.csv  power.csv             speed_loop_debug.csv
    rtk_gnss.csv        backup_gnss.csv
```

**On the base** (`ground-station`, same `--runs-dir` flag — the old
`--log-file` is gone): `ground_station_telemetry.csv`.

The base capture is **not** redundant. It records what the operator actually
saw, including link gaps the rover's own log cannot show because the rover was
still happily writing through them. Diagnosing a comms problem, the difference
between the two logs *is* the finding.

> **Filenames are branch-`rs`, not ROS 2.** `docs/CSV_LOGGING.md` documents the
> old set (`chassis_imu.csv`, `chassis_sensors.csv`, …) and is historical here
> except where names coincide — read column meanings there, filenames from
> above. There is no `chassis_imu.csv`: `ImuSample` routes only to `control`,
> so raw IMU is not captured on this branch.

**Files are created on first write, not at startup.** A missing CSV is
positive evidence that the feed never delivered a sample; a run that received
nothing leaves nothing and consumes no run number. Do not "fix" this by
pre-creating headers — an earlier version did, and it made a dead feed
indistinguishable from a quiet one.

**Read `board_diagnostics.csv` first when something felt wrong.** POST results,
reset cause and link state for both boards, decoded to names — a row saying
`IWDG` is worth far more at 2 a.m. than one saying `4`. `IWDG` means the
firmware hung long enough for the independent watchdog to fire: a firmware bug,
not a field condition, never noise.

### Retrieving it

```sh
rsync -av curry@192.168.1.1:~/almondmatcha/runs/  ./runs-rover/
rsync -av yupi@192.168.1.5:~/almondmatcha/runs/   ./runs-jetson/
```

⚠️ **Check the directory name on those two machines before trusting these
literally.** They hardcode `~/almondmatcha`, but this repository was renamed
to `almondmatcha-rs`, and a fresh `git clone` on the RPi or the Jetson lands
in `~/almondmatcha-rs/` instead. Nobody has verified what is actually on those
hosts' filesystems since the rename, so these two lines are a best guess, not
a checked fact. (The launch scripts themselves are immune to this: they derive
the repo root from their own location rather than assuming a path. The run
directory is always `<repo root>/runs/` on each machine — the launch script
prints its absolute path at startup, which is the authoritative answer.)

Pull before powering anything down. No video is recorded; if it is ever added
back, ~83 MB/s at 1280×720/30 fills the Jetson's 128 GB in ~26 minutes
(`HARDWARE.md` §8).

### Watching live instead

```sh
cargo run --release -p rover-tap -- --as telemetry --hz
```

Routing is unicast fan-out, so a tap sees only what is addressed to the service
it binds as. `[debug] mirror` + `--mirror` shows the whole bus — **leave the
mirror empty for real field runs** so nothing depends on a laptop being there.

---

## 6. After the run

Record, in whatever the project's run log is:

- Run number and timestamp on **both** machines, and the surface and weather.
- The commit hash on each host. `git rev-parse --short HEAD`.
- Whether §0.1 calibration was current, and which decoding mode it belongs to.
- Any `NOT SEEN` or NO-GO from `rover-doctor`, even ones you went ahead past.
- Anything surprising, while you still remember it.

Then check `board_diagnostics.csv` from both boards even if the run felt
clean. A board that rebooted mid-run looks, from the RPi's side, exactly like
a brief link drop — the sequence numbers restart and the feeds come back.
The reset cause is the only thing that distinguishes "the watchdog fired" from
"somebody nudged the USB cable", and by the time anyone reads the log the
event is long over.

---

## Known-open, as of this document

Kept short on purpose; `docs/STATUS_OPEN.md` is authoritative and this list
will rot before that one does.

1. Drivetrain calibration is `0.0` (§0.1).
2. Nothing has been flashed (§0.3); the LAN8742A PHY is the hard gate.
3. Servo direction unverified, and the D5 re-tune (§0.2). The sign itself is
   fixed.
4. Replay parity is against a synthetic trace only. No recorded ROS 2 run
   exists in this repository and none ever did; producing a real baseline
   means checking out `main` in the ROS 2 fallback repository,
   `RoboticsGG/almondmatcha` — this repository's `origin` has no `main`.
