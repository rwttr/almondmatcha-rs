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

## 0. Before you leave the bench

These are not warm-up steps. Each one is a blocker: if it is not cleared, the
run either cannot produce usable data or cannot safely happen at all. The
current status of all of them is `docs/STATUS_OPEN.md` §1.

### 0.1 Drivetrain calibration — blocks everything metric

`ticks_per_rev`, `metres_per_tick` and `track_width_m` in `config/rover.toml`
are all `0.0`. Until they are measured there is **no metric speed anywhere in
the system**: the estimator disables odometry rather than dividing by zero,
and every `speed_mps` in every log is meaningless.

Run both procedures in `docs/RUST_REWRITE_PLAN.md` §2.6 — about twenty minutes
with a tape measure. Procedure A gives ticks per revolution; Procedure B gives
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

```sh
cd firmware/chassis && cargo run --release    # probe-rs, via the on-board ST-LINK
cd firmware/sensors && cargo run --release
```

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

### On the RPi (192.168.1.1) — three processes, three terminals

```sh
cargo run --release -p rover-control                      # estimate → guide → actuate
cargo run --release -p rover-navigation                   # GNSS ×2, mission state machine
cargo run --release -p rover-telemetry                    # CSV logging + 5 Hz Telemetry feed
```

The RPi runs three processes that each bind their own UDP port. That is design
defect D1, and it is why `[services]` maps a *process* to a `host:port` rather
than a machine to a port.

`rover-navigation` takes `--ublox-port` and `--spresense-port` if udev has
enumerated the receivers somewhere other than the defaults. Check before you
assume: `/dev/ttyACM0` is an enumeration order, not a stable identity.

`rover-telemetry` takes `--runs-dir` (default `runs`). See §5.

### On the Jetson (192.168.1.5)

```sh
python -m rover_perception.main
```

Add `--preview` only on the bench — it opens a window and costs frame rate.
`--video PATH` replays a file instead of the D415, which is how you test the
pipeline without a camera. `--csv PATH` logs every processed frame and is
**off by default**; turn it on for a run you intend to analyse frame by frame.

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
rsync -av yupi@192.168.1.5:~/almondmatcha/…       ./runs-jetson/   # if --csv was used
```

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
