# What remains, and what to worry about

Companion to `docs/STATUS_DONE.md`, which records what is built and the
evidence for it. Together the two are the authoritative status of branch `rs`;
`docs/RUST_REWRITE_PLAN.md` §13 points here rather than repeating it.

This file is deliberately the pessimistic half. It exists so that nobody has
to infer what is missing by reading a list of accomplishments and noticing an
absence.

**The one-line version, as of 2026-10-26: the whole stack runs on the rover's
hardware on a bench, and nothing has driven yet.** The first bench session
(`STATUS_DONE.md` §9) flashed both boards, passed the PHY gate, calibrated the
drivetrain, deployed the RPi and Jetson, and fixed four defects only hardware
could show. What is left is the part that moves: steering, motors, the
watchdog, the D5/D6 re-tune — §7 has the order — plus what the bench turned up
and did not close (§9). (The 2026-10-17 audit's software fixes are
`STATUS_DONE.md` §8.)

---

## 1. Hard blockers

Four things blocked a meaningful field run. After 2026-10-26, §1.1 is mostly
cleared and §1.2's bring-up is done; §1.3's re-tune and §1.4's missing
recording remain, and neither is a coding task.

### 1.1 The drivetrain is uncalibrated

> **Mostly resolved 2026-10-26.** Procedure A on the bench: ~1,396 ticks/rev at
> 4× on both wheels (`docs/HARDWARE.md` §3). `metres_per_tick = 0.0002813` is
> π × the measured 0.125 m diameter / 1396 — the rover was on a bench rig and
> could not be rolled, so Procedure B is still owed and remains authoritative.
> `track_width_m` is still `0.0`. The rest of this section describes the state
> before that session.

`ticks_per_rev`, `metres_per_tick` and `track_width_m` in `config/rover.toml`
are all `0.0`. There is **no metric speed anywhere in the system** until they
are measured. The estimator disables odometry rather than divide by zero, and
the speed PID works in ticks/sec so it is unaffected — but every `speed_mps`
in every log is meaningless, and acceptance criterion 6 cannot be attempted.

`docs/CALIBRATION.md` is the step-by-step for Procedures A and B — probe-rs
setup, which ST-Link is which board, the exact flash commands, the log line to
read; `RUST_REWRITE_PLAN.md` §2.6 is the reasoning behind them. About twenty
minutes with a tape measure. `docs/HARDWARE.md` §3 records the expected answer
(~1500 ticks/rev at this firmware's 4× decoding) so the run confirms or
contradicts something rather than producing a number with nothing to check it
against.

⚠️ Calibrate against **4×** decoding. A constant measured against the old 2×
firmware reads exactly double the true speed.

`rover-doctor`'s drivetrain check now reports GO on the calibrated config (it
failed loudly before, correctly, with no flag to skip it).

### 1.2 Flashing and first bring-up — done on the bench, 2026-10-26

Until 2026-10-26 nothing had been flashed, and every firmware claim was a claim
about a binary that compiled. That is no longer true: both boards run this
branch's firmware, pass POST, link at 100 Mb/s full duplex, and talk to the
RPi and the base station. The PHY gate (`RUST_REWRITE_PLAN.md` §13.4a) passed
on both. `STATUS_DONE.md` §9 has the evidence; `docs/CALIBRATION.md` §§1-3 has
the commands, including the `--speed 1000` this bench's ST-Links need.

What bring-up did **not** cover, because the rover never moved: the motors and
servo under PWM, the command watchdog firing on a lost link, the IWDG on a hung
task. Those are §6 criteria 3-4 and §7 steps 1-2.

For the next person flashing these boards: always pass `--probe` (two
ST-Links, unstable enumeration order) and `--speed 1000`; reflash the sensors
board with the field image after calibration work; and note that a fresh
flash resets the boards' uptime and `BoardDiagnostics` reset cause to
`software`.

### 1.3 D5 — the heading sign: **fixed in software, re-tune outstanding**

`RUST_REWRITE_PLAN.md` §13.3b **D5**. The detector's `theta` was the negative
of `d(cross_track)/d(distance)`, while `Ekf::correct_camera` and
`RoverState::at_lookahead` both assume it is positive.

**The sign is fixed.** `LaneDetector.detect` negates before publishing;
`cross_track_m`, `steer`, both consumers and both gains are unchanged, because
the model was right and the data was wrong.
`perception/tests/test_lane_sign_convention.py` is the regression test that
never existed. The parity suite and replay gate are unaffected.

**What is still open is the re-tune.** The fix is a **3.21× increase in
straight-line steering authority** (1.834 → 5.882 deg/deg), against gains that
have never run with a correctly-signed heading term. Start the first bench run
slow. `config/rover.toml` carries the arithmetic and `[0.312×, 1.0×]` as the
bracket to search. Take it together with D6.

The 60 seconds on blocks is still worth doing, but it now tests something
different: not the sign, which is settled in software, but the firmware's
`steer > 0` → physical-right mapping — the one link in the chain with no test
that has never run.

> Do not let the README's "sign convention is inverted on purpose" note be
> read as covering `heading_err_rad`. It never did, and that framing is what
> let D5 hide. `rover-msgs`' crate docs now say so explicitly.

### 1.4 Replay parity is synthetic only

`tools/replay` passes against a synthetic trace and a frozen oracle of the old
control law. That proves internal consistency and that the harness has teeth.
It does **not** prove field parity.

No recorded ROS 2 run exists in this repository and none ever did — re-verified
when the ROS 2 tree was removed. Producing a real baseline means checking out
`main` in the ROS 2 fallback repository, `RoboticsGG/almondmatcha` — this
repository's `origin` has no `main` branch at all — running the old stack, and
recording one. Acceptance criterion 2 stays open until then.

---

## 2. The risk register, reassessed

`RUST_REWRITE_PLAN.md` §11 listed ten risks at the start of the rewrite. Where
they actually stand:

| §11 Risk | Status |
|---|---|
| **PHY bring-up on F767ZI** | **Closed on the bench, 2026-10-26.** Both boards link at 100 Mb/s full duplex with zero symbol errors; straps read `MODE[2:0]=111`, `ANAR=0x01e1` (§3.1's failure mode did not occur). Not yet proven under motor noise or vibration. |
| **Control regression from the port** | **Partly closed.** Replay gate passes against a frozen oracle; the lane port is exact against the vendored original. Field parity unproven — see §1.4. |
| **Sign-convention inversion** | **Sign closed; gain re-tune open.** Worth remembering how it went wrong: the mitigation was "documented §10, unit-tested", and that documentation is exactly what let D5 hide — the prose asserted a convention nothing checked against the consuming model. See §1.3. The 2026-10-17 audit found a second one of the same kind inside the EKF (the curvature term of the process model had the wrong sign, and its tests had shared the error — plan §2.2), so the entry stays "closed" only for the two signs anyone has checked; the gyro's own sign is §3 item 8. |
| **Encoder constant wrong after 2×→4×** | **Closed.** Measured at 4×: ~1,396 ticks/rev on both wheels (`HARDWARE.md` §3). The expected ~1,500 was a recollection and was ~7 % high; the measurement disagreed with it, as intended. |
| **RTK bandwidth over 433 MHz** | Deferred to phase 2. Untouched. |
| **No command uplink when disconnected** | **Architecturally answered** — goals preload from `[mission]` and the base station is non-essential by design — but the LoRa link itself is phase 2. |
| **EKF tuning eats time** | **Open.** "Seed `R` from measured variance" still requires a recording that does not exist (see §1.4 for why none does). The other half — "log innovations from day one" — *is* done, via `EkfDebug`. |
| **Magnetometer disappoints** | Deferred by design, behind an off-by-default flag. Not a concern. |
| **Jetson Python bus drifts from Rust** | **Closed.** Golden-byte fixtures checked from both languages, both directions. |
| **No ROS 2 escape hatch** | Accepted. `main` is intact in `RoboticsGG/almondmatcha` (not this repository). Criterion 10 ("`main` still builds") has not been re-verified recently. |

**Five closed or mostly closed out of ten, including the largest** — the PHY,
closed by the first bench session. The rest close on a driving rover or a
radio link, not a bench. It remains a caution against reading a green test
suite as progress against this table.

The sign-convention entry is the one worth learning from: it was listed as
mitigated by documentation and unit tests, and both were true — the docs stated
the convention and the tests asserted it. Neither checked the *data* against
the *model*, so both passed while the system was wrong. A mitigation that
cannot fail is not a mitigation.

---

## 3. Hardware-verification debt

`RUST_REWRITE_PLAN.md` §13.4 holds the full list in risk order. In brief, and
none of it is closable from a desk. Items 1-3 closed on the 2026-10-26 bench
(`STATUS_DONE.md` §9.1); the numbering is kept so references stay valid.

1. ~~**LAN8742A PHY** against `GenericPhy`~~ — **passed on both boards**,
   2026-10-26. See §3.1.
2. ~~**I2C1 on PB8/PB9**~~ — **confirmed on both boards**: the LSM6DSV16X and
   the INA226 both answer.
3. ~~**Encoder sign convention**~~ — **verified by hand**: both wheels count up
   turning forward. The same check found the left/right **sides** swapped
   (plan §13.3b D8, fixed).
4. **Software EXTI decode under load** — could in principle miss edges while
   competing with UDP and I²C interrupts. Unmeasured.
5. **TIM1 left-motor PWM** — the only channel on an advanced-control timer.
6. **Watchdog end to end** — cannot be faked in a test.
7. **Every timing constant** — 200 ms / 300 ms / 500 ms are reasoned, not
   measured.
8. **Gyro z sign.** Since the 2026-10-17 audit the EKF depends explicitly on
   the gyro's z axis being **z-up, counter-clockwise positive**
   (`e_psi_dot = (gyro - b_g) + v*kappa`, plan §2.2; `rover-model`'s comments
   spell out the derivation). How the LSM6DSV16X sits on the IKS4A1 relative
   to the chassis has never been checked against that. Check on the bench:
   with the IMU feed visible, turn the rover counter-clockwise (seen from
   above) by hand and confirm z reads **positive**. A negative reading means
   every yaw-rate term is backwards; fix it at the source (the chassis
   firmware's IMU read), not in the estimator, because the model and the
   zero-rate update both assume z-up. The feed is published by the board
   straight to `control`, so reading it needs `rover-tap --as control
   --type ImuSample` on the RPi's address (`docs/CALIBRATION.md` §4) — the
   `[debug] mirror` cannot see it.
9. **Motor enable pins float from reset until `Motors::new`.** The four
   H-bridge enables (`PF12 PD15 PF13 PE9`) are floating inputs after reset;
   the firmware now constructs `Motors` (all enables low, 0 % duty) as the
   first thing in `main`, before Ethernet bring-up, but software cannot close
   the window between reset and that call. Hardware pull-downs on the four
   enable lines would. Worth looking at with a meter or scope at power-on
   before the motors are connected to anything that matters.
10. **Encoder decode runs in the executor, not an ISR.** The decode arms EXTI
    before sampling the pins, which closed the lost-edge window the audit
    found, but a *second* edge arriving before the task samples decodes as a
    diagonal jump and counts as zero. Under executor load (UDP, I2C) at high
    wheel speed, ticks could fall short. Check it in Procedure A: spin a
    wheel slowly and then fast, and compare the counts for the same ten
    turns — they should match. If they fall short, move the decode into the
    EXTI interrupt handler or onto a hardware timer. (At the recalled ~1500
    ticks/rev and 0.20 m/s the tick rate is only ~760/s, so this may never
    bite at cruise; it is the fast hand-spin and any burst of load that would
    show it.) **Partial evidence, 2026-10-26:** six 10-turn hand runs at
    irregular speeds (up to ~860 ticks/s in a single second) agreed to 0.07 %
    on one wheel — no sign of lost counts at hand speed. A fast spin and a
    loaded drive are still unchecked.

### 3.1 The `ANAR` / strap gap — the most likely way the PHY bites

`GenericPhy::phy_init` enables auto-negotiation but **never writes `ANAR`**
(register `0x04`), so what the PHY advertises comes entirely from the
`MODE[2:0]` straps latched at reset. If those straps read `100` or `101`, the
PHY advertises **100BASE-TX half duplex only** — the link comes up, `poll_link`
returns true, and the MAC stays configured for 100 full.

This is plausible on this board rather than theoretical: `MODE[2:0]` is
multiplexed onto `RXD0`/`RXD1`/`CRS_DV`, which are STM32 **PC4, PC5, PA7**.

Both boards now log `ANAR` and the decoded strap value once over `defmt` and
warn if 100-full is not advertised. Read-only — the bring-up sequence is
untouched. **Writing `ANAR` was deliberately not done**: that is a bring-up
change and should follow evidence from the bench, not precede it. Plan §13.4a
has the full table and the reasoning.

*(Found by evaluating the `eth-phy-lan87xx` crate, which was **not** adopted —
plan §13.4a says why — but whose documentation flagged this.)*

**Bench result, 2026-10-26: did not occur.** Both boards log
`ANAR=0x01e1 advertises 100BASE-TX full duplex; MODE[2:0]=111 (all capable,
auto-neg enabled)` and link at 100 full. `ANAR` stays unwritten.

### 3.2 What the self-test cannot tell you

The chassis PWM checks confirm a timer is configured, a duty register accepted
a write, and TIM1's `MOE` is set. **Nothing in the firmware can observe whether
a pin actually drove a waveform.** It is a configuration check and says so in
its own comment. A green POST is not proof the motors will turn.

---

## 4. Open defects

Narratives live in `RUST_REWRITE_PLAN.md` §13.3b. Status only:

| | Defect | State |
|---|---|---|
| D1 | `PeerId` addressed machines, not processes | **Done** — `[services]` in `config/rover.toml` |
| D2 | `GnssFix` could not say which receiver it came from | **Done** — `GnssSource` / `GnssFix.source` |
| D3 | Mission monitor navigated on the uncorrected receiver | **Done** — `select_navigation_fix`, tested |
| D4 | Base-station CSV contradicts `CSV_LOGGING.md` | **Half implemented.** Perception's `--csv` is opt-in as decided. `ground-station` creates its run directory lazily, so a base that receives nothing leaves nothing — but any run that receives anything logs, with no flag to decline. (Plan §13.3b's intro lists D1-D3 as implemented and D4 as the half-implemented one; this table agrees.) |
| D5 | `heading_err_rad`'s sign disagreed with its consumer | **Sign fixed; re-tune outstanding.** §1.3 |
| D6 | Camera measurement timestamped after detection | **Fixed 2026-10-26; re-tune with D5.** §5.1 |

---

## 5. Concerns with no risk-register entry

### 5.1 D6 — the camera pipeline's latency is invisible

**Fixed 2026-10-26** (plan §13.3b D6 has the original write-up and the fix).
`perception` now stamps `t_us` at capture — from the D415's own timestamp when
its domain is host-comparable — and publishes `capture_age_us` (capture to
publish, one clock, no sync needed) and `frames_dropped` with every
`LaneMeasurement`. `Ekf::correct_camera` takes the innovation against its
belief at capture time and ignores frames older than `lane_max_age_ms`.
`lane_detection.csv` logs both new figures, so the lag is now measurable from
a field run.

**Measured on the bench, 2026-10-26:** the D415 reports the `global_time`
domain, so frames are stamped from the sensor's own capture time, and
**capture-to-publish is a median 59 ms** (54-62 ms, p95 61 ms) with 0 dropped
frames over 617 frames at 29.8 fps — well under the ~100 ms that had been
reasoned, and under `lane_max_age_ms = 250`.

**Still open:**

- That figure is with **no lane in view** (the rover faces a bench). Detection
  on a real track may take longer; re-read `capture_age_ms` from the first
  field run.
- Compensation changes closed-loop behaviour, so it belongs in the D5
  re-tune. `lane_latency_compensation = false` restores the old behaviour for
  an A/B comparison.

### 5.2 Merging camera and lane detection was right — and was a trade

It removed a 2.76 MB serialize + localhost UDP + decode thirty times a second,
roughly **83 MB/s** of overhead between two nodes that only ever talked to each
other. That should stay.

But those were separate processes, so they **pipelined** — frame N+1 captured
while frame N was detected. `HARDWARE.md` §7 half-admits it: the stages sum to
153–163 ms against a 100–150 ms measured end-to-end, the gap attributed to
"presumed pipeline overlap". One process and one `while` loop means capture and
detect now serialize. The IPC and the overlap went together.

Probably still a large net win, since `wait_for_frames` usually returns an
already-queued frame rather than blocking. The camera half has now been measured
(§5.1: 59 ms capture-to-publish); **end to end, through the EKF and the
chassis, it has not** — reasoning, not a result.

Related, and closed with D6: `camera.py` now drains librealsense's queue to
the newest frame on every read, so a slow detector drops frames (counted in
`frames_dropped`) rather than accumulating latency, whatever the library's
default queue depth.

---

## 6. Acceptance criteria not yet met

Against `RUST_REWRITE_PLAN.md` §12. `docs/STATUS_DONE.md` covers the ones that
are met.

| # | Criterion | Blocked by |
|---|---|---|
| 2 | Replay reproduces recorded ROS 2 steering | §1.4 — no recording exists |
| 3 | Watchdog: Ethernet pulled, throttle ramps to zero | boards run (§1.2) — needs the motors powered and commanded; §7 step 2 |
| 4 | IWDG: hang the control task, board resets | boards run — needs a deliberately hung task; §7 step 2 |
| 5 | E-stop end to end with `cmd_seq` echoed | stack runs end to end — needs `ground-station` driving a mission; §7 step 3 |
| 6 | Commanded 0.20 m/s measured as 0.20 ± 0.02 | metric speed now exists (§1.1) — needs a rolling rover, and ideally Procedure B first |
| 7 | 100 Hz IMU at the estimator, latency measured | IMU reaches `rover-control` — rate and latency not yet measured; §5.1 for why latency is harder than it looks |
| 8 | EKF coasts a ≥3 s lane dropout without divergence | needs a driving rover and a real lane; and §9's speed-at-rest drift first |
| 9 | One full field circuit at parity, CSVs compared | everything above |
| 10 | `main` still builds and still drives | not re-verified recently |

---

## 7. What to do next, in order

Updated after the 2026-10-26 bench session; the rover is still on its rig.

1. **Steering and motors on blocks.** With the wheels still off the ground,
   command a small steer and a low throttle (`ground-station`, or a mission
   with a low speed limit) and confirm: steer > 0 turns the wheels **right**
   (§1.3), both motors spin **forward**, and the left motor works at all
   (TIM1, §3 item 5). Read `speed_loop_debug.csv` for the measured tick rates.
2. **Watchdog and IWDG on blocks** (§6 criteria 3-4): with the wheels turning
   slowly, pull the chassis Ethernet cable — the throttle must ramp to zero
   over 300 ms and the board must report `watchdog_tripped`.
3. **The gyro z sign** (§3 item 8): turn the rover counter-clockwise by hand,
   z must read positive. Sixty seconds.
4. **Fix the speed estimate at rest** (§9) before trusting `RoverState` or
   `rover-doctor`'s estimator check.
5. **Re-tune for D5 and D6 together**, on hardware, once it can roll. D5's
   fix alone is a 3.21× authority increase.
6. **Procedure B** once the rover can roll, to confirm `metres_per_tick`.
7. **Record a real baseline on `roboticsgg-almondmatcha/main`** (§1.4) if
   criterion 2 is to ever close.

`docs/FIELD_TEST.md` is the end-to-end procedure; its deployment section now
records how the RPi and Jetson were actually deployed.

---

## 8. Smaller open items, none needing a bench to start

Left over from the 2026-10-17 audit (`STATUS_DONE.md` §8). None blocks a first
bench session; all are worth knowing before trusting a result.

- **Synthetic replay traces are not kinematically consistent.** `e_y` and
  `e_psi` in the synthetic truth are independent curves, and the synthetic
  gyro lacks the `e_psi_dot` term. That is why the dropout scenario's RMSE
  rose from 0.76 to 1.68 degrees when the EKF's curvature sign was corrected
  (it still passes the 3.0 degree gate): the coasting accuracy of that
  scenario is not a meaningful number. Fixing the generator would make the
  gate's coasting half worth reading; it does not touch §1.4's real gap, which
  is the missing recording.
- **I2C bus recovery is not implemented.** The firmware bounds the IMU reset
  loop and runs I2C with a 5 ms timeout, but cannot clock out a stuck slave
  (the 9-clock SCL recovery): the HAL does not hand the pins back. A slave
  holding SDA low survives an MCU reset, so the likely symptom is a POST or
  IMU failure that only a power cycle clears. Unobserved, since nothing has
  run.
- **CSV rows can be lost on kill.** Writers flush about once a second with
  `sync_data` and no binary installs a signal handler, so Ctrl-C, `tmux
  kill-session` or a power cut can lose up to about 1 s of rows (the final
  flush only runs when a logger is dropped normally). Waiting a couple of
  seconds before stopping a process, as `docs/FIELD_TEST.md` §4 now says,
  keeps the loss to nothing in practice.
- **`TelemetryLite` and the LoRa links are unimplemented**, by design (phase 2,
  plan §6, §13.3 item 7): there is no `LoraSerialLink` and no packed 18-byte
  frame, so nothing yet exercises the one-way-link assumptions the command
  protocol was built around.
- **`tools/requirements.txt` is unpinned** (`pyrealsense2`, `opencv-python`,
  `numpy`, no versions), so a rebuilt Jetson venv for the D415 capture utility
  is not reproducible. `perception/pyproject.toml` has lower bounds only.

The toolchain, which this list used to include, is pinned: `rust-toolchain.toml`
says `1.99.1`, and the size figures in `STATUS_DONE.md` are measured under it.

---

## 9. Found on the 2026-10-26 bench, still open

`STATUS_DONE.md` §9 has what the session verified and fixed. These it found and
did not close:

- **The speed state is unobservable at rest.** While the rover is stationary
  `Ekf::correct_wheel_sensors` runs only the zero-rate gyro-bias update, never
  an odometry update, so `v` random-walks: `RoverState.speed_mps` read 0.45 m/s
  with the wheels still, its variance growing. Within minutes the health word
  gained `ESTIMATOR_DIVERGED`. Drive control is unaffected (the speed PID uses
  ticks), but `speed_mps` and `rover-doctor`'s "estimator converged" check
  mean nothing at rest until a zero-speed update joins the zero-rate one.
  (Bench lane "detections" of clutter also feed the filter; that part goes
  away on a real track.)
- **The Spresense backup receiver has no fix.** It is alive and
  `rover-navigation` parses its JSON correctly, but it reports 0-1 satellites
  and `"fix":false` where the u-blox sees 12 — so `BACKUP_GNSS_STALE` stays
  set. Most likely antenna placement; recheck with sky view. **Opening its port
  resets the board** (DTR/RTS on open), so every `rover-navigation` restart
  cold-starts it and costs minutes to first fix.
- **No RTCM source**, so the u-blox stops at DGPS (h_acc ~2.9 m) and
  `rover-doctor`'s RTK check is NO-GO. Expected until the 433 MHz link exists.
- **Procedure B and `track_width_m`.** `metres_per_tick` is diameter-derived
  (§1.1).
- **`python3-venv` is missing** on the base PC and the Jetson (`ensurepip`
  unavailable). Worked around without sudo — `FIELD_TEST.md` has how — but
  `sudo apt install python3.10-venv` would make the documented commands work
  as written.
- **ST-Link firmware `V2J39M27`** on both boards needs `--speed 1000` for
  probe-rs. Updating it (STSW-LINK007) would probably remove that.

