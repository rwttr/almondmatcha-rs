# What remains, and what to worry about

The pessimistic half of the status split: **only what is still open.** Closed
items move to [`STATUS_DONE.md`](STATUS_DONE.md) (§10 lists what moved from
here). `docs/RUST_REWRITE_PLAN.md` §13 points here rather than repeating it.

**As of 2026-10-26: the whole stack runs on the rover's hardware on a bench,
and nothing has moved yet.** Steering, motors, the watchdog and the D5/D6
re-tune are what is left — §7 has the order — plus what the bench turned up
and did not close (§9).

---

## 1. Hard blockers

### 1.1 The drivetrain — Procedure B still owed

`ticks_per_rev = 1396` is measured (Procedure A, `docs/HARDWARE.md` §3), but
`metres_per_tick = 0.0002813` is π × the 0.125 m wheel / 1396: the rover was on
a bench rig and could not be rolled. A diameter omits tyre compression and slip
(typically a few percent), so **run Procedure B** (`docs/CALIBRATION.md`;
reasoning in `RUST_REWRITE_PLAN.md` §2.6) before a run that depends on absolute
speed — it remains the authoritative number. `track_width_m` is still `0.0`
(optional: a second yaw-rate estimate). Calibrate against **4×** decoding: a
constant from the old 2× firmware reads exactly double.

### 1.2 Nothing has moved yet

Both boards run and pass POST (`STATUS_DONE.md` §9), but the rover never left
its rig and was never commanded. Unexercised: the motors and servo under PWM,
TIM1 (§3 item 3), the command watchdog on a lost link, the IWDG on a hung task,
and anything that rolls. §6 criteria 3-8 all wait on this. When flashing,
always pass `--probe` and `--speed 1000` (`docs/CALIBRATION.md` §3.1).

### 1.3 The D5/D6 re-tune

The D5 sign fix (plan §13.3b) is a **3.21× increase in straight-line steering
authority** (1.834 → 5.882 deg/deg) against gains that never ran with a
correctly signed heading term; D6's latency compensation changes closed-loop
behaviour too. Re-tune both together, on hardware, starting slow:
`config/rover.toml` carries the arithmetic and `[0.312×, 1.0×]` as the bracket,
and `lane_latency_compensation = false` gives the A/B. The 60 s check on blocks
now tests only the firmware's `steer > 0` → physical-right mapping, the one
link with no test.

> The README's "sign convention is inverted on purpose" note never covered
> `heading_err_rad`; that framing is what let D5 hide.

### 1.4 Replay parity is synthetic only

`tools/replay` passes against a synthetic trace and a frozen oracle of the old
law — internal consistency and a harness with teeth, **not** field parity. No
recorded ROS 2 run exists in this repository and none ever did. A real baseline
means running the old stack on `main` in `RoboticsGG/almondmatcha` (this
repository's `origin` has no `main`) and recording one. Criterion 2 stays open
until then.

---

## 2. The risk register — still open

From `RUST_REWRITE_PLAN.md` §11's ten; the closed ones are in `STATUS_DONE.md`
§10.

| §11 Risk | Status |
|---|---|
| **Control regression from the port** | **Partly closed.** Replay passes against a frozen oracle; the lane port is exact against the vendored original. Field parity unproven — §1.4. |
| **Sign-convention inversion** | **Sign closed; gain re-tune open** (§1.3). The mitigation was "documented, unit-tested", and both were true while the system was wrong — the prose asserted a convention nothing checked against the consuming model. The 2026-10-17 audit found a second of the same kind in the EKF (plan §2.2). The gyro's own sign is still unchecked: §3 item 1. |
| **RTK bandwidth over 433 MHz** | Deferred to phase 2. Untouched. |
| **No command uplink when disconnected** | **Architecturally answered** — goals preload from `[mission]`, the base is non-essential by design — but the LoRa link itself is phase 2. |
| **EKF tuning eats time** | **Open.** "Seed `R` from measured variance" needs a recording that does not exist (§1.4). "Log innovations from day one" *is* done, via `EkfDebug`. |
| **Magnetometer disappoints** | Deferred by design, behind an off-by-default flag. Not a concern. |
| **No ROS 2 escape hatch** | Accepted. `main` is intact in `RoboticsGG/almondmatcha`; criterion 10 not re-verified recently. |

A mitigation that cannot fail is not a mitigation — the lesson of the
sign-convention row.

---

## 3. Hardware-verification debt

The open part of `RUST_REWRITE_PLAN.md` §13.4. None of it is closable from a
desk.

1. **Gyro z sign.** The EKF assumes z-up, counter-clockwise positive
   (`e_psi_dot = (gyro - b_g) + v*kappa`, plan §2.2); how the LSM6DSV16X sits
   on the IKS4A1 relative to the chassis has never been checked. Turn the rover
   counter-clockwise (seen from above) by hand and confirm z reads
   **positive**. If negative, fix it in the chassis firmware's IMU read, not
   the estimator. Reading the feed needs `rover-tap --as control --type
   ImuSample` on the RPi's address with `rover-control` stopped
   (`docs/CALIBRATION.md` §4); the `[debug] mirror` cannot see it.
2. **Motor enable pins float from reset until `Motors::new`.** `PF12 PD15 PF13
   PE9` are floating inputs after reset; `Motors::new` runs first in `main`,
   but software cannot close the window before it. Hardware pull-downs would.
   Check with a meter or scope at power-on.
3. **TIM1 left-motor PWM** — the only channel on an advanced-control timer. If
   the left motor alone is dead, TIM1's break/MOE gate is the first place to
   look.
4. **Watchdog end to end** — cannot be faked in a test (§6 criteria 3-4).
5. **Encoder decode under load.** The decode runs in the executor, not an ISR:
   a second edge before the task samples counts as zero, so under UDP/I2C load
   at high wheel speed ticks could fall short. Hand-speed runs agree to 0.07 %
   (`STATUS_DONE.md` §9.1); **a fast spin and a loaded drive are unchecked.**
   If they fall short, move the decode into the EXTI handler or a timer.
6. **Every timing constant** — 200 ms / 300 ms / 500 ms are reasoned, not
   measured.

### 3.1 What the self-test cannot tell you

The chassis PWM checks confirm a timer is configured, a duty register accepted
a write and TIM1's `MOE` is set. **Nothing in the firmware can observe whether
a pin drove a waveform**, so a green POST is not proof the motors will turn.

---

## 4. Open defects

Narratives in `RUST_REWRITE_PLAN.md` §13.3b; the closed ones (D1-D3, D6-D10)
are in `STATUS_DONE.md`.

| | Defect | State |
|---|---|---|
| D4 | Base-station CSV contradicts `CSV_LOGGING.md` | **Half implemented.** Perception's `--csv` is opt-in as decided; `ground-station` creates its run directory lazily, but any run that receives anything logs, with no flag to decline. |
| D5 | `heading_err_rad`'s sign disagreed with its consumer | **Sign fixed; re-tune outstanding** — §1.3. |

---

## 5. Latency still unmeasured end to end

The camera half is measured — 59 ms capture-to-publish (`STATUS_DONE.md` §9.1)
— but **with no lane in view**; detection on a real track may take longer, so
re-read `capture_age_ms` from the first field run. Merging the camera and
detector into one process removed an 83 MB/s IPC hop but also the pipelining
two processes gave (`HARDWARE.md` §7's stages sum past its end-to-end figure).
Latency **through the EKF and the chassis** has never been measured: reasoning,
not a result.

---

## 6. Acceptance criteria not yet met

Against `RUST_REWRITE_PLAN.md` §12; the met ones are in `STATUS_DONE.md` §7.

| # | Criterion | Blocked by |
|---|---|---|
| 2 | Replay reproduces recorded ROS 2 steering | §1.4 — no recording exists |
| 3 | Watchdog: Ethernet pulled, throttle ramps to zero | §1.2 — needs the motors powered and commanded; §7 step 2 |
| 4 | IWDG: hang the control task, board resets | §1.2 — needs a deliberately hung task; §7 step 2 |
| 5 | E-stop end to end with `cmd_seq` echoed | needs `ground-station` driving a mission; §7 step 1 |
| 6 | Commanded 0.20 m/s measured as 0.20 ± 0.02 | a rolling rover, ideally after Procedure B (§1.1) |
| 7 | 100 Hz IMU at the estimator, latency measured | the IMU reaches `rover-control`; rate and latency not measured (§5) |
| 8 | EKF coasts a ≥3 s lane dropout without divergence | a driving rover, a real lane, and §9's speed-at-rest fix first |
| 9 | One full field circuit at parity, CSVs compared | everything above |
| 10 | `main` still builds and still drives | not re-verified recently |

---

## 7. What to do next, in order

The rover is still on its rig.

1. **Steering and motors on blocks.** Wheels off the ground; command a small
   steer and a low throttle (`ground-station`, or a mission with a low speed
   limit). Confirm steer > 0 turns the wheels **right** (§1.3), both motors spin
   **forward**, the left motor works at all (§3 item 3). `speed_loop_debug.csv`
   has the tick rates.
2. **Watchdog and IWDG on blocks** (§6 criteria 3-4): wheels turning slowly,
   pull the chassis Ethernet cable — throttle must ramp to zero over 300 ms and
   the board report `watchdog_tripped`.
3. **The gyro z sign** (§3 item 1). Sixty seconds.
4. **Fix the speed estimate at rest** (§9) before trusting `RoverState` or
   `rover-doctor`'s estimator check.
5. **Re-tune for D5 and D6 together** (§1.3), once it can roll.
6. **Procedure B** (§1.1).
7. **Record a real baseline** (§1.4), if criterion 2 is ever to close.

`docs/FIELD_TEST.md` is the end-to-end procedure.

---

## 8. Smaller open items, none needing a bench to start

- **Synthetic replay traces are not kinematically consistent.** `e_y` and
  `e_psi` are independent curves and the synthetic gyro lacks the `e_psi_dot`
  term, so the dropout scenario's coasting RMSE (1.68°, under the 3.0° gate) is
  not a meaningful number. Fixing the generator would not touch §1.4's real
  gap.
- **No I2C bus recovery.** The firmware bounds the IMU reset loop and uses a
  5 ms I2C timeout but cannot clock out a stuck slave (the HAL does not hand
  the pins back). A slave holding SDA low survives an MCU reset: the symptom
  would be a POST/IMU failure only a power cycle clears.
- **CSV rows can be lost on kill.** Writers flush about once a second and no
  binary installs a signal handler, so Ctrl-C, `tmux kill-session` or a power
  cut can lose ~1 s of rows. Waiting a couple of seconds before stopping
  (`docs/FIELD_TEST.md` §4) avoids it.
- **`TelemetryLite` and the LoRa links are unimplemented**, by design (phase 2,
  plan §6, §13.3 item 7), so nothing yet exercises the one-way-link
  assumptions the command protocol was built around.
- **`tools/requirements.txt` is unpinned**, and `perception/pyproject.toml` has
  lower bounds only (it asks for OpenCV ≥ 4.8; the Jetson runs 4.5.4, which
  passes every test).

---

## 9. Found on the 2026-10-26 bench, still open

`STATUS_DONE.md` §9 has what the session verified and fixed.

- **The speed state is unobservable at rest.** While stationary,
  `Ekf::correct_wheel_sensors` runs only the zero-rate gyro-bias update, never
  an odometry update, so `v` random-walks: `speed_mps` read 0.45 m/s with the
  wheels still, and within minutes the health word gained
  `ESTIMATOR_DIVERGED`. Drive control is unaffected (the speed PID uses ticks),
  but `speed_mps` and `rover-doctor`'s "estimator converged" check mean nothing
  at rest until a zero-speed update joins the zero-rate one. (Bench "lane"
  detections of clutter also feed the filter; that goes away on a track.)
- **The Spresense backup receiver has no fix.** Alive, sketch running, JSON
  over 115200, but 0-1 satellites and `"fix":false` indoors where the u-blox
  sees 12, so `BACKUP_GNSS_STALE` stays set. Recheck with sky view; a real fix
  reaching `Telemetry.backup` has not yet been seen end to end. **Opening its
  port resets it** (DTR/RTS on open), so every `rover-navigation` restart
  cold-starts it.
- **No RTCM source**, so the u-blox stops at DGPS (h_acc ~2.9 m) and
  `rover-doctor`'s RTK check is NO-GO. Expected until the 433 MHz link exists.
- **`python3-venv` is missing** on the base PC and the Jetson. Worked around
  without sudo (`FIELD_TEST.md` "Getting the code onto the machines");
  `sudo apt install python3.10-venv` would make the documented commands work
  as written.
- **ST-Link firmware `V2J39M27`** on both boards needs `--speed 1000`; updating
  it (STSW-LINK007) would probably remove that.
