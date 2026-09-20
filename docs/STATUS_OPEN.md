# What remains, and what to worry about

Companion to `docs/STATUS_DONE.md`, which records what is built and the
evidence for it. Together the two are the authoritative status of branch `rs`;
`docs/RUST_REWRITE_PLAN.md` §13 points here rather than repeating it.

This file is deliberately the pessimistic half. It exists so that nobody has
to infer what is missing by reading a list of accomplishments and noticing an
absence.

**The one-line version: the software is written and tested, and none of it has
ever run on the rover.** Every item in §1 must be cleared before a field run
means anything.

---

## 1. Hard blockers

Four things block a meaningful field run. **None of them is now a coding
task** — §1.3's code is fixed and what remains of it is a bench session. That
is the uncomfortable part: no amount of further development clears any of
them.

### 1.1 The drivetrain is uncalibrated

`ticks_per_rev`, `metres_per_tick` and `track_width_m` in `config/rover.toml`
are all `0.0`. There is **no metric speed anywhere in the system** until they
are measured. The estimator disables odometry rather than divide by zero, and
the speed PID works in ticks/sec so it is unaffected — but every `speed_mps`
in every log is meaningless, and acceptance criterion 6 cannot be attempted.

Procedures A and B in `RUST_REWRITE_PLAN.md` §2.6. About twenty minutes with a
tape measure. `docs/HARDWARE.md` §3 records the expected answer (~1500
ticks/rev at this firmware's 4× decoding) so the run confirms or contradicts
something rather than producing a number with nothing to check it against.

⚠️ Calibrate against **4×** decoding. A constant measured against the old 2×
firmware reads exactly double the true speed.

`rover-doctor` fails loudly on this today. That failure is correct and there
is no flag to skip it.

### 1.2 Nothing has ever been flashed

Every firmware claim is a claim about a binary that compiles, not one that has
run. The LAN8742A PHY against Embassy's `GenericPhy` is the gate that decides
whether any of it is real — see §3 below and `RUST_REWRITE_PLAN.md` §13.4a.

This applies to the board self-diagnostics too, and the irony is worth stating
plainly: **the power-on self-test has itself never executed.** It is
scaffolding for the first bring-up, not evidence about it.

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
| **PHY bring-up on F767ZI** | **Open — still the hard gate.** Exposure is unchanged; diagnosis is much better. The boards report PHY identity, negotiated speed, duplex and symbol errors, and now also `ANAR` and the `MODE[2:0]` straps — see §3.1, which is a concrete, likely failure mode that was invisible before. See plan §13.4a. |
| **Control regression from the port** | **Partly closed.** Replay gate passes against a frozen oracle; the lane port is exact against the vendored original. Field parity unproven — see §1.4. |
| **Sign-convention inversion** | **Sign closed; gain re-tune open.** Worth remembering how it went wrong: the mitigation was "documented §10, unit-tested", and that documentation is exactly what let D5 hide — the prose asserted a convention nothing checked against the consuming model. See §1.3. |
| **Encoder constant wrong after 2×→4×** | **Open**, but now checkable: `HARDWARE.md` §3 records an expected value, so the calibration run either confirms it or disagrees loudly. |
| **RTK bandwidth over 433 MHz** | Deferred to phase 2. Untouched. |
| **No command uplink when disconnected** | **Architecturally answered** — goals preload from `[mission]` and the base station is non-essential by design — but the LoRa link itself is phase 2. |
| **EKF tuning eats time** | **Open.** "Seed `R` from measured variance" still requires a recording that does not exist. The other half — "log innovations from day one" — *is* done, via `EkfDebug`. |
| **Magnetometer disappoints** | Deferred by design, behind an off-by-default flag. Not a concern. |
| **Jetson Python bus drifts from Rust** | **Closed.** Golden-byte fixtures checked from both languages, both directions. |
| **No ROS 2 escape hatch** | Accepted. `main` is intact in `RoboticsGG/almondmatcha` (not this repository). Criterion 10 ("`main` still builds") has not been re-verified recently. |

**Three closed or mostly closed out of ten, and the largest is untouched.**
That is not a failure of the rewrite — most of these were always going to close
on hardware, and there has been no hardware. It is a caution against reading a
green test suite as progress against this table.

The sign-convention entry is the one worth learning from: it was listed as
mitigated by documentation and unit tests, and both were true — the docs stated
the convention and the tests asserted it. Neither checked the *data* against
the *model*, so both passed while the system was wrong. A mitigation that
cannot fail is not a mitigation.

---

## 3. Hardware-verification debt

`RUST_REWRITE_PLAN.md` §13.4 holds the full list in risk order. In brief, and
none of it is closable from a desk:

1. **LAN8742A PHY** against `GenericPhy` — the hard gate. See §3.1.
2. **I2C1 on PB8/PB9** — inherited from the mbed target's generic pin names,
   never confirmed against the physical board.
3. **Encoder sign convention** — defined from first principles; needs a wheel
   turned by hand while watching tick counts.
4. **Software EXTI decode under load** — could in principle miss edges while
   competing with UDP and I²C interrupts. Unmeasured.
5. **TIM1 left-motor PWM** — the only channel on an advanced-control timer.
6. **Watchdog end to end** — cannot be faked in a test.
7. **Every timing constant** — 200 ms / 300 ms / 500 ms are reasoned, not
   measured.

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
| D4 | Base-station CSV contradicts `CSV_LOGGING.md` | **Half done.** Perception's `--csv` is opt-in as decided. `ground-station` creates its run directory lazily, so a base that receives nothing leaves nothing — but any run that receives anything logs, with no flag to decline. |
| D5 | `heading_err_rad`'s sign disagreed with its consumer | **Sign fixed; re-tune outstanding.** §1.3 |
| D6 | Camera measurement timestamped after detection | **Open.** §5.1 |

---

## 5. Concerns with no risk-register entry

### 5.1 D6 — the camera pipeline's latency is invisible

Plan §13.3b D6 has the full write-up. `perception` stamps `t_us` *after* lane
detection returns rather than at capture, discarding the D415's hardware
timestamp, and `correct_lane` ignores `t_us` entirely — so the EKF treats a
~100 ms-old observation as current.

Not a staleness problem (`lane_stale_ms` is 500 ms; nothing trips). It is
phase lag in a closed loop, it grows with speed, and it can push innovations
into the `nis_gate = 11.34` rejection region in the tight curves where the
camera matters most.

**The part that should bother you: none of it is measurable from a field log.**
`achieved_fps` is loop rate, not latency; there is no dropped-frame counter and
no capture-to-publish figure. The ~100 ms is reasoned from `HARDWARE.md` §7's
ROS 2 baseline, not measured on this stack. Adding the measurement half alone
is behaviour-neutral; the compensation half belongs with the D5 re-tune.

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
already-queued frame rather than blocking. **But end-to-end latency has never
been measured on this stack** — reasoning, not a result.

Related: `camera.py` never pins the librealsense frame-queue depth, so whether
a slow detector drops frames or accumulates latency rests on a library default
this code does not set.

---

## 6. Acceptance criteria not yet met

Against `RUST_REWRITE_PLAN.md` §12. `docs/STATUS_DONE.md` covers the ones that
are met.

| # | Criterion | Blocked by |
|---|---|---|
| 2 | Replay reproduces recorded ROS 2 steering | §1.4 — no recording exists |
| 3 | Watchdog: Ethernet pulled, throttle ramps to zero | §1.2 — never flashed |
| 4 | IWDG: hang the control task, board resets | §1.2 |
| 5 | E-stop end to end with `cmd_seq` echoed | §1.2 |
| 6 | Commanded 0.20 m/s measured as 0.20 ± 0.02 | §1.1 — no metric speed exists |
| 7 | 100 Hz IMU at the estimator, latency measured | §1.2, and §5.1 for why "latency measured" is harder than it looks |
| 8 | EKF coasts a ≥3 s lane dropout without divergence | §1.2 |
| 9 | One full field circuit at parity, CSVs compared | everything above |
| 10 | `main` still builds and still drives | not re-verified recently |

---

## 7. What to do next, in order

1. **Calibrate the drivetrain** (§1.1). Twenty minutes, no hardware risk,
   unblocks criterion 6 and turns every logged speed into a real number.
2. **Flash both boards on the bench** (§1.2) and get past the PHY gate. This is
   the decision point for the whole branch. `rover-doctor` and
   `board_diagnostics.csv` are there to tell you what happened.
3. **Confirm the servo direction on blocks** (§1.3). Sixty seconds. The sign
   fix is settled in software; this checks the firmware mapping below it.
4. **Re-tune for D5 and D6 together**, on hardware, once the boards run. D5's
   fix alone is a 3.21× authority increase — this is the step that matters
   most now, not a formality.
5. **Record a real baseline on `roboticsgg-almondmatcha/main`** (§1.4) if
   criterion 2 is to ever close — that remote is the ROS 2 fallback; this
   repository's `origin` has no `main`.

`docs/FIELD_TEST.md` is the end-to-end procedure once steps 1–3 are done.
