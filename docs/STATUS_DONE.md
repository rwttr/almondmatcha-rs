# What is built — branch `rs`

Evidence side of the status split: [`STATUS_OPEN.md`](STATUS_OPEN.md) covers
what remains and what to worry about; this document names, for every claim,
the thing that would have failed if it were false. `RUST_REWRITE_PLAN.md`
§13 is the detailed design record; this file does not depend on it.

**Update, 2026-10-26: the first bench session happened** — §9. Both boards are
flashed and running, the full stack is deployed and running on the RPi and
the Jetson, every feed reaches the base station, and the drivetrain is
calibrated. The session found and fixed four defects only hardware could show
(§9.2). **Nothing has driven yet**: the rover sat on a bench rig throughout,
and steering, the motors under load, the watchdog and a mission are all still
unexercised. Sections 1-8 below were written before that session and describe
laptop-only evidence; where §9 supersedes one of them it says so.

**As of 2026-10-17** (the state §§1-8 describe), nothing in `firmware/` had
ever run on a real board, and neither the perception pipeline nor the control
law had ever driven the actual rover. Every result in §§1-8 came from a
developer laptop. The ROS 2 `main` now lives only in `RoboticsGG/almondmatcha`,
not in this repository. §8 records what the 2026-10-17 audit found and fixed in the
software; it makes the code more correct, not any more proven.

Three kinds of confidence appear below and must not be blurred: **proven**
(a test exists that fails if the claim is false, and it ran), **built and
reviewed, never run on hardware** (compiles, is unit-tested, has been read
carefully, but no board, sensor or motor has ever seen it), and **believed to
work** (a design judgement, backed by neither). Each section says which kind
of claim it's making.

---

## 1. The headline numbers

Re-measured on 2026-10-17 at the tip of `rs` (after the audit commits in §8),
not copied forward (commands in the footnote).

The host workspace — 12 crates (`ground-station`, `rover-bus`,
`rover-control`, `rover-doctor`, `rover-estimator`, `rover-link`,
`rover-model`, `rover-msgs`, `rover-navigation`, `rover-runs`, `rover-tap`,
`rover-telemetry`) plus `tools/replay`, 13 packages in all — is
clippy/rustfmt clean, and `cargo test --workspace` passes **434 tests**, 0
failed. Perception passes **122** Python tests. (2026-10-26, after the D6
fix and the bench session: **440** and **127** — D6 added 4 Rust and 2 Python
tests, the bench fixes 2 Rust (`rover-link`) and 3 Python (the D415 serial);
§9.) `testdata/` holds **26**
golden wire fixtures: 18 message types, `FrameHeader`, and seven variants.
Both firmware images build for `thumbv7em-none-eabihf` and pass clippy —
sizes and **never run on the boards they target** in §2's table. The
toolchain is pinned: `rust-toolchain.toml` says `1.99.1` (with `llvm-tools`,
so `cargo size` works), where it used to float on `stable` and let the size
figures drift between measurements. That is the entire scope of what's
proven: it builds, and it passes tests on a laptop.

---

## 2. Component status

Carried over from the status table `RUST_REWRITE_PLAN.md` §13 used to hold
(now split into this file and `STATUS_OPEN.md`), re-verified for this
document. The **Evidence** column is the point: every "done" names the test
or artefact that backs it, not just an assertion that it exists.

| Component | State | Evidence |
|---|---|---|
| `rover-msgs` | **done** | 18 message types (plus the `FrameHeader` envelope, which is not a `Wire` type but has its own fixture), builds for host **and** `thumbv7em-none-eabihf`. 11 tests: 2 unit, 6 round-trip/contract (round-trip, declared-vs-actual length, trailing bytes, truncation, frame budget, ID uniqueness), 3 golden-fixture. `CommandFrame` carries a `session` (21 B on the wire), and `CommandAck` (`TYPE_ID 0x0803`) is the consumers' echo. |
| `testdata/*.bin` | **done** | 26 fixtures — the 18 message types, `FrameHeader`, and seven variants (`CommandFrame_{Nop,SetSpeedLimit,CancelMission,EStop,ClearEStop}`, `MissionStatus_NoTarget`, `LaneMeasurement_Invalid`). Regenerating them is a breaking protocol change (see §4). |
| `rover-link` | **done** | `Link` trait, `UdpLink`, `send_to_addr` with an `Unsupported` default so a future LoRa link need not implement it. |
| `rover-bus` | **done** | config loading (unknown route destinations, unknown message names and duplicate service addresses are rejected at load), unicast fan-out, newest-wins receive, idempotent session + `cmd_seq` command protocol, debug mirror. 43 tests. |
| `rover-tap` | **done** | per-type rate (keyed on peer and type) and seq-gap loss; every message type registered, enforced by a test over `rover_msgs::ALL_MESSAGES`; `--mirror` sees every frame a host process publishes (the firmware has no mirror). |
| `rover-model` | **done** | shared `A(v)`/`B(v)`, Euler discretisation, sign-corrected `e_psi_dot = (gyro - b_g) + v*kappa` and `B[e_psi] = -v/L` (plan §2.2). 8 tests. |
| `rover-estimator` | **done** | 5-state EKF, Joseph form, chi-square gate, coast-on-dropout, learnable gyro bias, dt-discontinuity guard (gaps over 500 ms re-anchor instead of integrating). 16 behavioural tests. |
| `rover-control` | **done** | lib + thin bin so `tools/replay` drives production code. `LateralController` trait; `StaticGain` is the bit-exact port. `actuate.rs` owns every guard rail, and the drive gate is closed until the first IMU, first accepted camera frame and first `ChassisStatus`, and again when IMU (200 ms), `MissionStatus` (500 ms) or `ChassisStatus` (1 s) go stale. Tracks `ChassisArm` and sends zero throttle while the board is tripped so it can re-arm. 78 tests. |
| `rover-navigation` | **done** | two GNSS receivers, mission state machine, RTCM injection point; fixes expire after 2 s, non-finite positions are unusable, the mission pauses after 5 s without a usable fix. 87 tests. |
| `rover-telemetry` | **done** | CSV at native rates, health bits, base feed, `board_diagnostics.csv` with the three latched board health bits; `AckTracker` folds the two consumers' `CommandAck`s into the `Telemetry` echo. 66 tests. |
| `ground-station` | **done** | mission goals, speed limit, E-stop, live display, `runs/` CSV via the shared `rover-runs` crate. FIFO command queue with E-stop pre-emption. 42 tests. |
| `rover-runs` | **done** | the `run_NNN_<stamp>/` convention, shared by `rover-telemetry` and `ground-station` rather than copy-pasted into each. CSVs open in append mode; a header mismatch rotates to `name.N.csv`; run numbers above 999 parse. 15 tests. |
| `rover-doctor` | **done** | preflight GO/NO-GO over 9 checks; verdict logic is a pure function over observations, 27 tests. Distinguishes NOT SEEN from NO-GO and never reports GO on missing data; a board that ran no POST checks is NO-GO. Binds its own `doctor` service (`192.168.1.10:7031`). |
| `tools/replay` | **done** | **the §12 gate.** Synthetic trace + `legacy.rs` oracle. PASS at the 3.0° tolerance (rmse 1.370°, 267 rows compared, max 2.719°); deliberately verified to FAIL at 0.5°, so the harness demonstrably has teeth. A NaN RMSE fails the gate. 19 tests. |
| `perception/rover_perception/wire.py` | **done** | 79 golden-fixture tests green against the Rust fixtures — the two languages provably agree on every byte. |
| `perception/rover_perception/lane.py` | **done** | behaviour-preserving port, **parity proven** — see §3. |
| `perception/rover_perception/{camera,bus,main}.py` | **done** | one process replacing `camera_stream_node` + `lane_detection_node`. End-to-end verified: 20 frames in, 20 `LaneMeasurement` out, seq monotonic, decoded by `wire.py`, clean exit. |
| `firmware/chassis` | **runs on the board** (2026-10-26, §9) | 71,144 B flash (3.4%), 19,324 B RAM (3.7%) — text 71,056 / data 88 / bss 19,236 (grew with the `rt` vector table, §9.2). Clippy clean. Boots to `ready` with PHY link, LSM6DSV16X `id=0x70`, motors held stopped and tripped; POST passes; re-armed by `rover-control` over the LAN. Motors, servo and the watchdog under load are **not yet exercised**. |
| `firmware/sensors` | **runs on the board** (2026-10-26, §9) | 58,920 B flash (2.8%), 19,368 B RAM (3.7%) — text 58,840 / data 80 / bss 19,288. Clippy clean. Boots to `ready`; POST passes; encoders verified by hand (4× decode, ~1,396 ticks/rev, both wheels count up forward, sides fixed — §9.2); INA226 reads 28.5 V / 0.82 A at the base station. |

---

## 3. Why the lane detector port is believable

`perception/rover_perception/lane.py` is the one component where "ported carefully" isn't
enough evidence, because the C++/rclpy node it replaces no longer exists in
this repo to compare against by eye. What makes it **proven** rather than
merely **believed** is `perception/tests/test_lane_parity.py` (9 tests, all
passing when re-run for this document):

- It compares `process_frame` against the **real ROS 2 original**, not a
  paraphrase: `lane_detector.py` and `config.py` are vendored verbatim at
  `perception/tests/oracle/`, diffed at vendoring time — the only edit in
  either file is one import line. `tools/replay/src/legacy.rs` freezes the
  old control law the same way, for the same reason: the ROS 2 tree could
  not be safely deleted (§6) without first freezing something to check the
  port against forever.
- The comparison is **exact `==`**, no tolerance, across six deterministic
  synthetic scenarios plus a `search_center` tracking case. Four exercise the
  *detected* path with materially different geometry, so it isn't passing
  vacuously on "NaN equals NaN"; the other two (blank frame, pure noise) are
  negative controls, one asserted not-detected outright.
- The suite was checked by **mutation**, not by reading and trusting it:
  perturbing the port's polyfit coefficients by one part in 10⁷ fails 5 of
  the 9 tests. A real algorithmic divergence could not pass it by accident.

Still a laptop-only proof: it shows the new code computes the same lane
geometry as the old code given the same pixels. It says nothing about the
camera, the lens, or lighting in the field.

---

## 4. Cross-language wire agreement

The rover speaks one wire protocol across Rust (host binaries and both MCUs)
and Python (Jetson). Every message type is checked byte-for-byte across that
boundary rather than trusted to match by construction.

`crates/rover-msgs` defines **18 message types**. With `FrameHeader` — the
envelope wrapping every message, not itself a `Wire` type but given its own
fixture because framing bugs are exactly what this check exists to catch —
and seven variant fixtures that pin the awkward cases (each `CommandFrame`
body, a `MissionStatus` with no target, an invalid `LaneMeasurement`),
that's **26 golden fixtures** in `testdata/*.bin`. Each is generated once
from Rust and checked from both ends: Rust's own round-trip and
golden-fixture tests decode, re-encode and compare bytes; `perception/rover_perception/wire.py`'s
79 golden-fixture tests decode the same files independently and check the
resulting field values. Both sides agreeing with a byte file neither wrote at test time is
what makes this cross-language rather than two languages testing themselves.

**Regenerating these fixtures is a breaking protocol change**, not routine
maintenance — it silently invalidates every historical comparison against
the old bytes. Treat `testdata/*` like a protocol version bump.

---

## 5. Board self-diagnostics

The most recently landed work on the branch, squarely "built and reviewed,
never run on hardware": everything below compiles and is unit-tested on the
host, but has never received a real POST result or a real PHY register read
from a board.

Both firmware images (`firmware/chassis/src/diag.rs`,
`firmware/sensors/src/diag.rs`) now run a power-on self-test at boot, record
the MCU's own reset cause, and read the LAN8742A PHY's link-state registers —
speed, duplex, symbol-error count — read-only, never written. All of it is
packed into one new wire message, `BoardDiagnostics`, published by both
boards. Two host crates consume it:

- **`rover-telemetry`** (`src/main.rs`) subscribes from both boards, logs
  every sample to `board_diagnostics.csv`, and folds it into three
  **latched** health bits — `BOARD_POST_FAIL`, `BOARD_RESET`,
  `LINK_DEGRADED` — via `health.rs`'s `BoardHealth`, keyed per board so a
  healthy sample from one can never mask a fault the other is still
  reporting. Latched means a fault stays set for the rest of the run even
  once later samples look healthy, because clearing it the moment a board
  stops re-announcing would hide the event the bit exists to record. A
  reboot legitimately reporting `ResetCause::PowerOn` is still caught, since
  `BoardHealth` also watches `uptime_s` going backwards between samples from
  the same board. A link speed of `0 Mbit/s` is treated as "couldn't read its
  own PHY yet" rather than "link down" — the sample necessarily arrived over
  that link — so it's excluded from `LINK_DEGRADED` rather than latching a
  false alarm on every boot.
- **`rover-doctor`** (new crate) is a preflight tool: it listens for a fixed
  window, then runs `verdict.rs`'s `evaluate()` — a pure value-in/value-out
  function with no socket, clock or sleep, which is what lets its 27 tests
  exercise every check without a live rover — and prints a GO/NO-GO per
  check. Its most consequential design decision is a third outcome,
  `Verdict::NotSeen`, alongside `Go`/`NoGo`: a check that never received data
  fails the launch exactly like one that received bad data, but is reported
  under its own label so an operator doesn't waste time hunting a fault on a
  board that simply isn't talking. One check, drivetrain calibration, is read
  from `config/rover.toml` rather than observed on the bus, so it can never
  report `NotSeen` — it's a hard-coded NO-GO today, because
  `metres_per_tick` is still `0.0` (§7, criterion 6).

None of the nine checks, the three latched bits, or the self-test itself has
ever evaluated a real board. What's proven is narrower: given the inputs a
board *would* send, the verdict logic behaves as documented — including the
edge cases (missing data, a clean reboot, an unreadable PHY) that are easy to
get wrong by hand.

---

## 6. The ROS 2 tree has been removed

Removed on 2026-09-20 (`chore(rs)!: remove the ROS 2, mROS 2 and
embeddedRTPS tree`, `e7628b3`): 919 files, ~137,000 lines — `ws_rpi`,
`ws_jetson`, `ws_base`, `common_ifaces`, both `mros2-mbed-*` trees, ten
DDS/RTPS/mbed docs, and the ROS 2 launch tooling.

Deliberately held until two things were true: the `tools/replay` gate was
passing (§2, §7 criterion 1) and the lane-parity oracle was vendored (§3).
Until both existed, the ROS 2 tree was the *only* thing the port could be
checked against, so deleting it earlier would have deleted the evidence
along with the code.

Nothing is actually gone: everything removed was tracked and committed
first — verified beforehand that no untracked or ignored file, and no
`runs/`, CSV, bag or video, lived under the tree — so it's recoverable via
`git show roboticsgg-almondmatcha/main:<path>` (this repository's `origin`
has no `main`; that remote is the ROS 2 fallback, `RoboticsGG/almondmatcha`)
or this branch's own history. Durable knowledge was harvested first into
`docs/HARDWARE.md`. `ws_spresense` was left untouched, by instruction, and
still exists on this branch.

---

## 7. Acceptance criteria (§12) — where they actually stand

`RUST_REWRITE_PLAN.md` §12 lists ten criteria for calling the rewrite done.
Most are **not** met, uniformly because nothing has been flashed and the
drivetrain has never been calibrated. Full reasoning for each open criterion
lives in `STATUS_OPEN.md` §6; this is the verdict.

| # | Criterion | Status |
|---|---|---|
| 1 | `cargo test --workspace` green, fixtures cross-checked against Python | **Met.** 440 Rust tests, 127 Python tests, 26 fixtures — §1, §4. |
| 2 | `tools/replay` reproduces recorded ROS 2 steering | **Not met — cannot be attempted.** No recorded ROS 2 run ever existed; replay proves internal consistency against a frozen oracle (§2, §3), not field parity. `STATUS_OPEN.md` §1.4. |
| 3 | Watchdog: Ethernet pulled while driving → ramp-to-zero | **Not met** — never run on hardware. `STATUS_OPEN.md` §1.2. |
| 4 | IWDG: hung control task → board resets | **Not met** — same. `STATUS_OPEN.md` §1.2. |
| 5 | E-stop end to end, `cmd_seq` echoed | **Not met** — protocol is unit-tested (`rover-bus`) but never run on real firmware. `STATUS_OPEN.md` §1.2. |
| 6 | 0.20 m/s commanded → 0.20 ± 0.02 m/s measured | **Not met, now attemptable** — the drivetrain is calibrated (§9.1; diameter-derived, Procedure B still owed), so metric speed exists, but nothing has been driven. `STATUS_OPEN.md` §1.1. |
| 7 | 100 Hz IMU at the estimator, latency measured | **Not met** — `rover-control` receives the chassis IMU over the LAN (§9), but neither its rate at the estimator nor its latency has been measured. `STATUS_OPEN.md` §1.2, §5.1. |
| 8 | EKF coasts a ≥3 s lane dropout without divergence | **Partially met.** `rover-estimator/tests/ekf.rs::lane_dropout_coasts_without_diverging` proves it at unit level — 300 steps (3 s at 100 Hz) of invalid lane feed, covariance provably growing, state finite and bounded (`< 0.5` cross-track/heading error). Not a demonstration on the integrated pipeline or a real occluded camera. |
| 9 | One full field circuit at parity, CSVs compared | **Not met — cannot be attempted.** Requires a driving rover. `STATUS_OPEN.md` §1.4, §6. |
| 10 | `main` still builds and drives | **Not independently re-verified for this document.** `STATUS_OPEN.md` §6. |

One criterion fully met, one proven at the unit level but not end-to-end,
eight not met or not checkable without hardware.

---

## 8. The 2026-10-17 audit

A review of the whole branch for faults that tests written by the same hands
would not catch — a sign error the tests had shared, a datagram that would
have been dropped by its own receiver, health that read GO with no data.
Everything below was fixed in software and covered by new tests; **none of it
has run on hardware**, so "proven" here means a test fails without it. Bench
state at the time, confirmed by the user the same day: nothing flashed, the
2026-09-21 session did not happen, drivetrain uncalibrated. What the audit
left open is in `STATUS_OPEN.md` (§3 for bench checks, §8 for the rest).

**Perception and run directories** (`9d7fb8e`)
- *Critical:* perception sent from an unbound socket, so every datagram left
  from an ephemeral source port, and `rover-link` drops any datagram whose
  source `ip:port` is not a configured service — every `LaneMeasurement`
  would have been discarded by `rover-control`. The bus now binds its own
  `[services]` entry (`--bind-any` for hosts that do not own the address).
- Perception survives per-frame exceptions, reopens a lost camera with backoff
  (1, 2, 4, 8, 16 s) and then exits non-zero; non-finite lane geometry
  degrades to `valid = false`. The Jetson launcher restarts it on a non-zero
  exit; the RPi launcher deliberately restarts nothing.
- `rover-runs`: CSVs open in append mode (restarting a process in its pane no
  longer wipes the run), a header mismatch rotates to `name.N.csv`, and run
  numbers above 999 parse in Rust and in both bash allocators.

**Firmware** (`c3384a7`)
- The chassis boots **tripped**; re-arm needs an explicit zero-throttle
  `ChassisCommand` (plan §5.2). `watchdog_tripped` is published before the
  300 ms ramp. `ChassisCommand` is accepted only from `192.168.1.1:7001`;
  stale or duplicate `seq` is dropped while armed; the timeout deadline moves
  only on accepted commands; `seq` resyncs after a trip.
- `ChassisStatus` now reaches telemetry *and* control. `Motors` is constructed
  (stopped) before Ethernet bring-up. Throttle travels at 1/10000 resolution,
  and the firmware clamps duty to 40 %, mirroring `[speed] limit_cap_pct`.
- IMU reset loop bounded; I2C timeout 5 ms. `BoardDiagnostics` reaches
  telemetry, base and doctor (`.10:7031`) from both boards; PHY MDIO
  diagnostics rate-limited to 1 Hz; `PLLQ` set to /9 (48 MHz; it was 0, which
  RM0410 calls invalid, and feeds the RNG clock).
- Sensors: encoder decode arms EXTI before sampling, closing a lost-edge
  window. The remaining limit is recorded in `STATUS_OPEN.md` §3.

**Control, estimation, navigation** (`5fa7af9`)
- *EKF process model sign:* `e_psi_dot = (gyro - b_g) + v*kappa` (it was
  `- v*kappa`); Jacobian entries `(e_psi, kappa) = +v`, `(e_psi, v) = +kappa`;
  `control_b` gives `B[e_psi] = -v/L`. The tests had shared the error. Plan
  §2.2 has the derivation. Gyro bias is learnable (the zero-rate update had
  been fed a hard-coded `0.0`).
- Time and staleness: IMU/wheel gaps over 500 ms skip and re-anchor (a ~4,295 s
  step before); `SafetyGate` closes on IMU stale over 200 ms and `MissionStatus`
  stale over 500 ms (`mission_active` had latched forever) and starts closed;
  `lane_age_ms` starts at `u16::MAX`, so nothing drives before the first
  accepted camera frame; wheel speed uses the MCU's `t_us`, not RPi arrival time.
- The stall latch no longer clears itself through the duty it forced; a NaN
  steer no longer panics `rover-control`; non-finite NIS/measurements are
  gated; config validation covers `predict_hz`, `command_rate_hz`,
  `steer_max_deg`, slew, `limit_cap_pct` and `wheelbase`.
- Navigation: GNSS fixes expire after 2 s, non-finite positions are unusable,
  the NMEA checksum no longer panics on non-ASCII, reader threads survive
  panics, a mission with no usable fix for 5 s pauses (and resumes), a goal
  already inside the arrival radius is `Arrived` directly, and `CommandFrame`
  is subscribed rather than latest-only.
- Replay: a NaN RMSE fails the gate. The dropout scenario's RMSE moved from
  0.76 to 1.68 degrees (still PASS) after the sign fix — because the synthetic
  truth is not kinematically consistent; see `STATUS_OPEN.md` §8.

**Command protocol** (`7bc014c`)
- `CommandFrame` gained `session: u16` (random non-zero per `ground-station`
  process; 0 reserved): `WIRE_LEN` 19 to 21. A new session resets receiver
  ordering, which fixes a base restart leaving commands `1..L` ignored and
  falsely acked.
- New `CommandAck { session, cmd_seq }` (`TYPE_ID 0x0803`, 4 B), published by
  `rover-control` and `rover-navigation` when they apply a command and at 2 Hz.
  `CommandFrame` is routed to those two only; `rover-telemetry` no longer runs
  a receiver, and its `AckTracker` echoes `(session, seq)` in `Telemetry` only
  when both acks are fresh (2 s) and share a non-zero session, using the older
  `seq`, else `(0, 0)`. `Telemetry` gained `last_cmd_session` (177 B).
- `ground-station` sends FIFO with one in flight; an E-stop discards the queue
  and replaces the in-flight command (logged), and nothing replaces an
  in-flight E-stop. `rover-doctor` has its own `doctor` service
  (`192.168.1.10:7031`, 8 services in all), so it and `ground-station` run
  together. `rover-control` tracks `ChassisArm` (`Armed`/`Tripped`/`Unknown`),
  sends zero throttle while tripped so the board re-arms, and closes the drive
  gate after 1 s without `ChassisStatus`.
- Config validation (Rust and Python) rejects an unknown route destination, an
  unknown message name and a duplicate service address at load. `Bus` and
  `UdpLink` keep draining past runt datagrams and a failed decode never
  overwrites the latest value. `rover-tap` registers every type (test-enforced
  via `ALL_MESSAGES`), keys `--hz` on `(peer, type)`, and treats a backward
  sequence jump under 1000 as reordering rather than a restart.

**Health, doctor, ground-station, CSV** (`688a649`)
- `rover-telemetry`: an estimator never heard from, or silent over 500 ms,
  raises `LANE_STALE` and `ESTIMATOR_DIVERGED` (a zeroed default `RoverState`
  used to read healthy, so the doctor said GO); NaN cross-track variance reads
  as diverged; the stall verdict runs on a monotonic clock and expires after
  1 s without `SpeedLoopDebug`.
- `rover-doctor`: a board with no POST checks run is NO-GO; the calibration
  check names the file it read (the base PC's `config/rover.toml`) and says
  the RPi's copy must match.
- `ground-station`: health bits 8-10 are shown (they were hidden, so a faulted
  rover displayed "Health: OK"), unknown bits print as `UNKNOWN_BIT_n`, notices
  persist 5 s, the hint line is built from the command table (`clearestop`
  was missing), the dashboard shows the in-flight command, queue length,
  session and last echoed `seq`, and goals with NaN/infinite/out-of-range
  coordinates are rejected.
- CSV: buffered writers flush about once a second with `sync_data`; bounded
  queues (telemetry 8,192 rows, ground-station 1,024) drop and count rows when
  full; ground-station CSV errors are logged, rate-limited.
  `speed_loop_debug.csv` has the header
  `Timestamp_us,Measured_Left_TPS,Measured_Right_TPS,Measured_Avg_TPS,Target_TPS,Error_Pct,PID_Output_Pct`;
  `ground_station_telemetry.csv` gained `Last_Cmd_Session`.

**Toolchain** (`c3824e9`): `rust-toolchain.toml` pinned to `1.99.1` with
`llvm-tools`, replacing a floating `stable`.

Final counts at `c3824e9`, re-measured when this section was written: 434
Rust tests, 122 Python tests, 18 `TYPE_ID`s, 26 fixtures, both firmware images
build.

---

## 9. The 2026-10-26 bench session

The first time any of this branch met hardware. Rover on a bench rig with the
drive wheels off the ground, both NUCLEOs on USB to the base PC, all five
machines on the LAN. **Everything here is proven in the strongest sense this
document uses — observed on the real boards — but only on a bench**: nothing
drove, and the motors and servo were never commanded.

### 9.1 What was verified

| Area | Result | How |
|---|---|---|
| Probes | Both ST-Links identified: sensors `066DFF3932504E3043014542`, chassis `066AFF3932504E3043101915` — the chassis value, previously unconfirmed, is right | `probe-rs list`; the chassis serial flashed the board that then reported its IMU |
| Chips | Both `DBGMCU_IDCODE = 0x10016451` (F76x/77x), 3.24-3.26 V | OpenOCD, read-only |
| PHY gate (plan §13.4 item 1) | **Passed on both boards.** `ANAR=0x01e1`, `MODE[2:0]=111` — the healthy strap case, so the `ANAR`/strap failure mode did not occur (§10). 100 Mb/s full duplex, zero symbol errors | `defmt` boot log; `BoardDiagnostics` at the base |
| POST | Passes on both boards (after the fix in §9.2) | `rover-doctor` |
| I2C1 on PB8/PB9 (§13.4 item 2) | Confirmed on both boards: LSM6DSV16X answers `id=0x70` on chassis, INA226 reads on sensors | boot log; `Telemetry.power` = 28.5 V / 0.82 A |
| Encoders (§13.4 item 3) | 4× decode, **~1,396 ticks/rev**; both wheels count **up** when turned forward; no counts at rest | Procedure A, calibration image: 13,960 / 13,962 / 13,963 / 13,969 on one wheel, 13,913 / 13,954 on the other, 10 hand turns each |
| Drivetrain calibration | `ticks_per_rev = 1396`, `metres_per_tick = 0.0002813` (π × measured 0.125 m / 1396) in `config/rover.toml` | Procedure A + measured diameter. **Procedure B not run** — the rover could not be rolled |
| LAN, board → host | Every board message type reaches its host consumer (after the fix in §9.2) | `rover-control` logs `IMU feed live`; `rover-doctor` sees both boards |
| Chassis arming | `rover-control` re-arms the tripped chassis with zero throttle over the LAN | `rover-control` logs `chassis armed` |
| Deployment | RPi and Jetson run commit `575f028` (binaries cross-compiled on the base PC); perception's 124 tests pass on the Jetson against its system numpy 1.26.4 / OpenCV 4.5.4 / CUDA PyTorch | `FIELD_TEST.md` "Getting the code onto the machines" |
| D415 + D6 | Frames stamped from the D415's own `global_time` timestamp; **capture-to-publish median 59 ms** (54-62 ms, p95 61 ms), 0 dropped frames over 617, at 29.8 fps | `lane_detection.csv` `capture_age_ms` — with no lane in view, so detection may run longer on a real track |
| u-blox GNSS | DGPS, 12 satellites, h_acc 2.85 m | `Telemetry.rtk` |
| Preflight | `rover-doctor`: 7 of 9 GO. NO-GO on RTK (no RTCM source exists yet — expected) and health bits (`BACKUP_GNSS_STALE`; later also `LANE_STALE`, `ESTIMATOR_DIVERGED` — see `STATUS_OPEN.md` §9) | `rover-doctor` |

### 9.2 Defects found and fixed

Each was reachable only on hardware; each has its own commit and a design
record in plan §13.3b (D7-D10).

| | Defect | Fix |
|---|---|---|
| D7 | **Both boards hung ~10 ms after boot.** `default-features = false` on `embassy-stm32` dropped its `rt` feature and with it the device vector table; the first TIM5 (time-driver) interrupt fell into `DefaultHandler`'s infinite loop, before the IWDG was ever started. | `rt` enabled (`953810d`); verified by halting the core (PC in `DefaultHandler`, IPSR = IRQ 50) before, and by the vector table after |
| D8 | **Encoder sides swapped.** The rover's right wheel counted on the pins the ROS 2 firmware called left. | `PA15`/`PB5` → right, `PB3`/`PB4` → left (`9beb1d3`); re-verified by hand |
| D9 | **Every board datagram dropped by every host process.** Hosts identify a sender by exact `ip:port`; the boards send from ephemeral ports. The rover could never have driven: `rover-control` would never have seen the IMU. | `rover-link` falls back to the sole configured service at the source IP (`eb67653`); 2 tests |
| D10 | **POST raced the link.** With a static IP `wait_config_up` resolves before auto-negotiation (~1.7-2 s), so POST read zeroed PHY status and both boards reported `PHY_ID \| LINK` failed on a clean link. | `net::wait_up` also waits for the link and a cached speed, bounded at 5 s (`eb67653`) |

Plus two configuration changes the session forced: the D415 serial is now
pinned in `[perception] d415_serial` (the Jetson has a D405 too, and
"first device found" could pick it — `575f028`), and every `probe-rs` command
on this bench needs `--speed 1000` (ST-Link firmware `V2J39M27`;
`CALIBRATION.md` §3.1).

### 9.3 Not verified — still open

Steering direction, motors and TIM1 under PWM, the watchdog end to end, the
gyro z sign, encoder decode at high wheel speed, IMU rate and latency at the
estimator, Procedure B, `track_width_m`, the Spresense (no fix, see
`STATUS_OPEN.md` §9), and anything that moves. `STATUS_OPEN.md` §7 is the
order to take them in.

---

## 10. Closed — moved from `STATUS_OPEN.md`

What used to be open and no longer is, compacted out of `STATUS_OPEN.md` on
2026-10-26. Each line points at its evidence.

| Item | Closed by | Evidence |
|---|---|---|
| Nothing had ever been flashed | Both boards flashed and running, 2026-10-26 | §9 |
| **PHY bring-up on F767ZI** (the hard gate, §11 risk; plan §13.4 item 1) | Passed on both boards: 100 Mb/s full duplex, zero symbol errors | §9.1 |
| `ANAR` / strap gap (`GenericPhy` never writes `ANAR`; a `MODE[2:0]` of `100`/`101` would advertise half duplex) | Did not occur: `ANAR=0x01e1`, `MODE[2:0]=111` on both boards. `ANAR` stays unwritten; plan §13.4a has the reasoning | §9.1 |
| I2C1 on PB8/PB9 (plan §13.4 item 2) | LSM6DSV16X and INA226 both answer | §9.1 |
| Encoder sign convention (plan §13.4 item 3) | Both wheels count up turning forward; the sides were swapped, fixed as D8 | §9.1, §9.2 |
| **Encoder constant wrong after 2×→4×** (§11 risk) | Measured ~1,396 ticks/rev at 4×; the recalled ~1,500 was ~7 % high | §9.1, `HARDWARE.md` §3 |
| Drivetrain uncalibrated (`0.0` everywhere) | `ticks_per_rev` and `metres_per_tick` set; `rover-doctor` GO. Procedure B still open | §9.1 |
| **Jetson Python bus drifts from Rust** (§11 risk) | Golden-byte fixtures checked from both languages, both directions | §4 |
| D1 `PeerId` addressed machines, not processes | `[services]` in `config/rover.toml` | §2 |
| D2 `GnssFix` could not name its receiver | `GnssSource` / `GnssFix.source` | §2 |
| D3 Mission monitor navigated on the uncorrected receiver | `select_navigation_fix`, tested | §2 |
| D6 Camera measurement timestamped after detection | Stamped at capture from the D415's `global_time` clock; `capture_age_us` + `frames_dropped` on the wire; EKF compensates and gates at `lane_max_age_ms`; measured 59 ms median. Re-tune open with D5 | plan §13.3b D6, §9.1 |
| librealsense queue depth not pinned | `camera.py` drains to the newest frame on every read; drops are counted | plan §13.3b D6 |
| D7-D10 (vector table, encoder sides, host drops board datagrams, POST races the link) | Fixed on the bench | §9.2 |
| Toolchain floating on `stable` | `rust-toolchain.toml` pins `1.99.1` | §1 |

---

*How the numbers in this document were measured, 2026-10-17, rustc 1.99.1
(`rust-toolchain.toml` pin; `rustc --version` reports `1.99.1 (58cf8f9c3
2026-10-13)`). Test counts: `cargo test --workspace 2>&1 | grep '^test result'`,
summing the "passed" column (434, 0 failed; per package: rover-msgs 11,
rover-model 8, rover-estimator 16, rover-doctor 27, rover-bus 43,
rover-control 78, rover-telemetry 66, ground-station 42, rover-navigation 87,
rover-runs 15, rover-tap 12, rover-link 10, replay 19); Python:
`cd perception && .venv/bin/python -m pytest -q` (122: `test_wire_golden.py`
79, `test_bus.py` 28, `test_lane_parity.py` 9, `test_lane_sign_convention.py`
3, `test_main_recovery.py` 3). Fixtures: `ls testdata | wc -l` (26); crates:
`ls crates` (12) plus `tools/replay` in the workspace `members` list (13
packages). Replay: a fresh `tools/replay synthetic` + `run` against
`config/rover.toml` (267 of 1000 rows compared, rmse 1.370 deg, max 2.719 deg,
PASS at 3.0 deg and FAIL at 0.5 deg; the rmse was 1.349 deg before the
curvature-sign fix). Firmware: `cargo size --release` in each crate directory
(and `--features calibration` for sensors); **flash = text + data, RAM = data +
bss**, percentages against 2 MB flash and 512 KB SRAM. This is the same
accounting as the older `-A` section sums (`text` is `.vector_table + .text +
.rodata`; `bss` is `.bss + .uninit`), just reproducible with one command; the
chassis breakdown is 1,024 + 57,960 + 10,904 flash and 18,204 + 1,024 RAM
beyond `.data`.*
