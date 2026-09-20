# What is built — branch `rs`

Evidence side of the status split: [`STATUS_OPEN.md`](STATUS_OPEN.md) covers
what remains and what to worry about; this document names, for every claim,
the thing that would have failed if it were false. `RUST_REWRITE_PLAN.md`
§13 is the detailed design record; this file does not depend on it.

**Nothing in `firmware/` has ever run on a real board, and neither the
perception pipeline nor the control law has ever driven the actual rover.**
Every result below came from a developer laptop. The Raspberry Pi, both
NUCLEO-F767ZI boards, the Jetson and the drivetrain are all still running
whatever `main` was running last — nothing from this branch has been flashed
or deployed. The rover is not ready to drive, and nothing below changes that.

Three kinds of confidence appear below and must not be blurred: **proven**
(a test exists that fails if the claim is false, and it ran), **built and
reviewed, never run on hardware** (compiles, is unit-tested, has been read
carefully, but no board, sensor or motor has ever seen it), and **believed to
work** (a design judgement, backed by neither). Each section says which kind
of claim it's making.

---

## 1. The headline numbers

Re-verified while writing this document, not copied forward (commands in the
footnote).

The host workspace — 12 crates (`ground-station`, `rover-bus`,
`rover-control`, `rover-doctor`, `rover-estimator`, `rover-link`,
`rover-model`, `rover-msgs`, `rover-navigation`, `rover-runs`, `rover-tap`,
`rover-telemetry`) — is clippy/rustfmt clean, and `cargo test --workspace`
passes **331 tests**, 0 failed. Perception passes **88** Python tests.
`testdata/` holds **18** golden wire fixtures. Both firmware images build for
`thumbv7em-none-eabihf` and pass clippy — sizes and **never run on the boards
they target** in §2's table. That is the entire scope of what's proven: it
builds, and it passes tests on a laptop.

---

## 2. Component status

Carried over from `RUST_REWRITE_PLAN.md` §13.1, re-verified for this
document. The **Evidence** column is the point: every "done" names the test
or artefact that backs it, not just an assertion that it exists.

| Component | State | Evidence |
|---|---|---|
| `rover-msgs` | **done** | 17 message types (plus the `FrameHeader` envelope, which is not a `Wire` type but has its own fixture — hence 18 files in `testdata/`), builds for host **and** `thumbv7em-none-eabihf`. 7 contract tests: round-trip, declared-vs-actual length, trailing bytes, truncation, frame budget, ID uniqueness, golden fixtures. |
| `testdata/*.bin` | **done** | 18 fixtures — the 17 message types plus `FrameHeader`. Regenerating them is a breaking protocol change (see §4). |
| `rover-link` | **done** | `Link` trait, `UdpLink`, `send_to_addr` with an `Unsupported` default so a future LoRa link need not implement it. |
| `rover-bus` | **done** | config loading, unicast fan-out, newest-wins receive, idempotent command handshake, debug mirror. |
| `rover-tap` | **done** | per-type rate and seq-gap loss; `--mirror` sees the whole bus. |
| `rover-model` | **done** | shared `A(v)`/`B(v)`, Euler discretisation. 6 tests. |
| `rover-estimator` | **done** | 5-state EKF, Joseph form, chi-square gate, coast-on-dropout, zero-rate bias update. 10 behavioural tests. |
| `rover-control` | **done** | lib + thin bin so `tools/replay` drives production code. `LateralController` trait; `StaticGain` is the bit-exact port. `actuate.rs` owns every guard rail. |
| `rover-navigation` | **done** | two GNSS receivers, mission state machine, RTCM injection point. |
| `rover-telemetry` | **done** | CSV at native rates, health bits, base feed, `board_diagnostics.csv` with the three latched board health bits. |
| `ground-station` | **done** | mission goals, speed limit, E-stop, live display, `runs/` CSV via the shared `rover-runs` crate. |
| `rover-runs` | **done** | the `run_NNN_<stamp>/` convention, shared by `rover-telemetry` and `ground-station` rather than copy-pasted into each. |
| `rover-doctor` | **done** | preflight GO/NO-GO over 9 checks; verdict logic is a pure function over observations, 25 tests. Distinguishes NOT SEEN from NO-GO and never reports GO on missing data. |
| `tools/replay` | **done** | **the §12 gate.** Synthetic trace + `legacy.rs` oracle. PASS at the 3.0° tolerance (rmse 1.349°, 267 rows compared); deliberately verified to FAIL at 0.5°, so the harness demonstrably has teeth. |
| `perception/wire.py` | **done** | 54 tests green against the Rust fixtures — the two languages provably agree on every byte. |
| `perception/lane.py` | **done** | behaviour-preserving port, **parity proven** — see §3. |
| `perception/{camera,bus,main}.py` | **done** | one process replacing `camera_stream_node` + `lane_detection_node`. End-to-end verified: 20 frames in, 20 `LaneMeasurement` out, seq monotonic, decoded by `wire.py`, clean exit. |
| `firmware/chassis` | **builds** | 69,928 B flash (3.3%), 18,480 B RAM (3.5%). Clippy clean. POST + reset cause + PHY diagnostics incl. ANAR/strap check. **Never run on hardware.** |
| `firmware/sensors` | **builds** | 58,932 B flash (2.8%), 18,648 B RAM (3.6%). Clippy clean. POST + reset cause + PHY diagnostics incl. ANAR/strap check. **Never run on hardware.** |

---

## 3. Why the lane detector port is believable

`perception/lane.py` is the one component where "ported carefully" isn't
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

`crates/rover-msgs` defines **17 message types**. With `FrameHeader` — the
envelope wrapping every message, not itself a `Wire` type but given its own
fixture because framing bugs are exactly what this check exists to catch —
that's **18 golden fixtures** in `testdata/*.bin`. Each is generated once
from Rust and checked from both ends: Rust's own round-trip and
golden-fixture tests decode, re-encode and compare bytes; `perception/wire.py`'s
54 tests decode the same files independently and check the resulting field
values. Both sides agreeing with a byte file neither wrote at test time is
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
  function with no socket, clock or sleep, which is what lets its 25 tests
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
`git show main:<path>` or this branch's own history. Durable knowledge was
harvested first into `docs/HARDWARE.md`. `ws_spresense` was left untouched,
by instruction, and still exists on this branch.

---

## 7. Acceptance criteria (§12) — where they actually stand

`RUST_REWRITE_PLAN.md` §12 lists ten criteria for calling the rewrite done.
Most are **not** met, uniformly because nothing has been flashed and the
drivetrain has never been calibrated. Full reasoning for each open criterion
lives in `STATUS_OPEN.md` §6; this is the verdict.

| # | Criterion | Status |
|---|---|---|
| 1 | `cargo test --workspace` green, fixtures cross-checked against Python | **Met.** 331 Rust tests, 88 Python tests, 18 fixtures — §1, §4. |
| 2 | `tools/replay` reproduces recorded ROS 2 steering | **Not met — cannot be attempted.** No recorded ROS 2 run ever existed; replay proves internal consistency against a frozen oracle (§2, §3), not field parity. `STATUS_OPEN.md` §1.4. |
| 3 | Watchdog: Ethernet pulled while driving → ramp-to-zero | **Not met** — never run on hardware. `STATUS_OPEN.md` §1.2. |
| 4 | IWDG: hung control task → board resets | **Not met** — same. `STATUS_OPEN.md` §1.2. |
| 5 | E-stop end to end, `cmd_seq` echoed | **Not met** — protocol is unit-tested (`rover-bus`) but never run on real firmware. `STATUS_OPEN.md` §1.2. |
| 6 | 0.20 m/s commanded → 0.20 ± 0.02 m/s measured | **Not met** — no metric speed exists yet; `rover-doctor`'s drivetrain check is a hard NO-GO (§5). `STATUS_OPEN.md` §1.1. |
| 7 | 100 Hz IMU at the estimator, latency measured | **Not met** — never run on hardware. `STATUS_OPEN.md` §1.2, §5.1. |
| 8 | EKF coasts a ≥3 s lane dropout without divergence | **Partially met.** `rover-estimator/tests/ekf.rs::lane_dropout_coasts_without_diverging` proves it at unit level — 300 steps (3 s at 100 Hz) of invalid lane feed, covariance provably growing, state finite and bounded (`< 0.5` cross-track/heading error). Not a demonstration on the integrated pipeline or a real occluded camera. |
| 9 | One full field circuit at parity, CSVs compared | **Not met — cannot be attempted.** Requires a driving rover. `STATUS_OPEN.md` §1.4, §6. |
| 10 | `main` still builds and drives | **Not independently re-verified for this document.** `STATUS_OPEN.md` §6. |

One criterion fully met, one proven at the unit level but not end-to-end,
eight not met or not checkable without hardware.

---

*Numbers reproduced while writing this document: `cargo test --workspace`,
perception's pytest suite, `ls testdata | wc -l`, `ls crates`, and a fresh
`tools/replay synthetic` + `tools/replay run` against `config/rover.toml`
(267/1000 rows compared, rmse 1.349°, PASS at 3.0°, FAIL at 0.5°, reproduced
exactly). Firmware flash/RAM figures were measured fresh with
`cargo size --release -- -A` on both images (flash =
`.vector_table + .text + .rodata + .data`, RAM = `.data + .bss + .uninit`), not
inherited from an earlier document.*
