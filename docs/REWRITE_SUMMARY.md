# The Rust rewrite, in summary

**What this is:** a short account of what branch `rs` did to this project and
why, for someone who needs the shape of it without reading
[RUST_REWRITE_PLAN.md](RUST_REWRITE_PLAN.md)'s twelve hundred lines.

**Status:** complete as software, unproven as a rover. Every component is
written and tested on a laptop. **Nothing has been flashed to a board, and
the drivetrain is uncalibrated.** Those two facts govern everything below.

Dates: designed and built 19–20 September 2026. `main` still holds the
working ROS 2 rover and is the fallback.

---

## 1. The one-paragraph version

The rover ran on ROS 2 across five machines, including two NUCLEO-F767ZI
boards running mROS 2 — a C/C++ port of ROS 2 — over mbed OS, lwIP and
embeddedRTPS. Branch `rs` deletes all of it and replaces it with plain UDP
datagrams on a static LAN, a single Cargo workspace covering both the Linux
binaries and the microcontroller firmware, and one Python process on the
Jetson for perception. There is no middleware, no discovery, no broker, no
code generation, and no TCP. Along the way the estimator was upgraded from an
exponential moving average to a 5-state EKF, and the firmware gained a
command watchdog it had never had.

---

## 2. Why, specifically

The ROS 2 *usage* was thin: 12 topics, one service, one action, no
third-party ROS packages. Essentially first-order publish/subscribe with
custom message types.

The ROS 2 *cost* was not thin. On the two MCUs, embeddedRTPS needed **five
local patches** and hand-derived RTPS memory pools that had to be
re-calculated whenever a node was added **anywhere on the network** — a
change on the base station PC could HardFault a microcontroller during
discovery. The build was mbed OS + lwIP + CMake + Docker.

So the trade was: give up capabilities the project never used (discovery,
QoS negotiation, multi-vendor DDS interop, a rich ecosystem) in exchange for
deleting an entire class of failure that cost real field time.

### Alternatives considered and rejected

| Option | Why not |
|---|---|
| **Eclipse Zenoh** (the original idea) | Its value is discovery, routing, WAN bridging and shared memory. On a static five-host wired LAN with known addresses, none of that is used — it would have been a dependency paying for itself with features this rover doesn't have. |
| **zenoh-pico** (for the MCUs) | It is C. It keeps the mbed/CMake/Docker toolchain, which was a stated reason for leaving in the first place. |
| **zenoh-nostd** | Early development: TCP only, no serial transport, Interest protocol unimplemented. |
| **MQTT** | Five competing `no_std` crates, none dominant. Picking one is a bet. |
| **Cyphal / canadensis** | UDP transport under-specified, and it reintroduces DSDL code generation — the thing being escaped. |
| **dora-rs, Copper** | No microcontroller story at all. |

**Conclusion: no middleware.** The messaging need here is small enough that a
framework is a liability rather than leverage.

---

## 3. What replaced what

| Before | After |
|---|---|
| ROS 2 Humble + Fast-DDS on Linux | Plain UDP datagrams, static routing table in `config/rover.toml` |
| mROS 2 + embeddedRTPS + lwIP + mbed OS on two MCUs | [Embassy](https://embassy.dev) async Rust, `no_std` |
| `.msg`/`.srv`/`.action` files + `rosidl` codegen | `crates/rover-msgs` — 16 hand-written types, one shared crate |
| Three DDS domains (D4/D5/D6) for isolation | Unicast fan-out from a table. Isolation is not needed when nothing discovers anything. |
| Multicast discovery, `initialPeersList` | Static addresses. Nothing discovers anything, ever. |
| `camera_stream_node` → DDS → `lane_detection_node` | One Python process. The hop was 1:1 on the same machine. |
| 8 ROS nodes on the RPi | 3 Rust binaries: `rover-control`, `rover-navigation`, `rover-telemetry` |
| An EMA filter on lane measurements | A 5-state EKF |
| *(nothing)* | A firmware command watchdog |
| CMake, colcon, Docker, `build.bash` | `cargo build`, `probe-rs` |

### The structural win

**One Cargo workspace builds the host binaries and the MCU firmware against
one shared message crate.** A wire-layout disagreement between the Raspberry
Pi and an STM32 is now a compile error on a laptop instead of a HardFault in
a field. This is the single change that most reduces the cost of being wrong.

---

## 4. What was gained beyond parity

**A 5-state EKF** (`crates/rover-estimator`) replacing an exponential moving
average. The EMA had three problems: at α = 0.05 over 30 Hz it cost roughly
0.67 s of phase lag — 10–13 cm of travel at cruise — inside the steering
loop; it conflated lateral offset with path curvature, which the old control
law's own documentation admitted; and it froze outright whenever lane
detection dropped out. The filter estimates
`[cross_track, heading_err, curvature, speed, gyro_bias]` at the front axle,
uses IMU yaw rate as an input, corrects from camera and odometry through a
chi-square gate, and coasts on gyro plus odometry with growing covariance
when the lane is lost — a continuous confidence signal instead of a cliff.

**A firmware command watchdog**, which the ROS 2 firmware did not have in any
form. Previously, if the Raspberry Pi died mid-drive, the STM32 held its last
PWM value indefinitely and the rover kept going. Now: a 200 ms command
timeout, a 300 ms throttle ramp to zero rather than a step into a loaded
drivetrain, steering centred immediately, and an independent 500 ms hardware
watchdog for hung firmware. Recovery requires an explicit zero-throttle
command — it never resumes by itself just because packets came back.

**A test suite**, where the first-party ROS 2 code had none. (One file was
named `test_lane_pipeline_video.py`, but it contains zero assertions — it is
a manual eyeball harness.)

**Pluggable control.** `LateralController` is a trait. The field-tuned static
gain law is the parity baseline; LQR and MPC become new files rather than a
rewrite.

---

## 5. What it cost, honestly

**More lines, not fewer.** The rewrite wrote ~19,800 lines to replace ~9,600
lines of first-party ROS 2 code. Roughly 6,800 of the new lines are tests
that had no counterpart, and about a third of the non-test Rust is doc
comments recording *why* a constant has the value it does. What genuinely
disappeared is **766 vendored files** of mbed OS, lwIP, embeddedRTPS and
mROS 2 that this project no longer carries, patches, or reasons about.

**No ecosystem.** No `rviz`, no `ros2 bag`, no `ros2 topic echo`. The
replacements are `rover-tap` (a passive bus sniffer) and CSV logging. If this
project later wants a visualisation ecosystem, it will have to build or
import one.

**No field validation is possible from a desk.** There is no recorded ROS 2
run anywhere in this repository and there never was — no `runs/` directory,
no CSV, no bag file. `tools/replay` therefore compares the new pipeline
against an independently written port of the *old* control law fed synthetic
traces. That proves internal consistency and that the harness can catch a
regression. It does **not** prove field parity, and the tool says so itself
every time it runs.

**A known behaviour change.** The old mission monitor navigated on the
*uncorrected* Spresense GNSS and never read the RTK receiver, against a 20 m
arrival radius. The rewrite prefers RTK **and** tightens arrival to 2 m.
Mission arrival will therefore not match the ROS 2 baseline — this is a fix,
not a parity failure, but it must not be mistaken for one.

---

## 6. The numbers

| | |
|---|---|
| Rust written | 16,387 lines — 10,640 non-test (of which 3,527 doc comments), 5,747 test |
| Python written | 3,424 lines — 2,405 production, 1,019 test (excludes `tools/capture_d415_rgb.py`, 83 lines) |
| Vendored parity oracle | 1,130 lines (the ROS 2 lane detector, frozen verbatim) |
| Removed | 919 files, ~137,000 lines |
| ├─ first-party ROS 2 | 44 files, 9,632 lines |
| ├─ vendored MCU stack | 766 files |
| └─ docs, launch tooling, `command/`, `.vscode/` | 109 files |
| Tracked source, before → after | ~12.5 MB → 2.3 MB |
| Tests | 290 Rust, 82 Python |
| Message types | 16, plus the frame header — 17 golden byte fixtures |
| Chassis firmware | 65,048 B flash (3.1 %), 17,724 B RAM (3.4 %) |
| Sensors firmware | 54,316 B flash (2.6 %), 17,844 B RAM (3.4 %) |

---

## 7. Four things that will bite you

**The sign convention is inverted from ISO 8855, deliberately.**
`heading_err_rad`, `cross_track_m` and `steer` are all positive when the
correct response is *steer right*, so both feedback terms carry a **plus**
sign. This is field-derived. "Correcting" it to ISO 8855 without inverting
every consumer in the same commit turns the lateral controller into positive
feedback.

**Tick counts doubled.** The old firmware interrupted on encoder channel A
only — 2× decoding. The Rust firmware decodes both channels on every edge —
4× decoding. Any calibration taken against the old firmware reads exactly
double. `config/rover.toml` records `decoding = "quadrature_4x"` for this
reason.

**Hardware quadrature is impossible on this harness.** STM32 timer encoder
mode needs both channels of one encoder on the same timer. Verified
exhaustively against the F767ZI pin/alternate-function tables: each encoder's
own A/B pair straddles TIM2 and TIM3. This is a fact about how the harness
was soldered, not a software limitation, and it cannot be fixed without a
rewire. The firmware does software 4× decode over EXTI instead — correct, but
without the hardware guarantee that a count can never be missed under load.

**`docs/CONTROL_LAW.md` describes a law the code never implemented.** It
documents a full PID (`k_e1`, `k_e2`, `k_p`, `k_i`, `k_d`, `k_ff`); the
actual ROS 2 node had no PID at all. The port followed the code. This drift
predates the rewrite and exists on `main` too — the document now carries a
banner saying so.

---

## 8. Where it stands

**Done:** all ten Rust crates, both firmware images, the perception process,
the replay gate. 290 Rust tests and 82 Python tests pass; clippy and `rustfmt`
are clean; the replay gate exits 0.

The lane detector port is **proven**, not merely reviewed: the original ROS 2
detector is vendored verbatim at `perception/tests/oracle/` and compared
against exactly, with no tolerance. Perturbing the port's polyfit
coefficients by one part in 10⁷ fails five of nine tests.

**Blocking field use — neither is a coding task:**

1. **Calibrate the drivetrain.** `ticks_per_rev`, `metres_per_tick` and
   `track_width_m` are all `0.0`. No metric speed exists anywhere in the
   system until they are measured. Twenty minutes with a tape measure;
   procedures in plan §2.6. Measure against the **4×** decoder.
2. **Flash something.** Every firmware claim above is about a binary that
   builds, not one that has run. The LAN8742A PHY against Embassy's
   `GenericPhy` is the gate that decides whether any of it is real. Plan
   §13.4 lists the rest of the hardware-verification debt.

**Deferred by choice:** the two ESP32 LoRa links (out of scope by
instruction, but the reason there is no TCP anywhere), and the LQR and MPC
control laws (phase 2 and 3 — the trait exists so they are additions).

---

## 9. Further reading

| Document | For |
|---|---|
| [RUST_REWRITE_PLAN.md](RUST_REWRITE_PLAN.md) | The full design. **§13 is authoritative** over the rest of the file — everything above it is the plan, §13 is what exists. |
| [HARDWARE.md](HARDWARE.md) | Machines, sensors, both pin maps, calibration status. |
| [VISION_PIPELINE.md](VISION_PIPELINE.md) | How a camera frame becomes lane geometry. ROS 2 era, still the reference. |
| [CONTROL_LAW.md](CONTROL_LAW.md) | Steering and speed derivation. ROS 2 era — see the warning above. |
| [CSV_LOGGING.md](CSV_LOGGING.md) | Log schemas. ROS 2 era. |

The ROS 2 tree itself was removed from this branch on 20 September 2026 once
the replay gate passed and the parity oracle was vendored. It survives in
full on `main` and in this branch's history: `git show main:<path>`.
