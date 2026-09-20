# almondmatcha-rs

Outdoor autonomous rover: vision-based lane following, RTK GNSS waypoint
navigation, and an EKF tying them together. Five machines on one wired LAN,
two of them bare-metal microcontrollers.

![Almondmatcha](docs/image/almondmatcha2025.jpg)

> **This is a personal, private mirror**, split off 2026-09-20 from the
> original project's `rs` branch so the Rust rewrite could keep moving without
> waiting on the org repo. It carries the rewrite only — **there is no `main`
> here.** The ROS 2 system this replaces, and the rest of the project's
> history, lives at `RoboticsGG/almondmatcha` (public), still on `main`, kept
> as the working fallback. `rs` is this repo's only branch and its default.
>
> Working locally, both remotes are checked out side by side:
> `origin` → this repo, `roboticsgg-almondmatcha` → the upstream org repo.
> Nothing pushes upstream unless a remote is named explicitly.

> **New here?** [`docs/REWRITE_SUMMARY.md`](docs/REWRITE_SUMMARY.md) is the
> ten-minute version: what this rewrite did, why, what it cost, and what still
> blocks it from driving.

---

## What this repo is

This is the entire ROS 2 / DDS / mROS 2 stack, replaced with a single Rust
workspace plus one Python process for perception — the `rs` branch from
`RoboticsGG/almondmatcha`, standalone. **Nothing here has ever been flashed
to a board.**

**There is no middleware.** Messages are plain UDP datagrams on a static LAN,
routed from a table in `config/rover.toml`. No discovery, no QoS negotiation,
no multicast, no broker — and **no TCP anywhere**, because the planned LoRa
base link is one-directional and cannot carry a handshake.

The ROS 2 usage it replaces was thin: 12 topics, one service, one action, no
third-party ROS packages. What it cost was not thin — embeddedRTPS on the two
NUCLEOs needed five local patches and hand-derived RTPS memory pools that had
to be re-checked whenever a node was added *anywhere on the network*, and the
whole MCU toolchain was mbed OS + lwIP + CMake + Docker.

The single biggest structural win: **one Cargo workspace builds the host
binaries and the MCU firmware against one shared message crate**, so a wire
layout disagreement between the RPi and an STM32 is a compile error on a
laptop rather than a HardFault in a field.

> **`docs/RUST_REWRITE_PLAN.md` is the design document.** For what actually
> exists today, read `docs/STATUS_DONE.md` (what is built, and the evidence)
> and `docs/STATUS_OPEN.md` (what remains, and what to worry about). Those two
> are authoritative over any status claim in the plan.

---

## Architecture

> **Jetson = perception. RPi = estimation + control. STM32 = actuation and
> sensing. Base station = observability and optional override, never a
> dependency.**

That last clause is load-bearing: once the base moves to the LoRa link, the
rover must complete a mission with **zero uplink**.

```
  Jetson .5              RPi .1                         Base .10
┌────────────┐   ┌──────────────────────┐              ┌──────────────┐
│ perception │   │ rover-control        │   telemetry  │ ground-      │
│  (Python)  │──>│  ├ estimate  EKF     │─────────────>│  station     │
│  camera +  │UDP│  ├ guide  pluggable  │<─────────────│  (Rust)      │
│  lane      │   │  └ actuate  safety   │   commands   └──────────────┘
└────────────┘   ├──────────────────────┤
                 │ rover-navigation     │<-- RTCM --- ESP32 433 MHz (planned)
                 │ rover-telemetry      │----------> ESP32 923 MHz (planned)
                 └──────────┬───────────┘
                            │ UDP over Ethernet
                 ┌──────────┴───────────┐
                 │ STM32 .2  chassis-fw │
                 │ STM32 .6  sensors-fw │
                 └──────────────────────┘
```

Both LoRa links are **planned, not built** — out of scope on this branch, but
the reason there is no TCP in the design. Machines, sensors, pin maps and
calibration status: [`docs/HARDWARE.md`](docs/HARDWARE.md).

Seven *services* run on five *machines* — the RPi alone runs three processes
that each bind their own UDP port. `config/rover.toml`'s `[services]` table
therefore maps a **process** to a `host:port`, not a machine to a port.

---

## The crates

Twelve library and binary crates, two firmware images, one tool. `rover-msgs`
is at the bottom; everything stacks above it.

### The bus — how anything reaches anything

**`rover-msgs`** — the wire contract, shared by every component. Seventeen
message types plus the 4-byte `FrameHeader`, hand-written encode/decode, no
code generation. Fixed-width little-endian, no padding, no variable-length data
anywhere, SI units named in the field. Every type carries a permanent
`TYPE_ID`. It is `no_std` and builds for `thumbv7em-none-eabihf` as well as the
host — the mechanism behind the "single biggest structural win" above.
`testdata/*.bin` holds golden fixtures Rust generates and Python asserts
against, so the two languages provably agree on every byte.

**`rover-link`** — the link layer. `rover-bus` is written against the `Link`
*trait*, not UDP, because the base station will not always be on the LAN:
adding the LoRa serial link later is a new `impl`, not a redesign. Also where
"no TCP" is decided and explained.

**`rover-bus`** — pub/sub and the command protocol, generic over any `Link`. A
stream is unicast fan-out from the static routing table, **newest-wins**: one
slot per message type, a new arrival overwrites it. No queue, so no queue can
grow unbounded behind a slow subscriber. Commands are separate — idempotent
frames acked by sequence echo and retransmitted until acknowledged, which is
what makes them safe over a lossy one-way radio.

**`rover-tap`** — debug CLI replacing `ros2 topic echo`; per-type rates and
loss inferred from sequence gaps. ⚠️ Unicast fan-out means a tap sees only
traffic addressed to the service it binds as; set `[debug] mirror` to copy
every published frame to it and see the whole bus.

**`rover-runs`** — the `run_NNN_<stamp>/` directory convention, shared by
`rover-telemetry` and `ground-station` so two divergent implementations cannot
drift apart.

### The vehicle — model, estimation, control

**`rover-model`** — the linearised lateral error dynamics `A(v)`/`B(v)`. One
model, three consumers (EKF predict today; LQR and MPC later), so estimator and
controller can never quietly disagree about the vehicle.

**`rover-estimator`** — the 5-state EKF, referenced to the **front axle**:
cross-track error, heading error, curvature, speed, gyro bias. IMU yaw rate as
input, camera and odometry corrections behind a chi-square gate, Joseph-form
covariance, and a zero-rate gyro-bias update that runs every time the rover
pauses. On a lane dropout it coasts with growing covariance instead of freezing.

**`rover-control`** (RPi) — **estimate** → **guide** → **actuate**. Guidance is
a `LateralController` trait, so LQR and MPC are new files rather than a
rewrite; `actuate` owns every guard rail — saturation, slew limiting, speed
cap, stall detection, safety gate — because *controllers propose, actuation
disposes*. A library plus a thin binary, deliberately: that is what lets
`tools/replay` drive the **exact** shipping code rather than a lookalike.

### The rest of the rover

**`rover-navigation`** (RPi) — two GNSS receivers on their own threads (u-blox
RTK, Spresense backup), the waypoint mission state machine, and the RTCM
injection point for the 433 MHz link. Three ROS 2 nodes become one process. It
prefers RTK and falls back to the backup — which the ROS 2 mission monitor
never did, a deliberate behaviour change.

**`rover-telemetry`** (RPi) — per-topic CSV at native rates, the 5 Hz
`Telemetry` feed to base, and the health bits that age each inbound feed and
flag a dead one. One process where ROS 2 used two.

**`ground-station`** (base PC) — mission goals, speed limits, E-stop; live
telemetry; logs what arrives. Works over either link. Read its E-stop notes
before wiring it to anything real.

**`rover-doctor`** (base PC) — preflight GO/NO-GO run before driving. Nine
checks; distinguishes "not seen yet" from "seen and bad", and never reports GO
on missing data.

### Firmware — `no_std` Rust on Embassy

**`firmware/chassis`** (STM32 `.2`) — motor PWM, steering servo, LSM6DSV16X IMU
at 100 Hz, command watchdog. 69,928 B flash (3.3 %), 18,480 B RAM (3.5 %).

**`firmware/sensors`** (STM32 `.6`) — wheel encoders via software 4× quadrature
decode over EXTI (hardware quadrature is impossible on this harness — see
`HARDWARE.md`), INA226 power monitor, link watchdog. 58,932 B flash (2.8 %),
18,648 B RAM (3.6 %).

Both run a power-on self-test and report it, with reset cause and PHY link
state, over `BoardDiagnostics`.

⚠️ **Neither image has ever run on real hardware.** Both build and are clippy
clean; that is all anyone knows about them.

### Everything else

| Path | What it is |
|---|---|
| `perception/` | Jetson. Camera capture + lane detection in **one** Python process. Stays Python for PyTorch/TensorRT. `tests/oracle/` holds the ROS 2 detector frozen verbatim as the parity oracle. |
| `tools/replay` | The §12 gate — replays traces through the real control code against a frozen oracle of the old law. Nothing should drive a motor until this passes. |
| `tools/capture_d415_rgb.py` | Standalone D415 capture utility, outside the workspace; useful for test video. |
| `config/rover.toml` | Single source of truth: `[services]`, `[routes]`, gains, noise, safety timeouts, calibration. Nothing here may be duplicated in code. |
| `testdata/*.bin` | Golden wire fixtures. Regenerating them is a breaking protocol change. |
| `ws_spresense/` | Standalone Arduino GNSS sketches. Untouched by the rewrite. |

---

## ⚠️ The sign convention is inverted from ISO 8855

`cross_track_m` and `steer` are positive when the correct response is *steer
right* — the opposite of ISO 8855. This is field-derived and carried over
deliberately: "correcting" it without inverting every consumer in the same
commit turns the lateral controller into positive feedback.

> **`heading_err_rad` is a different matter, and this heading used to hide
> that.** The detector's `theta` was the *negative* of
> `d(cross_track)/d(distance)` while both the EKF's measurement matrix and
> `RoverState::at_lookahead` assume the positive — and because "inverted on
> purpose" reads as covering all three quantities, the disagreement looked
> intentional for months. **Fixed:** `LaneDetector.detect` now negates before
> publishing, and `perception/tests/test_lane_sign_convention.py` checks the
> detector against the model rather than against its own assumption.
> It is a 3.21× increase in steering authority, so it needs a re-tune —
> `docs/RUST_REWRITE_PLAN.md` §13.3b **D5**.

---

## Build and test

```bash
# Rust workspace — host binaries and tools
cargo test --workspace                      # 331 tests
cargo clippy --workspace --all-targets      # clean
cargo fmt --all --check

cargo build --release -p rover-control      # also: -p rover-navigation,
                                            # -p rover-telemetry, -p ground-station

# Firmware — separate targets, so each builds in its own directory
(cd firmware/chassis && cargo build --release)   # thumbv7em-none-eabihf
(cd firmware/sensors && cargo build --release)

# Perception (first time: create the venv the tests expect)
cd perception
python3 -m venv .venv && .venv/bin/pip install -e '.[dev]'
.venv/bin/python -m pytest tests/ -q        # 82 tests

# The parity gate — `run` needs a trace; `synthetic` makes one
cargo run -p replay -- synthetic --out /tmp/trace.csv
cargo run -p replay -- run --trace /tmp/trace.csv   # exit 0 = within tolerance
```

Firmware flashes with `probe-rs`; logs come back over RTT via `defmt`. No
Docker, no CMake, no mbed.

---

## Before this drives

- [ ] **Calibrate the drivetrain.** `ticks_per_rev`, `metres_per_tick` and
      `track_width_m` are all `0.0`. There is no metric speed in the system
      until they are measured — plan §2.6, about twenty minutes with a tape
      measure. Measure against the **4× decoding** Rust firmware, not the old
      2× firmware, or every speed reads double.
- [ ] **Nothing has met silicon.** The LAN8742A PHY against Embassy's
      `GenericPhy` is the hard gate that decides whether any of the firmware
      is real — plan §13.4 lists the rest of the hardware-verification debt.
- [x] **`heading_err_rad` sign fixed** (plan §13.3b D5). The detector now
      matches the model its output feeds. Regression-tested against the
      geometry, not against its own assumption.
- [ ] **Re-tune after D5.** The fix is a 3.21× increase in straight-line
      steering authority against gains that have never run with a correctly
      signed heading term. Start slow; `config/rover.toml` has the bracket.
- [ ] **Camera measurements carry no capture time** (plan §13.3b D6). `t_us` is
      stamped after lane detection rather than at capture, and the EKF ignores
      it, so a ~100 ms lag is applied as though it were current — and nothing
      logs enough to measure it. Take it together with D5's re-tune.
- [ ] **Replay parity is against synthetic traces only.** No recorded ROS 2
      field data exists in this repository and none ever did, so the gate
      compares against a hand-rolled oracle of the old law rather than the
      rover's actual past behaviour. Producing a real baseline now means
      checking out `main` on the upstream org repo (`roboticsgg-almondmatcha`
      remote) to record one — this repo has no `main` of its own.

---

## Docs

| File | What it is |
|---|---|
| [`REWRITE_SUMMARY.md`](docs/REWRITE_SUMMARY.md) | **Start here.** What the rewrite did, why, what it cost, what blocks it. |
| [`RUST_REWRITE_PLAN.md`](docs/RUST_REWRITE_PLAN.md) | The design document: architecture, rationale, calibration procedures (§2.6), defect narratives (§13.3b), hardware-verification debt (§13.4). |
| [`STATUS_DONE.md`](docs/STATUS_DONE.md) | What is built, and the evidence for each claim. |
| [`STATUS_OPEN.md`](docs/STATUS_OPEN.md) | What remains, the risk register reassessed, and open concerns. **Read before planning a field run.** |
| [`FIELD_TEST.md`](docs/FIELD_TEST.md) | How to run a field test end to end, and where the data lands. |
| [`HARDWARE.md`](docs/HARDWARE.md) | Machines, sensors, both pin maps, calibration status. |
| [`VISION_PIPELINE.md`](docs/VISION_PIPELINE.md) | Camera frame → lane geometry. ROS 2 era, still the reference. |
| [`CONTROL_LAW.md`](docs/CONTROL_LAW.md) | Steering and speed derivation. ROS 2 era — see its banner. |
| [`CSV_LOGGING.md`](docs/CSV_LOGGING.md) | Log schemas and analysis guidance. ROS 2 era. |

The three marked "ROS 2 era" describe the system this branch replaced. They are
kept because the knowledge in them is field-derived and expensive to
regenerate, not because they describe what runs now. Each carries a header
saying so, and where one disagrees with the code, the code is what ships.
