# Almondmatcha

Outdoor autonomous rover: vision-based lane following, RTK GNSS waypoint
navigation, and an EKF tying them together. Five machines on one wired LAN,
two of them bare-metal microcontrollers.

![Almondmatcha](docs/image/almondmatcha2025.jpg)

---

## What this branch is

Branch `rs` replaces the entire ROS 2 / DDS / mROS 2 stack with a single Rust
workspace plus one Python process for perception. `main` still holds the
working ROS 2 rover and is the fallback; nothing here has flown yet.

**There is no middleware.** Messages are plain UDP datagrams on a static
five-host LAN, routed from a table in `config/rover.toml`. No discovery, no
QoS negotiation, no multicast, no broker — and **no TCP anywhere**, because
the planned LoRa base link is one-directional and cannot carry a handshake.

The ROS 2 usage it replaces was thin: 12 topics, one service, one action, no
third-party ROS packages. What it cost was not thin — embeddedRTPS on the two
NUCLEOs needed five local patches and hand-derived RTPS memory pools that had
to be re-checked whenever a node was added *anywhere on the network*, and the
whole MCU toolchain was mbed OS + lwIP + CMake + Docker.

The single biggest structural win: **one Cargo workspace builds the host
binaries and the MCU firmware against one shared message crate**, so a wire
layout disagreement between the RPi and an STM32 is a compile error on a
laptop rather than a HardFault in a field.

> **`docs/RUST_REWRITE_PLAN.md` is the design document, and its §13 is
> authoritative over the rest of the file.** Everything above §13 is the plan;
> §13 records what actually exists and where the implementation knowingly
> diverges from it.

---

## Layout

```
almondmatcha/
├── config/rover.toml       Single source of truth: addresses, routes, gains,
│                           noise, safety timeouts. Nothing here is duplicated
│                           in code — if a constant is in both, one is wrong.
│
├── crates/
│   ├── rover-msgs          The wire contract. 17 message types, hand-written
│   │                       encode/decode, no codegen. Shared by host and MCU.
│   ├── rover-link          UDP sockets and peer addressing.
│   ├── rover-bus           Routing table, publish fan-out, command receive.
│   ├── rover-model         Bicycle kinematics, shared by estimator and control.
│   ├── rover-estimator     5-state EKF.
│   ├── rover-control       RPi: estimate → guide → actuate, three async tasks.
│   ├── rover-navigation    RPi: two GNSS receivers, mission state machine, RTCM.
│   ├── rover-telemetry     RPi: CSV logging and link encoding.
│   ├── ground-station      Base: mission goals, speed limit, E-stop, display.
│   └── rover-tap           Passive bus sniffer for debugging.
│
├── firmware/
│   ├── chassis             STM32 .2 — motors, steering servo, IMU, watchdog.
│   └── sensors             STM32 .6 — encoders, INA226, watchdog.
│
├── perception/             Jetson — camera capture + lane detection in ONE
│                           Python process. Stays Python for PyTorch/TensorRT.
│
├── tools/replay            The parity gate: replays traces through the real
│                           control code against a frozen oracle of the old law.
│
├── testdata/*.bin          Golden wire fixtures. Rust generates them, Python
│                           asserts against them — the two languages provably
│                           agree on every byte.
│
└── ws_spresense/           Standalone Arduino GNSS sketches. Untouched.
```

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
│  camera +  │UDP│  ├ guide     pluggable│<────────────│  (Rust)      │
│  lane      │   │  └ actuate   safety  │   commands   └──────────────┘
└────────────┘   ├──────────────────────┤
                 │ rover-navigation     │<── RTCM ── ESP32 433 MHz
                 │ rover-telemetry      │──> ESP32 923 MHz
                 └──────────┬───────────┘
                            │ UDP over Ethernet
                 ┌──────────┴───────────┐
                 │ STM32 .2  chassis-fw │
                 │ STM32 .6  sensors-fw │
                 └──────────────────────┘
```

Machines, sensors, pin maps and calibration status: **`docs/HARDWARE.md`**.

---

## Two things that are new, not ported

**A 5-state EKF** replaces an exponential moving average. The EMA cost ~0.67 s
of phase lag (α = 0.05 at 30 Hz), which is 10–13 cm of travel at cruise speed,
and it froze outright on a detection dropout. The filter estimates
`[cross_track, heading_err, curvature, speed, gyro_bias]` at the front axle,
takes IMU yaw rate as input, corrects from camera and odometry, and coasts on
gyro + odometry with growing covariance when the lane is lost.

**A firmware command watchdog**, which the ROS 2 firmware did not have at all.
If the RPi died mid-drive, the old STM32 held its last PWM value indefinitely
and the rover kept going. Now: 200 ms command timeout, a 300 ms throttle ramp
to zero (not a step into a loaded drivetrain), steering centred immediately,
plus an independent 500 ms IWDG. Recovery requires an explicit zero-throttle
command — it never resumes by itself just because packets came back.

---

## ⚠️ The sign convention is inverted from ISO 8855, on purpose

`heading_err_rad`, `cross_track_m` and `steer` are **all positive when the
correct response is "steer right."** Both feedback terms therefore carry a
**plus** sign.

This is field-derived and it is carried over deliberately. Inverting it — or
"correcting" it to ISO 8855 without inverting every consumer at the same time
— turns the lateral controller into **positive feedback**.

---

## Build and test

```bash
cargo test --workspace                       # 290 tests
cargo clippy --workspace --all-targets       # clean
cargo fmt --all --check

cargo build --release -p rover-control       # and navigation, telemetry, ground-station

cd firmware/chassis && cargo build --release # thumbv7em-none-eabihf
cd firmware/sensors && cargo build --release

cd perception && .venv/bin/python -m pytest tests/ -q
cargo run -p replay -- <trace>               # the parity gate; exit 0 = parity
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
- [ ] **Nothing has met silicon.** The Rust firmware has never been flashed.
      The LAN8742A PHY against Embassy's `GenericPhy` is the hard gate that
      decides whether any of it is real — plan §13.4 lists the rest.
- [ ] **Replay parity is against synthetic traces only.** No recorded ROS 2
      field data exists anywhere in this repository, so the gate compares
      against a hand-rolled oracle of the old control law, not against the
      rover's actual past behaviour. The harness prints this caveat itself.

---

## Docs

| File | What it is |
|---|---|
| `docs/RUST_REWRITE_PLAN.md` | The design document. **§13 is authoritative.** |
| `docs/HARDWARE.md` | Machines, sensors, pin maps, calibration status. |
| `docs/VISION_PIPELINE.md` | Camera frame → lane geometry. Field-tuned; ROS 2 era. |
| `docs/CONTROL_LAW.md` | Steering and speed law derivation. ROS 2 era. |
| `docs/CSV_LOGGING.md` | Log schema and analysis guidance. ROS 2 era. |

The three marked "ROS 2 era" describe the system this branch replaced. They are
kept because the knowledge in them is field-derived and expensive to
regenerate, not because they describe what runs now. Each carries a header
saying so. Where one disagrees with the code, the code is what ships —
`CONTROL_LAW.md` in particular documents a full PID that the ROS 2 node never
actually implemented, and that drift exists on `main` too.
