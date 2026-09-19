# Rust Rewrite Plan — `almondmatcha` v5

**Status:** proposal, not yet started
**Written:** 2026-09-19 · **Revised:** 2026-09-19 (rev 2)
**Scope:** full replacement of the ROS 2 / DDS / mros2 stack with a Rust workspace,
adding state estimation (EKF), a pluggable control law, a firmware command
watchdog, metric speed, and a link layer that survives the base station leaving
the LAN for LoRa. Big-bang rewrite on a branch; `main` is preserved untouched.

**Rev 3 adds §13, an implementation status table.** Everything above it is the
plan; §13 is what actually exists on branch `rs`. Where the two disagree, §13
is right and the plan text is aspirational.

**Rev 2 changes:** §2.5 metric speed calibration (encoder spec is *not* in the
repo — measurement procedure supplied), §5.2 full watchdog specification, §6 new
link-layer abstraction for the LoRa future, §4 base↔rover protocol changed from
TCP to idempotent datagrams, §2.6 LIS2MDL magnetometer (present on the shield,
currently unused), §8 pluggable controller trait for LQR/MPC.

---

## 0. Decision summary

| Question | Answer |
|---|---|
| Keep ROS 2? | No. 12 topics, 1 service, 1 action, two QoS profiles, zero third-party ROS packages. |
| Middleware? | **None.** Datagrams over a pluggable link layer. |
| STM32 stack? | **Embassy + `embassy-net`.** No mbed OS, no lwIP, no CMake, no Docker. |
| Jetson? | **Stays Python** (TensorRT / PyTorch). Perception only. |
| New: estimation | **EKF** on the RPi — lane + IMU + wheel odometry (+ optional magnetometer). |
| New: control | **Pluggable** `LateralController`: static-gain (current) → LQR → MPC. |
| New: safety | **Firmware command watchdog** on both STM32 boards. |
| New: comms | **Link abstraction** — Ethernet today, LoRa for the base link tomorrow. |
| Migration style | **Big bang** on a branch. `main` keeps the working ROS 2 rover. |

**The fact that makes this viable:** Embassy ships a working STM32F7 Ethernet
example built against `stm32f777zi` — the same die as the F767ZI with crypto
fused on. Same MAC, same package, same pinout.

**The biggest structural win:** one Cargo workspace builds the MCU firmware *and*
the host binaries against one shared message crate. A struct-layout disagreement
between RPi and STM32 becomes a compile error on a laptop, not a HardFault in a field.

---

## 1. Architecture

> **Jetson = perception. RPi = estimation + control. STM32 = actuation + sensing.**
> **Base station = observability and optional override — never a dependency.**

That second line is new in rev 2 and it is load-bearing. Once the base leaves the
LAN for a LoRa link (§6), the rover must complete its mission with **zero uplink**.
This is already nearly true today — `mission_active` comes from the RPi, not the
base — and the rewrite makes it an explicit invariant.

### 1.1 After

```
  Jetson .5              RPi .1                         Base (LAN today,
┌────────────┐   ┌──────────────────────┐                LoRa tomorrow)
│ perception │   │ rover-control        │               ┌──────────────┐
│  (Python)  │   │  ├ estimator (EKF)   │   telemetry   │ ground-      │
│  camera +  │──>│  ├ guidance (plug)   │──────────────>│  station     │
│  lane/YOLO │UDP│  └ chassis actuation │<──────────────│  (Rust)      │
│ one process│   ├──────────────────────┤   commands    └──────────────┘
└────────────┘   │ rover-navigation     │
                 │  GNSS ×2 + mission   │<── RTCM ── ESP32 433 MHz (USB)
                 │  + RTCM injection    │
                 ├──────────────────────┤
                 │ rover-telemetry      │──> ESP32 923 MHz (USB)
                 │  CSV + link encode   │
                 └──────────┬───────────┘
                            │ UDP over Ethernet
                 ┌──────────┴───────────┐
                 │ STM32 .2  chassis-fw │  motors, servo, IMU, watchdog
                 │ STM32 .6  sensors-fw │  encoders, INA226, watchdog
                 └──────────────────────┘
```

### 1.2 Process inventory

| Unit | Host | Lang | Contents |
|---|---|---|---|
| `rover-control` | RPi | Rust | 3 async tasks: **estimate** (EKF @100 Hz), **guide** (pluggable law @50 Hz), **actuate** (safety cap, speed PID, `ChassisCommand` @50 Hz). In-process channels — the hot path never touches the network. |
| `rover-navigation` | RPi | Rust | u-blox + Spresense serial, waypoint mission state machine, **RTCM injection from the 433 MHz link**. |
| `rover-telemetry` | RPi | Rust | CSV at native rates; encodes `Telemetry` (LAN) and `TelemetryLite` (LoRa). |
| `ground-station` | Base | Rust | Mission goals, speed limit, E-stop, live display. Works over either link. |
| `perception` | Jetson | Python | Camera + lane detection (later YOLO/TensorRT) in **one** process. |
| `chassis-fw` | STM32 .2 | Rust `no_std` | Motor PWM, steering servo, LSM6DSV16X @100 Hz, **watchdog**. |
| `sensors-fw` | STM32 .6 | Rust `no_std` | HW quadrature encoders, INA226, **watchdog**. |

---

## 2. State estimation

### 2.1 Why, beyond denoising

1. **The EMA costs ~0.67 s of phase lag.** `ema_alpha = 0.05` over 30 Hz is ~a
   20-sample time constant. At 0.15–0.20 m/s that is **10–13 cm of travel of pure
   lag** in the steering loop.
2. **`b` and `κ` are entangled, and the docs say so.** `compute_lane_params()`
   fits **1.22 m ahead of the front axle**, so `b` is non-zero on a curve even when
   perfectly on line; `CONTROL_LAW.md` warns `k_e2` and `k_ff` "act on partly the
   same information." The EKF **disentangles them** by modelling the lookahead
   geometry in the measurement equation and estimating at the front axle.
3. **Dropouts freeze the current filter.** The EMA updates only on detected frames.
   The EKF coasts on gyro + odometry with growing covariance, yielding a continuous
   confidence signal instead of the `speed_lost_ratio` / 10 s timeout cliff.

### 2.2 Formulation

EKF, 5 states, referenced to the **front axle**:

```
x = [ e_y      cross-track error at front axle     (m)
      e_psi    heading error vs. lane tangent      (rad)
      kappa    lane curvature ahead                (1/m)
      v        forward speed                       (m/s)
      b_g    ] gyro z bias                         (rad/s)
```

**Process** — IMU yaw rate as an *input*:

```
e_y_dot    =  v * sin(e_psi)   ~=  v * e_psi
e_psi_dot  =  (w_gyro - b_g)  -  v * kappa
kappa_dot  =  0 + w_kappa        (random walk)
v_dot      =  0 + w_v            (corrected by odometry)
b_g_dot    =  0 + w_b            (random walk)
```

```
          e_y  e_psi  kappa    v      b_g
 e_y    [  0     v      0     e_psi    0  ]
 e_psi  [  0     0     -v    -kappa   -1  ]
 kappa  [  0     0      0      0       0  ]     F_d ~= I + F_c*dt
 v      [  0     0      0      0       0  ]
 b_g    [  0     0      0      0       0  ]
```

**Camera** (~30 Hz, `valid` only), `L_a = 1.22 m`. Linear under small-angle:

```
h(x)  = [ e_y + L_a*e_psi + kappa*L_a^2/2 ,  e_psi + L_a*kappa ,  kappa ]

H_cam = [ 1  L_a  L_a^2/2  0  0 ]
        [ 0   1     L_a    0  0 ]
        [ 0   0      1     0  0 ]
```

**Odometry** (10 Hz, raised from 4 Hz): `z_v = v`, `H = [0 0 0 1 0]`.

**Zero-rate update:** when throttle is zero and both encoders are quiet, apply
`z = w_gyro` against `h(x) = b_g`. This is how `b_g` actually becomes observable.
Runs free every time the rover pauses.

### 2.3 Implementation rules

- **Fixed-rate predict at 100 Hz**, asynchronous correct on arrival. Also fixes a
  real defect — the current loop is event-driven off the camera topic, so its `dt`
  is "whatever the camera did last" (the node's docstring admits this).
- **Gate camera updates**: reject if `NIS = nu' * S^-1 * nu > 11.34`
  (chi-square, 3 DoF, 99%). Keep the existing shape filter too — different failures.
- **Publish `P` diagonal** in `RoverState`; guidance reduces speed as
  `sqrt(P[e_y])` and `lane_age_ms` grow.
- **Log innovations to CSV from the first bench run.** You cannot tune `Q`/`R` you
  cannot see. Non-negotiable.
- `nalgebra` `SMatrix<f32, 5, 5>` — no allocation, `no_std` capable.
- Ship as a **pure library** (`rover-estimator`), no I/O, driven by `tools/replay`
  over existing `runs/*.csv`. Must be tunable on a laptop.

### 2.4 Heading observability — and what the magnetometer does about it

Without an absolute heading reference, `e_psi` is observable **only** through the
camera. With the lane lost, heading drifts at the residual gyro bias rate — expect
**5–15 s** of useful coasting once bias converges, not minutes.

### 2.5 You already have a magnetometer, and it is unused

**Yes — the X-NUCLEO-IKS4A1 carries a LIS2MDL 3-axis magnetometer.** Confirmed
both by ST's product page and by the vendored
`libs/X-Nucleo-IKS4A1_mbedOS/README.md`, which lists it. The board also carries
LSM6DSO16IS, LIS2DUXS12, SHT40-AD1B, LPS22DF and STTS22H. Your firmware includes
only three headers (`LSM6DSV16X.h`, `STTS22H.h`, `LPS22DF.h`) — **the magnetometer
has never been wired up.**

Driver: [`lis2mdl-rs`](https://crates.io/crates/lis2mdl-rs), `no_std`,
`embedded-hal`, hosted in ST's official
[`st-mems-rust-drivers`](https://github.com/STMicroelectronics/st-mems-rust-drivers) —
same family as the IMU driver, so it costs almost nothing to add.

**But be careful what you expect from it.** Three caveats, in order of importance:

1. **It does not measure `e_psi`.** The magnetometer gives yaw in the *earth* frame.
   `e_psi` is heading error relative to the *lane tangent*. Converting one to the
   other requires knowing the lane's earth-frame heading, which you do not have
   from vision alone.
2. **Hard- and soft-iron distortion on a motorised rover is severe**, and the worst
   part is that it is not static: the field from the drive motors varies with
   current draw. A calibration done stationary will not hold under load.
3. **RTK course-over-ground is probably a better heading reference.** With an RTK
   fix and speed above ~0.3 m/s, the u-blox track angle is excellent and needs no
   calibration at all.

**Recommendation:** add the LIS2MDL as an **optional gyro-bias aid**, behind a
config flag, fused as a yaw-rate-consistency check rather than an absolute heading
measurement. Its real value is extending the coast window in §2.4 by constraining
`b_g` better than the zero-rate update alone. Prefer RTK COG when moving. Do **not**
make the control law depend on it. Treat it as a phase-2 experiment, not a
requirement for first field runs.

### 2.6 Metric speed — the encoder constant is NOT in this repo

You asked me to check the docs for ticks per wheel revolution. **It is not recorded
anywhere.** I searched `docs/`, `ws_rpi/`, the firmware tree and all READMEs: no
PPR, no CPR, no gear ratio, no motor part number. The only related constant is
`max_ticks_per_sec: 1000.0` in `chassis_speed_control_params.yaml`, explicitly
labelled `PLACEHOLDER`, with `use_closed_loop_speed: false` as the shipped default.
There is no metric speed anywhere in the system today.

**One thing I *can* tell you from the code, and it matters:** the current firmware
is **2× decoding, not 4×**. In `encoder_control.cpp` only channel A has interrupts
attached (`rise` + `fall`); channel B is read only as a direction input. So you get
2 counts per quadrature cycle.

> ⚠️ **If you move to STM32 hardware quadrature timer mode (recommended, §5.4),
> you get 4× decoding and every tick count doubles.** Calibrate *after* choosing
> the decoding mode, and record which mode the constant belongs to in
> `config/rover.toml`. Getting this wrong makes the rover run at half or double
> the commanded speed.

**The maths, with your 12.5 cm wheel:**

```
wheel_circumference = pi * 0.125 m           = 0.39270 m / revolution
metres_per_tick     = 0.39270 / ticks_per_revolution
speed_mps           = ticks_per_second * metres_per_tick
```

**Procedure A — ticks per revolution (answers your actual question, ~5 min).**
Preferred: it isolates the constant and absorbs gear ratio automatically.

1. Jack the drive wheel clear of the ground. Mark the tyre and the chassis.
2. Log `chassis_sensors.csv`, note the starting tick count.
3. Rotate the wheel **by hand**, forward, exactly **10 full revolutions**.
4. Note the ending count. `ticks_per_revolution = (end - start) / 10`.
5. Repeat for the other wheel. They should agree within a few percent; if not,
   you have a wiring or mounting asymmetry worth finding before you go further.

**Procedure B — metres per tick directly (~10 min, do this too).**
Absorbs tyre compression and real rolling radius, which Procedure A does not.

1. On the actual field surface, mark a **10.00 m** straight line.
2. Drive the rover open-loop along it, logging `chassis_sensors.csv`.
3. `metres_per_tick = 10.00 / mean(total_ticks_left, total_ticks_right)`.
4. Three trials, average.

Use **B** for `metres_per_tick` in production; use **A** to sanity-check it and to
have the number you asked for. If they disagree by more than ~5%, suspect slip.

Store in `config/rover.toml`:

```toml
[drivetrain]
wheel_diameter_m   = 0.125
decoding           = "quadrature_4x"   # or "edge_2x" — MUST match the firmware
ticks_per_rev      = 0.0               # from Procedure A
metres_per_tick    = 0.0               # from Procedure B  <- authoritative
track_width_m      = 0.0               # measure: needed for differential yaw rate
```

`track_width_m` is worth measuring at the same time — with left/right tick rates
you get a second yaw-rate estimate, which is a useful cross-check against the gyro
and costs nothing.

**This blocks the estimator.** Do it early; it is twenty minutes of work.

### 2.7 Port fidelity: the existing gains stay valid

```
u = k_lat * e_y_at_lookahead + k_head * e_psi_at_lookahead + atan(L * kappa)

  e_y_at_lookahead   = e_y + L_a*e_psi + kappa*L_a^2/2
  e_psi_at_lookahead = e_psi + L_a*kappa
```

Feeding the controller the **reconstructed lookahead values** keeps
`k_lat = 181.17 deg/m` and `k_head = 2.024 deg/deg` — both field-derived — valid
unchanged. The EKF is a drop-in for the EMA, not a reason to re-tune from scratch.

---

## 3. Message catalogue

### 3.1 Conventions

| Rule | Rationale |
|---|---|
| No `tpc_` / `srv_` prefixes | Hungarian notation for a system where everything is a message |
| Type names `PascalCase` nouns | `ChassisCommand`, not `ChassisCtrl` |
| Fields plain `snake_case`, **no `_msg` suffix** | `steer`, not `ro_ctrl_msg` |
| **SI units, unit in the name** where ambiguous | `cross_track_m`, `speed_mps` |
| No scaled integers on the LAN | the `int32 = value*1000` IMU encoding was an mros2 workaround |
| No `*_valid` bool pairs | `Option<T>` in Rust, presence bitmask on the wire |
| Compose, don't re-flatten | `Telemetry` embeds `RoverState` |
| **Packed integers only on the LoRa frame** | bandwidth, §6.3 — and only there |

### 3.2 Types

```rust
// ---- STM32 chassis <-> RPi ----------------------------------------------
struct ImuSample   { accel_mps2: [f32;3], gyro_radps: [f32;3], t_us: u32 }
struct MagSample   { field_gauss: [f32;3], t_us: u32 }          // optional, §2.5
struct ChassisCommand {
    steer:    f32,   // -1.0 full left .. +1.0 full right
    throttle: f32,   // -1.0 full reverse .. +1.0 full forward
    seq:      u16,   // watchdog + liveness, §5.2
}
struct ChassisStatus {                                          // NEW, §5.2
    seq_echo: u16, watchdog_tripped: bool, fault: FaultBits, t_us: u32,
}

// ---- STM32 sensors -> RPi ------------------------------------------------
struct WheelSensors { ticks_left: i32, ticks_right: i32, t_us: u32 }   // 10 Hz
struct PowerSample  { bus_volts: f32, current_amps: f32 }              //  5 Hz

// ---- GNSS (one type, two streams) ---------------------------------------
enum FixQuality { None, Autonomous, Dgps, RtkFloat, RtkFixed }
struct GnssFix {
    lat_deg: f64, lon_deg: f64, alt_m: f32,
    fix: FixQuality, sats: u8, h_acc_m: f32,
    speed_mps: f32, course_deg: f32,   // course = heading reference, §2.5
    utc_ms: u64,
}

// ---- Jetson -> RPi -------------------------------------------------------
struct LaneMeasurement {
    curvature_inv_m: f32,   // converted to 1/m AT SOURCE, §3.3
    heading_err_rad: f32,
    cross_track_m:   f32,   // at the 1.22 m lookahead point
    valid: bool, t_us: u32,
}

// ---- Estimator -> guidance ----------------------------------------------
struct RoverState {
    cross_track_m: f32,     // at the FRONT AXLE
    heading_err_rad: f32, curvature_inv_m: f32,
    speed_mps: f32, gyro_bias_radps: f32,
    p_diag: [f32;5], lane_age_ms: u16,
}
struct MotionSetpoint { steer_rad: f32, speed_mps: f32 }

// ---- Mission -------------------------------------------------------------
struct MissionGoal   { lat_deg: f64, lon_deg: f64 }
struct MissionStatus { active: bool, distance_remaining_m: f32,
                       target: Option<MissionGoal>, state: MissionState }

// ---- Base link -----------------------------------------------------------
struct Telemetry {                              // LAN, ~5 Hz, ~170 B
    seq: u32, t_us: u64,
    state: RoverState, mission: MissionStatus,
    power: PowerSample, rtk: GnssFix, backup: GnssFix,
    last_cmd_seq: u16,                          // command ack, §4.2
    health: HealthBits,
}
// TelemetryLite: see §6.3 — 18 bytes, LoRa only.

struct CommandFrame {                           // base -> rover, idempotent
    cmd_seq: u16,
    body: Command,   // SetSpeedLimit | SetMissionGoal | CancelMission | EStop
}

// ---- Debug ---------------------------------------------------------------
struct SpeedLoopDebug { measured_left_tps: f32, measured_right_tps: f32,
                        target_tps: f32, error_pct: f32, pid_output_pct: f32 }
struct EkfDebug       { innovation: [f32;3], nis: f32, gated: bool }
```

### 3.3 Substantive changes, not just renames

**`ChassisCtrl` loses its direction/magnitude split.** Today the wire carries
`fdr_msg` (1=right/2=straight/3=left) *and* `ro_ctrl_msg` (0.0–1.0), plus
`bdr_msg` (0=stop/1=fwd/2=bwd) *and* `spd_msg` (0–255) — four fields encoding two
signed quantities. H-bridge direction pins are a *firmware* concern. The wire
carries `steer: f32` and `throttle: f32` in `[-1, 1]`.

**Curvature converts to 1/m at the source.** Today `κ` ships as a parabola
coefficient in BEV pixels and every consumer folds in `BEV_PX_PER_M = 200.0`
independently — `CONTROL_LAW.md` documents the asymmetry with `b`, which *is*
converted. One conversion, one place.

**`ChassisStatus` is new** and exists so the watchdog is observable (§5.2). Today
the rover cannot tell you whether the chassis board thinks it is being commanded.

### 3.4 Old → new

| Old | New |
|---|---|
| `msgs_ifaces/ChassisCtrl` | `ChassisCommand` |
| `msgs_ifaces/ChassisIMU` | `ImuSample` |
| `msgs_ifaces/ChassisSensors` | `WheelSensors` + `PowerSample` (split: different rates and consumers) |
| `msgs_ifaces/SpresenseGNSS`, `UbloxGNSS` | `GnssFix` (one type, two streams) |
| `msgs_ifaces/TelemetryRelay` | `Telemetry` (composed; dead `lane_*` fields dropped) |
| `tpc_rover_nav_lane` `Float32MultiArray[4]` | `LaneMeasurement` |
| `tpc_rover_ctrl_cmd` `Float32MultiArray[3]` | `MotionSetpoint` |
| `tpc_chassis_speed_debug` `Float32MultiArray[6]` | `SpeedLoopDebug` |
| `action_ifaces/DesData` | `CommandFrame::SetMissionGoal` + `MissionStatus` in telemetry |
| `services_ifaces/SpdLimit` | `CommandFrame::SetSpeedLimit` |
| — | **`RoverState`**, **`ChassisStatus`**, **`MagSample`**, **`EkfDebug`**, **`TelemetryLite`** |

---

## 4. Bus protocol

### 4.1 Streams — datagrams, unicast fan-out

```
+--------+--------+------------------+
| type   | seq    | payload          |
| u16 LE | u16 LE | fixed-layout LE  |
+--------+--------+------------------+
```

Stable `u16` per type. Publish = encode, `send_to` each subscriber in a static
routing table. Subscribe = bind, filter on type ID. Newest-wins; no retransmit,
no ordering, no session state.

**Unicast fan-out, not multicast.** With 4 hosts and sub-100-byte messages this
costs nothing, and it removes the IGMP dependency that forced the hand-maintained
`initialPeersList` workaround in `fastdds_rover.xml`.

### 4.2 Commands — idempotent datagrams with sequence echo (**changed in rev 2**)

Rev 1 used TCP request/response for `SetSpeedLimit` and mission goals. **That is
now dropped**, because TCP cannot cross a LoRa link (§6). Instead:

- Base sends `CommandFrame { cmd_seq, body }` as a plain datagram, repeatedly,
  at ~1 Hz, until acknowledged.
- Rover applies any frame whose `cmd_seq` is newer than the last applied, and
  echoes `last_cmd_seq` in every `Telemetry` / `TelemetryLite`.
- Base stops retransmitting once it sees its `cmd_seq` echoed.

Roughly 30 lines. It is link-agnostic, loss-tolerant, needs no connection state,
and works identically over Ethernet and LoRa. **This is strictly better than the
rev-1 TCP design even on the LAN**, and it means `rover-bus` carries no TCP at all.

Bulk transfers (log retrieval) are not a bus concern — use `rsync` over SSH when
on the LAN.

### 4.3 Encoding

Fixed-layout little-endian. Strings become fixed `[u8; N]`. Hand-written
`encode`/`decode`, ~10 lines each. No serde, no codegen, no schema compiler.

**Golden-byte tests are mandatory**, Rust *and* Python, over shared
`testdata/*.bin`. This is the only thing standing between you and silent
cross-language layout drift, and it replaces what `.msg` codegen did for you.

```python
CHASSIS_CMD = struct.Struct("<ffH")
```

---

## 5. Firmware

### 5.1 Drivers — all exist, no C porting required

**LSM6DSV16X:** ST publishes **official Rust drivers**
([`st-mems-rust-drivers`](https://github.com/STMicroelectronics/st-mems-rust-drivers)).
Use [`lsm6dsv16x-rs`](https://crates.io/crates/lsm6dsv16x-rs) v2.1.0 — `no_std`,
BSD-3-Clause, `embedded-hal` 1.0, same vendor and register abstraction as the C
driver already vendored in `libs/`.

```rust
let mut imu = Lsm6dsv16x::new_i2c(i2c, I2CAddress::I2cAddH, delay)?;
imu.reset_set(Reset::RestoreCtrlRegs)?;
while imu.reset_get()? != Reset::Ready {}
imu.block_data_update_set(1)?;
imu.xl_data_rate_set(Odr::_120hz)?;
imu.gy_data_rate_set(Odr::_120hz)?;
imu.xl_full_scale_set(XlFullScale::_4g)?;
imu.gy_full_scale_set(GyFullScale::_500dps)?;
```

Wrap in a thin `Imu` type owning ODR/full-scale config and raw→SI conversion, so
nothing downstream sees LSB counts. (Current code publishes raw LSBs nominally
scaled by 1000 — the conversion was never actually applied anywhere.)

*Fallback:* a direct register driver is ~250 lines and covers 100% of current
usage. Register map in Appendix A.

**LIS2MDL magnetometer:** [`lis2mdl-rs`](https://crates.io/crates/lis2mdl-rs),
also in ST's official repo. Optional — see §2.5 for why to be cautious.

**INA226:** [`ina226`](https://crates.io/crates/ina226) or
[`ina226-tp`](https://lib.rs/crates/ina226-tp), both `no_std` + `embedded-hal` 1.0.

### 5.2 Command watchdog — SPECIFICATION

> **This closes a live safety gap.** In the current firmware, `motor_control_task`
> acts only when `command_updated` is set. If no command arrives it does nothing —
> **the last applied PWM is held indefinitely.** If the RPi dies, the link drops,
> or `rover-control` crashes mid-drive, the rover keeps driving at its last
> throttle until something physically stops it. There is no timeout anywhere in
> the path.

**Behaviour (chassis board):**

| Parameter | Value | Rationale |
|---|---|---|
| `CMD_TIMEOUT` | **200 ms** | 10 missed frames at the 50 Hz command rate |
| `RAMP_TIME` | **300 ms** | throttle → 0 on a ramp, not a step: a step into a loaded drivetrain is a mechanical shock |
| steering on trip | **centre** | a latched steering angle turns a runaway into a circle |
| recovery | **automatic**, but only after an explicit zero-throttle command | prevents a flapping link from producing lurch-stop-lurch |
| observability | `ChassisStatus.watchdog_tripped` | today you cannot tell this happened |

```rust
// chassis-fw, control task — sketch
const CMD_TIMEOUT:  Duration = Duration::from_millis(200);
const RAMP_TIME:    Duration = Duration::from_millis(300);

loop {
    match select(cmd_rx.receive(), Timer::after(CMD_TIMEOUT)).await {
        Either::First(cmd) => {
            if !tripped {
                apply(cmd.steer, cmd.throttle);
                last_seq = cmd.seq;
            } else if cmd.throttle.abs() < EPS {
                tripped = false;            // explicit zero re-arms
            }
        }
        Either::Second(_) => {
            tripped = true;
            ramp_throttle_to_zero(RAMP_TIME).await;
            centre_steering();
            defmt::warn!("watchdog: no ChassisCommand for {}ms", CMD_TIMEOUT.as_millis());
        }
    }
}
```

**Sensors board:** it publishes only, so it has no actuation to cut. It gets the
mirror-image watchdog instead — if the RPi stops *consuming* (no `Telemetry`
heartbeat seen for 2 s), it logs and flashes the status LED. Cheap, and it makes a
one-way link failure visible at the board rather than silent.

**A second, independent layer: the STM32 IWDG.** The command watchdog protects
against a dead *peer*. The hardware independent watchdog protects against a dead
*firmware* — a hung task, a deadlock, a stuck I2C transaction. Enable IWDG at
~500 ms and pet it only from the control task, so a hang in that task resets the
board (which drops PWM to a safe state on reset). The current firmware has neither
layer. Wire both.

**Acceptance test (§11):** rover on blocks, driving; pull the Ethernet cable.
Motors must ramp to zero within 200 ms + 300 ms, steering must centre,
`watchdog_tripped` must appear in the next telemetry frame that gets through.

### 5.3 Rate and dead-code changes

- **IMU at 100 Hz, not 10 Hz.** The board already samples at 100 Hz and discards
  9 of every 10 samples. The EKF wants all of them. Free.
- **Delete the sensors board's GNSS reader.** `gnss_reader_task` runs on USART6
  with a 4 KB stack, stores NMEA into `sensor_data.nmea_sentence`, and **never
  publishes it** — it appears only in a heartbeat `printf`, while the RPi reads the
  u-blox directly on `/dev/ttyACM0`. ~260 lines plus a thread, serving a debug
  print, on the most memory-constrained node.
- **`defmt` over RTT** replaces `printf` at 115200 into minicom. Structured,
  timestamped, near-zero cost. The on-board ST-Link works with `probe-rs` — no
  extra hardware.
- Dead `MAIN_LOOP_PERIOD_MS` and the unreachable `mros2::spin()` after
  `while(true)` both vanish. Noted so they are not faithfully reproduced.

### 5.4 Encoders: 4× decoding — but in software, not hardware

**This section was wrong in rev 1–3 and is corrected here.** The plan called for
STM32 timer encoder mode (zero CPU, no missed counts). That is impossible with
this rover's physical wiring, and the compiler says so:

| Pin pair | TIM2 | TIM3 |
|---|---|---|
| Encoder A as wired, `{PA15, PB5}` | `PB5: TimerPin<TIM2>` unsatisfied | `PA15: TimerPin<TIM3>` unsatisfied |
| Encoder B as wired, `{PB3, PB4}` | `PB4: TimerPin<TIM2>` unsatisfied | `PB3: TimerPin<TIM3>` unsatisfied |

`PA15` and `PB3` are TIM2 channels; `PB4` and `PB5` are TIM3 channels. Each
encoder's A/B pair therefore straddles two timers, and no alternate function on
any of the four pins changes that. The pairs that *would* work — `{PA15, PB3}`
on TIM2, `{PB4, PB5}` on TIM3 — each combine one channel from encoder A with one
from encoder B, which is a harness rewire, not something firmware can do.

The implementation instead does **software quadrature decode over EXTI
interrupts on all four channels**, through a 4-state transition table that
rejects an ambiguous transition as zero rather than guessing a direction.

This still delivers the thing that actually mattered: **4× decoding**, matching
`config/rover.toml`'s `decoding = "quadrature_4x"`. What it gives up is the
"zero CPU, cannot miss a count" property — so tick-rate behaviour under
sustained load, competing with UDP and I2C interrupts, is now a bench question
rather than a hardware guarantee.

> ⚠️ Decoding still changes from **2× to 4×** versus the mros2 firmware, which
> attached interrupts to channel A only. Tick counts double. See §2.6 —
> calibrate against *this* firmware, and only after it is running.

---

## 6. Link layer and the LoRa future

You have two ESP32 LoRa boards on the RPi over USB: **433 MHz** carrying RTK
corrections base→rover, and **923 MHz** carrying telemetry rover→base. The
intended end state is that base and rover are **not on the same LAN** and are
separated by distance.

**Short answer: yes, the proposal supports this — and it is the reason §4.2
changed from TCP to idempotent datagrams.** TCP cannot live on a link like this.
Datagrams with sequence-echo can.

### 6.1 The `Link` abstraction

```rust
pub trait Link {
    fn send(&mut self, dest: PeerId, frame: &[u8]) -> Result<(), LinkError>;
    fn recv(&mut self) -> Option<(PeerId, Frame)>;
    fn mtu(&self) -> usize;
    fn class(&self) -> LinkClass;   // Lan { .. } | Constrained { bps, duty_pct }
}
```

Implementations: `UdpLink` (Ethernet, today) and `LoraSerialLink` (framed bytes to
the ESP32 over USB CDC). `rover-bus` is written against `Link`, so swapping the
base station onto LoRa is a config change plus one new impl — the message types,
the command protocol and the application code are untouched.

`LinkClass` matters: `rover-telemetry` picks `Telemetry` on a LAN link and
`TelemetryLite` on a constrained one, automatically.

### 6.2 The invariant this depends on

**The rover must complete its mission with zero uplink.** Already nearly true —
`mission_active` is produced on the RPi, not the base. The rewrite makes it
explicit and testable:

- Mission goals may be **pre-loaded** in `config/rover.toml` and armed before departure.
- The base link is **observability plus optional override**. Losing it must never
  stop or endanger the rover.
- E-stop over a lossy link is best-effort by nature. The *reliable* stop is the
  onboard one: the §5.2 watchdog plus the mission state machine. Do not design
  safety around a radio.

### 6.3 `TelemetryLite` — 18 bytes

The LAN `Telemetry` frame is ~170 bytes. That does not fit a long-range LoRa
budget. Packed frame for constrained links:

| Field | Type | Encoding | B |
|---|---|---|---|
| `seq` | u8 | wraps | 1 |
| `lat` | i32 | deg × 1e7 (~1.1 cm) | 4 |
| `lon` | i32 | deg × 1e7 | 4 |
| `speed` | u8 | 0.02 m/s (0–5.1) | 1 |
| `heading_err` | i8 | 0.5° | 1 |
| `cross_track` | i8 | 2 cm (±2.54 m) | 1 |
| `dist_remaining` | u16 | metres | 2 |
| `volts` | u8 | 0.1 V offset | 1 |
| `amps` | u8 | 0.1 A | 1 |
| `fix` + `sats` | u8 | 3 bits + 5 bits | 1 |
| `state` + `health` | u8 | packed | 1 |
| **total** | | | **18** |

Indicative LoRa raw rates at BW125: SF7 ≈ 5.5 kbps, SF9 ≈ 1.8 kbps,
SF10 ≈ 0.98 kbps, SF12 ≈ 0.29 kbps. An 18-byte payload at SF10 is roughly 330 ms
of airtime, so **0.5–1 Hz is comfortable** with headroom for duty-cycle limits.
Treat these as planning figures and confirm the dwell-time and duty-cycle rules
that apply to you in the 920–925 MHz band before relying on a rate.

### 6.4 RTK corrections over 433 MHz — be realistic

This is the harder of the two links. A minimal RTCM3 set (1005 plus MSM4 for
GPS + GLONASS at 1 Hz) is roughly **300–600 bytes/s ≈ 2.4–4.8 kbps**. That
**exceeds long-range LoRa spreading factors** — SF10 gives you about 1 kbps.

Options, best first:

1. **Run the 433 link in FSK, not LoRa.** SX127x/SX126x radios do FSK at
   19.2–50 kbps, which is ample. This is what most RTK correction links actually
   use. You give up LoRa's processing gain, but 433 MHz propagates well and you
   are not trying for tens of kilometres.
2. **Trim the RTCM set.** GPS-only MSM4 plus 1005 at 1 Hz roughly halves it.
3. **Drop the correction rate to 0.5 Hz.** Most receivers hold RTK fix fine;
   corrections are slowly varying.
4. **Accept RTK Float.** Even float is decimetre-class — likely fine for a
   lane-following rover whose primary lateral reference is the camera anyway.

**Architecturally it is simple either way:** `rover-navigation` opens the 433
ESP32's USB CDC port, reads framed RTCM3, and writes bytes straight to the u-blox
serial port. A byte pump with framing — no parsing required. Add it as a task in
`rover-navigation` alongside the existing GNSS readers.

### 6.5 Open question you need to answer

**Is there a command uplink at all in the disconnected configuration?**
As described, 923 MHz is rover→base telemetry and 433 MHz is base→rover RTCM.
That leaves no path for `CommandFrame`. Three ways out:

- **(a)** Run 923 MHz bidirectionally — LoRa radios are half-duplex transceivers,
  so this is a firmware change on the ESP32, not new hardware. Cleanest.
- **(b)** Multiplex commands into the 433 stream alongside RTCM. Workable; adds
  framing complexity to the link that most needs its bandwidth.
- **(c)** No uplink. Missions are pre-loaded before departure; the base is
  telemetry-only. Simplest, and viable given §6.2 — but you lose remote E-stop,
  which is a real operational loss on an outdoor rover.

I would take **(a)**. Decide before §9 step 3, because it sets what `LoraSerialLink`
has to do.

---

## 7. Repository layout

New work on branch `rust-rewrite`. `main` keeps the ROS 2 system.

```
almondmatcha/
├── Cargo.toml                   # workspace: firmware + host, one message crate
├── rust-toolchain.toml
├── .cargo/config.toml           # target aliases, probe-rs runner
├── config/
│   └── rover.toml               # hosts, routes, drivetrain calib, gains, link cfg
├── crates/
│   ├── rover-msgs/              # no_std. wire types + codec + golden tests
│   ├── rover-link/              # Link trait: UdpLink, LoraSerialLink
│   ├── rover-bus/               # pub/sub + command-echo over any Link
│   ├── rover-model/             # linearised error dynamics — shared, §8
│   ├── rover-estimator/         # EKF. pure math, no I/O, laptop-testable
│   ├── rover-control/           # bin  (RPi)  estimate + guide + actuate
│   ├── rover-navigation/        # bin  (RPi)  GNSS + mission + RTCM injection
│   ├── rover-telemetry/         # bin  (RPi)  CSV + link encode
│   ├── ground-station/          # bin  (Base)
│   └── rover-tap/               # bin  debug CLI — replaces `ros2 topic echo`
├── firmware/
│   ├── chassis/                 # no_std, thumbv7em-none-eabihf
│   └── sensors/                 # no_std
├── perception/                  # Jetson Python
│   ├── pyproject.toml
│   └── rover_perception/{camera,lane,bus,main}.py
├── tools/replay/                # CSV replay harness for estimator + control
├── testdata/                    # golden wire-format fixtures
└── docs/
```

**Deleted:** `common_ifaces/`, `sync_stm32_interfaces.sh`, both `mros2_add_msgs/`
trees, `platform/patches/` (5 RTPS patches), `platform/rtps/`, `libs.zip`,
`fastdds_rover.xml`, every `package.xml` / `CMakeLists.txt` / `setup.py`, all
`launch/*.launch.py`.

**Untouched:** `ws_spresense/` — standalone Arduino, no ROS dependency, works.

---

## 8. Control law — built to be swapped

You intend to move from the current static-gain law to LQR, then MPC. The
guidance task is therefore a trait, selected in config:

```rust
pub trait LateralController {
    fn steer(&mut self, s: &RoverState, dt: f32) -> f32;   // radians
    fn reset(&mut self);
    fn name(&self) -> &'static str;
}
```

| Impl | Status | Notes |
|---|---|---|
| `StaticGain` | **port first** | exactly today's law — `k_lat`, `k_head`, Ackermann FF. The parity baseline. |
| `Lqr` | phase 2 | solve the DARE on the §2.2 error model at startup, re-solved per speed bin (the model is speed-dependent — `A` contains `v`). |
| `Mpc` | phase 3 | horizon N≈10 at 50 Hz, condensed to a box-constrained QP in N variables. |

### 8.1 One model, three consumers

`rover-model` holds the linearised lateral error dynamics `A(v)`, `B(v)` — and it
is **the same model the EKF already uses**. That gives a genuinely tidy structure:

```
rover-model ──> rover-estimator   (F matrix for the EKF predict)
            ──> Lqr               (DARE solution -> gains)
            ──> Mpc               (prediction model over the horizon)
```

Change the vehicle model once and estimator and controller stay consistent. This
is worth more than it looks: today the "LQR" gains in
`rover_kinematic_control_node.py` are hardcoded numbers from an offline derivation
with no model in the codebase to check them against.

### 8.2 MPC solver

Do **not** reach for a general solver. Condensed over horizon N with one input,
the QP has N variables (≈10) and box constraints on steering only. A dense
projected-gradient or active-set solve is ~80 lines with `nalgebra` and runs in
microseconds on an RPi 4 — comfortably inside a 20 ms budget. If you later need
general inequality constraints, [`clarabel`](https://crates.io/crates/clarabel) is
pure Rust and a clean escape hatch. Avoid `osqp` — it is C, and C is what you are
leaving.

### 8.3 Guard rails, whichever law is active

The actuation task owns these regardless of controller, so a bad experimental law
cannot hurt the hardware:

- `steer_max_deg = 45.0` saturation
- slew-rate limit on steering
- `spd_limit_cap` safety ceiling
- speed reduction as `sqrt(P[e_y])` and `lane_age_ms` grow
- stall detection and the §5.2 watchdog

Controllers propose. The actuation task disposes.

---

## 9. Work breakdown

Big-bang, so this is build order. Each step is testable.

| # | Work | Testable how |
|---|---|---|
| 0 | **Drivetrain calibration** (§2.6, Procedures A + B) | tape measure — blocks step 5 |
| 1 | Workspace skeleton, `rover.toml`, toolchain, CI | `cargo build` |
| 2 | `rover-msgs` + golden fixtures (Rust **and** Python) | `cargo test` + `pytest` |
| 3 | `rover-link` + `rover-bus` (UDP + command echo) + `rover-tap` | two processes on a laptop |
| 4 | **Spike:** Embassy blink → static IP → UDP echo on one NUCLEO | **hard gate — §10** |
| 5 | `rover-model` + `rover-estimator` EKF + `tools/replay` | offline, laptop |
| 6 | `sensors-fw` — HW quadrature, INA226, watchdog | bench, `rover-tap` |
| 7 | `chassis-fw` — PWM, servo, IMU @100 Hz, **watchdog + IWDG** | bench, motors on blocks |
| 8 | `rover-control` — estimate + guide (`StaticGain`) + actuate | replay, then bench |
| 9 | `rover-navigation` — GNSS ×2 + mission + RTCM injection | bench with live GNSS |
| 10 | `rover-telemetry` + `ground-station` | bench |
| 11 | `perception` — merge camera+lane, drop `cv_bridge`, new bus | Jetson, recorded video |
| 12 | Bench integration: watchdog, IWDG, E-stop, command echo | rover on blocks |
| 13 | Field bring-up: straight → gentle curve → full circuit | field |
| 14 | *Phase 2:* `LoraSerialLink`, `TelemetryLite`, RTCM over 433 | range test |
| 15 | *Phase 3:* `Lqr`, then `Mpc`; LIS2MDL experiment; YOLO/TensorRT | replay + field |

**Estimate: 4–8 weeks part-time** for steps 0–13, dominated by firmware bring-up
and field revalidation — not by the Rust. Steps 14–15 are separate efforts.

**YOLO/TensorRT, LQR/MPC, LoRa and the magnetometer are all deliberately after
step 13.** Land the port at parity with the current system first. Changing the
middleware, the language, the process topology, the control law, the perception
algorithm *and* the radio link simultaneously means you cannot bisect a regression.

---

## 10. Port fidelity: what must not change

Earned over field runs that are expensive to repeat. **Port bit-exactly, verify
against recorded data, refactor only in a separate commit.**

| Constant | Value | Source |
|---|---|---|
| `k_lat` | 181.17 deg/m | LQR solution, field-validated |
| `k_head` | 2.024 deg/deg | LQR solution, field-validated |
| `wheelbase_m` | 0.4875 | measured |
| `bev_px_per_m` | 200.0 | camera calibration |
| lookahead | 1.22 m ahead of front axle | ROI geometry |
| `speed_kp/ki/kd` | 0.3 / 0.5 / 0.0 | field-derived (`5c3a3fa`, `f46768c`, `dc858af`) |
| `autocal_min_duty_pct` | 13.0 | cruises 15–16%, stalls at 11% |
| `steer_max_deg` | 45.0 | mechanical limit |
| lane segmentation | L\* channel, not chroma | `6a49552`, validated on D415 |
| `max_ticks_per_sec` | **placeholder — not calibrated** | see §2.6 |

**Sign convention is a known hazard.** `rover_kinematic_control_node.py` carries an
explicit warning: `theta`, `b` and `steer_angle` are all "+ = correct by steering
right", the *opposite* of the ISO 8855 convention the textbook
`-k1*e_lat - k2*e_heading` form assumes. Both feedback terms therefore carry a
**plus** sign. Get it backwards and the controller becomes positive feedback.
Document it in `rover-model`, and unit-test the sign of the response to a known
offset.

**Required before hardware:** `tools/replay` must drive `rover-estimator` and
`StaticGain` from existing `runs/*.csv` and match recorded ROS 2 behaviour within
tolerance. No motor turns until that passes.

---

## 11. Risks

| Risk | Severity | Mitigation |
|---|---|---|
| **PHY bring-up on F767ZI** — Nucleo uses LAN8742A, Embassy's example uses `GenericPhy` | Kills the plan | **Step 4 is a hard gate.** 1–2 days, before anything else. Standard clause-22 part; should work. |
| **Control regression from the port** | High | Bit-exact port + replay harness (§10). No motors until replay passes. |
| **Sign-convention inversion** | High | Documented §10, unit-tested |
| **Encoder constant wrong after the 2×→4× decoding change** | High | §2.6 warning; `decoding` recorded in `rover.toml`; verify commanded vs. measured speed on the first bench run |
| **RTK bandwidth over 433 MHz** | Medium | §6.4 — plan for FSK, not long-range LoRa SF |
| **No command uplink when disconnected** | Medium | §6.5 — decide before step 3 |
| **EKF tuning eats time** | Medium | Seed `R` from measured variance of lane params, stationary, facing a straight line — you have the CSVs. Log innovations from day one. |
| **Magnetometer disappoints** | Low | Optional, behind a flag, after step 13. §2.5 says why to expect little. |
| **Jetson Python bus drifts from Rust** | Medium | Shared golden-byte fixtures (§4.3) |
| **No ROS 2 escape hatch** | Medium | Accepted. `main` keeps the working system. |

---

## 12. Acceptance criteria

1. `cargo test --workspace` green, golden-byte fixtures cross-checked against the
   Python decoder.
2. `tools/replay` reproduces recorded ROS 2 steering output from
   `runs/*/lane_detection.csv` within tolerance.
3. **Watchdog:** rover on blocks, driving, Ethernet pulled → throttle ramps to zero
   within 200 ms + 300 ms, steering centres, `watchdog_tripped` observable.
4. **IWDG:** artificially hang the control task → board resets, PWM safe.
5. **E-stop:** `ground-station` E-stop → full stop, end to end, with `cmd_seq`
   echoed in telemetry.
6. **Metric speed:** commanded 0.20 m/s produces 0.20 ± 0.02 m/s measured over a
   10 m run on the field surface.
7. 100 Hz IMU confirmed at the estimator, end-to-end latency measured.
8. EKF coasts through a ≥3 s artificial lane dropout without steering divergence.
9. One full field circuit at parity with the current system, logged, CSVs compared.
10. `main` still builds and still drives the rover.

---

## 13. Implementation status — branch `rs`

Updated 2026-09-20. **This section is the truth; everything above it is the
plan.** Where they disagree, believe this.

### 13.1 What exists and is verified

| Component | State | Evidence |
|---|---|---|
| `rover-msgs` | **done** | 17 types, builds for host **and** `thumbv7em-none-eabihf`. 7 contract tests: round-trip, declared-vs-actual length, trailing bytes, truncation, frame budget, ID uniqueness, golden fixtures. |
| `testdata/*.bin` | **done** | 17 fixtures. Regenerating them is a breaking protocol change. |
| `rover-link` | **done** | `Link` trait, `UdpLink`, `send_to_addr` with an `Unsupported` default so a future LoRa link need not implement it. |
| `rover-bus` | **done** | config loading, unicast fan-out, newest-wins receive, idempotent command handshake, debug mirror. |
| `rover-tap` | **done** | per-type rate and seq-gap loss; `--mirror` sees the whole bus. |
| `rover-model` | **done** | shared `A(v)`/`B(v)`, Euler discretisation. 6 tests. |
| `rover-estimator` | **done** | 5-state EKF, Joseph form, chi-square gate, coast-on-dropout, zero-rate bias update. 10 behavioural tests. |
| `rover-control` | **done** | lib + thin bin so `tools/replay` drives production code. `LateralController` trait; `StaticGain` is the bit-exact port. `actuate.rs` owns every guard rail. |
| `rover-navigation` | **done** | two GNSS receivers, mission state machine, RTCM injection point. |
| `rover-telemetry` | **done** | CSV at native rates, health bits, base feed. |
| `ground-station` | **done** | mission goals, speed limit, E-stop, live display. |
| `tools/replay` | **done** | **the §12 gate.** Synthetic trace + `legacy.rs` oracle. PASS at the 3.0° tolerance (rmse 1.349°, 267 rows compared); deliberately verified to FAIL at 0.5°, so the harness demonstrably has teeth. |
| `perception/wire.py` | **done** | 51 tests green against the Rust fixtures — the two languages provably agree on every byte. |
| `perception/lane.py` | **done** | behaviour-preserving port, **parity proven** — see 13.1b. |
| `perception/{camera,bus,main}.py` | **done** | one process replacing `camera_stream_node` + `lane_detection_node`. End-to-end verified: 20 frames in, 20 `LaneMeasurement` out, seq monotonic, decoded by `wire.py`, clean exit. |
| `firmware/chassis` | **builds** | 65,048 B flash (3.1%), 17,724 B RAM (3.4%). Clippy clean. **Never run on hardware.** |
| `firmware/sensors` | **builds** | 54,316 B flash (2.6%), 17,844 B RAM (3.4%). Clippy clean. **Never run on hardware.** |

Host workspace: **290 tests**, clippy and `fmt` clean. Perception: **82 tests**.

### 13.1b The lane parity test, and why it is believable

`perception/tests/test_lane_parity.py` compares `process_frame` against the
**real ROS 2 original**, not a description of it: `lane_detector.py` and
`config.py` are vendored verbatim at `perception/tests/oracle/` (diffed at
vendoring time — the only edit is one import line), the same way
`tools/replay/legacy.rs` freezes the old control law. That is what let the
ROS 2 tree be deleted without taking the evidence with it.

Comparison is **exact** `==`, no tolerance, over six deterministic synthetic
scenarios plus a `search_center` tracking case. Four of the six exercise the
*detected* path with materially different geometry, so the test is not
passing vacuously on "NaN equals NaN"; the other two (blank, pure noise) are
the negative controls, and one is additionally asserted not-detected outright.

Checked by mutation, not by reading: perturbing the port's polyfit
coefficients by **one part in 10⁷** fails 5 of the 9 tests. A real
algorithmic divergence cannot slip through this.

### 13.2 What does not exist yet

| Component | State |
|---|---|
| Firmware on real silicon | **nothing has been flashed.** See 13.4. |
| Drivetrain calibration | `ticks_per_rev`, `metres_per_tick`, `track_width_m` are all `0.0`. No metric speed exists until they are measured — §2.6, and **against the 4× decoder**, not the old 2× firmware. |
| Parity against real field data | impossible here. No recorded ROS 2 run exists in this repository and none ever did (re-verified at tree-removal time). `replay` proves internal consistency and regression-catching, not field parity. §12 criterion 2 stays open. |
| LoRa link layer | deliberately deferred — §6. Both ESP32s are out of scope by instruction. |
| LQR / MPC control laws | phase 2 and 3. The `LateralController` trait exists so they are a new file, not a rewrite. |

### 13.2b The ROS 2 tree has been removed

Removed on 2026-09-20 (`chore(rs)!: remove the ROS 2, mROS 2 and embeddedRTPS
tree`): 919 files, ~137k lines — `ws_rpi`, `ws_jetson`, `ws_base`,
`common_ifaces`, both `mros2-mbed-*` trees, ten DDS/RTPS/mbed docs, and the
ROS 2 launch tooling.

This was held until the replay gate passed **and** the lane parity oracle was
vendored, because until then that tree was the only thing the port could be
checked against. What it still holds is recoverable: everything deleted was
tracked and committed, so `git show main:<path>` and this branch's history
both return it, and it was verified beforehand that no untracked or ignored
file — and no `runs/`, CSV, bag or video — lived anywhere under it.

Durable knowledge was harvested first, into `docs/HARDWARE.md`. `ws_spresense`
was left untouched by instruction.

### 13.3 Deviations from the plan, and why

1. **`lsm6dsv16x-rs` is pinned to v1.0.0, not the v2.1.0 named in §5.1.**
   v2.x depends on `bisync`, and every published version of `bisync` is
   yanked on crates.io, so v2.x cannot be resolved at all. v1.0.0 predates
   the async/blocking split: same method names, register enums behind
   `prelude` instead of the crate root, two type parameters instead of
   three. Revisit if `bisync` is ever unyanked.

2. **Firmware constants are a hand-transcribed `config.rs`, not a compile-time
   parser.** §6 says firmware should `include_str!` `rover.toml` and parse it
   in a `const fn`. That parser is a project in itself and getting it subtly
   wrong is a worse failure than a small constants file a human can diff by
   eye. Every constant cites its `rover.toml` key. **The manual sync step is
   real and is the documented cost of not having the parser.**

3. **The watchdog centres the steering *before* the ramp, not after.** §5.2's
   code sketch ramps throttle and then centres, which leaves the servo
   latched for the full 300 ms while the vehicle is still moving — against
   the stated reason for centring at all ("a latched steering angle turns a
   runaway into a circle"). The plan's sketch was wrong; the implementation
   follows the plan's *rationale*.

4. **Process noise is `P += Q*dt`, not `P += Q`.** Predict is driven by IMU
   packet arrival, not a hardware timer, so `dt` jitters and a dropped packet
   makes it several times nominal. Per-tick noise leaves the filter
   overconfident exactly when it has least information. `[estimator.q]` values
   are therefore per-second spectral densities.

5. **`lane_age_ms` resets only on an *accepted* camera update.** Guidance reads
   `RoverState` and never sees `EkfDebug`, so counting gated readings as fresh
   would let a detector producing consistent garbage read as healthy while the
   filter coasted with no corrections. Camera liveness is `HealthBits::LANE_STALE`.

6. **Encoders decode in software, not in timer hardware.** §5.4 as originally
   written was not achievable on this board — each encoder's channel pair
   straddles TIM2 and TIM3. Verified by compilation, not by reading a
   datasheet. 4× decoding is preserved; the "cannot miss a count" guarantee is
   not. See the corrected §5.4.

7. **`TelemetryLite` is not implemented.** The LoRa radios are deferred, and an
   unused type rots. Add it with `LoraSerialLink`.

### 13.3b Open design defects found during implementation

Three real flaws in this plan, surfaced by building against it. Recorded here
with their decisions; **not yet implemented**.

#### D1 — `PeerId` addresses machines, but the bus must address processes

`config/rover.toml` gives each *host* one UDP port. The RPi runs **three**
binaries that each need to receive — `rover-control` (`ImuSample`,
`WheelSensors`, `LaneMeasurement`), `rover-navigation` (`CommandFrame`), and
`rover-telemetry` (everything it logs). They all bind `PeerId::Rpi`, so on real
hardware the second and third to start die with `EADDRINUSE`. Unicast would not
fan out to several listeners even if they could bind, and `UdpLink` has no
`SO_REUSEPORT` — nor should it, since that load-balances rather than duplicates.

This is a modelling error on my part: the bus routes to *endpoints*, and an
endpoint is a process, not a machine.

**Decision — make `PeerId` a service identity.** `[hosts]` and `[ports]` collapse
into one `[services]` table mapping a service name to a full `host:port`:

```toml
[services]
control    = "192.168.1.1:7001"
navigation = "192.168.1.1:7002"
telemetry  = "192.168.1.1:7003"
chassis    = "192.168.1.2:7010"
sensors    = "192.168.1.6:7011"
perception = "192.168.1.5:7020"
base       = "192.168.1.10:7030"
```

`[routes]` then targets services, and a type may have several — `WheelSensors`
goes to `control` (speed loop) and `telemetry` (CSV). Firmware gains a small
per-type destination table instead of a single `RPI_IP`/`RPI_PORT` pair; no
message needs more than two destinations, so the cost on the MCUs is one extra
`send_to`.

#### D2 — `GnssFix` cannot say which receiver it came from

One type and one `TYPE_ID` serve both the u-blox and the Spresense on two
streams, so a subscriber in another process cannot tell them apart from the bus
alone. `rover-telemetry` currently guesses: anything better than `Autonomous`
must be the u-blox, since the Spresense reports only a boolean fix. That
direction is sound, but a u-blox in cold start reporting `Autonomous` is
misfiled as the backup — and it is the RTK stream that the mission logic and the
heading reference depend on.

**Decision — add a `source: GnssSource { Rtk, Backup }` field.** One byte,
self-describing, no heuristic. It changes the wire format and the golden
fixtures, which is free now and expensive after the boards are in a field.

#### D3 — the ROS 2 mission monitor navigated on the *uncorrected* receiver

`gnss_mission_monitor_node.cpp` tracked position from `tpc_gnss_spresense` only
and never read the RTK stream, while arrival radius is 2 m here (20 m in ROS 2).
Uncorrected GPS is several metres accurate, so arrival detection was being
decided by the noisier of the two receivers with a centimetre-grade one sitting
unused.

**Decision — prefer RTK when usable, fall back to backup.** This is a deliberate
behavioural improvement, not a port. It means mission-arrival behaviour will
**not** match the ROS 2 baseline in replay, and that difference is expected
rather than a parity failure.

#### D4 — base-station CSV (minor)

`docs/CSV_LOGGING.md` says the base station is display-only, and the ROS 2 node
was. A base-side log is still independently useful — it records what the
operator actually saw, including link gaps the rover's own log cannot show.
**Decision: keep it, behind an off-by-default flag**, so the shipped behaviour
matches the ROS 2 baseline and the capability is there when a comms problem
needs diagnosing.

### 13.4 Hardware-verification debt

Nothing in `firmware/` has met silicon. In rough order of risk:

1. **LAN8742A PHY against `GenericPhy`** — §9 step 4, still the hard gate.
   Standard clause-22 part, but unproven here.
2. **I2C1 on PB8/PB9** — inherited from the mbed target's generic
   `I2C_SDA`/`I2C_SCL` names for `NUCLEO_F767ZI`. Standard Nucleo-144 Arduino
   bus, not confirmed against the physical board.
3. **Encoder sign convention** — the quadrature table's forward direction was
   defined from first principles and cannot be checked without turning a wheel
   by hand and watching `ticks_left`/`ticks_right`. If it counts backwards,
   swap that encoder's two pin arguments.
4. **Software decode under load** — an EXTI decode, unlike a timer peripheral,
   can in principle miss edges while competing with UDP and I2C interrupts.
   Unmeasured.
5. **TIM1 left-motor PWM** — the only channel on an advanced-control timer. If
   the left motor alone produces no PWM while the right motor and servo work,
   TIM1's break/MOE gate is the first place to look.
6. **Watchdog end-to-end** — §12 criterion 3. Cannot be faked in a test.
7. **Every timing constant** — 200 ms / 300 ms / 500 ms are reasoned, not measured.

### 13.5 Standing blockers

- **Drivetrain calibration is `0.0`.** `ticks_per_rev`, `metres_per_tick` and
  `track_width_m` are all unmeasured, so there is still no metric speed
  anywhere in the system. The estimator disables odometry loudly rather than
  dividing by zero, and the speed PID works in ticks/sec so it is unaffected —
  but nothing can report m/s until §2.6 is done. Twenty minutes with a tape
  measure.
- **Nothing has been flashed.** Every firmware claim in 13.1 is a claim about
  a binary that builds, not one that has run. The LAN8742A PHY against
  Embassy's `GenericPhy` is the gate that decides whether any of it is real.
  See 13.4 for the full list.
- **Replay parity is against a synthetic trace and a hand-rolled oracle, not
  against the rover's past behaviour.** No recorded ROS 2 run exists in this
  repository and none ever did. The harness prints this caveat itself. §12
  criterion 2 stays open until a real run is recorded on the new stack and
  compared against a real ROS 2 run recorded on `main` — which now requires
  checking out `main` to produce one, since the ROS 2 tree is gone from this
  branch.

## Appendix A — LSM6DSV16X register fallback

Transcribed from `libs/X-Nucleo-IKS4A1_mbedOS/plt_lsm6dsv16x/registers.h` in
the mbed firmware. That path no longer exists on this branch — it lived under
`mros2-mbed-chassis-dynamics`, removed in `chore(rs)!: remove the ROS 2,
mROS 2 and embeddedRTPS tree`. The table below is the reason it did not need
to survive; if more of it is ever wanted, `git show main:<path>` still has
the header. This is the fallback for talking to the part directly, should
`lsm6dsv16x-rs` prove unusable on hardware:

| Register | Addr | Use |
|---|---|---|
| `WHO_AM_I` | `0x0F` | expect `0x70` (`LSM6DSV16X_ID`) |
| `CTRL1` | `0x10` | accelerometer ODR / mode |
| `CTRL2` | `0x11` | gyroscope ODR / mode |
| `CTRL3` | `0x12` | `BDU`, `IF_INC`, `SW_RESET`, `BOOT` |
| `CTRL6` | `0x15` | gyroscope full scale |
| `CTRL8` | `0x17` | accelerometer full scale |
| `STATUS_REG` | `0x1E` | `XLDA` / `GDA` data ready |
| `OUTX_L_G` | `0x22` | gyro X/Y/Z, 6 bytes LE |
| `OUTX_L_A` | `0x28` | accel X/Y/Z, 6 bytes LE |

Set `BDU=1`, `IF_INC=1`, then burst-read 6 bytes from `0x22` and `0x28`.
Sensitivity conversions mirror the C driver's `from_fsN_to_mg` /
`from_fsN_to_mdps` helpers.

## Appendix B — X-NUCLEO-IKS4A1 sensor complement

Your firmware uses **one** of these. Confirmed against ST's product page and the
vendored `libs/X-Nucleo-IKS4A1_mbedOS/README.md`:

| Part | Function | Used today? | Rust driver |
|---|---|---|---|
| **LSM6DSV16X** | 6-axis IMU | **yes** | [`lsm6dsv16x-rs`](https://crates.io/crates/lsm6dsv16x-rs) |
| **LIS2MDL** | **3-axis magnetometer** | **no** — see §2.5 | [`lis2mdl-rs`](https://crates.io/crates/lis2mdl-rs) |
| LSM6DSO16IS | 6-axis IMU w/ ISPU | no | ST repo |
| LIS2DUXS12 | 3-axis accelerometer | no | ST repo |
| LPS22DF | pressure | header included, unused | ST repo |
| STTS22H | temperature | header included, unused | ST repo |
| SHT40-AD1B | humidity + temp | no | community |

## Appendix C — References

- [Embassy](https://github.com/embassy-rs/embassy) ·
  [`embassy-net`](https://docs.embassy.dev/embassy-net/) ·
  [STM32F7 Ethernet example](https://github.com/embassy-rs/embassy/blob/main/examples/stm32f7/src/bin/eth.rs)
- [ST official Rust MEMS drivers](https://github.com/STMicroelectronics/st-mems-rust-drivers) ·
  [`lsm6dsv16x-rs`](https://crates.io/crates/lsm6dsv16x-rs) ·
  [`lis2mdl-rs`](https://crates.io/crates/lis2mdl-rs)
- [`ina226`](https://crates.io/crates/ina226) · [`ina226-tp`](https://lib.rs/crates/ina226-tp)
- [`probe-rs`](https://probe.rs/) · [`defmt`](https://defmt.ferrous-systems.com/) ·
  [`nalgebra`](https://nalgebra.org/) · [`clarabel`](https://crates.io/crates/clarabel)
- [X-NUCLEO-IKS4A1](https://www.st.com/en/evaluation-tools/x-nucleo-iks4a1.html)
- Considered and rejected: [Zenoh](https://zenoh.io/) (value is discovery/routing/WAN — unused here),
  [zenoh-nostd](https://github.com/eclipse-zenoh/zenoh-nostd) (early development),
  MQTT (five competing `no_std` crates, no dominant one),
  [canadensis](https://github.com/samcrow/canadensis) / Cyphal (UDP transport unspecified, reintroduces DSDL codegen),
  [dora-rs](https://dora-rs.ai/) / [Copper](https://www.copper-robotics.com/) (no MCU story)

---
