# Rust Rewrite Plan — `almondmatcha` v5

**Status:** implemented on branch `rs`. §13 records what actually exists.
**Written:** 2026-09-19 · **Revised:** 2026-09-20 (rev 4)
**Scope:** full replacement of the ROS 2 / DDS / mros2 stack with a Rust workspace,
adding state estimation (EKF), a pluggable control law, a firmware command
watchdog, metric speed, and a link layer that survives the base station leaving
the LAN for LoRa. Big-bang rewrite on a branch; `main` was preserved
untouched. (This branch has since become its own repository — `main` now
lives only in `RoboticsGG/almondmatcha`; this repository's `origin` has no
`main`.)

**Rev 3 added §13, the record of how the implementation actually diverged from
this plan.** Everything above it is the plan; §13 and the status documents it
points to are what exists on branch `rs`. Where the two disagree, §13 is right
and the plan text is aspirational.

**Rev 2 changes:** §2.6 metric speed calibration (encoder spec is *not* in the
repo — measurement procedure supplied), §5.2 full watchdog specification, §6 new
link-layer abstraction for the LoRa future, §4 base↔rover protocol changed from
TCP to idempotent datagrams, §2.5 LIS2MDL magnetometer (present on the shield,
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
| Migration style | **Big bang** on a branch. `main` kept the working ROS 2 rover — now only in `RoboticsGG/almondmatcha`. |

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
| `sensors-fw` | STM32 .6 | Rust `no_std` | Software 4× EXTI quadrature decode (§5.4), INA226, **watchdog**. |

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
  over existing `runs/*.csv`. Must be tunable on a laptop. (No such `runs/*.csv`
  has ever existed here — §12 criterion 2 and `STATUS_OPEN.md` §1.4 for what
  producing one would take.)

### 2.4 Heading observability — and what the magnetometer does about it

Without an absolute heading reference, `e_psi` is observable **only** through the
camera. With the lane lost, heading drifts at the residual gyro bias rate — expect
**5–15 s** of useful coasting once bias converges, not minutes.

### 2.5 You already have a magnetometer, and it is unused

**Yes — the X-NUCLEO-IKS4A1 carries a LIS2MDL 3-axis magnetometer.** Confirmed
both by ST's product page and by the vendored
`libs/X-Nucleo-IKS4A1_mbedOS/README.md`, which lists it (that path no longer
exists on this branch — see Appendix A;
`git show roboticsgg-almondmatcha/main:<path>` still has it, since this
repository's `origin` has no `main` of its own).
The board also carries
LSM6DSO16IS, LIS2DUXS12, SHT40-AD1B, LPS22DF and STTS22H. Your firmware
includes
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

Ticks per wheel revolution is **not recorded anywhere** — not in this repo, not
in the ROS 2 tree it replaced: no PPR, no CPR, no gear ratio, no motor part
number. The only related constant was `max_ticks_per_sec: 1000.0` in the old
`chassis_speed_control_params.yaml`, explicitly labelled `PLACEHOLDER`, with
`use_closed_loop_speed: false` as the shipped default. There is still no metric
speed anywhere in the system.

> ⚠️ **The 2×/4× trap.** The mros2 firmware attached interrupts to channel A
> only (`rise` + `fall`) and read channel B purely as a direction input — 2
> counts per quadrature cycle. **This firmware decodes every edge on all four
> channels: 4×, so every tick count doubles.** A constant measured against the
> old firmware and pasted in reads exactly double the true speed. Record which
> mode the constant belongs to in `config/rover.toml` — `decoding =
> "quadrature_4x"`. (Hardware quadrature timer mode would give the same 4× for
> free, but it is impossible on this harness; §5.4 has the pin analysis.)

**The maths, with the 12.5 cm wheel:**

```
wheel_circumference = pi * 0.125 m           = 0.39270 m / revolution
metres_per_tick     = 0.39270 / ticks_per_revolution
speed_mps           = ticks_per_second * metres_per_tick
```

**The procedures themselves live in `docs/CALIBRATION.md`** — Procedure A
(ticks per revolution, on the bench) and Procedure B (metres per tick on the
real surface, authoritative), with the ST-Link serials, the build-and-flash
commands, the defmt log line to read, the counts to expect, and the
direction-sign check. This section is the reasoning; that document is the
how-to. Use **B** for `metres_per_tick` in production and **A** to sanity-check
it; disagreement beyond ~5% suggests slip. `track_width_m` is worth measuring
in the same pass — left/right tick rates then give a second yaw-rate estimate
to cross-check the gyro, and it costs nothing.

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

**Unicast fan-out, not multicast.** With seven services on five hosts and
sub-200-byte messages this costs nothing, and it removes the IGMP dependency
that forced the hand-maintained `initialPeersList` workaround in
`fastdds_rover.xml`.

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
driver already vendored in `libs/` (that path no longer exists on this branch —
see Appendix A; `git show roboticsgg-almondmatcha/main:<path>` still has it,
since this repository's `origin` has no `main` of its own).

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

New work on branch `rs`. `main` kept the ROS 2 system — at the time in the
same repository. This branch has since become its own repository; `main` now
lives only in `RoboticsGG/almondmatcha`, and this repository's `origin` has
no `main` at all.

```
almondmatcha-rs/
├── Cargo.toml                   # workspace: firmware + host, one message crate
├── rust-toolchain.toml
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
│   ├── rover-runs/              # run_NNN_<stamp>/ convention, shared by telemetry + ground-station
│   ├── ground-station/          # bin  (Base)
│   ├── rover-doctor/            # bin  (Base)  preflight GO/NO-GO, 9 checks
│   └── rover-tap/               # bin  debug CLI — replaces `ros2 topic echo`
├── firmware/
│   ├── chassis/                 # no_std, thumbv7em-none-eabihf
│   └── sensors/                 # no_std
├── perception/                  # Jetson Python
│   ├── pyproject.toml
│   └── rover_perception/{camera,lane,bus,wire,main}.py
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
| 6 | `sensors-fw` — software 4× EXTI quadrature decode (§5.4), INA226, watchdog | bench, `rover-tap` |
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
| `steer_max_deg` | 45.0 | mechanical limit — **tightened, not ported**: the ROS 2 value was ±60° (`CONTROL_LAW.md` §1.6) |
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
tolerance. No motor turns until that passes. **No such `runs/*.csv` exists, and
none ever did** — §12 criterion 2 has what producing one would take, and
`STATUS_OPEN.md` §1.4 is authoritative on where it stands.

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
| **EKF tuning eats time** | Medium | Seed `R` from measured variance of lane params, stationary, facing a straight line. No such CSVs exist yet (`STATUS_OPEN.md` §1.4) — this means recording one first, not pulling from an existing log. Log innovations from day one. |
| **Magnetometer disappoints** | Low | Optional, behind a flag, after step 13. §2.5 says why to expect little. |
| **Jetson Python bus drifts from Rust** | Medium | Shared golden-byte fixtures (§4.3) |
| **No ROS 2 escape hatch** | Medium | Accepted. `main` keeps the working system, now in `RoboticsGG/almondmatcha` — this repository's `origin` has no `main`. |

---

## 12. Acceptance criteria

1. `cargo test --workspace` green, golden-byte fixtures cross-checked against the
   Python decoder.
2. `tools/replay` reproduces recorded ROS 2 steering output from
   `runs/*/lane_detection.csv` within tolerance. **No recorded run exists in
   this repository and none ever did (`STATUS_OPEN.md` §1.4); this criterion cannot be
   attempted until one is produced on `main` in the ROS 2 fallback repository,
   `RoboticsGG/almondmatcha` (this repository's `origin` has no `main`).**
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

> **Status lives in two dedicated documents, and they are authoritative over
> any status claim anywhere else, including this file.**
>
> | | |
> |---|---|
> | [`STATUS_DONE.md`](STATUS_DONE.md) | What is built, and the evidence for every claim |
> | [`STATUS_OPEN.md`](STATUS_OPEN.md) | What remains: risks reassessed, open defects, concerns |
>
> They were split out because one section mixing accomplishments with blockers
> made it too easy to read the finished work and miss what it did not say —
> and because this project has already been bitten by a status document that
> outlived its accuracy.
>
> What stays below is the part that is *not* status and does not rot the same
> way: why the implementation diverged (§13.3), the narrative behind each
> design defect (§13.3b), and the hardware-verification debt with its
> reasoning (§13.4, §13.4a). These explain *why*; the status docs track
> *where*.

### 13.3 Deviations from the plan, and why

1. **`lsm6dsv16x-rs` is pinned to v1.0.0, not the v2.1.0 named in §5.1.**
   v2.x depends on `bisync`, and every published version of `bisync` is
   yanked on crates.io, so v2.x cannot be resolved at all. v1.0.0 predates
   the async/blocking split: same method names, register enums behind
   `prelude` instead of the crate root, two type parameters instead of
   three. Revisit if `bisync` is ever unyanked.

2. **Firmware constants are a hand-transcribed `config.rs`, not a compile-time
   parser.** `config/rover.toml`'s own header comment says firmware should
   `include_str!` it and parse it in a `const fn`. That parser is a project in
   itself and getting it subtly
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

Six real flaws, surfaced by building against this plan and then by auditing
what was built. **D1-D4 are implemented** — D1 as `[services]` in
`config/rover.toml`, D2 as `GnssSource` / `GnssFix.source`, D3 as
`select_navigation_fix` in `crates/rover-navigation/src/mission.rs` (with
passing tests), D4 as perception's `--csv` flag, off by default (the base
station only half-follows that decision — see D4).

**D5 and D6 are open, and both are deferred for the same reason:** each
changes closed-loop behaviour against gains tuned around the current
behaviour, so neither should be taken without a re-tune on hardware. They
should be taken together.

**D5 is the only one that can make the rover steer the wrong way**, and should
be settled before anything is flashed. D6 is a systematic lag the system
cannot currently even measure.

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

**What actually shipped only half-follows that decision.** The perception
process does implement it correctly: `--csv` defaults to `None` and logging is
opt-in. `crates/ground-station/src/main.rs` does not — `--runs-dir` has a
`default_value` and `CsvLogger::spawn` is called unconditionally, so the base
station logs CSVs on every run with no flag to turn it off.

> Updated when the base station moved to the shared `run_NNN_<stamp>/`
> convention: the flag is now `--runs-dir` (it was `--log-file`, naming a
> single flat CSV) and the logger creates its directory and file lazily on
> first write, so a base station that receives nothing now leaves nothing
> behind. **That narrows the gap but does not close it** — a run that
> receives anything at all still logs, with no way to say no. The decision
> above is still only half-implemented.
`crates/ground-station/src/csv_log.rs` documents the tension itself (it quotes
`CSV_LOGGING.md`'s "No CSV logging on base station" line) and resolves it the
other way, citing the crate's own task brief. So: perception is off by
default as decided; the base station is on unconditionally, undecided-in-code.

#### D5 — `heading_err_rad`'s sign disagreed with the model consuming it

**Found 2026-09-20 by measurement, not review. Sign fixed the same day; the
gain re-tune is still open.** This was the only defect here that could make the
rover steer the wrong way.

> **Resolution.** `LaneDetector.detect` negates `theta` before it becomes
> `heading_err_rad`. `compute_lane_params`, `process_frame`, `cross_track_m`,
> `steer`, `Ekf::correct_camera`, `at_lookahead` and both gains are
> **unchanged** — the model was right and the data was wrong, so the fix
> belongs in the detector alone. `perception/tests/test_lane_sign_convention.py`
> is the regression test that was missing. The parity suite and replay gate are
> unaffected, which is the evidence the fix landed at the right boundary.

##### The claim that was half true

`rover-msgs`' crate docs, `guide.rs`, §10 and the README all stated one
convention: `cross_track_m`, `heading_err_rad` and `steer` are *all* positive
when the correct response is "steer right", so both feedback terms carry a plus
sign. Against the real detector, **the `cross_track_m` half was right and the
`heading_err_rad` half was backwards.**

##### The measurement, and the algebra behind it

Canvas orientation confirmed first, since everything depends on it: a marker at
the ROI's far edge lands at BEV row 26/340, the near edge at 334/340 — **canvas
top is ahead**. (The ROI trapezoid is 550 px wide far, 1202 px near, which is
what a ground plane in perspective looks like.)

Feeding `process_frame` a lane veering **right** as it recedes gave
`theta = -11.662°` against a measured `d(cross_track)/d(distance) = +0.2052`,
with `tan(theta) = -0.2064` — the negative, to three decimals. Re-measured
independently on 2026-09-20 with a different frame: `theta = -16.984°` against
`+0.3065`.

It is structural, not an artifact. `compute_lane_params` fits
`x = A*y'² + B*y' + C` with `y' = y - height`, so `y'` increases *backwards*
and forward is `-y'`. Hence `d(cross)/d(distance) = -B` while
`theta = arctan(B)`.

##### Why that was wrong

Two consumers assume the opposite, both with `+l_a`:

- `RoverState::at_lookahead` — `cross + l_a*heading_err + 0.5*kappa*l_a²`
- `Ekf::correct_camera` — `h[(0, HEADING_ERR)] = l_a`

Both encode `d(cross)/d(l) = +heading_err`, as does the error model in the
ROS 2 node's own docstring: `b_dot = v*theta`.

##### Why the rover nevertheless drove

`b` is measured 1.22 m ahead, so it already carries heading information and
dominates. For a straight path angled `psi` right:

```text
was:     k_lat*(1.22*psi) + k_head*(-psi) = 3.857*psi - 2.024*psi = 1.833*psi deg
correct: k_lat*(1.22*psi) + k_head*(+psi) = 3.857*psi + 2.024*psi = 5.881*psi deg
```

Still positive, so it steered the right way — at **31 % of the intended
authority**, the heading term cancelling the lookahead's anticipation instead
of reinforcing it. Consistent with `k_lat = 181.17` being tuned unusually high
to compensate for a term working against it.

The clean failure case was `b ≈ 0, theta ≠ 0` — on the line but pointed wrong,
which happens at every line crossing. There the `b` term contributes nothing,
the heading term alone decides, and the rover steered **away** until `b` grew
enough to overrule it. A weave, not a divergence.

##### Two things that made it worse than it looked

1. **The EKF amplified it.** The EMA passed `theta` through untouched. The EKF
   cross-couples cross-track and heading through `l_a`, so it was fusing two
   measurement rows that contradicted each other — inflated innovations, a
   biased heading estimate, more NIS gating than the data deserved.
2. **The tests could not catch it.** `positive_heading_error_steers_right`
   asserts the law matches the assumption, which is circular. The parity suite
   compares the port against the frozen ROS 2 oracle — and they agreed
   perfectly, because *both* disagreed with the model downstream. The one test
   pinning `detect`'s conversion ran on a centred frame where `theta` is `0.0`,
   so its sign assertion was vacuous. All three are now fixed or added.

##### What the fix costs, and what is left open

With `cross_look` pinned to the measured offset (what `correct_camera` actually
fits), the straight-line response to a path angled `psi`:

| | response |
|---|---|
| before, heading term subtracting | `3.858 - 2.024` = **1.834** deg/deg |
| after, heading term reinforcing | `3.858 + 2.024` = **5.882** deg/deg |

**A 3.21× increase in straight-line steering authority**, against gains that
have never run with a correctly-signed heading term. `config/rover.toml`
records this beside `k_lat`/`k_head`, with `[0.312×, 1.0×]` as the bracket to
search on the bench — a bracket, not a recommendation. `0.312×` reproduces the
net authority the ROS 2 rover exhibited but also cuts the pure-lateral response
to 31 %, which was never wrong, so it is a floor rather than a target.

The gains were deliberately not guessed at: they are field-derived (§10 requires
bit-exact porting), and `tools/replay` proves the control law bit-exact against
a frozen oracle, which changing them would break.

**Where the servo direction changes.** The steer sign differs from the old
behaviour only inside `|b| < k_head * theta_deg / k_lat` — about 11 cm of offset
at 10° of heading error. That is the "on the line but pointed wrong" case at
every line crossing, where the old behaviour steered *away*. Outside that band
the direction is unchanged and only the magnitude grows. Nothing on the
`steer` → servo path was touched: `steer > 0` still means right, through
`SteerLimiter`, the wire, `steer_dir()` and `SERVO_CENTER_DEG - angle_deg`.

**Still confirm on hardware** (60 seconds, on blocks): show the camera a line
angled clearly right and watch the servo. It should turn **right**. This is no
longer a test of the sign — that is settled and regression-tested — but of the
one link with no test that has never run: the firmware's `steer > 0` →
physically-right mapping.

Adopting ISO 8855 wholesale was considered and **rejected**: a relabelling that
invalidates every field-tuned gain and buys nothing on a single vehicle with no
external interop.

#### D6 — the camera measurement is timestamped after detection, not at capture

**Found 2026-09-20 by inspection, while answering whether consolidating the
camera and lane-detection nodes could let a frame go stale. Recorded, not
fixed** — for the same reason as D5: the fix changes closed-loop behaviour
against gains that were tuned without it.

##### What the code does

`perception/rover_perception/main.py`'s capture loop:

```python
result = detector.detect(frame)      # ~30-40 ms per HARDWARE.md §7
t_us = time.monotonic_ns() // 1000 & 0xFFFFFFFF
```

The timestamp is taken **after** detection returns. It therefore records when
the answer was computed, not when the photons arrived, and the entire
capture-plus-detect latency is erased from it. The D415's own hardware frame
timestamp is available — `frames.get_timestamp()` in `camera.py` — and is
discarded.

The consumer completes the omission. `rover-control`'s
`Estimator::correct_lane` forwards the message straight to
`Ekf::correct_camera`, which **never reads `t_us`**. There is no latency
compensation anywhere on the camera path. Contrast `ImuSample.t_us`, which
`estimate.rs` goes out of its way to use correctly, and documents at length.

##### Why it matters, and why it is not "the frame expired"

It is *not* a staleness problem. `lane_stale_ms` is 500 ms and
`confidence_speed_scale`'s `AGE_LO_MS` is 500 ms, against a pipeline latency
plausibly around 70–100 ms. Nothing trips, and `LANE_STALE` will not fire.
The damage is subtler:

1. **Phase lag in a closed loop.** The EKF applies a correction describing
   where the rover was ~100 ms ago as though it described now. That eats
   phase margin, and the error grows with speed — worst exactly where the
   controller is working hardest.
2. **The innovation gate may reject good measurements.** A systematically
   lagged measurement produces systematically larger innovations. With
   `nis_gate = 11.34` (χ², 3 DoF, 99 %), a tight curve at speed could start
   rejecting valid camera updates, silencing the camera when it matters most.
3. **None of this is measurable from a field log.** `achieved_fps` is loop
   rate, not latency. There is no dropped-frame counter and no
   capture-to-publish figure anywhere, so the size of the lag — and whether
   points 1 and 2 actually bite — cannot be established from a recorded run.

##### The fix, when it is taken

Two halves, and the first is useless without the second:

- Stamp `t_us` from the frame's capture time, preferring the D415 hardware
  timestamp, and publish a capture-to-publish latency and a dropped-frame
  count alongside it.
- Have `Ekf::correct_camera` compensate for measurement age.

**Doing the second changes control behaviour**, so it belongs with the D5
re-tune, on hardware, not before. Adding only the measurement half is safe and
behaviour-neutral, and would at least turn the estimates above into numbers.

##### Related, same area

`camera.py` never sets the librealsense frame-queue depth explicitly, so
whether a slow detector drops frames or accumulates latency rests on a library
default this code does not pin. Worth pinning deliberately when D6 is taken.

### 13.4 Hardware-verification debt

Nothing in `firmware/` has met silicon. In rough order of risk:

1. **LAN8742A PHY against `GenericPhy`** — §9 step 4, still the hard gate.
   Standard clause-22 part, but unproven here. See §13.4a: bring-up is left
   exactly as Embassy ships it, and the diagnostic gap around it has been
   closed separately.
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

### 13.4a The PHY question, answered

§13.4 has listed the LAN8742A as the top hardware risk since this branch
started, without saying what could be done about it. This section closes that.
The short answer: **a LAN8742A-specific driver is possible and nothing blocks
it, but replacing the bring-up sequence would be a bad trade. Only the
diagnostics were added.**

#### Nothing is sealed

`embassy_stm32::eth::Phy` is a public trait with exactly three methods —
`phy_reset`, `phy_init`, `poll_link` — and `StationManagement`, which is raw
`smi_read`/`smi_write` over MDIO, is publicly exported. `Ethernet::new_with_phy`
takes any `P: Phy`. No fork is needed and no sealed trait stands in the way.
A complete part-specific driver is roughly 150 lines.

#### `GenericPhy` is already half a Microchip driver

Worth knowing before assuming it is a lowest-common-denominator fallback: its
`phy_init` writes `PHY_REG_WUCSR` at `0x8010` through MMD indirect access
(`0x0D`/`0x0E`). That is an SMSC/Microchip vendor register, not generic
clause 22. Embassy wrote it around exactly this family of parts, which is why
it is the sensible default for a LAN8742A and why bring-up is likely to just
work.

#### What it genuinely cannot tell you

The diagnostic half, and all three gaps are the silent kind:

- **It never reads PHY ID1/ID2.** The constants are declared and
  `#[allow(dead_code)]`. A mis-strapped, dead or substituted PHY therefore
  produces no identity mismatch — it simply fails to link, with no reason
  given.
- **`poll_link` returns `bool`**, discarding resolved speed and duplex. This
  is the real silent failure on this hardware: a marginal cable negotiates
  **10 Mbit/s half duplex**, `poll_link` reports "up", and the MAC stays
  configured for 100 full.
- **No symbol error counter**, which is the number that goes non-zero on a
  marginal cable long before anyone notices packet loss.

#### The decision

All three gaps are closed by *read-only* register queries, so `Lan8742a<SM>`
**wraps** `GenericPhy<SM>` and delegates all three `Phy` methods to it
unchanged, adding queries through `GenericPhy::station_management()`. The
firmware constructs the inner `GenericPhy` exactly as `Ethernet::new` does
internally and calls `Ethernet::new_with_phy`, so the write sequence on the
wire is byte-identical to the one thousands of boards run, and the new code
cannot affect link establishment.

The rejected alternative was writing a bespoke bring-up sequence from the
datasheet. That trades a widely-used-but-unverified-here sequence for a
bespoke-and-also-unverified one — strictly worse, on firmware that has never
been flashed.

The device is moved into `embassy_net::Runner` and then into `net_task`, so
`Ethernet::phy_mut()` is unreachable from any other task. The wrapper
therefore writes its findings into atomics from inside its own `poll_link`,
which the runner calls every 500 ms, and the diagnostics task reads those.

#### Registers

Verified against the Microchip **LAN8742A/LAN8742Ai datasheet, Revision 1.1
(05-21-13)** — not from memory. An earlier draft of this work had two of them
wrong, in a way that would have produced a confidently meaningless number.

| Reg | Contents |
|-----|----------|
| `0x02` | PHY Identifier 1, default `0x0007` |
| `0x03` | PHY Identifier 2, default `0xC130`; bits [15:10] OUI, [9:4] model, **[3:0] revision, which varies between parts — mask it off before comparing** |
| `0x1A` | Symbol Error Counter |
| `0x1F` | PHY Special Control/Status; bits **[4:2]** = speed indication: `001` 10-half, `101` 10-full, `010` 100-half, `110` 100-full |

Two properties of `0x1A` that determine how it must be read, quoted from the
datasheet: *"This field counts up to 65,536 and rolls over to 0 if incremented
beyond it's maximum value. Note: This register is cleared on reset, but is not
cleared by reading the register. It does not increment in 10BASE-T mode."*

So it is **free-running, not read-to-clear** — the raw value is reported and
must never be accumulated across polls, which would multiply the true error
count by the poll count. And because it does not increment at 10BASE-T,
`phy_symbol_errors == 0` is **not** evidence of a healthy link when
`link_speed_mbps` is 10. It means something only at 100 Mbit/s.

Every uncertain read degrades to "unknown" (`0`) rather than a guess. A
diagnostic that lies is worse than one that says it does not know.

#### The `eth-phy-lan87xx` crate — evaluated, not adopted

[`eth-phy-lan87xx`](https://crates.io/crates/eth-phy-lan87xx) is a `no_std`
LAN8710A/8720A/8740A/8742A driver over MDIO. It was evaluated 2026-09-20.
**Not adopted**, for the same reason a bespoke driver was rejected above, and
one more:

- It does not implement `embassy_stm32::eth::Phy`. It implements
  `eth_mdio_phy::PhyDriver` over an `eth_mdio_phy::MdioBus`, with ESP32's SMI
  controller as its reference platform. Using it here means writing two
  adapters — nothing hard, but two more unproven pieces on the critical path.
- 0.2.0 was published 2026-05, 0.3.0 2026-06, and total downloads are in the
  low hundreds. Against that, `GenericPhy` is what every embassy STM32
  Ethernet board runs. Swapping a widely-used-but-unverified-here bring-up
  path for a rarely-used-and-also-unverified one is a worse trade, not a
  better one, on firmware that has never been flashed.
- Licensing is not the obstacle: `GPL-2.0-or-later OR Apache-2.0`, and the
  Apache arm matches this workspace.

**Its documentation was still worth the read, and one thing in it changes what
we do.** It calls out that a `BMCR.RESET` does not restore `ANAR` to
`0x01E1`, and writes the advertisement explicitly before enabling
auto-negotiation. That is a real gap here — see below.

#### The `ANAR` / `MODE[2:0]` strap gap

`GenericPhy::phy_init` enables auto-negotiation by writing `BCR`, and **never
writes `ANAR` (register `0x04`)** — embassy declares `PHY_REG_ANTX` and leaves
it dead. What the PHY advertises therefore comes entirely from the
`MODE[2:0]` straps latched at reset. Datasheet Table 3.4:

| MODE[2:0] | Meaning | `ANAR` [8,7,6,5] |
|---|---|---|
| `000`–`011` | fixed speed/duplex, **auto-negotiation disabled** | N/A |
| `100` | 100BASE-TX **half** advertised, auto-neg enabled | `0100` |
| `101` | repeater mode, 100 half advertised | `0100` |
| `110` | power-down | N/A |
| `111` | **all capable**, auto-neg enabled | `1111` |

`ANAR[8,7,6,5]` is 100-full, 100-half, 10-full, 10-half. **If the straps land
on `100` or `101`, the PHY advertises 100BASE-TX half duplex only, the link
comes up at 100 half, and `poll_link` still returns `true`.**

That is precisely the silent failure this section already worried about — but
with a cause far more likely than a marginal cable. And it is plausible on
*this* board specifically: Table 3.5 multiplexes `MODE[2:0]` onto `RXD0`,
`RXD1` and `CRS_DV`, which on the Nucleo-F767ZI are STM32 pins **PC4, PC5 and
PA7**. The strap value depends on what those GPIOs are doing when the PHY
leaves reset.

**What was done about it:** `Lan8742a` now reads `ANAR` (`0x04`) and the
Special Modes register (`0x12`, bits [7:5] = the latched `MODE[2:0]`) and logs
both once over `defmt`, warning if 100-full is not advertised and naming the
decoded strap value. Read-only; the bring-up write sequence is unchanged.

**What was deliberately *not* done:** writing `ANAR` before auto-negotiation.
That is a bring-up change, and the rule above stands — it should be an
evidence-driven decision after the bench shows a bad strap, not a speculative
fix for a condition nobody has observed on this hardware. It is also `defmt`
only rather than on the wire, because it is a bench question: a probe is
attached at first flash, and in the field `PostBits::LINK` and
`link_speed_mbps` already report the symptom and send you back to the bench.

## Appendix A — LSM6DSV16X register fallback

Transcribed from `libs/X-Nucleo-IKS4A1_mbedOS/plt_lsm6dsv16x/registers.h` in
the mbed firmware. That path no longer exists on this branch — it lived under
`mros2-mbed-chassis-dynamics`, removed in `chore(rs)!: remove the ROS 2,
mROS 2 and embeddedRTPS tree`. The table below is the reason it did not need
to survive; if more of it is ever wanted,
`git show roboticsgg-almondmatcha/main:<path>` still has the header — this
repository's `origin` has no `main` of its own. This is the fallback for
talking to the part directly, should
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
vendored `libs/X-Nucleo-IKS4A1_mbedOS/README.md` (that path no longer exists on
this branch — see Appendix A; `git show roboticsgg-almondmatcha/main:<path>`
still has it, since this repository's `origin` has no `main` of its own):

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
