# Hardware reference

Physical facts about the rover: what the boxes are, what is plugged into what,
and which pin does what. None of it depends on the software stack, which is why
this file survived the ROS 2 removal while `ARCHITECTURE.md`, `DOMAINS.md`,
`TOPICS.md` and the rest did not — those described DDS domains, RTPS memory
pools and topic names, all of which branch `rs` deleted outright.

Where a value here is also a value the software uses, **`config/rover.toml` is
authoritative and this file is a description of it**, not a second source. The
pin tables are the exception: those live only in the two firmware crates'
`config.rs` / module docs, cited per row below.

> Anything marked ⚠️ has **never been checked against real silicon**. The Rust
> firmware has never been flashed. See `RUST_REWRITE_PLAN.md` §13.4.

---

## 1. Machines

| Role | Hardware | Compute | IP | Login |
|---|---|---|---|---|
| Perception | Jetson Orin Nano 8 GB | Cortex-A78AE ×6 + Ampere GPU, 8 GB, 128 GB eMMC | 192.168.1.5 | `yupi@` |
| Estimation + control | Raspberry Pi 4B | Cortex-A72 ×4, 4–8 GB, 64 GB SD | 192.168.1.1 | `curry@` |
| Chassis firmware | NUCLEO-F767ZI | Cortex-M7 @ 216 MHz, 512 KB SRAM, 2 MB flash | 192.168.1.2 | SWD only |
| Sensors firmware | NUCLEO-F767ZI | Cortex-M7 @ 216 MHz, 512 KB SRAM, 2 MB flash | 192.168.1.6 | SWD only |
| Base station | PC | — | 192.168.1.10 | `yupi@` |

All five on one gigabit switch, `192.168.1.0/24`, static addresses, no DHCP and
no discovery protocol of any kind. The two NUCLEOs negotiate 100 Mbps.

The RPi runs **three** processes that each bind their own UDP port — this is
design defect D1, and it is why `[services]` in `config/rover.toml` maps a
*process* to a `host:port` rather than a machine to a port.

The base station is **observability and optional override, never a dependency**.
The rover must complete a mission with the base powered off. This becomes load
bearing once the base moves to the LoRa link.

---

## 2. Sensors and what they hang off

| Device | Measures | Bus | Attached to |
|---|---|---|---|
| Intel RealSense D415 | RGB 1280×720 @ 30 FPS | USB 3 | Jetson |
| LSM6DSV16X (X-NUCLEO-IKS4A1) | 3-axis accel + 3-axis gyro | I²C1 | Chassis NUCLEO |
| Wheel encoders ×2 | Quadrature, both drive wheels | GPIO + EXTI | Sensors NUCLEO |
| INA226 | Bus voltage, current | I²C1, addr `0x40` | Sensors NUCLEO |
| u-blox simpleRTK2b | RTK GNSS, cm-class | USB serial | RPi |
| Sony Spresense | GNSS, ~10 Hz, uncorrected | USB serial | RPi |
| ESP32 LoRa 433 MHz | RTCM downlink, base → rover | USB serial | RPi |
| ESP32 LoRa 923 MHz | Telemetry uplink, rover → base | USB serial | RPi |

Notes that cost a field session to rediscover:

- **The D415's depth stream is not used.** The lane detector takes BGR only.
  Depth was "reserved for obstacle avoidance" and never wired to anything.
- **The IKS4A1 carries no magnetometer** — no compass is available on this
  rover. `[estimator] use_magnetometer = false` is not a tuning choice.
  Even with one, the drive motors distort it under load (plan §2.5).
- **Both GNSS receivers moved to the RPi.** The ROS 2 sensors firmware read
  NMEA over UART; the Rust firmware deliberately does not. Serial parsing on
  an MCU bought nothing that `rover-navigation` cannot do with a real
  allocator and a real parser.
- **Both ESP32 links are one-directional** and out of scope for branch `rs`.
  They are the reason there is **no TCP anywhere** in this system: a one-way
  radio link cannot carry a handshake, so every message is a self-contained
  idempotent datagram (plan §6).

---

## 3. Drivetrain

| Quantity | Value | Source |
|---|---|---|
| Drive wheel diameter | 0.125 m | measured |
| Wheelbase | 0.4875 m | measured, carried from the ROS 2 system |
| Track width | ⚠️ **unmeasured** | `[drivetrain] track_width_m = 0.0` |
| Encoder ticks/rev | ⚠️ **unmeasured** | `[drivetrain] ticks_per_rev = 0.0` |
| Metres per tick | ⚠️ **unmeasured** | `[drivetrain] metres_per_tick = 0.0` |

**Until those three are measured there is no metric speed anywhere in the
system.** Both procedures are in `RUST_REWRITE_PLAN.md` §2.6 and need a tape
measure and about twenty minutes.

⚠️ **Calibrate against this firmware, not the old one.** The ROS 2 firmware
interrupted on channel A only (rise + fall) — 2× decoding. The Rust firmware
decodes both channels on every edge — 4× decoding. A calibration taken against
the old firmware and pasted here reads exactly **double** the true speed.

Field-derived behaviour worth keeping: the rover cruises at **15–16 % duty** and
**cannot climb the ramp below ~11 %**. That is what sets `[speed.autocal]
min_duty_pct = 13.0` and the stall detector's thresholds.

---

## 4. Chassis NUCLEO pin map

Source: `firmware/chassis/src/motor.rs` module docs, `firmware/chassis/src/config.rs`.

| Function | Pin | Peripheral |
|---|---|---|
| Steering servo PWM | `PA3` | TIM2 CH4, 50 Hz |
| Right motor PWM | `PA6` | TIM3 CH1, 20 kHz |
| Left motor PWM | `PE11` | TIM1 CH2, 20 kHz |
| Right motor forward enable | `PF12` | GPIO out |
| Right motor backward enable | `PD15` | GPIO out |
| Left motor forward enable | `PF13` | GPIO out |
| Left motor backward enable | `PE9` | GPIO out |
| IMU | `PB8` / `PB9` | I²C1 |
| Ethernet RMII | `PA1 PA2 PA7 PC1 PC4 PC5 PB13 PG11 PG13` | ETH + LAN8742A |

⚠️ **TIM1 is an advanced-control timer**, unlike TIM2/TIM3. It has a break input
and a main-output-enable gate that must be set before any output appears. If the
steering servo and the right motor work on first power-up but the **left motor
stays dead**, TIM1's MOE/break configuration is the first place to look.

Servo: centre `100.0°`, duty band `0.05`–`0.10`, steering limit `±45°`.

---

## 5. Sensors NUCLEO pin map

Source: `firmware/sensors/src/encoders.rs` module docs, `firmware/sensors/src/config.rs`.

| Function | Pin | Peripheral |
|---|---|---|
| Left encoder channel A | `PA15` | EXTI15 |
| Left encoder channel B | `PB5` | EXTI5 |
| Right encoder channel A | `PB3` | EXTI3 |
| Right encoder channel B | `PB4` | EXTI4 |
| INA226 | `PB8` / `PB9` | I²C1, addr `0x40`, 0.1 Ω shunt |
| Link-loss indicator LED | `PB0` | GPIO out |
| Ethernet RMII | `PA1 PA2 PA7 PC1 PC4 PC5 PB13 PG11 PG13` | ETH + LAN8742A |

Left/right mapping is inherited from the ROS 2 firmware, where `encoder_a` was
the left wheel (`app.cpp`: `mt_lf_encode_msg = enc_A`).

### ⚠️ Hardware quadrature is impossible on this harness

STM32 timer encoder mode needs both channels of **one** encoder on the **same**
timer. Verified exhaustively against `stm32-metapac`'s pin/AF tables for the
F767ZI — not a datasheet skim:

| Pin | Only timer-capable alternate function |
|---|---|
| `PA15` (left ch A) | TIM2_CH1 |
| `PB5` (left ch B) | TIM3_CH2 |
| `PB3` (right ch A) | TIM2_CH2 |
| `PB4` (right ch B) | TIM3_CH1 |

Each encoder's own A/B pair **straddles two timers**. `{PA15, PB3}` would be a
valid TIM2 pair and `{PB4, PB5}` a valid TIM3 pair — but those pair the two
encoders' channel As together, which is not a thing software can do.

This is a property of how the harness was soldered for GPIO-interrupt reading,
and it forecloses hardware quadrature **without a physical rewire**. The
firmware therefore does software 4× decode over EXTI, which is correct but
loses the hardware guarantee that a count can never be missed under load.

---

## 6. Safety

The ROS 2 firmware had **no watchdog of any kind**: if the RPi died mid-drive,
the STM32 held its last PWM value indefinitely and the rover kept going. Both
Rust firmware images now carry two independent layers:

| Layer | Timeout | Catches |
|---|---|---|
| Command watchdog | 200 ms (10 missed frames @ 50 Hz) | dead peer — RPi or link |
| IWDG | 500 ms | dead firmware — hung task, deadlocked I²C |

On a command timeout the throttle **ramps** to zero over 300 ms rather than
stepping (a step into a loaded drivetrain is a mechanical shock), steering
centres immediately, and recovery requires an explicit zero-throttle command —
the rover never resumes on its own just because packets came back.

---

## 7. Measured latency budget (ROS 2 baseline)

Kept as the number the rewrite is measured against, not as a spec:

| Stage | ROS 2 |
|---|---|
| Camera capture | 33 ms |
| Lane detection | 30–40 ms |
| Steering control | 20 ms |
| Chassis command | 20 ms |
| Motor actuation | 50 ms |
| **End to end** | **100–150 ms** |

The stages above sum to roughly 153–163 ms, higher than the stated 100–150 ms
end-to-end figure; the two were measured independently and the gap is
presumed pipeline overlap (e.g. the next camera frame capturing while the
current one is still being steered on), not an arithmetic error — neither
figure has been re-measured to confirm the overlap.

Two of those hops no longer exist. Camera capture and lane detection were two
ROS processes joined by a ~2.7 MB serialize + localhost UDP hop + decode, 30
times a second; they are now one loop iteration in one process.

**"20 Hz" here and "50 Hz" in `CONTROL_LAW.md` describe different things, not
a contradiction.** The 20 Hz above is the old chassis firmware's own motor
control *polling loop* — `MOTOR_RESPONSE_PERIOD_MS = 50` in
`motor_control.cpp`, i.e. it re-applied its held command at ~20 Hz regardless
of how often a new one arrived. `CONTROL_LAW.md`'s 50 Hz is the *topic* rate
at which the RPi published `tpc_chassis_cmd` — a firmware update rate versus
a publish rate, on the same old system. The Rust firmware now runs actuation
against a 50 Hz command stream, unifying the two.

---

## 8. Storage

If video recording is ever added back to the perception process, the arithmetic
that governed it before: **~83 MB/s** at native 1280×720 / 30 FPS fills the
Jetson's 128 GB eMMC in **~26 minutes**. The ROS 2 recorder throttled to
640×360 @ 10 FPS for this reason, and ran as its own process because video
encoding is CPU-bound and must never contend with the detection loop.
