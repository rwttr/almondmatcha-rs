# Dual CSV Logging Architecture

> **This documents the ROS 2 system that branch `rs` replaced.** The dual-tier
> D4/D5 relay architecture went with the DDS domains. Kept for its column
> schemas and analysis guidance, which `rover-telemetry` follows. See design
> defect D4 in [RUST_REWRITE_PLAN.md](RUST_REWRITE_PLAN.md) §13.3b for where
> the base-station CSV knowingly contradicts this file.

## Filenames on branch `rs`

The schemas and column meanings below still apply. The **filenames do not** —
`rover-telemetry` writes a different set. Where an `rs` file covers the same
data as a ROS 2 one, they're paired here; where it doesn't, that's noted too:

| ROS 2 (as documented below) | branch `rs` |
|---|---|
| `rtk_gnss.csv` | `rtk_gnss.csv` — same name, header changed (see `rover-telemetry`'s `csv_fmt.rs`) |
| `spresense_gnss.csv` | `backup_gnss.csv` |
| `chassis_sensors.csv` | `power.csv` — voltage/current only; encoders now feed the estimator directly and aren't logged to CSV at all |
| `mission_state.csv` | `mission_status.csv` — the schema heading below stays `mission_state.csv` because `geo.rs`/`mission.rs` cite it by that name |
| `chassis_speed_pid.csv` | `speed_loop_debug.csv` |
| `chassis_imu.csv` | *(none — `ImuSample` routes only to `control`; raw IMU isn't captured)* |
| `chassis_cmd.csv` | *(none — motor commands aren't logged on this branch)* |
| — | `rover_state.csv`, `chassis_status.csv`, `board_diagnostics.csv` — new on `rs`, no ROS 2 equivalent |

---

## Overview

The ROS 2 rover ran a **dual-tier CSV logging system**: the RPi wrote
high-fidelity per-topic CSVs at native sensor rates (Tier 1), and the Jetson
wrote an aggregated, down-sampled copy to its higher-capacity storage (Tier
2), for redundancy and eventual database migration.

---

## Architecture

### Tier 1: RPi High-Fidelity Logging

**Node**: `rover_monitoring_node` (rover_monitoring), C++, Domain 5.
**Location (ROS 2)**: `ws_rpi/runs/` — never existed in this repository
(`.gitignore` excluded it) and does not exist on branch `rs`.

**Characteristics**:
- Subscribed to Domain 5 topics directly (STM32 sensors/commands, Jetson
  steering command, RPi speed-PID debug) — deliberately not lane data, which
  stayed Domain-6-only and was logged on the Jetson side instead
- Logged each topic at its native rate (event-driven), 4–50 Hz depending on
  sensor, across 7 separate files
- **Files were created on the first message of their topic, not at startup.**
  A run that received nothing left no directory behind and consumed no run
  number, and a missing CSV was positive evidence that topic never delivered
  data — previously every file existed with headers regardless, so a dead run
  was indistinguishable from a healthy one that had no data yet.

**CSV Files**:
- `rtk_gnss.csv` (~10 Hz): RTK position data from u-blox ZED-F9P
- `spresense_gnss.csv` (~10 Hz): GPS data from Sony Spresense
- `chassis_imu.csv` (~10 Hz): Accelerometer and gyroscope from STM32
- `chassis_sensors.csv` (~4 Hz): Encoders, voltage, current, power
- `chassis_cmd.csv` (~50 Hz): Motor commands (speed, steering angle + direction, drive direction)
- `mission_state.csv` (event-driven): Mission status, destination, steering, lane detection
- `chassis_speed_pid.csv` (~4 Hz): Closed-loop speed PID internals (measured/target wheel speed, error, output)

---

### Tier 2: Jetson Aggregated Logging

**Node**: `rover_local_monitoring_node` (rover_monitoring), Python, Domain 4.
**Location (ROS 2)**: `ws_jetson/runs/` — never existed in this repository
and does not exist on branch `rs`.

**Characteristics**:
- Subscribed to `/tpc_telemetry_relay` on Domain 4 — one aggregated message,
  versus Tier 1's 10+ direct subscriptions
- Logged at 5 Hz (the telemetry relay rate)
- Files created on first message, same rule as Tier 1
- Python, for easy database migration; high-capacity Jetson storage vs. the
  RPi's limited SD card

**CSV Files**:
- `telemetry_unified.csv`: All telemetry data in one file (5 Hz)
- `rtk_gnss.csv`: RTK position data (5 Hz, valid only)
- `spresense_gnss.csv`: Spresense GPS data (5 Hz, valid only)
- `chassis_data.csv`: Combined sensors, IMU, commands (5 Hz)
- `mission_state.csv`: Mission status, destination, steering, lane (5 Hz)

---

## Comparison

RPi logged full-fidelity per-topic (4–50 Hz, 7 files, limited SD-card
storage); Jetson logged a 5 Hz aggregate (5 files, high-capacity storage) for
redundancy and easier database migration. Neither tier added subscription
load to the other's domain — Tier 1 rode the existing D5 subscriptions, Tier
2 the existing D4 relay.

---

## CSV Field Reference

### RPi: rtk_gnss.csv
Source: `tpc_gnss_ublox` topic (~10 Hz) — u-blox ZED-F9P RTK receiver

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) — base PC wall clock |
| `Date` | string | — | Date string as reported by the u-blox receiver |
| `Time` | string | — | UTC time string as reported by the u-blox receiver |
| `Latitude` | float64 | °  | WGS84 latitude, positive = North |
| `Longitude` | float64 | °  | WGS84 longitude, positive = East |
| `Altitude` | float64 | m  | Height above ellipsoid |
| `Fix_Quality` | string | — | `"No Fix"`, `"GPS"`, `"DGPS"`, `"RTK Float"`, `"RTK Fixed"` — use `RTK Fixed` for cm-level accuracy |
| `Centimeter_Error` | float32 | cm | Estimated horizontal position error reported by u-blox |
| `Satellites` | int32 | count | Number of satellites used in solution |
| `SNR` | float32 | dB | Signal-to-noise ratio |
| `Speed_ms` | float64 | m/s | Ground speed reported by the receiver |

---

### RPi: spresense_gnss.csv
Source: `tpc_gnss_spresense` topic (~10 Hz) — Sony Spresense standard GPS

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Date` | string | — | Date string as reported by the Spresense |
| `Time` | string | — | Time string as reported by the Spresense |
| `Num_Satellites` | int32 | count | Number of satellites in view |
| `Fix` | bool | 0/1 | `1` = GNSS fix acquired |
| `Latitude` | float64 | °  | WGS84 latitude |
| `Longitude` | float64 | °  | WGS84 longitude |
| `Altitude` | float64 | m  | Height above ellipsoid |

---

### RPi: chassis_imu.csv
Source: `tpc_chassis_imu` topic (~10 Hz) — LSM6DSV16X on STM32 chassis board

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Accel_X` | int32 | m/s² × 1000 | X-axis acceleration (divide by 1000 for m/s²). Rover longitudinal axis. |
| `Accel_Y` | int32 | m/s² × 1000 | Y-axis acceleration. Rover lateral axis. |
| `Accel_Z` | int32 | m/s² × 1000 | Z-axis acceleration. Vertical. At rest ≈ 9810 (1 g). |
| `Gyro_X` | int32 | rad/s × 1000 | Angular velocity around X (roll rate) |
| `Gyro_Y` | int32 | rad/s × 1000 | Angular velocity around Y (pitch rate) |
| `Gyro_Z` | int32 | rad/s × 1000 | Angular velocity around Z (yaw rate) |

> Values are stored as raw integer × 1000. Divide by 1000.0 to get physical units (m/s², rad/s).

---

### RPi: chassis_sensors.csv
Source: `tpc_chassis_sensors` topic (~4 Hz) — INA226 + encoders on STM32 sensors board

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Motor_Left_Encoder` | int32 | counts | Left motor encoder cumulative count |
| `Motor_Right_Encoder` | int32 | counts | Right motor encoder cumulative count |
| `System_Current_A` | float32 | A | Battery bus current measured by INA226 |
| `System_Voltage_V` | float32 | V | Battery bus voltage measured by INA226 |
| `Power_W` | float32 | W | Computed: `System_Voltage_V × System_Current_A` |

Raw counts are cumulative, not a rate — differentiate against `Timestamp_us`
to get wheel speed (ticks/sec), or use `chassis_speed_pid.csv` below, which
already has the measured rate the closed-loop speed controller computed.

---

### RPi: chassis_cmd.csv
Source: `tpc_chassis_cmd` topic (~50 Hz) — commands sent by RPi chassis_controller_node to STM32

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `FDR_Msg` | int | enum | **Steering direction** (`fdr_msg`): `1`=right, `2`=straight, `3`=left |
| `RO_Ctrl_Deg` | float32 | ° | Continuous steering angle magnitude (`ro_ctrl_msg`) sent to the STM32 servo driver |
| `SPD_Msg` | int (0–255 range, values 0–100 used) | % duty | Final speed command after closed-loop PID correction (if active) and the operator safety cap. STM32 divides by 100 for PWM duty cycle — both wheels share this one value. |
| `BDR_Msg` | int | enum | **Drive direction** (`bdr_msg`): `0`=stop, `1`=forward, `2`=backward |

---

### RPi: mission_state.csv
Source: multiple D5 topics (event-driven on any topic change)

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Mission_Active` | bool | 0/1 | `1` = GNSS waypoint mission running; `0` = idle |
| `Distance_Remaining_m` | float | m | Remaining straight-line distance to destination waypoint |
| `Dest_Latitude` | float | °  | Current target waypoint latitude (from base station action) |
| `Dest_Longitude` | float | °  | Current target waypoint longitude |
| `Steering_Cmd` | float32 | ° | Kinematic control output from Jetson (`tpc_rover_ctrl_cmd[0]`). Continuous steering angle command fed to chassis_controller_node. |

No `Lane_*` columns here by design: raw lane detection (`tpc_rover_nav_lane`)
existed only on Domain 6 (Jetson localhost), and this node ran entirely on
Domain 5, so it could never receive it — D5 and D6 logging were kept
deliberately separate, not bridged. Lane data was logged on the Jetson side
instead, see `ws_jetson_lane_detection_*.csv` below.

Rows arrived at whatever rate `Mission_Active`/destination/distance changed —
`Steering_Cmd` was the latest value at that moment, not a forced row per
steering update (that was the Jetson-side `ws_jetson_kinematic_ctrl_*.csv`,
at full control-loop rate).

---

### RPi: chassis_speed_pid.csv
Source: `tpc_chassis_speed_debug` topic (~4 Hz, paced by the encoder feed) — published by
`chassis_controller_node`'s closed-loop speed PID (`chassisSensorsCallback()`), which
otherwise computed and discarded these values internally with no external trace.

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `Timestamp_us` | int64 | µs | System clock (µs since Unix epoch) |
| `Measured_Left_TPS` | float32 | ticks/s | Left wheel speed, from the encoder delta since the previous message |
| `Measured_Right_TPS` | float32 | ticks/s | Right wheel speed, same basis |
| `Measured_Avg_TPS` | float32 | ticks/s | Average of left/right — the raw process variable |
| `Target_TPS` | float32 | ticks/s | Setpoint: `(target_speed_pct / 100) × max_ticks_per_sec` |
| `Error_Pct` | float32 | % of full scale | `target_speed_pct - (Measured_Avg_TPS / max_ticks_per_sec × 100)` — the error the PID actually operates on. **Not** ticks/s: the loop runs in the same 0–100% unit as its output so the gains survive re-calibrating `max_ticks_per_sec` |
| `PID_Output_Pct` | float32 | % duty | Final output (feedforward + PID trim) before the operator safety cap — compare against `chassis_cmd.csv`'s `SPD_Msg`, which is this value *after* the cap |

Used to tune `speed_kp`/`speed_ki`/`speed_kd` and `max_ticks_per_sec` in
`ws_rpi/src/chassis_control/config/chassis_speed_control_params.yaml`
(deleted with the ROS 2 tree) by plotting `Target_TPS` vs `Measured_Avg_TPS`
and `Error_Pct` over time.

`PID_Output_Pct - target_speed_pct` is the trim the loop was applying: near
zero meant the feedforward alone was right; persistently large meant the
terrain load (or a stale `max_ticks_per_sec` calibration) was doing real work.

---

### Jetson: telemetry_unified.csv
Source: `tpc_telemetry_relay` topic (5 Hz) — aggregated relay from RPi. One row = one relay message.

| Column | Type | Description |
|--------|------|-------------|
| `Timestamp` | ISO8601 string | Jetson wall-clock time when message was received |
| `Mission_Active` | bool | See mission_state above |
| `Distance_Remaining_km` | float64 | km to destination waypoint |
| `Spresense_Valid` | bool | `1` = Spresense GNSS data is fresh and valid |
| `Spresense_Lat/Lon/Alt` | float32 | Degrees / metres — Spresense GPS position |
| `Spresense_Sats` | int32 | Satellite count |
| `Ublox_Valid` | bool | `1` = RTK data is fresh and valid |
| `Ublox_Lat/Lon/Alt` | float64 | Degrees / metres — u-blox RTK position |
| `Ublox_Fix` | string | RTK fix quality string (e.g. `"RTK Fixed"`) |
| `Ublox_Err_cm` | float32 | cm — u-blox estimated horizontal error |
| `Ublox_Sats` | int32 | Satellite count |
| `Chassis_Cmd_Valid` | bool | `1` = chassis command data is fresh |
| `Cmd_Left_Speed` / `Cmd_Right_Speed` | float32 | Speed command 0–255 (both equal to `spd_msg`) |
| `Cmd_Steer_Dir` | int | Steering enum: 1=right, 2=straight, 3=left |
| `Cmd_Drive_Dir` | int | Drive enum: 0=stop, 1=forward, 2=backward |
| `Chassis_Sensors_Valid` | bool | `1` = sensor data is fresh |
| `Encoder_Left` / `Encoder_Right` | int32 | Motor encoder counts |
| `Voltage_V` / `Current_A` / `Power_W` | float32 | Battery measurements |
| `Chassis_IMU_Valid` | bool | `1` = IMU data is fresh |
| `Accel_X/Y/Z` | int32 | Raw × 1000 (÷1000 → m/s²) |
| `Gyro_X/Y/Z` | int32 | Raw × 1000 (÷1000 → rad/s) |
| `Steering_Valid` | bool | `1` = steering command is fresh |
| `Steering_Cmd` | float32 | Kinematic control output from Jetson |
| `Lane_Valid` | bool | `1` = lane detection data is fresh |
| `Lane_Theta` | float64 | Lane angle from Jetson detector |
| `Lane_B` | float64 | Lane y-intercept (px) |
| `Lane_Detected` | bool | `1` = lane visible in frame |
| `Dest_Valid` | bool | `1` = destination coordinate has been set |
| `Dest_Lat` / `Dest_Lon` | float32 | Target waypoint coordinates |

> **Down-sampling note:** Jetson logs at 5 Hz (relay rate). RPi logs at native topic rate (4–50 Hz). Use RPi logs for high-frequency analysis (motor commands, IMU). Use Jetson logs for aggregated mission analysis.

---

## Vision Navigation Logs (ws_jetson, `vision_navigation` package)

Separate from the dual-tier system above: these files were written directly
by the vision/control nodes, not by `rover_monitoring`. They landed inside
the same `run_NNN_<stamp>/` directory as the Tier 2 CSVs — one launch, one
directory per machine (ROS 2; this directory never existed in this
repository).

Each logging node was a separate process, so the launch scripts allocated the
run directory once and exported it as `$ROVER_RUN_DIR`, which every node
preferred over computing its own — without it, each process would pick its
own run number and timestamp, scattering one launch across several
directories. Starting a node by hand with `ros2 run` (no launcher, so no
variable) gave it its own run directory, which was expected.

Each machine kept its run output inside its own workspace, so Jetson and RPi
logs could never collide, and wiping one machine's runs never touched the
other's.

All three write asynchronously: the owning node enqueues a row/frame and a
background thread drains the queue and writes it to disk, so a slow eMMC/SD
card never blocks the image-processing or control callback.

### Jetson: ws_jetson_lane_detection_TIMESTAMP.csv
Source: `lane_detection_node` — one row per processed camera frame.

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `timestamp` | ISO8601 string | — | Wall-clock time the frame finished processing |
| `curvature` | float | 1/px | Parabola coefficient A (`x = A*y² + B*y + C`), rover-centered frame, BEV pixels -- not converted to a real 1/m arc |
| `theta` | float | ° | Heading error angle (+ = turn right) |
| `b` | float | m | Lateral offset from lane center |
| `detected` | float | 0/1 | `1.0` = lane detected, `0.0` = not detected |
| `fps` | float | frames/s | Rolling-window achieved throughput over the last 30 processed frames — measures actual image-processing rate, not the camera's configured capture target. `0.0` until the window has ≥2 samples. |

### Jetson: ws_jetson_kinematic_ctrl_TIMESTAMP.csv
Source: `rover_kinematic_control` — one row per `tpc_rover_nav_lane` message received.

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `time_sec` | float | s | `time.time()` at the control update |
| `theta_ema` | float | ° | EMA-filtered heading error |
| `b_ema` | float | m | EMA-filtered lateral offset |
| `curvature_ema` | float | — | EMA-filtered curvature (feedforward input) |
| `pid_u` | float | — | PID feedback output (`u_pid`) |
| `e_sum` | float | — | Combined error fed to the PID (`k_e1*theta_ema + k_e2*b_ema`) |
| `steer_angle` | float | ° | Final commanded steering angle (PID + feedforward, clamped) |
| `speed_cmd` | int | 0–100 | Chassis speed command (% PWM duty cycle) |
| `detected` | int | 0/1 | Detection validity used for this control update (post warm-up/timeout logic) |

### Jetson: ws_jetson_camera_TIMESTAMP.avi
Source: `camera_recorder_node` — raw (uncompressed) video of `tpc_rover_d415_rgb`, for
offline debugging (e.g. "what did the camera actually see when detection dropped out
at 14:32?"). Same timestamp convention as the two CSVs above, so the three files from
one run are found by matching timestamps in the filename — there's no shared run
number the way the RPi-side `run_NNN_.../` directories have one.

Deliberately a separate node from `camera_stream_node` (upstream of the whole control
loop — a recording bug must not be able to affect frame capture) and from
`lane_detection_node` (video encoding is CPU-bound work, unlike the tiny CSV row
writes that node already backgrounds safely — doing it in-process risks contending
with the actual detection loop for CPU time).

**Defaults:** 640×360 @ 10 FPS, throttled down from the camera's native
1280×720/30 FPS feed and written with no compression (no encode step, to keep CPU
low). At these defaults: **~25 GB/hour**. The Jetson's eMMC is 128 GB total, shared
with the OS and everything else — raw at the *camera's* native 1280×720/30 FPS would
fill the entire disk in about 26 minutes, which is why this is throttled/downscaled
rather than recorded at full fidelity. Tunable via ROS parameters (`record_fps`,
`record_width`, `record_height`, `enabled`) without a rebuild — recompute the
MB/hour math before raising them. Logs a one-time warning if free disk space drops
below `low_space_warn_mb` (default 2048 MB), but does not auto-stop recording.

> **Note:** the exact FOURCC used for "uncompressed" AVI output depends on the
> OpenCV/FFmpeg build actually installed on the Jetson and could not be verified on
> dev hardware (no camera attached, no way to test video write/playback here) —
> confirm the file plays back correctly the first time this runs in the field.

---

## Run Directory Numbering

Both RPi and Jetson used synchronized run numbering:

**Pattern**: `run_NNN_YYYYMMDD_HHMMSS/`

- `NNN`: Auto-incremented run number (001, 002, ...)
- `YYYYMMDD`: Date (20250104 = Jan 4, 2025)
- `HHMMSS`: Time (143052 = 2:30:52 PM)

**Detection**: `glob.glob("run_*")` finds existing runs, increments max number

**Example**:
- Run 1: `run_001_20250104_143052/`
- Run 2: `run_002_20250104_151823/`

---

## Launch Integration (ROS 2 — historical, not executable on branch `rs`)

None of the commands below run on branch `rs` — `ws_rpi/`, `ws_jetson/`,
`ws_base/` and the launch scripts they reference were all removed in the ROS
2 tree deletion. Recover them from the ROS 2 fallback repository,
`RoboticsGG/almondmatcha`'s `main` branch — this repository's `origin` has no
`main` at all — with, for example,
`git show roboticsgg-almondmatcha/main:ws_rpi/launch_rover_tmux.sh`, or clone
that repository and check out its `main` to run them for real.

### RPi (ws_rpi)

CSV logging was automatic once `rover_monitoring_node` launched, started
alongside `mission_monitoring_node_rpi` (relay only, no CSVs) by the same
launch script — no additional configuration needed.

### Jetson (ws_jetson)

`rover_local_monitoring_node` ran in the Jetson launch stack under
`ROS_DOMAIN_ID=4`, alongside the rest of that machine's nodes.

### Base Station (ws_base)

No CSV logging on base station (display-only). Monitoring node subscribed to
Domain 4 telemetry relay for real-time display.
