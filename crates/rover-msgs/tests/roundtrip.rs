//! Contract tests for the wire format.
//!
//! `encode` and `decode` are hand-written per type, which keeps the byte layout
//! readable but means the two could drift apart. These tests close that gap:
//! every type is round-tripped with non-trivial values, and the encoded length
//! is checked against its declared `WIRE_LEN`.
//!
//! If you add a message type, add it to `all_types!` below. Nothing else in the
//! suite needs to change.

use rover_msgs::*;

/// Every `Wire` type in the crate, with a populated instance of each.
///
/// Values are deliberately distinctive — no zeros, no repeated numbers, every
/// field different — so a decode that reads fields in the wrong order or at the
/// wrong offset produces a mismatch rather than accidentally passing.
macro_rules! all_types {
    ($mac:ident) => {
        $mac!(ImuSample {
            accel_mps2: [1.5, -2.25, 9.81],
            gyro_radps: [0.125, -0.5, 1.75],
            t_us: 123_456_789,
        });
        $mac!(MagSample {
            field_gauss: [0.21, -0.34, 0.47],
            t_us: 987_654_321,
        });
        $mac!(ChassisStatus {
            seq_echo: 40_001,
            watchdog_tripped: true,
            fault: FaultBits(0b1010),
            t_us: 55_555,
        });
        $mac!(ChassisCommand {
            steer: -0.375,
            throttle: 0.625,
            seq: 4242,
        });
        $mac!(WheelSensors {
            ticks_left: -123_456,
            ticks_right: 654_321,
            t_us: 7_777_777,
        });
        $mac!(PowerSample {
            bus_volts: 12.6,
            current_amps: 3.25,
        });
        $mac!(GnssFix {
            lat_deg: 7.006_873_21,
            lon_deg: 100.498_765_43,
            alt_m: 42.5,
            fix: FixQuality::RtkFixed,
            sats: 23,
            h_acc_m: 0.014,
            speed_mps: 0.185,
            course_deg: 271.5,
            utc_ms: 1_700_000_000_123,
            source: GnssSource::Rtk,
        });
        $mac!(LaneMeasurement {
            curvature_inv_m: 0.0325,
            heading_err_rad: -0.0873,
            cross_track_m: 0.142,
            valid: true,
            t_us: 314_159,
        });
        $mac!(RoverState {
            cross_track_m: 0.0731,
            heading_err_rad: -0.0219,
            curvature_inv_m: 0.0417,
            speed_mps: 0.183,
            gyro_bias_radps: 0.0026,
            p_diag: [1e-4, 2e-4, 3e-4, 4e-4, 5e-4],
            lane_age_ms: 33,
        });
        $mac!(MotionSetpoint {
            steer_rad: 0.196,
            speed_mps: 0.20,
        });
        $mac!(MissionGoal {
            lat_deg: 7.006_9,
            lon_deg: 100.498_8,
        });
        $mac!(MissionStatus {
            active: true,
            distance_remaining_m: 137.25,
            target: Some(MissionGoal {
                lat_deg: 7.006_9,
                lon_deg: 100.498_8,
            }),
            state: MissionState::Running,
        });
        $mac!(CommandFrame {
            cmd_seq: 9001,
            body: Command::SetMissionGoal(MissionGoal {
                lat_deg: 7.006_9,
                lon_deg: 100.498_8,
            }),
        });
        $mac!(Telemetry {
            seq: 1_000_001,
            t_us: 9_876_543_210,
            state: RoverState {
                cross_track_m: 0.0731,
                heading_err_rad: -0.0219,
                curvature_inv_m: 0.0417,
                speed_mps: 0.183,
                gyro_bias_radps: 0.0026,
                p_diag: [1e-4, 2e-4, 3e-4, 4e-4, 5e-4],
                lane_age_ms: 33,
            },
            mission: MissionStatus {
                active: true,
                distance_remaining_m: 137.25,
                target: Some(MissionGoal {
                    lat_deg: 7.006_9,
                    lon_deg: 100.498_8,
                }),
                state: MissionState::Running,
            },
            power: PowerSample {
                bus_volts: 12.6,
                current_amps: 3.25,
            },
            rtk: GnssFix {
                lat_deg: 7.006_873_21,
                lon_deg: 100.498_765_43,
                alt_m: 42.5,
                fix: FixQuality::RtkFixed,
                sats: 23,
                h_acc_m: 0.014,
                speed_mps: 0.185,
                course_deg: 271.5,
                utc_ms: 1_700_000_000_123,
                source: GnssSource::Rtk,
            },
            backup: GnssFix {
                lat_deg: 7.006_8,
                lon_deg: 100.498_7,
                alt_m: 41.0,
                fix: FixQuality::Autonomous,
                sats: 9,
                h_acc_m: 2.5,
                speed_mps: 0.19,
                course_deg: 270.0,
                utc_ms: 1_700_000_000_000,
                source: GnssSource::Backup,
            },
            last_cmd_seq: 9001,
            health: HealthBits(0b0000_0100),
        });
        $mac!(SpeedLoopDebug {
            measured_left_tps: 412.5,
            measured_right_tps: 408.25,
            target_tps: 410.0,
            error_pct: -0.75,
            pid_output_pct: 16.125,
        });
        $mac!(EkfDebug {
            innovation: [0.012, -0.004, 0.0009],
            nis: 2.87,
            gated: false,
        });
        $mac!(BoardDiagnostics {
            board: BoardId::Sensors,
            post_run: PostBits(0b0111_1111),
            post_pass: PostBits(0b0110_1011),
            reset_cause: ResetCause::IndependentWatchdog,
            phy_id: 0x0007_C130,
            link_speed_mbps: 100,
            link_full_duplex: true,
            phy_symbol_errors: 17,
            uptime_s: 8_675_309,
            tx_drops: 42,
        });
    };
}

/// Encode, decode, compare — and check the length matches the declaration.
macro_rules! check_roundtrip {
    ($ty:ident { $($field:tt)* }) => {{
        let original = $ty { $($field)* };

        let mut buf = [0u8; 512];
        let written = original.encode(&mut buf);

        assert_eq!(
            written,
            $ty::WIRE_LEN,
            "{}: encode() wrote {} bytes but WIRE_LEN says {}",
            $ty::NAME, written, $ty::WIRE_LEN,
        );

        let decoded = $ty::decode(&buf).unwrap_or_else(|e| {
            panic!("{}: decode failed on its own output: {e}", $ty::NAME)
        });

        assert_eq!(
            decoded, original,
            "{}: value changed across encode/decode", $ty::NAME,
        );
    }};
}

#[test]
fn every_type_roundtrips_and_matches_its_declared_length() {
    all_types!(check_roundtrip);
}

/// Trailing bytes must be ignored. Receivers get whole datagrams, which may be
/// padded by a link layer; a message must decode from the front regardless.
macro_rules! check_trailing_ignored {
    ($ty:ident { $($field:tt)* }) => {{
        let original = $ty { $($field)* };
        let mut buf = [0xABu8; 512];
        original.encode(&mut buf);
        let decoded = $ty::decode(&buf).expect("decode with trailing bytes");
        assert_eq!(decoded, original, "{}: trailing bytes changed the value", $ty::NAME);
    }};
}

#[test]
fn trailing_bytes_are_ignored() {
    all_types!(check_trailing_ignored);
}

/// A buffer one byte short must be rejected, not read out of bounds.
macro_rules! check_short_buffer {
    ($ty:ident { $($field:tt)* }) => {{
        let original = $ty { $($field)* };
        let mut buf = [0u8; 512];
        original.encode(&mut buf);

        let short = &buf[..$ty::WIRE_LEN - 1];
        match $ty::decode(short) {
            Err(DecodeError::TooShort { need, got }) => {
                assert_eq!(need, $ty::WIRE_LEN, "{}: wrong `need`", $ty::NAME);
                assert_eq!(got, $ty::WIRE_LEN - 1, "{}: wrong `got`", $ty::NAME);
            }
            other => panic!(
                "{}: expected TooShort for a truncated buffer, got {other:?}",
                $ty::NAME
            ),
        }
    }};
}

#[test]
fn truncated_buffers_are_rejected() {
    all_types!(check_short_buffer);
}

/// Bodies must fit the frame budget, so a message can never be silently
/// truncated by the link layer.
macro_rules! check_fits_frame {
    ($ty:ident { $($field:tt)* }) => {{
        let _ = $ty { $($field)* };
        assert!(
            $ty::WIRE_LEN <= rover_msgs::frame::MAX_BODY_LEN,
            "{}: WIRE_LEN {} exceeds MAX_BODY_LEN {}",
            $ty::NAME, $ty::WIRE_LEN, rover_msgs::frame::MAX_BODY_LEN,
        );
    }};
}

#[test]
fn every_body_fits_the_frame_budget() {
    all_types!(check_fits_frame);
}

/// Type IDs are permanent and must be unique. Reusing one for a different shape
/// makes two machines silently disagree about what they are reading, which is
/// the single worst failure this protocol can have.
#[test]
fn type_ids_are_unique() {
    // Declared before the macro: identifiers in a macro body resolve at the
    // macro's definition site, so `ids` must already be in scope here.
    let mut ids: Vec<(u16, &'static str)> = Vec::new();

    macro_rules! collect_id {
        ($ty:ident { $($field:tt)* }) => {{
            let _ = $ty { $($field)* };
            ids.push(($ty::TYPE_ID, $ty::NAME));
        }};
    }

    all_types!(collect_id);

    ids.sort_by_key(|(id, _)| *id);
    for pair in ids.windows(2) {
        assert_ne!(
            pair[0].0, pair[1].0,
            "type ID 0x{:04X} used by both `{}` and `{}`",
            pair[0].0, pair[0].1, pair[1].1,
        );
    }
}
