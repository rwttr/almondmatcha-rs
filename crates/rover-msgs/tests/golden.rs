//! Cross-language golden-byte fixtures.
//!
//! # Why this file exists, separately from `roundtrip.rs`
//!
//! `roundtrip.rs` proves `encode`/`decode` are inverses of each other *within
//! Rust*. It says nothing about whether the Python perception node agrees
//! with Rust about what those bytes mean. Two implementations can round-trip
//! perfectly and still disagree with each other — swap two fields of the same
//! type, or get endianness backwards, and each side is internally consistent
//! while the wire is silently corrupt between them. A byte-for-byte fixture,
//! checked into the repo and read by both languages' test suites, is the only
//! thing that catches that.
//!
//! # BREAKING PROTOCOL CHANGE
//!
//! **Regenerating `testdata/*.bin` is a breaking wire-format change.** The
//! files here are not test scaffolding to regenerate on a whim — they *are*
//! the contract. If a fixture changes, every node that speaks this protocol
//! (RPi binaries, both STM32 boards, the Jetson `perception` process) has
//! drifted out of sync with whatever is still deployed, and must be rebuilt
//! and redeployed together.
//!
//! To regenerate deliberately, after an intentional format change:
//!
//! ```text
//! UPDATE_GOLDEN=1 cargo test -p rover-msgs --test golden
//! ```
//!
//! That writes fresh fixtures instead of checking against the committed
//! ones. Review the resulting diff in `testdata/` like you would review a
//! schema migration — because that is what it is — then update
//! `perception/tests/test_wire_golden.py` and `perception/rover_perception/wire.py`
//! in the same change. A fixture commit with no corresponding Python change
//! is a bug, not a refactor.
//!
//! # Instance duplication with `roundtrip.rs` is intentional
//!
//! Each file under `tests/` compiles as its own independent binary, so a
//! `macro_rules!` defined in one is not visible to another — Cargo gives test
//! files no shared module unless one is added under `tests/`, which is out of
//! scope for this change (see the task boundary: this file only). The
//! `all_types!` instances below are therefore copied verbatim from
//! `roundtrip.rs` rather than imported. **Keep them identical.** They exist
//! twice so that the round-trip suite and the golden-byte suite agree on what
//! "a representative value" looks like for each type; if you change one, change
//! the other, or the two suites stop testing the same claim.

use rover_msgs::*;
use std::path::PathBuf;

/// Every `Wire` type, with the same populated instance as `roundtrip.rs`.
///
/// Values are deliberately distinctive — no zeros, no repeated numbers, every
/// field different — so a golden comparison that reads fields in the wrong
/// order or at the wrong offset fails loudly instead of accidentally matching.
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
    };
}

/// `testdata/` lives at the workspace root, two levels up from this crate.
fn testdata_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent() // crates/
        .and_then(|p| p.parent()) // workspace root
        .expect("rover-msgs is always two directories under the workspace root")
        .join("testdata")
}

/// True when `UPDATE_GOLDEN=1` (or any non-empty value) asks us to overwrite
/// fixtures instead of checking against them.
///
/// Deliberately not just `.is_ok()` on the env var: an accidentally-exported
/// `UPDATE_GOLDEN=` (empty) from a shell profile should not silently start
/// overwriting the protocol contract on every `cargo test`.
fn should_update() -> bool {
    std::env::var("UPDATE_GOLDEN")
        .map(|v| !v.is_empty() && v != "0")
        .unwrap_or(false)
}

macro_rules! check_golden {
    ($ty:ident { $($field:tt)* }) => {{
        let original = $ty { $($field)* };

        let mut buf = [0u8; 512];
        let written = original.encode(&mut buf);
        assert_eq!(written, $ty::WIRE_LEN, "{}: encode() length disagrees with WIRE_LEN", $ty::NAME);
        let encoded = &buf[..written];

        let path = testdata_dir().join(format!("{}.bin", $ty::NAME));

        if should_update() {
            std::fs::create_dir_all(path.parent().unwrap())
                .unwrap_or_else(|e| panic!("creating {}: {e}", path.parent().unwrap().display()));
            std::fs::write(&path, encoded)
                .unwrap_or_else(|e| panic!("writing {}: {e}", path.display()));
            eprintln!("wrote {}", path.display());
        } else {
            let fixture = std::fs::read(&path).unwrap_or_else(|e| {
                panic!(
                    "{}: could not read fixture {} ({e}). \
                     If this is a new type, generate it with `UPDATE_GOLDEN=1 cargo test -p rover-msgs --test golden`, \
                     commit the .bin file, and update perception/rover_perception/wire.py to match.",
                    $ty::NAME, path.display(),
                )
            });
            assert_eq!(
                encoded, fixture.as_slice(),
                "{}: encoded bytes no longer match testdata/{}.bin — \
                 this is either a wire-format regression, or an intentional protocol \
                 change that needs `UPDATE_GOLDEN=1` PLUS a coordinated update to \
                 perception/rover_perception/wire.py and a redeploy of every node.",
                $ty::NAME, $ty::NAME,
            );

            // The fixture must also decode back to the same value: a byte match
            // alone doesn't prove decode() agrees about field order.
            let decoded = $ty::decode(&fixture).unwrap_or_else(|e| {
                panic!("{}: fixture failed to decode: {e}", $ty::NAME)
            });
            assert_eq!(decoded, original, "{}: fixture decodes to a different value than the instance that produced it", $ty::NAME);
        }
    }};
}

#[test]
fn wire_bytes_match_committed_fixtures() {
    all_types!(check_golden);
}

/// The 4-byte frame header is part of the contract too — a Python decoder
/// that gets `type_id`/`seq` order or width wrong will otherwise only be
/// caught in the field, since no `Wire` type test exercises it.
#[test]
fn frame_header_matches_committed_fixture() {
    let header = FrameHeader {
        type_id: 0x0501,
        seq: 0xBEEF,
    };
    let mut buf = [0u8; FRAME_HEADER_LEN];
    let written = header.encode(&mut buf);
    assert_eq!(written, FRAME_HEADER_LEN);

    let path = testdata_dir().join("FrameHeader.bin");
    if should_update() {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, buf).unwrap();
        eprintln!("wrote {}", path.display());
    } else {
        let fixture = std::fs::read(&path)
            .unwrap_or_else(|e| panic!("could not read {}: {e}", path.display()));
        assert_eq!(
            &buf[..],
            fixture.as_slice(),
            "FrameHeader: bytes no longer match testdata/FrameHeader.bin"
        );
        let decoded = FrameHeader::decode(&fixture).expect("FrameHeader fixture failed to decode");
        assert_eq!(decoded, header);
    }
}
