"""Golden-byte cross-language contract tests.

Mirrors `crates/rover-msgs/tests/golden.rs` instance-for-instance. If Rust and
Python ever disagree about a byte layout, this is the test that catches it —
see the module docs on `golden.rs` for why regenerating `testdata/*.bin` is a
breaking protocol change, never a routine one.

**Keep the instances below identical to `golden.rs`'s `all_types!` macro.**
They are duplicated for the same reason the Rust side duplicates them from
`roundtrip.rs`: there is no shared-fixture mechanism across the Rust/Python
boundary, so the discipline is "copy exactly, change together."
"""

from __future__ import annotations

import dataclasses
import math
import pathlib

import pytest

from rover_perception.wire import (
    BoardDiagnostics,
    BoardId,
    ChassisCommand,
    ChassisStatus,
    Command,
    CommandFrame,
    EkfDebug,
    FixQuality,
    FrameHeader,
    GnssFix,
    GnssSource,
    ImuSample,
    LaneMeasurement,
    MagSample,
    MissionGoal,
    MissionState,
    MissionStatus,
    MotionSetpoint,
    PowerSample,
    ResetCause,
    RoverState,
    SpeedLoopDebug,
    Telemetry,
    WheelSensors,
)

TESTDATA = pathlib.Path(__file__).resolve().parents[2] / "testdata"


def _approx_equal(a, b) -> bool:
    """Structural equality tolerant of float32 round-off.

    The instances below are written as ordinary Python (double-precision)
    literals for readability, matching the Rust `f32` literals they mirror.
    Round-tripping through `encode`/`decode` passes every value through a
    real `f32`, so e.g. `0.196` and its round-tripped value differ in the
    last few bits of a double -- expected, and not a wire-format bug. The
    actual contract (bytes match `testdata/*.bin` exactly) is checked
    separately in `test_encode_matches_committed_fixture`, with no tolerance
    at all; this helper exists only for the supplementary round-trip checks.
    """
    if a is None or b is None:
        return a is b
    if isinstance(a, float):
        return math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-9)
    if isinstance(a, tuple):
        return len(a) == len(b) and all(_approx_equal(x, y) for x, y in zip(a, b))
    if dataclasses.is_dataclass(a):
        return all(_approx_equal(getattr(a, f.name), getattr(b, f.name)) for f in dataclasses.fields(a))
    return a == b


def _fixture(name: str) -> bytes:
    path = TESTDATA / f"{name}.bin"
    if not path.exists():
        pytest.fail(
            f"missing fixture {path} -- generate it from the Rust side with "
            f"`UPDATE_GOLDEN=1 cargo test -p rover-msgs --test golden`"
        )
    return path.read_bytes()


# Every `(instance, fixture_name)` pair, mirroring `golden.rs`'s `all_types!`.
INSTANCES = [
    (
        ImuSample(
            accel_mps2=(1.5, -2.25, 9.81),
            gyro_radps=(0.125, -0.5, 1.75),
            t_us=123_456_789,
        ),
        "ImuSample",
    ),
    (
        MagSample(field_gauss=(0.21, -0.34, 0.47), t_us=987_654_321),
        "MagSample",
    ),
    (
        ChassisStatus(seq_echo=40_001, watchdog_tripped=True, fault=0b1010, t_us=55_555),
        "ChassisStatus",
    ),
    (
        ChassisCommand(steer=-0.375, throttle=0.625, seq=4242),
        "ChassisCommand",
    ),
    (
        WheelSensors(ticks_left=-123_456, ticks_right=654_321, t_us=7_777_777),
        "WheelSensors",
    ),
    (
        PowerSample(bus_volts=12.6, current_amps=3.25),
        "PowerSample",
    ),
    (
        GnssFix(
            lat_deg=7.00687321,
            lon_deg=100.49876543,
            alt_m=42.5,
            fix=FixQuality.RTK_FIXED,
            sats=23,
            h_acc_m=0.014,
            speed_mps=0.185,
            course_deg=271.5,
            utc_ms=1_700_000_000_123,
            source=GnssSource.RTK,
        ),
        "GnssFix",
    ),
    (
        LaneMeasurement(
            curvature_inv_m=0.0325,
            heading_err_rad=-0.0873,
            cross_track_m=0.142,
            valid=True,
            t_us=314_159,
        ),
        "LaneMeasurement",
    ),
    (
        RoverState(
            cross_track_m=0.0731,
            heading_err_rad=-0.0219,
            curvature_inv_m=0.0417,
            speed_mps=0.183,
            gyro_bias_radps=0.0026,
            p_diag=(1e-4, 2e-4, 3e-4, 4e-4, 5e-4),
            lane_age_ms=33,
        ),
        "RoverState",
    ),
    (
        MotionSetpoint(steer_rad=0.196, speed_mps=0.20),
        "MotionSetpoint",
    ),
    (
        MissionGoal(lat_deg=7.0069, lon_deg=100.4988),
        "MissionGoal",
    ),
    (
        MissionStatus(
            active=True,
            distance_remaining_m=137.25,
            target=MissionGoal(lat_deg=7.0069, lon_deg=100.4988),
            state=MissionState.RUNNING,
        ),
        "MissionStatus",
    ),
    (
        CommandFrame(
            cmd_seq=9001,
            body=Command.set_mission_goal(MissionGoal(lat_deg=7.0069, lon_deg=100.4988)),
        ),
        "CommandFrame",
    ),
    (
        Telemetry(
            seq=1_000_001,
            t_us=9_876_543_210,
            state=RoverState(
                cross_track_m=0.0731,
                heading_err_rad=-0.0219,
                curvature_inv_m=0.0417,
                speed_mps=0.183,
                gyro_bias_radps=0.0026,
                p_diag=(1e-4, 2e-4, 3e-4, 4e-4, 5e-4),
                lane_age_ms=33,
            ),
            mission=MissionStatus(
                active=True,
                distance_remaining_m=137.25,
                target=MissionGoal(lat_deg=7.0069, lon_deg=100.4988),
                state=MissionState.RUNNING,
            ),
            power=PowerSample(bus_volts=12.6, current_amps=3.25),
            rtk=GnssFix(
                lat_deg=7.00687321,
                lon_deg=100.49876543,
                alt_m=42.5,
                fix=FixQuality.RTK_FIXED,
                sats=23,
                h_acc_m=0.014,
                speed_mps=0.185,
                course_deg=271.5,
                utc_ms=1_700_000_000_123,
                source=GnssSource.RTK,
            ),
            backup=GnssFix(
                lat_deg=7.0068,
                lon_deg=100.4987,
                alt_m=41.0,
                fix=FixQuality.AUTONOMOUS,
                sats=9,
                h_acc_m=2.5,
                speed_mps=0.19,
                course_deg=270.0,
                utc_ms=1_700_000_000_000,
                source=GnssSource.BACKUP,
            ),
            last_cmd_seq=9001,
            health=0b0000_0100,
        ),
        "Telemetry",
    ),
    (
        SpeedLoopDebug(
            measured_left_tps=412.5,
            measured_right_tps=408.25,
            target_tps=410.0,
            error_pct=-0.75,
            pid_output_pct=16.125,
        ),
        "SpeedLoopDebug",
    ),
    (
        EkfDebug(innovation=(0.012, -0.004, 0.0009), nis=2.87, gated=False),
        "EkfDebug",
    ),
    (
        BoardDiagnostics(
            board=BoardId.SENSORS,
            post_run=0b0111_1111,
            post_pass=0b0110_1011,
            reset_cause=ResetCause.INDEPENDENT_WATCHDOG,
            phy_id=0x0007_C130,
            link_speed_mbps=100,
            link_full_duplex=True,
            phy_symbol_errors=17,
            uptime_s=8_675_309,
            tx_drops=42,
        ),
        "BoardDiagnostics",
    ),
]


@pytest.mark.parametrize("instance, fixture_name", INSTANCES, ids=[name for _, name in INSTANCES])
def test_encode_matches_committed_fixture(instance, fixture_name):
    encoded = instance.encode()
    assert len(encoded) == instance.WIRE_LEN, (
        f"{fixture_name}: encode() produced {len(encoded)} bytes, WIRE_LEN says {instance.WIRE_LEN}"
    )
    fixture = _fixture(fixture_name)
    assert encoded == fixture, f"{fixture_name}: Python encoding no longer matches testdata/{fixture_name}.bin"


@pytest.mark.parametrize("instance, fixture_name", INSTANCES, ids=[name for _, name in INSTANCES])
def test_fixture_decodes_back_to_the_same_value(instance, fixture_name):
    fixture = _fixture(fixture_name)
    decoded = type(instance).decode(fixture)
    assert _approx_equal(decoded, instance), (
        f"{fixture_name}: decoding testdata/{fixture_name}.bin didn't reproduce the instance"
    )


@pytest.mark.parametrize("instance, fixture_name", INSTANCES, ids=[name for _, name in INSTANCES])
def test_round_trip_through_python_alone(instance, fixture_name):
    """Independent of the fixture: encode/decode must be inverses in Python
    too, the same property `roundtrip.rs` checks on the Rust side."""
    decoded = type(instance).decode(instance.encode())
    assert _approx_equal(decoded, instance)


def test_frame_header_matches_committed_fixture():
    header = FrameHeader(type_id=0x0501, seq=0xBEEF)
    encoded = header.encode()
    assert len(encoded) == 4
    fixture = _fixture("FrameHeader")
    assert encoded == fixture
    assert FrameHeader.decode(fixture) == header


def test_every_fixture_file_is_covered():
    """Catches the failure mode where a new Rust type gets a fixture but
    nobody adds the matching Python case here."""
    covered = {name for _, name in INSTANCES} | {"FrameHeader"}
    on_disk = {p.stem for p in TESTDATA.glob("*.bin")}
    missing = on_disk - covered
    assert not missing, f"testdata/*.bin has fixtures with no Python golden test: {sorted(missing)}"


def test_wire_len_constants_are_self_consistent():
    """`WIRE_LEN` must equal the fixture's length for every type -- catches a
    hand-edited constant that no longer matches the struct format used to
    produce it."""
    for instance, fixture_name in INSTANCES:
        assert instance.WIRE_LEN == len(_fixture(fixture_name)), fixture_name
