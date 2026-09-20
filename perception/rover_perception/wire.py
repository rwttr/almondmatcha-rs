"""Wire types mirrored from `crates/rover-msgs/src/{types.rs,frame.rs}`.

# Contract

This module is the Python half of the only thing preventing the Jetson from
silently drifting out of sync with the Rust rover: the wire format. It must
byte-for-byte agree with `rover-msgs`, verified against the same
`testdata/*.bin` fixtures both languages read (see
`tests/test_wire_golden.py`).

- Fixed-width little-endian, no padding, fields in declaration order —
  identical rule to `rover_msgs::Wire`.
- A message never carries `NaN`/`inf` on the wire. `LaneMeasurement.valid`
  is how "no detection this frame" is expressed; the float fields are
  always finite (typically zeroed), because a stale consumer reading only
  the floats and ignoring `valid` must not see garbage. This mirrors a bug
  the ROS 2 system actually had: `clamp(NaN, -100, 100)` on some platforms
  collapses to the *lower* bound, so a lost frame could silently become a
  full-scale steering command. See `lane.py`.
- Booleans are one byte; any nonzero byte decodes as `True`, matching
  `rover_msgs::codec::Reader::bool`.

# BREAKING PROTOCOL CHANGE

If you change a layout here, you are changing the wire protocol. That is
only ever done in lockstep with `crates/rover-msgs/src/types.rs` and a
`UPDATE_GOLDEN=1 cargo test -p rover-msgs --test golden` regeneration of
`testdata/*.bin` — see the module docs on `crates/rover-msgs/tests/golden.rs`
for why. A change here with no matching Rust change (or vice versa) is a
rover that can no longer talk to itself.

Only the types the Jetson perception process actually touches are given
`encode`; the rest are decode-only (or omitted), since this process never
constructs them. All are included for `test_wire_golden.py`'s benefit: full
coverage is what makes that test worth trusting.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import ClassVar, Optional, Tuple


class DecodeError(ValueError):
    """Mirrors `rover_msgs::DecodeError`. Raised for a short buffer or an
    invalid enum discriminant, so callers can catch one exception type
    regardless of which check failed."""


def _check_len(buf: bytes, need: int, name: str) -> None:
    if len(buf) < need:
        raise DecodeError(f"{name}: buffer too short: need {need} bytes, got {len(buf)}")


# ===========================================================================
# Framing (frame.rs)
# ===========================================================================

FRAME_HEADER_LEN = 4
MAX_BODY_LEN = 256
MAX_FRAME_LEN = FRAME_HEADER_LEN + MAX_BODY_LEN

_FRAME_HEADER_STRUCT = struct.Struct("<HH")


@dataclass(frozen=True)
class FrameHeader:
    type_id: int
    seq: int

    def encode(self) -> bytes:
        return _FRAME_HEADER_STRUCT.pack(self.type_id, self.seq)

    @classmethod
    def decode(cls, buf: bytes) -> "FrameHeader":
        _check_len(buf, FRAME_HEADER_LEN, "FrameHeader")
        type_id, seq = _FRAME_HEADER_STRUCT.unpack_from(buf, 0)
        return cls(type_id, seq)


def encode_frame(msg, seq: int) -> bytes:
    """Header + body, matching `rover_msgs::frame::encode_frame`."""
    return FrameHeader(msg.TYPE_ID, seq).encode() + msg.encode()


# ===========================================================================
# Enums
# ===========================================================================

class FixQuality(IntEnum):
    NONE = 0
    AUTONOMOUS = 1
    DGPS = 2
    RTK_FLOAT = 3
    RTK_FIXED = 4

    def is_rtk(self) -> bool:
        return self in (FixQuality.RTK_FLOAT, FixQuality.RTK_FIXED)


class MissionState(IntEnum):
    IDLE = 0
    ARMED = 1
    RUNNING = 2
    ARRIVED = 3
    CANCELLED = 4
    FAULT = 5


class GnssSource(IntEnum):
    """Which physical receiver a `GnssFix` came from. Mirrors
    `rover_msgs::GnssSource` -- see its doc comment for why this field
    exists (design defect D2, docs/RUST_REWRITE_PLAN.md sec 13.3b): a single
    `GnssFix` type serves both the u-blox and the Spresense, so a subscriber
    off the wire alone used to have no way to tell them apart."""

    RTK = 0
    BACKUP = 1


# ===========================================================================
# 0x01xx -- chassis board -> RPi
# ===========================================================================

_IMU_SAMPLE_STRUCT = struct.Struct("<6fI")


@dataclass(frozen=True)
class ImuSample:
    TYPE_ID: ClassVar[int] = 0x0101
    WIRE_LEN: ClassVar[int] = 28
    NAME: ClassVar[str] = "ImuSample"

    accel_mps2: Tuple[float, float, float]
    gyro_radps: Tuple[float, float, float]
    t_us: int

    def encode(self) -> bytes:
        return _IMU_SAMPLE_STRUCT.pack(*self.accel_mps2, *self.gyro_radps, self.t_us)

    @classmethod
    def decode(cls, buf: bytes) -> "ImuSample":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        *vals, t_us = _IMU_SAMPLE_STRUCT.unpack_from(buf, 0)
        return cls(tuple(vals[0:3]), tuple(vals[3:6]), t_us)


_MAG_SAMPLE_STRUCT = struct.Struct("<3fI")


@dataclass(frozen=True)
class MagSample:
    TYPE_ID: ClassVar[int] = 0x0102
    WIRE_LEN: ClassVar[int] = 16
    NAME: ClassVar[str] = "MagSample"

    field_gauss: Tuple[float, float, float]
    t_us: int

    def encode(self) -> bytes:
        return _MAG_SAMPLE_STRUCT.pack(*self.field_gauss, self.t_us)

    @classmethod
    def decode(cls, buf: bytes) -> "MagSample":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        x, y, z, t_us = _MAG_SAMPLE_STRUCT.unpack_from(buf, 0)
        return cls((x, y, z), t_us)


# seq_echo(u16) watchdog_tripped(bool, 1 byte) fault(u8) t_us(u32)
_CHASSIS_STATUS_STRUCT = struct.Struct("<H?BI")


@dataclass(frozen=True)
class ChassisStatus:
    TYPE_ID: ClassVar[int] = 0x0103
    WIRE_LEN: ClassVar[int] = 8
    NAME: ClassVar[str] = "ChassisStatus"

    seq_echo: int
    watchdog_tripped: bool
    fault: int  # FaultBits, carried as a plain u8 bitmask
    t_us: int

    def encode(self) -> bytes:
        return _CHASSIS_STATUS_STRUCT.pack(self.seq_echo, self.watchdog_tripped, self.fault, self.t_us)

    @classmethod
    def decode(cls, buf: bytes) -> "ChassisStatus":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        seq_echo, tripped, fault, t_us = _CHASSIS_STATUS_STRUCT.unpack_from(buf, 0)
        return cls(seq_echo, tripped, fault, t_us)


# ===========================================================================
# 0x02xx -- RPi -> chassis board
# ===========================================================================

_CHASSIS_COMMAND_STRUCT = struct.Struct("<ffH")


@dataclass(frozen=True)
class ChassisCommand:
    TYPE_ID: ClassVar[int] = 0x0201
    WIRE_LEN: ClassVar[int] = 10
    NAME: ClassVar[str] = "ChassisCommand"

    steer: float
    throttle: float
    seq: int

    def encode(self) -> bytes:
        return _CHASSIS_COMMAND_STRUCT.pack(self.steer, self.throttle, self.seq)

    @classmethod
    def decode(cls, buf: bytes) -> "ChassisCommand":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        steer, throttle, seq = _CHASSIS_COMMAND_STRUCT.unpack_from(buf, 0)
        return cls(steer, throttle, seq)


# ===========================================================================
# 0x03xx -- sensors board -> RPi
# ===========================================================================

_WHEEL_SENSORS_STRUCT = struct.Struct("<iiI")


@dataclass(frozen=True)
class WheelSensors:
    TYPE_ID: ClassVar[int] = 0x0301
    WIRE_LEN: ClassVar[int] = 12
    NAME: ClassVar[str] = "WheelSensors"

    ticks_left: int
    ticks_right: int
    t_us: int

    def encode(self) -> bytes:
        return _WHEEL_SENSORS_STRUCT.pack(self.ticks_left, self.ticks_right, self.t_us)

    @classmethod
    def decode(cls, buf: bytes) -> "WheelSensors":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        tl, tr, t_us = _WHEEL_SENSORS_STRUCT.unpack_from(buf, 0)
        return cls(tl, tr, t_us)


_POWER_SAMPLE_STRUCT = struct.Struct("<ff")


@dataclass(frozen=True)
class PowerSample:
    TYPE_ID: ClassVar[int] = 0x0302
    WIRE_LEN: ClassVar[int] = 8
    NAME: ClassVar[str] = "PowerSample"

    bus_volts: float
    current_amps: float

    def encode(self) -> bytes:
        return _POWER_SAMPLE_STRUCT.pack(self.bus_volts, self.current_amps)

    @classmethod
    def decode(cls, buf: bytes) -> "PowerSample":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        v, a = _POWER_SAMPLE_STRUCT.unpack_from(buf, 0)
        return cls(v, a)


# ===========================================================================
# 0x04xx -- GNSS
# ===========================================================================

_GNSS_FIX_STRUCT = struct.Struct("<ddfBBfffQB")


@dataclass(frozen=True)
class GnssFix:
    TYPE_ID: ClassVar[int] = 0x0401
    WIRE_LEN: ClassVar[int] = 43
    NAME: ClassVar[str] = "GnssFix"

    lat_deg: float
    lon_deg: float
    alt_m: float
    fix: FixQuality
    sats: int
    h_acc_m: float
    speed_mps: float
    course_deg: float
    utc_ms: int
    source: GnssSource

    def encode(self) -> bytes:
        return _GNSS_FIX_STRUCT.pack(
            self.lat_deg, self.lon_deg, self.alt_m, int(self.fix), self.sats,
            self.h_acc_m, self.speed_mps, self.course_deg, self.utc_ms, int(self.source),
        )

    @classmethod
    def decode(cls, buf: bytes) -> "GnssFix":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        lat, lon, alt, fix_raw, sats, h_acc, speed, course, utc_ms, source_raw = _GNSS_FIX_STRUCT.unpack_from(buf, 0)
        try:
            fix = FixQuality(fix_raw)
        except ValueError:
            raise DecodeError(f"GnssFix: invalid discriminant {fix_raw} for field `FixQuality`")
        try:
            source = GnssSource(source_raw)
        except ValueError:
            raise DecodeError(f"GnssFix: invalid discriminant {source_raw} for field `GnssSource`")
        return cls(lat, lon, alt, fix, sats, h_acc, speed, course, utc_ms, source)


# ===========================================================================
# 0x05xx -- perception (the type this process actually emits)
# ===========================================================================

_LANE_MEASUREMENT_STRUCT = struct.Struct("<fff?I")


@dataclass(frozen=True)
class LaneMeasurement:
    """Lane geometry from the Jetson. See `rover_msgs::LaneMeasurement` for
    the field semantics (lookahead point, sign convention). `curvature_inv_m`
    is converted to 1/metres by `lane.py` before this is constructed --
    never at encode time -- so this class is a dumb mirror of the wire, not
    where the physics lives."""

    TYPE_ID: ClassVar[int] = 0x0501
    WIRE_LEN: ClassVar[int] = 17
    NAME: ClassVar[str] = "LaneMeasurement"

    curvature_inv_m: float
    heading_err_rad: float
    cross_track_m: float
    valid: bool
    t_us: int

    def encode(self) -> bytes:
        return _LANE_MEASUREMENT_STRUCT.pack(
            self.curvature_inv_m, self.heading_err_rad, self.cross_track_m, self.valid, self.t_us,
        )

    @classmethod
    def decode(cls, buf: bytes) -> "LaneMeasurement":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        curvature, heading, cross_track, valid, t_us = _LANE_MEASUREMENT_STRUCT.unpack_from(buf, 0)
        return cls(curvature, heading, cross_track, valid, t_us)


# ===========================================================================
# 0x06xx -- estimation and guidance
# ===========================================================================

EKF_STATES = 5

_ROVER_STATE_STRUCT = struct.Struct("<5f5fH")


@dataclass(frozen=True)
class RoverState:
    TYPE_ID: ClassVar[int] = 0x0601
    WIRE_LEN: ClassVar[int] = 42
    NAME: ClassVar[str] = "RoverState"

    cross_track_m: float
    heading_err_rad: float
    curvature_inv_m: float
    speed_mps: float
    gyro_bias_radps: float
    p_diag: Tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)
    lane_age_ms: int = 0

    def encode(self) -> bytes:
        return _ROVER_STATE_STRUCT.pack(
            self.cross_track_m, self.heading_err_rad, self.curvature_inv_m,
            self.speed_mps, self.gyro_bias_radps, *self.p_diag, self.lane_age_ms,
        )

    @classmethod
    def decode(cls, buf: bytes) -> "RoverState":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        vals = _ROVER_STATE_STRUCT.unpack_from(buf, 0)
        cross_track, heading, curvature, speed, gyro_bias = vals[0:5]
        p_diag = tuple(vals[5:10])
        lane_age_ms = vals[10]
        return cls(cross_track, heading, curvature, speed, gyro_bias, p_diag, lane_age_ms)


_MOTION_SETPOINT_STRUCT = struct.Struct("<ff")


@dataclass(frozen=True)
class MotionSetpoint:
    TYPE_ID: ClassVar[int] = 0x0602
    WIRE_LEN: ClassVar[int] = 8
    NAME: ClassVar[str] = "MotionSetpoint"

    steer_rad: float
    speed_mps: float

    def encode(self) -> bytes:
        return _MOTION_SETPOINT_STRUCT.pack(self.steer_rad, self.speed_mps)

    @classmethod
    def decode(cls, buf: bytes) -> "MotionSetpoint":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        steer, speed = _MOTION_SETPOINT_STRUCT.unpack_from(buf, 0)
        return cls(steer, speed)


# ===========================================================================
# 0x07xx -- mission
# ===========================================================================

_MISSION_GOAL_STRUCT = struct.Struct("<dd")


@dataclass(frozen=True)
class MissionGoal:
    TYPE_ID: ClassVar[int] = 0x0702
    WIRE_LEN: ClassVar[int] = 16
    NAME: ClassVar[str] = "MissionGoal"

    lat_deg: float
    lon_deg: float

    def encode(self) -> bytes:
        return _MISSION_GOAL_STRUCT.pack(self.lat_deg, self.lon_deg)

    @classmethod
    def decode(cls, buf: bytes) -> "MissionGoal":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        lat, lon = _MISSION_GOAL_STRUCT.unpack_from(buf, 0)
        return cls(lat, lon)


_MISSION_STATUS_STRUCT = struct.Struct("<?f?ddB")


@dataclass(frozen=True)
class MissionStatus:
    TYPE_ID: ClassVar[int] = 0x0701
    WIRE_LEN: ClassVar[int] = 23
    NAME: ClassVar[str] = "MissionStatus"

    active: bool
    distance_remaining_m: float
    target: Optional[MissionGoal]
    state: MissionState

    def encode(self) -> bytes:
        goal = self.target if self.target is not None else MissionGoal(0.0, 0.0)
        return _MISSION_STATUS_STRUCT.pack(
            self.active, self.distance_remaining_m, self.target is not None,
            goal.lat_deg, goal.lon_deg, int(self.state),
        )

    @classmethod
    def decode(cls, buf: bytes) -> "MissionStatus":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        active, dist, has_target, lat, lon, state_raw = _MISSION_STATUS_STRUCT.unpack_from(buf, 0)
        try:
            state = MissionState(state_raw)
        except ValueError:
            raise DecodeError(f"MissionStatus: invalid discriminant {state_raw} for field `MissionState`")
        target = MissionGoal(lat, lon) if has_target else None
        return cls(active, dist, target, state)


# ===========================================================================
# 0x08xx -- base station link
# ===========================================================================

@dataclass(frozen=True)
class Command:
    """Tagged union mirroring `rover_msgs::Command`. The wire always writes
    the full 16-byte payload area regardless of variant (see
    `CommandFrame.encode`), so decode always reads every field; callers key
    off `tag`, exactly like matching on the Rust enum."""

    NOP: ClassVar[int] = 0
    SET_SPEED_LIMIT: ClassVar[int] = 1
    SET_MISSION_GOAL: ClassVar[int] = 2
    CANCEL_MISSION: ClassVar[int] = 3
    ESTOP: ClassVar[int] = 4
    CLEAR_ESTOP: ClassVar[int] = 5

    tag: int
    speed_limit_pct: int = 0
    mission_goal: Optional[MissionGoal] = None

    @classmethod
    def nop(cls) -> "Command":
        return cls(cls.NOP)

    @classmethod
    def set_speed_limit(cls, pct: int) -> "Command":
        return cls(cls.SET_SPEED_LIMIT, speed_limit_pct=pct)

    @classmethod
    def set_mission_goal(cls, goal: MissionGoal) -> "Command":
        return cls(cls.SET_MISSION_GOAL, mission_goal=goal)

    @classmethod
    def cancel_mission(cls) -> "Command":
        return cls(cls.CANCEL_MISSION)

    @classmethod
    def estop(cls) -> "Command":
        return cls(cls.ESTOP)

    @classmethod
    def clear_estop(cls) -> "Command":
        return cls(cls.CLEAR_ESTOP)


@dataclass(frozen=True)
class CommandFrame:
    TYPE_ID: ClassVar[int] = 0x0802
    PAYLOAD_LEN: ClassVar[int] = MissionGoal.WIRE_LEN
    WIRE_LEN: ClassVar[int] = 3 + MissionGoal.WIRE_LEN
    NAME: ClassVar[str] = "CommandFrame"

    cmd_seq: int
    body: Command

    def encode(self) -> bytes:
        buf = bytearray(self.WIRE_LEN)
        struct.pack_into("<H", buf, 0, self.cmd_seq)
        buf[2] = self.body.tag
        if self.body.tag == Command.SET_SPEED_LIMIT:
            buf[3] = self.body.speed_limit_pct
        elif self.body.tag == Command.SET_MISSION_GOAL:
            struct.pack_into("<dd", buf, 3, self.body.mission_goal.lat_deg, self.body.mission_goal.lon_deg)
        # Nop / CancelMission / EStop: payload area stays zeroed.
        return bytes(buf)

    @classmethod
    def decode(cls, buf: bytes) -> "CommandFrame":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        cmd_seq, tag = struct.unpack_from("<HB", buf, 0)
        if tag == Command.NOP:
            body = Command.nop()
        elif tag == Command.SET_SPEED_LIMIT:
            body = Command.set_speed_limit(buf[3])
        elif tag == Command.SET_MISSION_GOAL:
            lat, lon = struct.unpack_from("<dd", buf, 3)
            body = Command.set_mission_goal(MissionGoal(lat, lon))
        elif tag == Command.CANCEL_MISSION:
            body = Command.cancel_mission()
        elif tag == Command.ESTOP:
            body = Command.estop()
        elif tag == Command.CLEAR_ESTOP:
            body = Command.clear_estop()
        else:
            raise DecodeError(f"CommandFrame: invalid discriminant {tag} for field `Command`")
        return cls(cmd_seq, body)


@dataclass(frozen=True)
class Telemetry:
    """Composed exactly like `rover_msgs::Telemetry`: encode/decode delegate
    to the member types' own `encode`/`decode` rather than re-flattening
    the fields, so this can never drift from what those types do on their
    own."""

    TYPE_ID: ClassVar[int] = 0x0801
    WIRE_LEN: ClassVar[int] = (
        4 + 8 + RoverState.WIRE_LEN + MissionStatus.WIRE_LEN + PowerSample.WIRE_LEN
        + GnssFix.WIRE_LEN * 2 + 2 + 2
    )
    NAME: ClassVar[str] = "Telemetry"

    seq: int
    t_us: int
    state: RoverState
    mission: MissionStatus
    power: PowerSample
    rtk: GnssFix
    backup: GnssFix
    last_cmd_seq: int
    health: int  # HealthBits, carried as a plain u16 bitmask

    def encode(self) -> bytes:
        head = struct.pack("<IQ", self.seq, self.t_us)
        tail = struct.pack("<HH", self.last_cmd_seq, self.health)
        return b"".join([
            head, self.state.encode(), self.mission.encode(), self.power.encode(),
            self.rtk.encode(), self.backup.encode(), tail,
        ])

    @classmethod
    def decode(cls, buf: bytes) -> "Telemetry":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        seq, t_us = struct.unpack_from("<IQ", buf, 0)
        n = 12
        state = RoverState.decode(buf[n:])
        n += RoverState.WIRE_LEN
        mission = MissionStatus.decode(buf[n:])
        n += MissionStatus.WIRE_LEN
        power = PowerSample.decode(buf[n:])
        n += PowerSample.WIRE_LEN
        rtk = GnssFix.decode(buf[n:])
        n += GnssFix.WIRE_LEN
        backup = GnssFix.decode(buf[n:])
        n += GnssFix.WIRE_LEN
        last_cmd_seq, health = struct.unpack_from("<HH", buf, n)
        return cls(seq, t_us, state, mission, power, rtk, backup, last_cmd_seq, health)


# ===========================================================================
# 0x09xx -- debug
# ===========================================================================

_SPEED_LOOP_DEBUG_STRUCT = struct.Struct("<5f")


@dataclass(frozen=True)
class SpeedLoopDebug:
    TYPE_ID: ClassVar[int] = 0x0901
    WIRE_LEN: ClassVar[int] = 20
    NAME: ClassVar[str] = "SpeedLoopDebug"

    measured_left_tps: float
    measured_right_tps: float
    target_tps: float
    error_pct: float
    pid_output_pct: float

    def encode(self) -> bytes:
        return _SPEED_LOOP_DEBUG_STRUCT.pack(
            self.measured_left_tps, self.measured_right_tps, self.target_tps,
            self.error_pct, self.pid_output_pct,
        )

    @classmethod
    def decode(cls, buf: bytes) -> "SpeedLoopDebug":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        vals = _SPEED_LOOP_DEBUG_STRUCT.unpack_from(buf, 0)
        return cls(*vals)


_EKF_DEBUG_STRUCT = struct.Struct("<3ff?")


@dataclass(frozen=True)
class EkfDebug:
    TYPE_ID: ClassVar[int] = 0x0902
    WIRE_LEN: ClassVar[int] = 17
    NAME: ClassVar[str] = "EkfDebug"

    innovation: Tuple[float, float, float]
    nis: float
    gated: bool

    def encode(self) -> bytes:
        return _EKF_DEBUG_STRUCT.pack(*self.innovation, self.nis, self.gated)

    @classmethod
    def decode(cls, buf: bytes) -> "EkfDebug":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        x, y, z, nis, gated = _EKF_DEBUG_STRUCT.unpack_from(buf, 0)
        return cls((x, y, z), nis, gated)


# ===========================================================================
# 0x0903 -- board self-diagnostics
# ===========================================================================


class BoardId(IntEnum):
    """Mirrors `rover_msgs::BoardId`. One message type serves both boards --
    see that type's doc comment for why."""

    CHASSIS = 0
    SENSORS = 1

    def label(self) -> str:
        return {BoardId.CHASSIS: "chassis", BoardId.SENSORS: "sensors"}[self]


class ResetCause(IntEnum):
    """Mirrors `rover_msgs::ResetCause`. Read once at boot from `RCC_CSR` and
    latched for the run -- see that type's doc comment on why this is the
    single most diagnostic byte either board produces."""

    UNKNOWN = 0
    POWER_ON = 1
    PIN = 2
    SOFTWARE = 3
    INDEPENDENT_WATCHDOG = 4
    WINDOW_WATCHDOG = 5
    LOW_POWER = 6
    BROWN_OUT = 7

    def is_abnormal(self) -> bool:
        return self in (
            ResetCause.INDEPENDENT_WATCHDOG,
            ResetCause.WINDOW_WATCHDOG,
            ResetCause.BROWN_OUT,
        )


# board(u8) post_run(u16) post_pass(u16) reset_cause(u8) phy_id(u32)
# link_speed_mbps(u8) link_full_duplex(bool,1 byte) phy_symbol_errors(u16)
# uptime_s(u32) tx_drops(u16)
_BOARD_DIAGNOSTICS_STRUCT = struct.Struct("<BHHBIB?HIH")


@dataclass(frozen=True)
class BoardDiagnostics:
    """Mirrors `rover_msgs::BoardDiagnostics`. `post_run`/`post_pass` are
    carried as plain `u16` bitmasks (see `PostBits`) -- their bit meanings
    are board-specific above bit 3, so decoding them into names belongs to
    whichever consumer already knows `board`, not to this dumb wire mirror."""

    TYPE_ID: ClassVar[int] = 0x0903
    WIRE_LEN: ClassVar[int] = 20
    NAME: ClassVar[str] = "BoardDiagnostics"

    board: BoardId
    post_run: int
    post_pass: int
    reset_cause: ResetCause
    phy_id: int
    link_speed_mbps: int
    link_full_duplex: bool
    phy_symbol_errors: int
    uptime_s: int
    tx_drops: int

    def post_failures(self) -> int:
        return self.post_run & ~self.post_pass

    def post_ok(self) -> bool:
        return self.post_failures() == 0

    def encode(self) -> bytes:
        return _BOARD_DIAGNOSTICS_STRUCT.pack(
            int(self.board), self.post_run, self.post_pass, int(self.reset_cause),
            self.phy_id, self.link_speed_mbps, self.link_full_duplex,
            self.phy_symbol_errors, self.uptime_s, self.tx_drops,
        )

    @classmethod
    def decode(cls, buf: bytes) -> "BoardDiagnostics":
        _check_len(buf, cls.WIRE_LEN, cls.NAME)
        (board_raw, post_run, post_pass, reset_raw, phy_id, link_speed_mbps,
         link_full_duplex, phy_symbol_errors, uptime_s, tx_drops) = \
            _BOARD_DIAGNOSTICS_STRUCT.unpack_from(buf, 0)
        try:
            board = BoardId(board_raw)
        except ValueError:
            raise DecodeError(f"BoardDiagnostics: invalid discriminant {board_raw} for field `BoardId`")
        try:
            reset_cause = ResetCause(reset_raw)
        except ValueError:
            raise DecodeError(f"BoardDiagnostics: invalid discriminant {reset_raw} for field `ResetCause`")
        return cls(
            board, post_run, post_pass, reset_cause, phy_id, link_speed_mbps,
            link_full_duplex, phy_symbol_errors, uptime_s, tx_drops,
        )
