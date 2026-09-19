"""Unit tests for `rover_perception.bus`, mirroring the spirit of
`crates/rover-bus`'s own tests (its `config.rs`/`lib.rs` `#[cfg(test)]`
modules) -- routing/encoding logic against a fixture TOML, plus one real
loopback UDP round-trip so the wire bytes are checked end to end, not just
the routing table lookups.
"""

from __future__ import annotations

import contextlib
import logging
import socket

import pytest

from rover_perception.bus import Bus, BusConfig, ConfigError, find_config_path
from rover_perception.wire import FrameHeader, LaneMeasurement, encode_frame

SAMPLE = """
[services]
control = "127.0.0.1:17001"
base    = "127.0.0.1:17030"

[routes]
LaneMeasurement = ["control"]

# A section that belongs to another crate/process entirely -- must not
# break parsing here, same check as rover-bus's own SAMPLE fixture.
[drivetrain]
wheel_diameter_m = 0.125
"""


def _lane_measurement(t_us: int = 1) -> LaneMeasurement:
    # Exactly representable in float32 (powers of two / simple binary
    # fractions) so the byte-exact assertions below don't have to account
    # for f64->f32 round-off on the wire -- see wire.py's WIRE_LEN=17 pack
    # ("<fff?I"), which stores these as f32.
    return LaneMeasurement(
        curvature_inv_m=0.5, heading_err_rad=-0.25, cross_track_m=0.125, valid=True, t_us=t_us
    )


def _bound_udp_socket() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(1.0)
    return sock


# ===========================================================================
# Config parsing / routing (no sockets)
# ===========================================================================


def test_resolves_services_to_addresses():
    cfg = BusConfig.parse(SAMPLE)
    assert cfg.addr_of("control") == ("127.0.0.1", 17001)
    assert cfg.addr_of("base") == ("127.0.0.1", 17030)
    assert cfg.addr_of("nonexistent") is None


def test_resolves_routes_by_wire_name():
    cfg = BusConfig.parse(SAMPLE)
    assert cfg.route_for("LaneMeasurement") == ["control"]


def test_unrouted_type_resolves_to_no_destinations():
    # GnssFix has no [routes] entry in SAMPLE -- must be "no destinations",
    # not an error. See rover_bus::BusConfig::route_for's doc comment and
    # this module's docstring.
    cfg = BusConfig.parse(SAMPLE)
    assert cfg.route_for("GnssFix") == []


def test_mirror_absent_section_is_none():
    cfg = BusConfig.parse(SAMPLE)
    assert cfg.mirror() is None


def test_mirror_empty_string_is_none():
    text = SAMPLE + '\n[debug]\nmirror = ""\n'
    assert BusConfig.parse(text).mirror() is None


def test_mirror_present_resolves_to_address():
    text = SAMPLE + '\n[debug]\nmirror = "127.0.0.1:17099"\n'
    assert BusConfig.parse(text).mirror() == ("127.0.0.1", 17099)


def test_mirror_malformed_address_is_a_hard_error():
    text = SAMPLE + '\n[debug]\nmirror = "not-an-address"\n'
    with pytest.raises(ConfigError):
        BusConfig.parse(text)


def test_unknown_service_in_routes_is_rejected():
    bad = SAMPLE.replace('LaneMeasurement = ["control"]', 'LaneMeasurement = ["groundstation"]')
    with pytest.raises(ConfigError):
        BusConfig.parse(bad)


def test_bad_socket_address_is_rejected():
    bad = SAMPLE.replace('"127.0.0.1:17001"', '"127.0.0.1"')  # missing port
    with pytest.raises(ConfigError):
        BusConfig.parse(bad)


def test_malformed_toml_is_a_config_error_not_a_raw_toml_exception():
    with pytest.raises(ConfigError):
        BusConfig.parse("this is not [valid toml")


# ===========================================================================
# find_config_path
# ===========================================================================


def test_find_config_path_env_override(tmp_path, monkeypatch):
    fixture = tmp_path / "somewhere-else.toml"
    fixture.write_text(SAMPLE)
    monkeypatch.setenv("ROVER_CONFIG", str(fixture))
    assert find_config_path() == fixture


def test_find_config_path_locates_the_real_repo_config(monkeypatch):
    monkeypatch.delenv("ROVER_CONFIG", raising=False)
    path = find_config_path()
    assert path.name == "rover.toml"
    assert path.parent.name == "config"
    assert path.is_file()


def test_loads_the_real_repo_config():
    # The actual file this process ships against. If this ever fails, the
    # config file and this parser have drifted apart -- same intent as
    # rover-bus's own `loads_the_real_repo_config` test.
    cfg = BusConfig.load(find_config_path())
    assert cfg.route_for("LaneMeasurement") == ["control"]
    assert cfg.addr_of("control") is not None
    # Shipped default is `mirror = ""` -- disabled.
    assert cfg.mirror() is None


# ===========================================================================
# Bus.publish -- real loopback UDP sockets
# ===========================================================================


def test_publish_sends_exact_wire_bytes_to_the_routed_destination():
    route_sock = _bound_udp_socket()
    port = route_sock.getsockname()[1]
    cfg = BusConfig.parse(
        f"""
        [services]
        control = "127.0.0.1:{port}"

        [routes]
        LaneMeasurement = ["control"]
        """
    )
    bus = Bus(cfg)
    msg = _lane_measurement()
    try:
        bus.publish(msg)
        data, _addr = route_sock.recvfrom(1024)
    finally:
        bus.close()
        route_sock.close()

    # Byte-exact against wire.py's own encode_frame -- this is the same
    # round-trip test_wire_golden.py already trusts, so a mismatch here
    # would point at Bus.publish's framing, not the wire format itself.
    assert data == encode_frame(msg, 0)
    header = FrameHeader.decode(data)
    assert header.type_id == LaneMeasurement.TYPE_ID
    assert header.seq == 0
    assert LaneMeasurement.decode(data[4:]) == msg


def test_publish_increments_a_per_type_sequence_counter():
    route_sock = _bound_udp_socket()
    port = route_sock.getsockname()[1]
    cfg = BusConfig.parse(
        f"""
        [services]
        control = "127.0.0.1:{port}"

        [routes]
        LaneMeasurement = ["control"]
        """
    )
    bus = Bus(cfg)
    try:
        bus.publish(_lane_measurement(1))
        bus.publish(_lane_measurement(2))
        seqs = [FrameHeader.decode(route_sock.recvfrom(1024)[0]).seq for _ in range(2)]
    finally:
        bus.close()
        route_sock.close()

    assert seqs == [0, 1]


def test_publish_reaches_both_the_route_and_the_mirror():
    route_sock = _bound_udp_socket()
    mirror_sock = _bound_udp_socket()
    cfg = BusConfig.parse(
        f"""
        [services]
        control = "127.0.0.1:{route_sock.getsockname()[1]}"

        [routes]
        LaneMeasurement = ["control"]

        [debug]
        mirror = "127.0.0.1:{mirror_sock.getsockname()[1]}"
        """
    )
    bus = Bus(cfg)
    try:
        bus.publish(_lane_measurement())
        assert route_sock.recvfrom(1024)[0]
        assert mirror_sock.recvfrom(1024)[0]
    finally:
        bus.close()
        route_sock.close()
        mirror_sock.close()


def test_publish_succeeds_even_when_the_mirror_is_unreachable():
    route_sock = _bound_udp_socket()

    # Bind and immediately drop a socket to get a port nothing is
    # listening on, standing in for "the debugging laptop isn't here" --
    # same trick rover-bus's own equivalent test uses.
    dead_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dead_sock.bind(("127.0.0.1", 0))
    dead_port = dead_sock.getsockname()[1]
    dead_sock.close()

    cfg = BusConfig.parse(
        f"""
        [services]
        control = "127.0.0.1:{route_sock.getsockname()[1]}"

        [routes]
        LaneMeasurement = ["control"]

        [debug]
        mirror = "127.0.0.1:{dead_port}"
        """
    )
    bus = Bus(cfg)
    try:
        bus.publish(_lane_measurement())  # must not raise
        assert route_sock.recvfrom(1024)[0], "the real route must still get the frame"
    finally:
        bus.close()
        route_sock.close()


def test_unrouted_type_still_reaches_the_mirror():
    mirror_sock = _bound_udp_socket()
    cfg = BusConfig.parse(
        f"""
        [services]
        control = "127.0.0.1:9"

        [debug]
        mirror = "127.0.0.1:{mirror_sock.getsockname()[1]}"
        """
    )
    # LaneMeasurement has no [routes] entry at all here -- must still reach
    # the mirror (see this module's docstring / rover.toml's own comment).
    bus = Bus(cfg)
    try:
        bus.publish(_lane_measurement())
        assert mirror_sock.recvfrom(1024)[0]
    finally:
        bus.close()
        mirror_sock.close()


# ===========================================================================
# Route failures are loud, mirror failures are not
#
# A mirror has a deliberately absent operator most of the time; a route
# carries this process's entire output. Logging both at DEBUG made a totally
# undelivered LaneMeasurement stream invisible at any normal log level, while
# the process went on reporting detected frames and a healthy FPS. See
# bus.py's module docstring.
# ===========================================================================


@contextlib.contextmanager
def _capture_logs():
    """Collect every record `rover_perception.bus` emits inside the block.

    A plain handler rather than pytest's `caplog` fixture so these tests say
    what they capture: the module sets no level of its own, and a root-logger
    configuration elsewhere in the suite must not be able to change what a
    test about log levels observes.
    """
    records = []

    class _Collector(logging.Handler):
        def emit(self, record):
            records.append(record)

    handler = _Collector()
    logger = logging.getLogger("rover_perception.bus")
    previous_level, previous_propagate = logger.level, logger.propagate
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    try:
        yield records
    finally:
        logger.removeHandler(handler)
        logger.setLevel(previous_level)
        logger.propagate = previous_propagate


class _FailingSocket:
    """Stands in for a socket whose `sendto` always fails, the way an
    unreachable peer or a downed interface presents (`ENETUNREACH`,
    `EHOSTUNREACH`). Raising from `sendto` is the real failure mode -- a UDP
    `sendto` to a black hole succeeds silently, so this models the errors
    that *do* surface, which are exactly the ones worth reporting."""

    def __init__(self):
        self.attempts = 0

    def sendto(self, _frame, _addr):
        self.attempts += 1
        raise OSError(51, "Network is unreachable")

    def close(self):
        pass


def test_route_send_failure_warns():
    cfg = BusConfig.parse(
        """
        [services]
        control = "192.0.2.1:7001"

        [routes]
        LaneMeasurement = ["control"]
        """
    )
    sock = _FailingSocket()
    bus = Bus(cfg, sock)
    with _capture_logs() as records:
        bus.publish(_lane_measurement())  # must still not raise

    assert sock.attempts == 1
    warnings = [r for r in records if r.levelno >= logging.WARNING]
    assert len(warnings) == 1, f"expected exactly one WARNING, got {records}"
    assert "control" in warnings[0].getMessage()


def test_mirror_send_failure_does_not_warn():
    cfg = BusConfig.parse(
        """
        [services]
        control = "192.0.2.1:7001"

        [debug]
        mirror = "192.0.2.2:7099"
        """
    )
    bus = Bus(cfg, _FailingSocket())
    with _capture_logs() as records:
        bus.publish(_lane_measurement())

    assert not [r for r in records if r.levelno >= logging.WARNING], (
        "a mirror with nobody listening is the normal case, not a warning"
    )


def test_repeated_route_failures_are_throttled():
    """At ~30 FPS an unreachable peer would otherwise emit 30 WARNINGs a
    second and bury every other message -- which hides the failure just as
    effectively as logging it at DEBUG did."""
    cfg = BusConfig.parse(
        """
        [services]
        control = "192.0.2.1:7001"

        [routes]
        LaneMeasurement = ["control"]
        """
    )
    bus = Bus(cfg, _FailingSocket())
    with _capture_logs() as records:
        for _ in range(100):
            bus.publish(_lane_measurement())

    warnings = [r for r in records if r.levelno >= logging.WARNING]
    assert len(warnings) == 1, f"100 failed sends should warn once, got {len(warnings)}"


def test_route_recovery_is_reported():
    """The operator who saw the failure warning needs to be told it cleared;
    otherwise the only way to know is that the warnings stopped, which is
    indistinguishable from the process having died."""
    route_sock = _bound_udp_socket()
    cfg = BusConfig.parse(
        f"""
        [services]
        control = "127.0.0.1:{route_sock.getsockname()[1]}"

        [routes]
        LaneMeasurement = ["control"]
        """
    )
    failing = _FailingSocket()
    bus = Bus(cfg, failing)
    try:
        with _capture_logs() as records:
            bus.publish(_lane_measurement())          # fails -> warns
            bus._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            bus.publish(_lane_measurement())          # succeeds -> recovery

        messages = [r.getMessage() for r in records if r.levelno >= logging.WARNING]
        assert len(messages) == 2, messages
        assert "failed" in messages[0]
        assert "recovered" in messages[1]
        assert "1 datagram lost" in messages[1], messages[1]
        assert route_sock.recvfrom(1024)[0], "the recovered send must actually arrive"
    finally:
        bus.close()
        route_sock.close()
