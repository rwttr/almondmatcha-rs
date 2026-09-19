"""Unit tests for `rover_perception.bus`, mirroring the spirit of
`crates/rover-bus`'s own tests (its `config.rs`/`lib.rs` `#[cfg(test)]`
modules) -- routing/encoding logic against a fixture TOML, plus one real
loopback UDP round-trip so the wire bytes are checked end to end, not just
the routing table lookups.
"""

from __future__ import annotations

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
