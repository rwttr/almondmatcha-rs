"""Python side of the UDP bus. Mirrors `crates/rover-bus` (`src/lib.rs`,
`src/config.rs`) -- that crate is the authority this module is checked
against, not the other way around: if the two ever disagree, this module is
what needs fixing.

# What this reproduces from `rover-bus`

- `config/rover.toml`'s `[services]` (service name -> `"host:port"`),
  `[routes]` (message type `NAME` -> list of service names), and
  `[debug] mirror` (empty string = disabled) -- same three sections, same
  meaning, same "absent/empty means off" defaults as `rover_bus::config`.
- `publish(msg)`: encode with a **per-message-type** sequence counter (not
  one counter for the whole bus -- see `Bus::publish`'s doc comment on why:
  it is what lets a receiver see packet loss as gaps in one type's stream),
  then send one datagram to every service `[routes]` names for `msg.NAME`,
  plus one more to the mirror address if `[debug] mirror` is set.
- **A message type absent from `[routes]` is not an error.** It means "no
  destinations" -- `rover.toml`'s own comment on `[routes]` says so
  explicitly ("publishing it still costs nothing... unrouted type as 'no
  destinations', not an error"), and `rover_bus::BusConfig::route_for`
  encodes exactly that. This module matches it: `route_for` returns an
  empty list rather than raising, and the mirror still receives the
  message either way.

# What is deliberately different from the Rust side

- **No `PeerId` enum.** `rover-bus` validates every `[services]`/`[routes]`
  entry against a fixed `PeerId::ALL` (design defect D1,
  `docs/RUST_REWRITE_PLAN.md` sec 13.3b) because it also has to build a
  receive-side peer table (`rover_link::Link`) keyed by that enum. This
  module only ever *sends* -- it has no receive side and no fixed peer
  registry to validate against -- so `BusConfig.parse` instead validates
  each `[routes]` destination directly against `[services]` (unknown
  service name -> `ConfigError`, same failure mode, just checked against
  the config file itself rather than a hardcoded enum).
- **`publish()` never raises on a send failure**, route or mirror alike. The
  Rust side returns `Err` naming the first failed *route* destination (but
  never for a mirror failure -- see `Bus::publish`'s doc comment: a mirror
  is "a debugging convenience with a deliberately absent operator on the
  other end most of the time"). This module goes one step further and
  treats every send as best-effort, full stop: UDP already gives no
  delivery confirmation, this process has nothing useful to do with a
  `sendto` failure besides log it, and a `LaneMeasurement` producer racing
  ahead every ~33 ms should not stall or raise over one unreachable peer.
  Failures are logged, not silently swallowed.
"""

from __future__ import annotations

import logging
import os
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import tomllib
except ImportError:  # Python < 3.11 -- the Jetson runs 3.10.
    import tomli as tomllib

from .wire import encode_frame

logger = logging.getLogger(__name__)

# Overrides `find_config_path`'s walk-up-from-the-package search entirely.
# Set this to point at a specific file -- a bench rig's own config, or a
# test fixture -- rather than relying on directory layout.
ROVER_CONFIG_ENV = "ROVER_CONFIG"

Address = Tuple[str, int]


class ConfigError(ValueError):
    """Mirrors `rover_bus::config::ConfigError`: something is wrong with
    `rover.toml`'s bus-relevant sections (unparseable TOML, a malformed
    `host:port`, a route naming a service `[services]` never defined, ...).
    One exception type so callers can catch it regardless of which check
    failed, matching the Rust enum's role.
    """


def find_config_path(start: Optional[Path] = None) -> Path:
    """Locate `config/rover.toml` by walking up from `start` (default: this
    package's own directory) toward the filesystem root, the way the Rust
    tests locate it relative to `CARGO_MANIFEST_DIR` -- except there is no
    Cargo-equivalent workspace marker on the Python side to anchor on, so
    this walks up looking for a `config/rover.toml` at each level instead of
    assuming a fixed number of parent directories. `ROVER_CONFIG` in the
    environment bypasses the search completely, for a bench rig or test
    fixture that lives somewhere this walk would never find it.

    Deliberately does NOT hardcode an absolute path -- this file's location
    on disk (a git checkout, a Jetson's install path, a CI runner) is not
    something this module should need to know about.
    """
    override = os.environ.get(ROVER_CONFIG_ENV)
    if override:
        return Path(override)

    here = start or Path(__file__).resolve().parent
    for directory in (here, *here.parents):
        candidate = directory / "config" / "rover.toml"
        if candidate.is_file():
            return candidate

    raise ConfigError(
        f"could not find config/rover.toml by walking up from {here}; "
        f"set {ROVER_CONFIG_ENV} to point at it directly"
    )


def _parse_addr(addr_str: str, context: str) -> Address:
    """Parse a `"host:port"` string the way `rover_bus::config` parses a
    `SocketAddr` -- `rpartition` (not `split`) on the LAST colon, so an IPv6
    literal (which contains colons of its own) would still split correctly
    if one is ever used here; this project's addresses are all IPv4, but the
    parse has no reason to assume that."""
    host, sep, port_str = addr_str.rpartition(":")
    if not sep:
        raise ConfigError(f"{context}: {addr_str!r} is not a valid host:port address")
    try:
        port = int(port_str)
    except ValueError as exc:
        raise ConfigError(f"{context}: {addr_str!r} has a non-numeric port") from exc
    return host, port


@dataclass(frozen=True)
class BusConfig:
    """Parsed, resolved `[services]`/`[routes]`/`[debug]` sections. Mirrors
    `rover_bus::BusConfig`'s role: everything else in `rover.toml`
    (`[drivetrain]`, `[control]`, ...) is simply never looked at, so a
    change to one of those sections can never break parsing here -- same
    reasoning as `rover_bus::config`'s module doc comment.
    """

    services: Dict[str, Address]
    routes: Dict[str, List[str]]
    _mirror: Optional[Address]

    @classmethod
    def load(cls, path: Optional[Path] = None) -> "BusConfig":
        """Load and resolve `config/rover.toml` (or any file with the same
        shape). `path` defaults to `find_config_path()`."""
        path = Path(path) if path is not None else find_config_path()
        try:
            text = path.read_text()
        except OSError as exc:
            raise ConfigError(f"reading {path}: {exc}") from exc
        return cls.parse(text)

    @classmethod
    def parse(cls, toml_text: str) -> "BusConfig":
        """Parse from an already-read string -- split out from `load` so
        tests can exercise fixture configs without touching the filesystem,
        exactly like `rover_bus::BusConfig::parse`."""
        try:
            raw = tomllib.loads(toml_text)
        except tomllib.TOMLDecodeError as exc:
            raise ConfigError(f"parsing config: {exc}") from exc

        services_raw: Dict[str, str] = raw.get("services", {})
        services: Dict[str, Address] = {
            name: _parse_addr(addr, f"service `{name}`") for name, addr in services_raw.items()
        }

        routes_raw: Dict[str, List[str]] = raw.get("routes", {})
        routes: Dict[str, List[str]] = {}
        for msg_name, hosts in routes_raw.items():
            for host in hosts:
                if host not in services:
                    raise ConfigError(
                        f"route for `{msg_name}` names unknown service `{host}` "
                        f"(not present in [services])"
                    )
            routes[msg_name] = list(hosts)

        # Empty (the field default, and what a bare [debug] section with no
        # `mirror` key also produces) means disabled -- same reasoning as
        # rover_bus::config: a typo'd mirror address silently disabling
        # debugging is worse than a startup failure, so anything non-empty
        # must parse as a real address.
        debug_raw: Dict[str, str] = raw.get("debug", {})
        mirror_str = debug_raw.get("mirror", "").strip()
        mirror = _parse_addr(mirror_str, "[debug] mirror") if mirror_str else None

        return cls(services, routes, mirror)

    def addr_of(self, service: str) -> Optional[Address]:
        """The resolved address of a service, if `[services]` named it."""
        return self.services.get(service)

    def route_for(self, type_name: str) -> List[str]:
        """Destinations configured for a message type, by its `Wire.NAME`.

        An empty list -- never an error -- for a type with no `[routes]`
        entry. See this module's docstring: a partial config, or a message
        type nothing currently subscribes to, is a normal thing to publish
        into, not a broken one.
        """
        return list(self.routes.get(type_name, []))

    def mirror(self) -> Optional[Address]:
        """The debug firehose mirror address (`[debug] mirror`), if
        configured. `Bus.publish` sends an extra, best-effort copy of every
        frame here on top of its normal routes."""
        return self._mirror


class Bus:
    """Publish side of the UDP bus for one process. Mirrors
    `rover_bus::Bus::publish`'s fan-out semantics -- see this module's
    docstring for the one deliberate difference (never raises on a send
    failure).

    General-purpose despite this process only ever publishing
    `LaneMeasurement` (routed to `control` in `config/rover.toml`): nothing
    about the wire format or the routing table is perception-specific, so
    keeping this generic over any `Wire`-shaped message (duck-typed:
    anything with `.TYPE_ID`, `.NAME`, and `.encode()`, matching
    `wire.py`'s dataclasses) costs nothing and means a second Python
    process (a bench tool, a replay script) never has to fork this class to
    publish a different type.
    """

    def __init__(self, config: BusConfig, sock: Optional[socket.socket] = None) -> None:
        self._config = config
        # `sock` is injectable for tests that want to control the local
        # bind address; production code (main.py) always takes the default,
        # an ordinary unbound UDP socket -- fine for sending, since `sendto`
        # picks an ephemeral local port on first use.
        self._sock = sock if sock is not None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Per-TYPE_ID sequence counter -- see this module's docstring on why
        # per-type, not one counter for the whole bus.
        self._send_seq: Dict[int, int] = {}

    def publish(self, msg) -> None:
        """Encode `msg` and send it to every service `[routes]` names for
        `msg.NAME`, plus the mirror if one is configured. Never raises on a
        send failure -- see this module's docstring.
        """
        seq = self._send_seq.get(msg.TYPE_ID, 0)
        self._send_seq[msg.TYPE_ID] = (seq + 1) & 0xFFFF  # seq is u16 on the wire
        frame = encode_frame(msg, seq)

        for service in self._config.route_for(msg.NAME):
            addr = self._config.addr_of(service)
            if addr is None:
                # Cannot happen for a config BusConfig.parse accepted --
                # parse() already checked every route destination exists in
                # [services] -- guarded anyway rather than trust that
                # invariant silently forever if this ever gets a
                # hand-built BusConfig.
                logger.warning(
                    "route for %s names unconfigured service %r; dropping", msg.NAME, service
                )
                continue
            self._send(frame, addr, f"route to {service} ({addr[0]}:{addr[1]})")

        mirror = self._config.mirror()
        if mirror is not None:
            self._send(frame, mirror, f"mirror ({mirror[0]}:{mirror[1]})")

    def _send(self, frame: bytes, addr: Address, description: str) -> None:
        try:
            self._sock.sendto(frame, addr)
        except OSError as exc:
            # Best-effort by design -- see this module's docstring. A
            # laptop that has been closed, or a board that hasn't booted
            # yet, must not be allowed to affect the rest of this process.
            logger.debug("send to %s failed: %s", description, exc)

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> "Bus":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()
