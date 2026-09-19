"""Camera capture, replacing ROS 2 `camera_stream_node.py`.

# What changed and why

The ROS 2 node's whole job was: grab a frame, wrap it in a `sensor_msgs/Image`
via `CvBridge`, and publish it on a QoS-`BEST_EFFORT` topic for
`lane_detection_node` to subscribe to. See `rover_perception/__init__.py` and
`lane.py`'s module docstring for why that hop is gone: camera and detector are
now one loop iteration of one process (`main.py`), so this module's only job
is `read() -> Optional[np.ndarray]` (BGR) — a plain blocking call, no
publisher, no timer callback, no `rclpy`.

# Two backends, one interface

- A RealSense D415, via `pyrealsense2`.
- A video file, via `cv2.VideoCapture` — used for bench testing, replay, and
  as the D415's fallback (see below).

`pyrealsense2` has no wheel for every platform this code is developed and
tested on (notably macOS — see `perception/pyproject.toml`), so it is
imported lazily behind a `try/except ImportError` at module scope. Nothing in
this module (or in the test suite) may import it unconditionally: doing so
would make the whole perception package unimportable on a dev machine that
will never see a D415.

# Depth: deliberately not ported

`camera_stream_node.py` could also stream and publish a depth frame
(`ENABLE_DEPTH_STREAM`, an `rs.align` to the color stream, a second
publisher). Nothing downstream of this module consumes depth — the lane
detector takes BGR only (`lane.py`'s `process_frame`) — so depth support is
dropped entirely rather than carried over as unused surface area: an
`enable_depth` flag that nothing reads would be exactly the kind of
"plausible-looking but dead" config knob this rewrite is trying to get rid
of. If a future consumer needs depth, add it back deliberately, with that
consumer in mind, rather than resurrecting it unused.

# The D415 -> video fallback is real field behaviour

If the D415 fails to initialise (unplugged, USB bandwidth issue, firmware
hiccup) and a `fallback_video` path is configured, `Camera` logs a warning
and falls back to playing that file instead of failing the whole process.
This is not a test convenience — it is what lets a field session continue
(in a degraded, non-live-camera mode useful for tuning the rest of the
pipeline) instead of the perception process refusing to start at all because
one USB cable was loose.
"""

from __future__ import annotations

import dataclasses
import logging
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# See the module docstring: this must never be a hard import. `rs is None`
# is the whole test suite's signal to skip the D415 backend rather than
# fail to import the package at all.
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


class CameraError(RuntimeError):
    """Camera (or fallback) failed to initialise. Distinct from a plain
    `RuntimeError` so `main.py` can catch exactly this and exit non-zero
    with a clear message, rather than a bare `Exception` from deep inside
    `cv2`/`pyrealsense2` leaking out as a stack trace with no context."""


@dataclass
class CameraConfig:
    """Everything needed to open either backend. One dataclass for both
    (rather than two config types plus a mode enum) because the fallback
    path (D415 -> video) means a single `Camera` construction can legitimately
    need both a D415 serial *and* a video path at once."""

    width: int = 1280
    height: int = 720
    # `None` means "derive it" -- from the video file's own metadata in video
    # mode, or the D415 default of 30 in camera mode. An explicit value
    # overrides either. Kept as one field (not a separate "override_fps")
    # because "unset" and "use the source's own rate" are the same concept in
    # both backends.
    fps: Optional[int] = None

    # Video-file mode. Empty string (the default) means "use the D415
    # instead" -- same sentinel convention `CameraConfig.VIDEO_PATH` used in
    # the ROS 2 config.
    video_path: str = ""
    loop_video: bool = True

    # D415 mode.
    serial: str = ""  # empty = let pyrealsense2 grab the first device it finds
    json_config_path: str = ""  # optional RealSense advanced-mode JSON

    # If the D415 fails to open, fall back to playing this file instead of
    # raising. Empty (the default) means "no fallback -- fail loudly."
    fallback_video: str = ""


class Camera:
    """A blocking BGR frame source, backed by either a RealSense D415 or a
    video file. `read()` returns `None` on end-of-stream (non-looping video)
    or a frame-grab timeout (D415) rather than raising -- both are routine,
    expected conditions in a capture loop, not failures worth an exception's
    control-flow cost.
    """

    def __init__(self, config: CameraConfig) -> None:
        self._config = config
        self._backend: "_Backend" = self._open(config)

    @staticmethod
    def _open(config: CameraConfig) -> "_Backend":
        if config.video_path:
            return _VideoBackend(config)

        try:
            return _RealSenseBackend(config)
        except Exception as exc:
            if not config.fallback_video:
                raise
            # Real field behaviour, not a test convenience -- see the module
            # docstring. Deliberately catches every exception the D415 path
            # can raise (missing pyrealsense2, no device attached, a bad
            # advanced-mode JSON, ...): any of them means "no live camera
            # right now," and the fallback's whole point is to keep the
            # process alive through all of them.
            logger.warning(
                "D415 initialisation failed (%s); falling back to video file %r",
                exc,
                config.fallback_video,
            )
            fallback_config = dataclasses.replace(config, video_path=config.fallback_video)
            return _VideoBackend(fallback_config)

    def read(self) -> Optional[np.ndarray]:
        """The next BGR frame, or `None` if none is available right now
        (see class docstring)."""
        return self._backend.read()

    @property
    def fps(self) -> float:
        """The rate this capture source is actually running at -- the
        configured override if one was given, otherwise whatever the
        backend measured or defaulted to. Distinct from `main.py`'s rolling
        achieved-FPS counter, which measures how fast frames are actually
        being *processed*, not how fast the source claims to produce them."""
        return self._backend.fps

    def close(self) -> None:
        self._backend.close()

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()


# ===========================================================================
# Backends
# ===========================================================================
#
# Not a formal ABC -- there are exactly two implementations, both private to
# this module, and `Camera` is the only caller. A `typing.Protocol` would add
# a name with no behaviour of its own; this comment is the interface
# documentation instead: `read() -> Optional[np.ndarray]`, `fps: float`,
# `close() -> None`.


class _VideoBackend:
    """`cv2.VideoCapture` over a file on disk."""

    def __init__(self, config: CameraConfig) -> None:
        self._cap = cv2.VideoCapture(config.video_path)
        if not self._cap.isOpened():
            raise CameraError(f"could not open video file: {config.video_path!r}")

        self._loop = config.loop_video
        self._width = config.width
        self._height = config.height

        if config.fps is not None:
            self.fps = float(config.fps)
        else:
            file_fps = self._cap.get(cv2.CAP_PROP_FPS)
            # A file with a missing/corrupt header, or certain raw formats,
            # reports 0 (or occasionally a nonsense negative value) here --
            # fall back to a sane default rather than propagate a bogus rate
            # into the rolling-FPS-vs-configured-rate comparison in main.py.
            self.fps = file_fps if file_fps > 1e-3 else 30.0

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self._cap.read()
        if not ok:
            if not self._loop:
                return None
            # Rewind to frame 0 and try once more. A file that fails to
            # produce a frame even right after rewinding (zero-length, or
            # corrupt) is reported as end-of-stream rather than retried in a
            # loop that could spin forever.
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._cap.read()
            if not ok:
                return None

        if frame.shape[1] != self._width or frame.shape[0] != self._height:
            frame = cv2.resize(frame, (self._width, self._height))
        return frame

    def close(self) -> None:
        self._cap.release()


class _RealSenseBackend:
    """RealSense D415 color stream, via `pyrealsense2`."""

    def __init__(self, config: CameraConfig) -> None:
        if rs is None:
            raise CameraError(
                "pyrealsense2 is not installed -- install it on the Jetson "
                "with `pip install pyrealsense2` (see perception/pyproject.toml)"
            )

        self._pipeline = rs.pipeline()
        rs_config = rs.config()

        if config.serial:
            rs_config.enable_device(config.serial)
        else:
            # Not an error -- a bench with exactly one D415 attached works
            # fine this way -- but worth a loud warning, since silently
            # grabbing "whichever device is first" is exactly the kind of
            # thing that bites when a second RealSense shows up on the bus.
            logger.warning(
                "no D415 serial configured; grabbing the first RealSense "
                "device pyrealsense2 finds"
            )

        fps = config.fps if config.fps is not None else 30
        rs_config.enable_stream(
            rs.stream.color, config.width, config.height, rs.format.bgr8, fps
        )

        profile = self._pipeline.start(rs_config)

        if config.json_config_path:
            self._load_advanced_mode_json(profile, rs_config, config.json_config_path)

        self.fps = float(fps)

    def _load_advanced_mode_json(self, profile, rs_config, json_path: str) -> None:
        """Push a RealSense advanced-mode JSON (exposure/gain/etc. presets)
        to the device, enabling advanced mode first if it isn't already.

        UNVERIFIED ON HARDWARE (see docs/RUST_REWRITE_PLAN.md sec 13.4's
        verification-debt list for the pattern this project uses for that
        caveat): toggling advanced mode resets the device, which is why this
        stops and restarts the pipeline around the toggle rather than
        assuming the pre-toggle `profile`/device handle stays valid -- the
        same sequence as Intel's own `set_advanced_mode.py` example. The 2 s
        sleep is that example's settle time, not a measured minimum.
        """
        device = profile.get_device()
        adv = rs.rs400_advanced_mode(device)

        if not adv.is_enabled():
            self._pipeline.stop()
            adv.toggle_advanced_mode(True)
            time.sleep(2.0)
            profile = self._pipeline.start(rs_config)
            adv = rs.rs400_advanced_mode(profile.get_device())

        with open(json_path, "r") as f:
            adv.load_json(f.read())

    def read(self) -> Optional[np.ndarray]:
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=100)
        except RuntimeError:
            # librealsense raises (not returns a sentinel) on a frame-grab
            # timeout. A single missed frame is routine at the USB/sensor
            # level and must not crash the capture loop -- treated exactly
            # like the non-looping video backend's end-of-stream `None`.
            return None

        color = frames.get_color_frame()
        if not color:
            return None
        # `.copy()`, not a bare `np.asanyarray(...)`: `get_data()` exposes
        # the frame's own internal buffer through the Python buffer
        # protocol, valid only while `color`/`frames` stay alive. Both go
        # out of scope (and their buffer can be reused by librealsense's
        # frame pool) the moment this function returns, so the caller must
        # not be handed a view onto memory this function no longer holds a
        # reference to.
        return np.asanyarray(color.get_data()).copy()

    def close(self) -> None:
        self._pipeline.stop()
