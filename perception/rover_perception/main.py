"""Process entry point for the Jetson perception process.

`pyproject.toml` declares `rover-perception = "rover_perception.main:main"`
as the installed console script -- this is what runs on the Jetson.

# The loop

Read a frame -> `LaneDetector.detect` -> build a `LaneMeasurement` ->
`Bus.publish`. That is the entire control-relevant body of this process;
everything else in this module (CSV logging, the preview window, the
rolling-FPS counter, signal handling) is instrumentation and shutdown
plumbing around it.

# Carried over from `lane_detection_node.py`

- **Rolling-window achieved FPS** over the last 30 processed frames
  (`_RollingFps`): `(len(times) - 1) / (times[-1] - times[0])`, `0.0` until
  a second sample exists. This measures how fast frames are actually being
  *processed* -- which can differ from the camera's configured capture rate
  under load -- not how fast the source claims to produce them (that is
  `Camera.fps`). Keeping the two separate was the point of the original
  code and remains the point here.
- **Async CSV logging on a background thread** (`_CsvLogger`): a
  `queue.Queue`, a sentinel of `None` to stop, `thread.join(timeout=2.0)` on
  shutdown. The original blocked its image callback on disk I/O and
  dropped frames while writing; the whole reason this thread exists is so a
  slow filesystem never stalls the capture/detect loop. Off by default,
  behind `--csv` -- matching the D4 decision in
  `docs/RUST_REWRITE_PLAN.md` sec 13.3b for the (analogous) base-station
  CSV: the shipped default matches the pre-logging baseline, and the
  capability is there the moment a field problem needs diagnosing. The log
  file (and its directory) are created lazily, on the first row actually
  written, so a `--csv` run that never processes a frame leaves nothing on
  disk.
- **Optional preview window** (`--preview`): `cv2.imshow` plus a crosshair
  and one status line, ESC or `q` to quit. Pure debug convenience, kept
  thin on purpose.
- **Per-frame terminal log** -- but throttled by default, opted out of with
  `--verbose`. The original logged every frame at ~30 FPS unconditionally,
  which is noise, not signal, in a field run's terminal; `--verbose` is for
  someone who actually wants that firehose while debugging.

# Explicitly NOT carried over

- **The stale-frame age check** (`MAX_FRAME_AGE_SEC` in the ROS 2
  `LaneDetectionConfig`). It existed because `camera_stream_node` and
  `lane_detection_node` were two separate ROS 2 processes joined by a
  queued DDS topic, so a frame already in flight when something hiccuped
  could go stale before the detector acted on it. Camera capture and
  detection are now one loop iteration of one process (see `lane.py`'s
  module docstring) -- there is no queue, and therefore nothing for a frame
  to go stale *in*. A `None` from `camera.read()` here means either "no
  frame is available yet" (a D415 grab timeout -- try again) or "this
  source is exhausted" (a non-looping video's end) -- see `_MAX_CONSECUTIVE_NONE`
  below for how this loop tells those apart, which is a different problem
  from staleness and not a relaxation of it.
- `cv_bridge`, `sensor_msgs/Image`, QoS profiles, `rclpy` -- there is no ROS
  here at all; see `rover_perception/__init__.py`.
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import signal
import sys
import threading
import time
from collections import deque
from pathlib import Path
from queue import Queue
from typing import Deque, Optional, Sequence

import cv2

from .bus import Bus, BusConfig, ConfigError
from .camera import Camera, CameraConfig, CameraError
from .lane import LaneDetector, LaneResult
from .wire import LaneMeasurement

logger = logging.getLogger("rover_perception.main")

_FPS_WINDOW = 30

# How many consecutive `camera.read() -> None` reads this loop tolerates
# before treating the source as exhausted rather than momentarily busy.
# `Camera.read()` returns `None` for two very different reasons that look
# identical from here -- a routine D415 grab timeout (single frame, retry
# is correct) and a non-looping video file's end-of-stream (permanent,
# retrying forever would spin this loop at 100% CPU doing nothing). ~150
# reads is a few seconds' worth of retries at typical camera rates -- long
# enough that no plausible transient hiccup trips it, short enough that a
# genuinely dead source is noticed quickly rather than hanging the process.
_MAX_CONSECUTIVE_NONE = 150

_CSV_COLUMNS = ("timestamp", "curvature", "theta", "b", "detected", "fps")

# Sentinel telling the CSV writer thread to stop. `None` specifically (not
# an arbitrary object()) per the porting brief -- there is nothing else that
# would ever legitimately arrive on this queue for it to be confused with,
# since every real row is a tuple.
_CSV_STOP = None


class _RollingFps:
    """Achieved-throughput FPS over the last `window` processed frames. See
    this module's docstring for why this is deliberately not the same
    number as `Camera.fps`.
    """

    def __init__(self, window: int = _FPS_WINDOW) -> None:
        self._times: Deque[float] = deque(maxlen=window)

    def tick(self, now: Optional[float] = None) -> float:
        """Record one more processed frame at `now` (default:
        `time.monotonic()`) and return the current rolling FPS -- `0.0`
        until at least two samples are in the window, matching the
        original's guard against dividing by a single-sample time span of
        zero.
        """
        self._times.append(now if now is not None else time.monotonic())
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / (self._times[-1] - self._times[0])


class _CsvLogger:
    """Background-thread CSV writer. See this module's docstring for why it
    exists: the capture/detect loop must never block on disk I/O.
    """

    def __init__(self, path: Path) -> None:
        self._path = path
        self._queue: "Queue" = Queue()
        self._thread = threading.Thread(
            target=self._run, name="rover-perception-csv", daemon=True
        )
        self._thread.start()

    def log(
        self,
        timestamp: float,
        curvature_inv_m: float,
        theta_deg: float,
        b_m: float,
        detected: bool,
        fps: float,
    ) -> None:
        """Enqueue one row. Non-blocking -- `Queue.put` with no bound never
        waits, which is the entire point: a slow writer thread must fall
        behind in its own queue, not in this call."""
        self._queue.put((timestamp, curvature_inv_m, theta_deg, b_m, detected, fps))

    def _run(self) -> None:
        file = None
        writer = None
        try:
            while True:
                item = self._queue.get()
                if item is _CSV_STOP:
                    break
                if writer is None:
                    # Created on the first real row, not in __init__ -- see
                    # this module's docstring: a run that never processes a
                    # frame must leave no file (or directory) behind.
                    self._path.parent.mkdir(parents=True, exist_ok=True)
                    file = open(self._path, "w", newline="")
                    writer = csv.writer(file)
                    writer.writerow(_CSV_COLUMNS)
                writer.writerow(item)
                file.flush()
        finally:
            if file is not None:
                file.close()

    def stop(self) -> None:
        """Signal the writer thread to drain its queue and exit, and wait
        up to 2 s for it. A logger that never got a single row (writer/file
        still `None`) exits almost instantly; this bound exists for the
        pathological case of a huge backlog on a very slow disk, so
        shutdown is never unbounded."""
        self._queue.put(_CSV_STOP)
        self._thread.join(timeout=2.0)


class _ShutdownFlag:
    """`bool`-like flag a signal handler can set from inside the interpreter
    signal-check machinery, and the main loop polls once per iteration.
    Simpler than threading.Event for this single-writer/single-reader,
    same-process use, and avoids the loop blocking on `Event.wait()`
    anywhere.
    """

    def __init__(self) -> None:
        self._stop = False

    def request(self, signum: int, _frame: object) -> None:
        logger.info("received signal %d; shutting down", signum)
        self._stop = True

    def __bool__(self) -> bool:
        return self._stop


def _draw_preview(frame, result: LaneResult, achieved_fps: float) -> bool:
    """Render one debug frame with a centre crosshair and a status line.
    Returns `False` when the operator asked to quit (ESC or `q`), `True`
    otherwise. Pure `cv2.imshow` debug output -- no numeric effect on
    anything published."""
    vis = frame.copy()
    height, width = vis.shape[:2]
    cx, cy = width // 2, height // 2
    color = (0, 200, 0) if result.valid else (0, 0, 220)  # BGR: green=valid, red=lost

    cv2.drawMarker(vis, (cx, cy), color, markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)
    status = (
        f"valid={result.valid} "
        f"curv={result.curvature_inv_m:+.4f}/m "
        f"theta={math.degrees(result.heading_err_rad):+.1f}deg "
        f"b={result.cross_track_m:+.3f}m "
        f"fps={achieved_fps:.1f}"
    )
    cv2.putText(vis, status, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    cv2.imshow("rover-perception", vis)

    key = cv2.waitKey(1) & 0xFF
    return key not in (27, ord("q"))  # 27 == ESC


def _camera_config_from_args(args: argparse.Namespace) -> CameraConfig:
    return CameraConfig(
        width=args.width,
        height=args.height,
        fps=args.fps,
        video_path=args.video or "",
        loop_video=not args.no_loop,
        serial=args.serial or "",
        json_config_path=args.realsense_json or "",
        fallback_video=args.fallback_video or "",
    )


def run(args: argparse.Namespace) -> int:
    """Initialise everything, run the capture/detect/publish loop until
    shutdown is requested (or the source is exhausted), and clean up.
    Returns a process exit code: 0 on a clean shutdown, non-zero on a fatal
    initialisation failure (bad config, camera that never opens).
    """
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    try:
        config_path = Path(args.config) if args.config else None
        bus_config = BusConfig.load(config_path)
    except ConfigError as exc:
        logger.error("failed to load bus config: %s", exc)
        return 1

    try:
        camera = Camera(_camera_config_from_args(args))
    except CameraError as exc:
        logger.error("failed to open camera: %s", exc)
        return 1

    bus = Bus(bus_config)
    detector = LaneDetector()
    fps_counter = _RollingFps()
    csv_logger = _CsvLogger(Path(args.csv)) if args.csv else None

    shutdown = _ShutdownFlag()
    # Registered here, not at import time -- run() is what owns the camera
    # and bus this handler needs the loop below to notice and unwind past.
    signal.signal(signal.SIGINT, shutdown.request)
    signal.signal(signal.SIGTERM, shutdown.request)

    frame_count = 0
    consecutive_none = 0
    last_heartbeat = 0.0

    try:
        while not shutdown:
            frame = camera.read()
            if frame is None:
                consecutive_none += 1
                if consecutive_none >= _MAX_CONSECUTIVE_NONE:
                    logger.error(
                        "no frame from camera for %d consecutive reads; "
                        "stopping (end of a non-looping video, or the camera "
                        "went away)",
                        consecutive_none,
                    )
                    break
                # A short sleep, not a busy spin, while waiting on a source
                # that has nothing for us right now (a D415 grab timeout, or
                # one bad video frame) -- see _MAX_CONSECUTIVE_NONE's
                # comment on why this can't just be "retry immediately"
                # forever.
                time.sleep(0.01)
                continue
            consecutive_none = 0

            result = detector.detect(frame)

            # Free-running, wrapping u32 microsecond counter local to this
            # publisher -- see wire.py's LaneMeasurement and the porting
            # brief: consumers only ever take deltas within one publisher's
            # own stream, so the epoch is arbitrary, but it must monotonic
            # and must WRAP (not reset to 0) when it overflows, which is
            # exactly what `& 0xFFFFFFFF` on an ever-increasing counter
            # does.
            t_us = time.monotonic_ns() // 1000 & 0xFFFFFFFF
            msg = LaneMeasurement(
                curvature_inv_m=result.curvature_inv_m,
                heading_err_rad=result.heading_err_rad,
                cross_track_m=result.cross_track_m,
                valid=result.valid,
                t_us=t_us,
            )
            bus.publish(msg)

            achieved_fps = fps_counter.tick()
            frame_count += 1

            if csv_logger is not None:
                csv_logger.log(
                    time.time(),
                    result.curvature_inv_m,
                    math.degrees(result.heading_err_rad),
                    result.cross_track_m,
                    result.valid,
                    achieved_fps,
                )

            now = time.monotonic()
            if args.verbose:
                # Opted into the firehose explicitly -- log every frame.
                logger.debug(
                    "frame %d valid=%s curvature=%.5f theta=%.2fdeg b=%.3fm fps=%.1f",
                    frame_count,
                    result.valid,
                    result.curvature_inv_m,
                    math.degrees(result.heading_err_rad),
                    result.cross_track_m,
                    achieved_fps,
                )
            elif now - last_heartbeat >= 2.0:
                # Default: a throttled heartbeat so a field run's terminal
                # still shows life, without the original's 30 Hz noise.
                logger.info(
                    "frame %d valid=%s fps=%.1f", frame_count, result.valid, achieved_fps
                )
                last_heartbeat = now

            if args.preview:
                if not _draw_preview(frame, result, achieved_fps):
                    logger.info("preview window closed by operator")
                    break
    finally:
        if args.preview:
            cv2.destroyAllWindows()
        if csv_logger is not None:
            csv_logger.stop()
        camera.close()
        bus.close()

    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rover-perception",
        description=(
            "Jetson perception process: camera capture + lane detection, "
            "one process, no ROS."
        ),
    )
    source = parser.add_argument_group("camera source")
    source.add_argument(
        "--video", metavar="PATH", help="Read frames from a video file instead of a D415 camera."
    )
    source.add_argument(
        "--serial",
        metavar="SERIAL",
        help="RealSense D415 serial number (grabs the first device found if omitted).",
    )
    source.add_argument(
        "--fallback-video",
        metavar="PATH",
        help="Video file to fall back to if the D415 fails to initialise.",
    )
    source.add_argument(
        "--realsense-json",
        metavar="PATH",
        help="RealSense advanced-mode JSON to load on the D415.",
    )
    source.add_argument("--width", type=int, default=1280, help="Capture width (default: 1280).")
    source.add_argument("--height", type=int, default=720, help="Capture height (default: 720).")
    source.add_argument(
        "--fps",
        type=int,
        default=None,
        help="Override the capture FPS (default: derive from the source).",
    )
    source.add_argument(
        "--no-loop",
        action="store_true",
        help="Do not loop a video file at end-of-stream (default: loop).",
    )

    parser.add_argument(
        "--config",
        metavar="PATH",
        help=(
            "Path to rover.toml (default: found by walking up from the "
            "package, or $ROVER_CONFIG)."
        ),
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show a debug preview window (cv2.imshow). ESC or q quits.",
    )
    parser.add_argument(
        "--csv",
        metavar="PATH",
        default=None,
        help=(
            "Log every processed frame to this CSV file. Off by default -- "
            "see docs/RUST_REWRITE_PLAN.md sec 13.3b (D4)."
        ),
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Log every processed frame instead of a throttled heartbeat.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
