"""Recovery behaviour of `rover_perception.main.run` and the lane.py
finiteness guard -- exercised with fake camera/detector/bus, no hardware."""

from __future__ import annotations

import argparse
import signal
import time

import numpy as np
import pytest

from rover_perception import main as pmain
from rover_perception.camera import CameraError, CapturedFrame
from rover_perception.lane import LaneConfig, LaneDetector, LaneResult
from rover_perception.wire import LaneMeasurement


@pytest.fixture(autouse=True)
def _restore_signals():
    old = (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))
    yield
    signal.signal(signal.SIGINT, old[0])
    signal.signal(signal.SIGTERM, old[1])


class FakeBus:
    published: list = []

    def __init__(self, *a, **k):
        pass

    def publish(self, msg):
        FakeBus.published.append(msg)

    def close(self):
        pass


def _args():
    return pmain.build_arg_parser().parse_args(["--config", "x"])


def _patch(monkeypatch, camera_factory, detector):
    FakeBus.published = []
    monkeypatch.setattr(pmain, "Bus", FakeBus)
    monkeypatch.setattr(pmain.BusConfig, "load", classmethod(lambda cls, p=None: object()))
    monkeypatch.setattr(pmain, "Camera", camera_factory)
    monkeypatch.setattr(pmain, "LaneDetector", lambda: detector)
    monkeypatch.setattr(pmain, "_MAX_CONSECUTIVE_NONE", 3)
    monkeypatch.setattr(pmain, "_REOPEN_BACKOFF_S", (0.0, 0.0))
    monkeypatch.setattr(pmain.time, "sleep", lambda s: None)


def _frame(age_ns=0, dropped=0):
    """A factory for a fresh `CapturedFrame` stamped `age_ns` in the past."""
    return lambda: CapturedFrame(
        np.zeros((4, 4, 3), np.uint8), time.monotonic_ns() - age_ns, dropped
    )


class Detector:
    def __init__(self, fail_first=0):
        self.calls = 0
        self.fail_first = fail_first

    def detect(self, frame):
        self.calls += 1
        if self.calls <= self.fail_first:
            raise RuntimeError("boom")
        return LaneResult(0.0, 0.0, 0.0, True)

    def reset(self):
        pass


def test_frame_exception_is_logged_and_loop_continues_then_camera_loss_exits_nonzero(monkeypatch):
    frame = _frame()
    opens = {"n": 0}

    class Cam:
        def __init__(self, cfg):
            opens["n"] += 1
            if opens["n"] > 1:
                raise CameraError("gone")  # every reopen fails
            self.left = 3

        def read(self):
            if self.left:
                self.left -= 1
                return frame()
            return None

        def close(self):
            pass

    det = Detector(fail_first=1)
    _patch(monkeypatch, Cam, det)
    assert pmain.run(_args()) == 1
    assert det.calls == 3                       # exception did not stop the loop
    assert len(FakeBus.published) == 2          # frames 2 and 3 published
    assert opens["n"] == 1 + 2                  # initial + both backoff attempts


def test_camera_reopen_success_resumes_publishing(monkeypatch):
    frame = _frame()
    cams = []

    class Cam:
        def __init__(self, cfg):
            self.id = len(cams)
            cams.append(self)
            self.left = 2

        def read(self):
            if self.left:
                self.left -= 1
                return frame()
            if self.id >= 1:
                # second camera: ask the loop to stop after its frames
                import os
                os.kill(os.getpid(), signal.SIGINT)
            return None

        def close(self):
            pass

    _patch(monkeypatch, Cam, Detector())
    assert pmain.run(_args()) == 0
    assert len(cams) == 2
    assert len(FakeBus.published) == 4


def test_non_finite_or_huge_geometry_becomes_valid_false(monkeypatch):
    det = LaneDetector()
    for bad in (
        (float("nan"), 1.0, 0.1, True),
        (1.0, float("inf"), 0.1, True),
        (1e300, 1.0, 0.1, True),   # finite double, but overflows float32 on the wire
        (1.0, 1.0, float("nan"), True),
    ):
        monkeypatch.setattr("rover_perception.lane.process_frame", lambda f, search_center=None, _b=bad: _b)
        res = det.detect(np.zeros((4, 4, 3), np.uint8))
        assert res.valid is False
        # Must be packable: this is what used to raise OverflowError.
        LaneMeasurement(res.curvature_inv_m, res.heading_err_rad, res.cross_track_m, res.valid, 0, 0, 0).encode()
        assert det._search_center is None


def _one_shot_camera(frames):
    """A camera yielding `frames` (factories) in order, then SIGINT."""

    class Cam:
        def __init__(self, cfg):
            self.frames = list(frames)

        def read(self):
            if self.frames:
                return self.frames.pop(0)()
            import os
            os.kill(os.getpid(), signal.SIGINT)
            return None

        def close(self):
            pass

    return Cam


def test_t_us_is_capture_time_and_age_spans_capture_to_publish(monkeypatch):
    # Captured 80 ms before the loop sees it: the age must include that,
    # and t_us must be the capture instant, not publish (plan §13.3b D6).
    capture_ns = time.monotonic_ns() - 80_000_000
    frame = lambda: CapturedFrame(np.zeros((4, 4, 3), np.uint8), capture_ns, 0)
    _patch(monkeypatch, _one_shot_camera([frame]), Detector())
    assert pmain.run(_args()) == 0
    (msg,) = FakeBus.published
    assert msg.t_us == capture_ns // 1000 & 0xFFFFFFFF
    assert 80_000 <= msg.capture_age_us < 1_000_000


def test_frames_dropped_counts_skipped_frames_and_failed_detections(monkeypatch):
    frames = [_frame(dropped=2), _frame(dropped=1), _frame(dropped=0)]
    # The first detection raises: its frame (and the 2 skipped before it)
    # never reach the wire, so they are carried into the next message.
    _patch(monkeypatch, _one_shot_camera(frames), Detector(fail_first=1))
    assert pmain.run(_args()) == 0
    assert [m.frames_dropped for m in FakeBus.published] == [2 + 1 + 1, 0]
