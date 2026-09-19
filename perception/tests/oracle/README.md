# The parity oracle

`lane_detector.py` and `config.py` in this directory are **verbatim, frozen
copies** of the ROS 2 originals:

- `ws_jetson/src/vision_navigation/vision_navigation/lane_detector.py`
- `ws_jetson/src/vision_navigation/vision_navigation/config.py`

taken at commit `b82caf3` of this repository, on branch `rs`, shortly before
`ws_jetson/` was deleted.

## Why they exist

`../test_lane_parity.py` proves that `rover_perception/lane.py` (the Rust
rewrite's Python port) computes bit-for-bit the same lane geometry as the
ROS 2 code it replaces, over the same synthetic frames. That test needs a
copy of the original algorithm to compare against, and `ws_jetson/` is being
removed from this repository — so this directory is that copy, kept alive
solely to stand as the answer key.

## Rules

- **They are never imported by production code.** Nothing under
  `rover_perception/` may import from `tests/oracle/`. If it ever needs to,
  something has gone wrong with the boundary between "port" and "oracle".
- **They must never be "fixed."** If a parity test fails and the discrepancy
  traces back to something that looks wrong in these files — a threshold
  that seems off, an operation ordering that seems suspicious, anything —
  that is a **finding about the port** (or a pre-existing bug in the ROS 2
  system worth knowing about), not a bug to patch here. Patching the oracle
  to make a test pass defeats the entire point of having one: it stops being
  independent evidence and becomes a mirror of whatever `lane.py` already
  does.
- The only edit ever made to these files, relative to the ROS 2 originals,
  is the import line in `lane_detector.py`:
  `from vision_navigation.config import LaneDetectionConfig` was changed to
  `from .config import LaneDetectionConfig` so the file imports as part of
  this package instead of the ROS 2 `vision_navigation` package, which no
  longer exists in this repository. No algorithmic line was touched.
  `config.py` was copied with no edits at all — it has no imports of its
  own to fix.
- If these files ever need to change for a reason other than the above,
  that is itself worth a second look: the whole reason they are here is to
  hold still while everything else moves.
