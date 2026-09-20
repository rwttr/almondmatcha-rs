# Session handoff — 2026-09-20 (second session)

> **Ephemeral. Delete once the work below is done.** The durable record is
> `docs/STATUS_DONE.md` and `docs/STATUS_OPEN.md`. If something here still
> matters in a month, it belongs in one of those.
>
> The previous handoff was deleted for outliving its usefulness. Do not let
> this one do the same.

## ⚠️ First thing: nothing is committed

**53 paths are uncommitted**, on top of `747198e`. Working tree is green but
unsaved. Commit before anything else — a session crash loses a full day.

The changes split cleanly along these lines, which is the natural commit
boundary:

| Group | Roughly |
|---|---|
| Firmware diagnostics | `firmware/**` — POST, reset cause, PHY diagnostics, ANAR/strap logging |
| Host diagnostics | `crates/rover-telemetry`, `crates/rover-doctor` (new), `crates/rover-runs` (new), `crates/ground-station` |
| Wire contract | `crates/rover-msgs`, `testdata/BoardDiagnostics.bin`, `perception/wire.py`, `config/rover.toml` |
| D5 sign fix | `perception/rover_perception/lane.py`, `perception/tests/**`, `crates/rover-msgs` docs |
| Docs | `docs/**`, `README.md` |

The docs commit touches source comments in four crates, so keep it separate
from the feature work.

## State

Branch `rs`. **331 Rust tests, 88 Python tests, clippy clean, fmt clean, both
firmware images build, replay gate PASS** (rmse 1.349°, 267 rows).

Firmware: chassis 69,928 B flash / 18,480 B RAM; sensors 58,932 B / 18,648 B.
Of 2 MB flash and 512 KB SRAM.

## What landed this session

1. **Board self-diagnostics, both sides.** POST + reset cause + read-only
   LAN8742A PHY diagnostics in firmware; `BoardDiagnostics` on the wire;
   `rover-telemetry` logging `board_diagnostics.csv` and driving three latched
   health bits; `crates/rover-doctor` as a preflight GO/NO-GO.
2. **`crates/rover-runs`** — the `run_NNN_<stamp>/` convention, now shared by
   `rover-telemetry` and `ground-station` instead of copy-pasted.
3. **D5 fixed** (the heading sign). See below — the re-tune is not done.
4. **D6 recorded** (camera measurement timestamped after detection). Not fixed,
   deliberately.
5. **Docs split** into `STATUS_DONE.md` / `STATUS_OPEN.md`; plan §13 reduced to
   a pointer; `docs/FIELD_TEST.md` written; every doc compacted.
6. **`ANAR`/strap logging** on both boards — see "The PHY strap gap" below.

Two stale claims in `VISION_PIPELINE.md` were fixed in passing: a "Known
limits" bullet said §2 had never been validated against the D415, contradicting
§2's own dated validation record; and a cross-reference pointed at a
`fecfeb4` note in `CONTROL_LAW.md` that does not exist.

⚠️ **`docs/CONTROL_LAW.md`, `VISION_PIPELINE.md` and `CSV_LOGGING.md` are cited
by source code *by section number*** — `CONTROL_LAW.md` §2.1 from `actuate.rs`,
§1.7 from `lane.py`, `VISION_PIPELINE.md` "section 2" from
`perception/tests/oracle/config.py` (the **frozen** parity oracle, which must
not be edited). Do not renumber or rename a heading in those files without
grepping for citations first.

Three bugs were found and fixed along the way, all worth knowing about because
each is a class this codebase keeps producing:

- `BoardDiagnostics::WIRE_LEN` was `19`; the fields sum to `20`. `encode` wrote
  20 while `decode` demanded 19. Nothing exercised the two against each other
  until golden fixtures were added.
- `link_speed_mbps == 0` means "PHY read failed", not "link down" — the message
  arrives over the link it describes. Both the health latch and `rover-doctor`
  treated unknown as degraded, which would have false-alarmed on every boot.
- The handoff before this one gave the LAN8742A symbol-error counter as `0x1E`
  and "read-to-clear". Both wrong: it is `0x1A` and free-running. Checked
  against the datasheet (Rev 1.1, 05-21-13).

## Next, in order

1. **Commit.** See above.
2. **Calibrate the drivetrain** — `RUST_REWRITE_PLAN.md` §2.6, twenty minutes,
   no hardware risk. Expect **~15,000 counts for ten hand turns** (~1500
   ticks/rev at 4×); ~7,500 means the firmware is still decoding at 2×.
   `HARDWARE.md` §3 explains where that expectation comes from and why it is
   not in `rover.toml`.
3. **Flash both boards on the bench.** The decision point for the whole branch.
   Watch the `defmt` output for the PHY strap warning (below).
4. **Confirm servo direction on blocks**, then **re-tune for D5 and D6**.
5. **Record a real ROS 2 baseline on `main`** if acceptance criterion 2 is ever
   to close.

`docs/FIELD_TEST.md` is the end-to-end procedure once 2–4 are done.

## Open, and needing a decision

### D5 — sign fixed, gains are not

`LaneDetector.detect` now negates `theta`. `cross_track_m`, `steer`, both
consumers and both gains are unchanged — the model was right and the data was
wrong. `perception/tests/test_lane_sign_convention.py` is the regression test;
mutation-checked (revert the fix, 4 tests fail).

**The open half:** this is a **3.21× increase in straight-line steering
authority** (1.834 → 5.882 deg/deg), against gains never run with a correctly
signed heading term. `config/rover.toml` carries the arithmetic and
`[0.312×, 1.0×]` as the bracket to search. Start slow. Do not pick a number
from a desk — that is what the bracket is for.

### The PHY strap gap — watch for this at first flash

`GenericPhy::phy_init` enables auto-negotiation but **never writes `ANAR`**, so
what the PHY advertises comes entirely from the `MODE[2:0]` straps — which are
multiplexed onto `RXD0`/`RXD1`/`CRS_DV`, i.e. STM32 **PC4/PC5/PA7**. Straps of
`100` or `101` advertise **100BASE-TX half duplex only**, the link comes up,
and `poll_link` still returns true.

Both boards now log `ANAR` and the decoded strap once over `defmt` and warn if
100-full is not advertised. **If that warning fires, the decision is whether to
write `ANAR` before auto-negotiation** — a bring-up change, deliberately not
taken pre-emptively. Plan §13.4a has the table and the reasoning.

*(`eth-phy-lan87xx` was evaluated and not adopted — §13.4a says why. Its
documentation is what surfaced this.)*

### D6 — camera latency, deferred with D5

`t_us` is stamped after lane detection instead of at capture, the D415 hardware
timestamp is discarded, and `correct_lane` ignores `t_us` entirely. The
measurement half (stamp at capture, add a dropped-frame counter, pin the
librealsense queue depth) is behaviour-neutral and safe to take alone. The
compensation half changes control and belongs with the D5 re-tune.

## Still true, and still not a coding task

Nothing has been flashed. The drivetrain is uncalibrated. Replay parity is
synthetic only. `docs/STATUS_OPEN.md` §1 is the authoritative list.
