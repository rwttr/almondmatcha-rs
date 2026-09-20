# Session handoff — 2026-09-20 (third session)

> **Next session is a bench session: 2026-09-21, on the Linux base PC
> (`192.168.1.10`, `yupi@`), both NUCLEO boards on USB.** Read
> § "Tomorrow at the bench" below before anything else — the work from this
> session is on a Mac and does not exist on that machine yet.

> **Ephemeral. Delete once the work below is done.** The durable record is
> `docs/STATUS_DONE.md` and `docs/STATUS_OPEN.md`. If something here still
> matters in a month, it belongs in one of those.

## ⚠️ This is a different repo from the one most of the history was written in

| Remote | Points at | What it's for |
|---|---|---|
| `origin` | `rwttr/almondmatcha-rs` (private, personal) | **The repo this session works in.** `git push` with no args goes here. Has **only** `rs`, which is its default branch. |
| `roboticsgg-almondmatcha` | `RoboticsGG/almondmatcha` (public, org) | Upstream. Holds `main` (the ROS 2 fallback) and the project's full history. Only pushed to by explicit name. |

Local `main` exists in this clone and tracks `roboticsgg-almondmatcha/main`.
**A fresh clone of `almondmatcha-rs` would not have it** — which is why the
docs were swept this session for bare `git show main:<path>` and
`git checkout main`, all of which worked here by accident and failed for
everyone else.

Local git identity for this repo only (`.git/config`, nothing global):
`Rattachai Wongtanawijit <ratta.rwttr@gmail.com>`.

`.claude/settings.local.json` holds a `Bash(git push:*)` allow rule so pushes
don't hit the auto-mode data-exfiltration classifier. It's gitignored — if
it's missing in a future session, `git push` will need re-approving.

**Lesson from two sessions ago, still worth not repeating:** a compound
`remote add && push` was blocked in one shot by the classifier, and the
*report* claimed the remote had been added when nothing in that call had run.
**Verify a blocked compound command's pieces individually before trusting any
of them ran.**

## Tomorrow at the bench — do these in order

The whole of this session's work was done on a **Mac**. Tomorrow's machine is
the **Linux base PC**, and the two are only connected through `origin`
(`rwttr/almondmatcha-rs`). Nothing below works until step 0 has happened.

0. **Commit and push `rs`.** At the time of writing, everything this session
   produced is **uncommitted** on the Mac — including `docs/CALIBRATION.md`,
   which is still *untracked*. Until it is pushed, the Linux PC's checkout has
   no calibration guide and no `calibration` Cargo feature, and step 2 below
   cannot be run at all. Then, on the Linux box: `git pull`.
1. **Install the toolchain on the Linux PC** — `docs/CALIBRATION.md` §2. It is
   a fresh machine for this stack: the apt prerequisites, `rustup target add
   thumbv7em-none-eabihf`, `cargo install probe-rs-tools --locked`, the udev
   rule, then **replug both boards** so the kernel re-evaluates permissions.
   Budget more time than the commands suggest; this is the step that bites.
2. **Identify the two probes before flashing anything.** `probe-rs list` with
   both USB cables in will show two ST-Links. One is
   `066DFF3932504E3043014542` — that is the **sensors** board (`192.168.1.6`,
   the one with the encoders). **The other is the chassis board, and its
   serial has never been written down.** Record it into `docs/CALIBRATION.md`
   §1's table as the first act of the session, and commit that one-line change
   while you are still at the keyboard.
3. **Flash the sensors board with the calibration image** and run Procedure A
   — `docs/CALIBRATION.md` §3.1 and §5. This is the first time any of this
   firmware meets real silicon.
4. **Flash the chassis board** (§3, using the serial you just recorded) and
   watch its `defmt` output for the PHY strap warning — see below.

Both boards being on USB means **both ST-Links enumerate at once**, so never
run `probe-rs run` without `--probe`; it picks whichever it sees first and
that ordering is not stable across replugs. The `cargo run --release`
shortcut in §3.1 is therefore **not** safe tomorrow — use the explicit
`probe-rs run --probe …` form.

If the boards are also on the Ethernet LAN tomorrow, `cargo run -p rover-tap
-- --as control --type WheelSensors` (§4) is the no-reflash way to watch the
same counts — but the base PC must hold `192.168.1.1` for that, which is the
RPi's address, so it is the fallback, not the plan.

## State

`HEAD` = `origin/rs` = `7a888d0`. **This session's work is uncommitted** —
see the diff, and see step 0 above.

**331 Rust tests, 88 Python tests, clippy clean, fmt clean, both firmware
images build** — re-verified against the working tree, not just before it.

Firmware flash/RAM: chassis 69,928 B / 18,480 B; sensors 58,932 B / 18,648 B
(default features). Of 2 MB flash and 512 KB SRAM. The sensors board's new
`calibration` feature adds **+664 B flash / +80 B RAM** and is **off by
default**, so the field image is byte-identical to `7a888d0`'s — confirmed by
rebuilding both and comparing section sizes.

## What landed this session

1. **`docs/CALIBRATION.md`** — the step-by-step drivetrain encoder
   calibration: Linux/probe-rs setup, udev rule, which ST-Link is which
   board, build, flash, read ticks, Procedures A and B, track width, writing
   the values back, verifying with `rover-doctor`. This is the doc that
   unblocks "Next" item 1.
2. **`firmware/sensors` gained a `calibration` Cargo feature** (default off):
   a 1 Hz `defmt` readout of both tick counters and their per-second deltas,
   so Procedure A needs nothing but a USB ST-Link cable — no LAN, no RPi.
   Format is a contract the doc is written against:
   `INFO  encoders: L=15014 R=14982 dL=1502 dR=1498`.
3. **`rover-doctor`'s drivetrain NO-GO now points at `docs/CALIBRATION.md`**
   for the how, keeping plan §2.6 as the reasoning. Its test asserts on both.
4. **Docs swept for the repo split** — every `main` reference qualified with
   the remote it actually lives on, the layout tree's root renamed to
   `almondmatcha-rs/` and its crate list corrected from ten to twelve, four
   wrong `perception/*.py` paths fixed, a dead `§13.1` cross-reference
   reworded, `README.md`'s test count corrected from 82 to 88, and deleted
   ROS 2 paths annotated as deleted where their sibling docs already were.

### The ST-Link serials — recovered, and one of them does not exist

| Board | IP | ST-Link serial |
|---|---|---|
| Sensors (encoders live here) | `192.168.1.6` | `066DFF3932504E3043014542` |
| Chassis | `192.168.1.2` | **never recorded** |

Live at the tip of the org remote:
`git show roboticsgg-almondmatcha/main:copilot-session-note/SESSION_2026-07-07.md`,
line 56 — it flashes the sensors board with that serial and the chassis board
with none, in the same snippet. That is the only ST-Link serial anywhere in
the history; every commit was grepped for any 24-hex-character serial and
exactly one distinct value came back. **Capture the chassis board's with
`probe-rs list` the first time both boards are on the bench**, and write it
into `docs/CALIBRATION.md` §1's table.

### Two things that would have wasted a bench session

- **Plan §2.6's Procedure A told you to log `chassis_sensors.csv`.** That is a
  ROS 2 filename; in this stack encoders are not logged to CSV at all
  (`docs/CSV_LOGGING.md`). Following it as written got you nowhere. Fixed —
  §2.6 keeps the reasoning and points at `docs/CALIBRATION.md` for the steps.
- **`firmware/sensors/src/config.rs` claimed `[debug] mirror` + `rover-tap`
  could see wheel ticks without a firmware change.** It cannot: the mirror
  lives only in the host-side `crates/rover-bus` `Bus::publish`, and the
  firmware has no mirror concept. Corrected, with the two paths that do work.

## Next, in order

Steps 1 and 2 are tomorrow's session (see above for the machine-specific
order); 3 and 4 follow from what it finds.

1. **Calibrate the drivetrain** — `docs/CALIBRATION.md`. Twenty minutes, no
   hardware risk. Expect **~15,000 counts for ten hand turns**; **~7,500
   means the firmware is still decoding at 2×** — stop if you see that. Do
   the direction check in the same pass: forward must count **up**, and the
   sign has never been verified against the physical mounting.
2. **Flash both boards on the bench.** The decision point for the whole
   branch. Watch the `defmt` output for the PHY strap warning (below).
3. **Confirm servo direction on blocks**, then **re-tune for D5 and D6**.
4. **Record a real ROS 2 baseline** if acceptance criterion 2 is to close —
   `git checkout main` in this clone (it tracks `roboticsgg-almondmatcha/main`;
   `origin` has no `main`).

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

Both boards log `ANAR` and the decoded strap once over `defmt` and warn if
100-full is not advertised. **If that warning fires, the decision is whether to
write `ANAR` before auto-negotiation** — a bring-up change, deliberately not
taken pre-emptively. Plan §13.4a has the table and the reasoning.

### D6 — camera latency, deferred with D5

`t_us` is stamped after lane detection instead of at capture, the D415 hardware
timestamp is discarded, and `correct_lane` ignores `t_us` entirely. The
measurement half (stamp at capture, add a dropped-frame counter, pin the
librealsense queue depth) is behaviour-neutral and safe to take alone. The
compensation half changes control and belongs with the D5 re-tune.

## ⚠️ Before editing docs: some headings are load-bearing from source

- **`docs/CONTROL_LAW.md` §2.1 and §1.7** are cited by number from
  `crates/rover-control/src/actuate.rs` (including a runtime `log::warn!` an
  operator reads) and `perception/rover_perception/lane.py`. §2.1's
  "16 % / nominal cruise duty" table row is hard-coded in `actuate.rs`.
- **`docs/VISION_PIPELINE.md`** — `## 2. Adaptive color segmentation …` and
  `### Regenerating the ROI` are cited from `perception/tests/oracle/config.py`,
  the **frozen** parity oracle, which must not be edited. The doc is the only
  side that could move, so it must not.
- **`docs/CSV_LOGGING.md` is cited by heading text and quoted prose, never by
  section number** — `## Run Directory Numbering`, `### RPi: mission_state.csv`,
  and a sentence quoted verbatim in `rover-telemetry/src/csv_writer.rs`.
  (The previous handoff said "by section number" for this file. It was wrong.)
- **`docs/RUST_REWRITE_PLAN.md` is cited by number from ~30 sites**, and
  **`docs/STATUS_OPEN.md` §1/§1.1** from five. `rover-doctor` has a test
  asserting its NO-GO string contains "RUST_REWRITE_PLAN.md" and "2.6".

Grep for citations before renumbering or retitling anything in those files.

## Still true, and still not a coding task

Nothing has been flashed. The drivetrain is uncalibrated. Replay parity is
synthetic only. `docs/STATUS_OPEN.md` §1 is the authoritative list. What
changed this session is that the calibration is now a written procedure
instead of a note saying it should happen.
