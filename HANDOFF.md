# Session handoff — 2026-09-20

> **Ephemeral. Delete this file once the work below is done.** It is session
> state, not documentation. This project deleted its last handoff doc
> (`HANDOFF_field_run_verification.md`) because it outlived its usefulness and
> started misleading people — don't let this one do the same.
>
> The durable record is `docs/RUST_REWRITE_PLAN.md` §13. If something here
> matters in a month, it belongs there instead.

## Where things stand

Branch `rs`, HEAD `83dcebc`. **Working tree clean, 290 Rust tests + 82 Python
tests passing, clippy clean, fmt clean, both firmware images build, replay
gate exits 0.** Nothing is half-applied.

The last commit added the *wire contract* for board self-diagnostics —
`BoardDiagnostics` (0x0903), `BoardId`, `ResetCause`, `PostBits`, and three
new `HealthBits`. It is deliberately inert: nothing produces or consumes it
yet. Read those doc comments in `crates/rover-msgs/src/types.rs` (end of file)
before implementing either side; they specify the semantics.

## What was in flight when the session hit its rate limit

Two subagents were launched in parallel and both died at the session limit
while still reading files. **Neither wrote anything.** Their briefs are
reproduced below and are re-runnable as-is.

### Job A — firmware POST + LAN8742A diagnostics

Scope: `firmware/chassis/**` and `firmware/sensors/**` only.

1. **Reset cause.** Read `RCC_CSR` once at boot before anything clears it, map
   to `ResetCause`, then clear via `RMVF`. Check the watchdog and software
   flags *first*, then pin, then power-on/brown-out — a power-on reset sets
   several flags at once on this part, and the most diagnostic cause must win
   over the most common one. Put it in a small `diag.rs` per crate.

2. **LAN8742A PHY diagnostics, without risking bring-up.** This is the
   delicate one — see "The PHY question" below for the full reasoning. The
   rule: write `Lan8742a<SM>` that **wraps** `GenericPhy<SM>` and delegates
   all three `Phy` methods to it unchanged, then adds *read-only* register
   queries via `GenericPhy::station_management()`. The bring-up path stays
   byte-identical to what Embassy ships.

   Registers (⚠️ **from memory, unverified against a datasheet — treat as a
   starting point**): `0x02`/`0x03` PHY ID (LAN8742A = `0x0007_C130`, revision
   in the low nibble, so mask it off before comparing); `0x1F` bits [4:2] =
   resolved speed/duplex (`001` 10-half, `101` 10-full, `010` 100-half, `110`
   100-full); `0x1E` symbol error counter, read-to-clear so accumulate
   saturating. Every uncertain read must degrade to "unknown" (`0`) rather
   than guessing. A diagnostic that lies is worse than one that says so.

3. **Power-on self-test.** Implement the per-board bit table in `PostBits`'
   doc comment. Record `post_run` and `post_pass` separately. **A POST failure
   must not prevent boot** — publish it and carry on degraded; a rover that
   refuses to start because its power monitor died is worse than one that
   drives without power telemetry. For the PWM bits, check what is actually
   checkable (timer configured, duty register accepted a write, TIM1's MOE
   set) and say plainly in the comment that it is a configuration check, not
   an output check — do not claim to verify something you cannot.

4. **Publish** once after POST, then at 1 Hz. Add the destination constant to
   each `config.rs` citing its route key, as the existing `*_DEST` constants
   do. Track `tx_drops` (saturating count of failed sends). Diagnostics must
   never delay the control loop or the IWDG pet.

Acceptance: both images build, clippy clean, and **report the flash/RAM delta**
(was chassis 65,048 B / 17,724 B, sensors 54,316 B / 17,844 B).

### Job B — host side: telemetry, preflight, base-station `runs/`

Scope: `crates/`, `perception/`, `testdata/`, `config/rover.toml`. Not
`firmware/`, not `docs/`.

1. **Close the wire contract.** Add `BoardDiagnostics = ["telemetry", "base"]`
   to `[routes]`. Add it to `golden.rs`'s `all_types!` and generate
   `testdata/BoardDiagnostics.bin`. Mirror the type in
   `perception/rover_perception/wire.py` and add the instance to
   `perception/tests/test_wire_golden.py`, keeping it identical to
   `golden.rs`'s. Get field order from the Rust `encode`/`decode`, not from
   the struct declaration.

2. **Telemetry ingests it.** Subscribe, log to `board_diagnostics.csv` in the
   run directory (decode enums to their `name()` strings — a CSV that says
   `IWDG` is worth more at 2 a.m. than one that says `4`), and drive the three
   new health bits. `BOARD_RESET` must also fire when a board's `uptime_s`
   goes *backwards* versus its last sample: on a clean power glitch the reset
   cause is legitimately `PowerOn`, so uptime is the check that catches it.
   **Key the state by `BoardId`** — one slot for both boards lets a fault on
   one hide behind the other's good news.

3. **`crates/rover-doctor`** — a preflight GO/NO-GO the operator runs on the
   base PC before driving. Binds as `base`, listens ~10 s, checks: every
   service reachable, both boards passed POST, no abnormal resets, links at
   100 Mbit/s full duplex with no symbol errors, no health bits, **drivetrain
   calibrated** (this fails today — `metres_per_tick` is `0.0` — and that is
   correct and must be loud, naming plan §2.6), RTK fix quality, lane
   detection live, estimator converged. One line per check, exit non-zero on
   NO-GO so it can gate a launch script. **Distinguish "not seen yet" from
   "seen and bad"** — the operator acts differently on each — and never report
   GO on missing data. Unit-test the verdict logic as a pure function over
   observations, separate from the socket code.

4. **Base station `runs/`.** `ground-station` writes one flat
   `ground_station_telemetry.csv`; give it the same `run_NNN_<stamp>/`
   convention `crates/rover-telemetry/src/runs.rs` implements. That module now
   needs two consumers — promote it to a shared crate or expose it as a
   library, but **do not copy-paste it**; two divergent run-numbering
   implementations is exactly the bug class this project keeps finding.

### Job C — docs (not started, mine)

- `docs/FIELD_TEST.md` — how to run a field test end to end, and the `runs/`
  data collection story on rover and base.
- Fold the PHY answer below into plan §13.4, which currently lists the PHY as
  the top hardware risk without saying what can be done about it.

## Open decision that blocks driving: D5

`docs/RUST_REWRITE_PLAN.md` §13.3b **D5** records a measured sign
inconsistency: the detector's `theta` is the negative of
`d(cross_track)/d(distance)`, while both `Ekf::correct_camera` and
`RoverState::at_lookahead` assume it is positive. Deliberately unfixed —
the change triples heading authority against gains tuned around the
cancellation.

**Cheapest resolution: 60 seconds with the rover on blocks.** Show the camera
a line angled clearly to the right and watch the servo. If it turns left, D5
is confirmed on hardware and the fix is one negation in
`LaneDetector.detect`, followed by re-tuning.

Do not let the README's "sign convention is inverted on purpose" note be read
as covering `heading_err_rad`. It does not.

## The PHY question, answered

**You can write a LAN8742A-specific driver. Nothing blocks it.**
`embassy_stm32::eth::Phy` is a public trait with exactly three methods
(`phy_reset`, `phy_init`, `poll_link`), and `StationManagement` — raw
`smi_read`/`smi_write` over MDIO — is publicly exported. No sealed traits, no
fork needed. It is roughly 150 lines.

Two things are worth knowing before doing it.

**`GenericPhy` is already half a Microchip driver.** Its `phy_init` writes
`PHY_REG_WUCSR` at `0x8010` through MMD indirect access (`0x0D`/`0x0E`) —
that is an SMSC/Microchip vendor register, not generic clause 22. Embassy
wrote it around exactly this family of parts, which is why it is the sensible
default for a LAN8742A and why bring-up is likely to just work.

**What it genuinely cannot tell you** is the diagnostic half:

- It never reads PHY ID1/ID2. The constants are declared and
  `#[allow(dead_code)]`. So a mis-strapped, dead or substituted PHY produces
  no identity mismatch — it simply fails to link, with no reason given.
- `poll_link` returns `bool`, discarding the resolved speed and duplex. This
  is the real silent failure: a marginal cable negotiates **10 Mbit/s half
  duplex**, `poll_link` says "up", and the MAC stays configured for 100 full.
- No symbol error counter, which is the thing that goes non-zero on a
  marginal cable *long before* anyone notices packet loss.

All three are read-only queries. That is why Job A wraps `GenericPhy` rather
than replacing it: **the unproven bring-up path stays byte-identical to the
one thousands of boards run, and the new code cannot affect link
establishment.** Replacing a widely-used-but-unverified-here sequence with a
bespoke-and-also-unverified one would be a strictly worse trade, and this
firmware has never been flashed.

## Everything else that still blocks a field run

1. **Drivetrain calibration.** `ticks_per_rev`, `metres_per_tick`,
   `track_width_m` are all `0.0`. No metric speed exists until they are
   measured — plan §2.6, ~20 minutes with a tape measure, **against the 4×
   decoder** or every speed reads double.
2. **Nothing has been flashed.** LAN8742A bring-up is the hard gate; plan
   §13.4 has the rest.
3. **D5**, above.
4. **Replay parity is synthetic only.** No recorded ROS 2 run exists in this
   repo and none ever did. Producing a real baseline means checking out `main`.
