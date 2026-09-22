# Drivetrain encoder calibration

Step-by-step, paste-able procedure for measuring `ticks_per_rev`,
`metres_per_tick` and `track_width_m` — the three `[drivetrain]` values in
`config/rover.toml` that are currently `0.0`. `docs/RUST_REWRITE_PLAN.md`
§2.6 has the reasoning (why the constant isn't in this repo, the 2×/4× trap,
the maths); this document is the how. `docs/HARDWARE.md` §3 has the
drivetrain facts and the one recalled-not-measured expected value. Read
those first if you haven't; this document assumes them and doesn't repeat
the reasoning.

This is a bench-then-field session, ~20 minutes total, needing a tape
measure, chalk or marking tape, and a laptop with an ST-Link cable.

> ⚠️ **None of this has been run.** Nothing in `firmware/` has ever been
> flashed to real silicon (`docs/STATUS_DONE.md`, `docs/FIELD_TEST.md` §0.3).
> Every command below has been checked against the source in this repo —
> the Cargo config, the routing table, the message wire format — but not
> against a board on a bench. Where a step depends on something only real
> hardware can confirm (probe enumeration, actual tick counts, cable
> behaviour), it says so.

---

## 1. Which ST-Link is which board

Two identical NUCLEO-F767ZI boards, so the on-board ST-Link's USB serial is
the only reliable way to tell them apart once both are plugged into the same
PC.

| Board | IP | ST-Link serial |
|---|---|---|
| Sensors (encoders live here) | `192.168.1.6` | `066DFF3932504E3043014542` |
| Chassis | `192.168.1.2` | `066AFF3932504E3043101915` |

Both serials above are recovered from the user's own local shell aliases
(`flash_chassis` / `flash_sensors`, OpenOCD-based, still targeting the old
`mros2-mbed-*` build outputs), not from a fresh `probe-rs list` capture on
this repo's toolchain — but they name the same physical boards, since the
ST-Link serial lives in the debugger hardware, not the firmware. The sensors
value matches the one already recovered from git history below, which is a
useful cross-check. The chassis value has no other corroborating source in
this repo's history (see below) — if it misflashes at the bench, re-capture
it with `probe-rs list` as originally planned and correct this table.

The sensors-board serial is also recovered from the old ROS 2 history on the
`roboticsgg-almondmatcha` remote. The citation worth knowing is still live at
the current tip of that remote's `main`:

```sh
git show roboticsgg-almondmatcha/main:copilot-session-note/SESSION_2026-07-07.md \
    | sed -n '50,62p'
```

Line 56 flashes the sensors/GNSS board (`192.168.1.6`) with
`-c "hla_serial 066DFF3932504E3043014542"` — and the chassis-dynamics block
immediately below it, in that same snippet, carries **no serial at all**.
That one snippet is the evidence for both rows of the table above.

Two further copies exist in the history but have since been dropped from
`main`, so they need `git show <commit>:<path>`: commit `a90bf816`'s
`copilot-session-note/SESSION_2026-05-22.md:92`, and commit `a307c32`'s
`docs/STM32_CHANGES_SUMMARY.md:270` — the latter using OpenOCD's newer
`adapter serial` spelling rather than `hla_serial` (see the footnote below).

`066DFF3932504E3043014542` is the *only* ST-Link serial that appears anywhere
in that git history: every commit was grepped for any bare 24-hex-character
ST-Link serial and exactly one distinct value came back. The chassis board
was always flashed there as "the default ST-LINK", with no serial ever
written down in-repo — `066AFF3932504E3043101915` above comes from the
user's local `flash_chassis` alias instead, not from this repo's history.

If the bench session hasn't confirmed it yet, treat the chassis row as
unverified and capture it directly the first time both boards are on the
bench together, as a sanity check against the alias value:

```sh
# Unplug the sensors board (or note both serials and match by elimination).
# With ONLY the chassis board's USB cable connected:
probe-rs list
```

Everything below that flashes "the chassis board" should name the serial
explicitly with `--probe`, for the same reason the sensors board's serial
matters: with both boards plugged in, `probe-rs run` with no `--probe` picks
whichever one it enumerates first, and that is not guaranteed to be stable
across replugging.

> Footnote for anyone cross-referencing the old OpenOCD-era notes: OpenOCD
> renamed this option between versions — `hla_serial` in older notes,
> `adapter serial` from OpenOCD ≥0.12. Both spellings appear in the old
> history. This repo's toolchain is `probe-rs`, not OpenOCD, so the renaming
> doesn't affect anything here — it matters only when you're reading those
> old session notes yourself.

---

## 2. One-time setup on the Linux base station

`probe-rs-tools` links against `libudev`, so on a fresh Debian/Ubuntu box the
`cargo install` below fails at the linker step until these are present. Do
this first — the error it produces otherwise ("could not find system library
`libudev`") reads like a Rust problem and isn't one:

```sh
sudo apt install -y pkg-config libudev-dev build-essential
```

Then:

```sh
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools --locked
```

(On Fedora/RHEL the equivalent is `sudo dnf install pkgconf-pkg-config
systemd-devel`. On Arch, `pkgconf` and `systemd-libs` are usually already
there.)

`rust-toolchain.toml` at the repo root already pins `channel = "stable"` with
`rustfmt`/`clippy`/`thumbv7em-none-eabihf` as components — `rustup` will pick
that up automatically inside the repo, so the `target add` above is mostly a
belt-and-braces step for a fresh machine.

### udev rule, so you don't need `sudo` for every flash

ST-Link V2-1 (the on-board debugger on both Nucleo-144s) is USB `0483:374b`.
Add `3748` and `374f` too, in case an older ST-Link/V2 or a V3 unit ever ends
up on this bench:

```sh
sudo tee /etc/udev/rules.d/49-stlink.rules >/dev/null <<'EOF'
# ST-LINK/V2-1 (on-board debugger, NUCLEO-F767ZI) and other ST-Link revisions.
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", GROUP="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", GROUP="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Make sure your user is in the `plugdev` group (`sudo usermod -aG plugdev
$USER`, then log out and back in — group membership doesn't apply
retroactively to an open session), and physically replug both boards after
installing the rule so the kernel re-evaluates permissions on them.

⚠️ **If `probe-rs list` still shows nothing after that:** an old ST-Link
firmware is a known cause of probe-rs failing to enumerate a board at all.
Update it from Windows or via ST's own tool with ST's **STSW-LINK007**
firmware updater before assuming the udev rule or the cable is at fault.

---

## 3. Building and flashing

Both firmware crates are **deliberately not workspace members** (each has an
empty `[workspace]` table — see the comment at the top of
`firmware/sensors/Cargo.toml` / `firmware/chassis/Cargo.toml`), so you must
build from inside the crate directory. `cargo build` from the repo root will
not build them.

```sh
cd firmware/sensors
```

`firmware/sensors/.cargo/config.toml` already sets
`[build] target = "thumbv7em-none-eabihf"`, so a plain `cargo build --release`
here builds for the STM32 — you do not pass `--target`. That same file sets
`runner = "probe-rs run --chip STM32F767ZITx"` and `DEFMT_LOG = "info"`, which
is what makes `cargo run --release` flash-and-attach in one step.

### 3.1 The calibration image

Calibration needs a 1 Hz tick-count readout that the normal field firmware
doesn't carry. It's gated behind a `calibration` Cargo feature on
`sensors-fw`, default off (`firmware/sensors/Cargo.toml`'s `[features]`
block), which spawns a task logging over defmt in exactly this format:

```
INFO  encoders: L=15014 R=14982 dL=1502 dR=1498
```

— absolute left/right tick counts, then the delta over the last second.
**The quadrature decoding itself is identical with and without the
feature** — `calibration` only adds a log line, it doesn't touch
`encoders.rs`'s decode table — so a constant measured on the calibration
image is valid for the field image with no conversion.

Build and flash it:

```sh
cargo build --release --features calibration

# Explicit probe selection — do this whenever both boards might be plugged
# in, so you don't flash the wrong one. 0483:374b is the ST-Link V2-1
# VID:PID; the trailing field is the serial from §1. `--features` is a
# cargo flag, not a probe-rs one: it already did its job at build time, so
# `probe-rs run` here just takes the resulting ELF and flashes it.
probe-rs run --chip STM32F767ZITx \
    --probe 0483:374b:066DFF3932504E3043014542 \
    target/thumbv7em-none-eabihf/release/sensors-fw
```

`probe-rs run` takes the **binary**, not `cargo` flags — build first, then
point `probe-rs run` at the artifact directly, as above. If only one
board is attached, the shortcut is:

```sh
cargo run --release --features calibration
```

which goes through the configured runner and skips the manual `probe-rs run`
invocation, but does not let you pick a probe by serial — don't use it with
both boards plugged in.

The binary is named `sensors-fw`; artifacts land at
`firmware/sensors/target/thumbv7em-none-eabihf/release/sensors-fw` regardless
of which of the two build methods above produced it.

Keep the `probe-rs run` session attached in its terminal — that is your log
window for §4 and §5. Ctrl-C stops it; the board keeps running the flashed
image either way, since this is a normal flash, not a debug halt.

### 3.2 Going back to the field image

When you're done, reflash without the feature so the board doesn't carry a
1 Hz log line nobody in the field reads:

```sh
cargo build --release
probe-rs run --chip STM32F767ZITx \
    --probe 0483:374b:066DFF3932504E3043014542 \
    target/thumbv7em-none-eabihf/release/sensors-fw
```

---

## 4. Reading tick counts without reflashing (fallback)

If the sensors board is already installed in the rover and you don't want to
pull it for a bench session, the field image already publishes `WheelSensors`
(`ticks_left`, `ticks_right`, `t_us`) at 10 Hz, unicast to `control`
(`192.168.1.1:7001` per `[services]`/`[routes]` in `config/rover.toml`).
`rover-tap` can read that stream directly:

```sh
cargo run -p rover-tap -- --as control --type WheelSensors
```

`rover-tap` binds `0.0.0.0:<control's port>` and accepts frames by *source*
address, so the machine running it doesn't need to *be* the RPi in any other
sense — but it does need to actually own `192.168.1.1` for the sensors
board's unicast packets to arrive there at all. In practice that means
running this on the RPi itself, or temporarily taking over its address with
`rover-control` not running, so the two don't fight over the port.

This gives you absolute `ticks_left`/`ticks_right` only, at 10 Hz — no
per-second delta the way the calibration image's log line does — so for
Procedure A below you'd read the count before and after the ten turns by
eye, the same arithmetic as §5.

⚠️ **`[debug] mirror` does not work for this.** The mirror is implemented
only in the host-side `crates/rover-bus` `Bus::publish` (`config/rover.toml`'s
`[debug] mirror` key, read by that crate) — checked directly against
`crates/rover-bus/src/lib.rs`'s `publish`, which is where the mirror send
happens. The STM32 firmware doesn't link `rover-bus` at all (see
`firmware/sensors/Cargo.toml`'s dependency list) and builds its own UDP
frames by hand (`encoders.rs`'s `publish_task`, `encode_frame` +
`socket.send_to`) with no mirror concept whatsoever. A `[debug] mirror`
pointed anywhere will show you every host-side republish and relay, but
**never** a message a board published directly — `WheelSensors` included.
`--as control` above is the only way to see it without touching the wire
format. (`firmware/sensors/src/config.rs`'s comment on `WHEEL_SENSORS_DEST`
covers the same ground for anyone reading the firmware source directly.)

---

## 5. Procedure A — ticks per revolution (bench, ~5 min)

Isolates the encoder constant and absorbs gear ratio automatically. Do this
first; it's the fast sanity check before you take anything to the field.

1. Rover on blocks, drive wheels clear of the ground (same setup as
   `docs/FIELD_TEST.md` §0.2/§0.3).
2. Flash the calibration image (§3.1) and keep `probe-rs run` attached so
   you can watch the 1 Hz `encoders:` log line.
3. Mark the tyre and the chassis with chalk so you can see a full turn.
4. Note the resting `L=`/`R=` counts from the log.
5. Rotate **one** wheel by hand, forward, exactly **10 full revolutions**.
6. Note the new `L=`/`R=` count for that wheel.
   `ticks_per_rev = (end - start) / 10`.
7. Repeat for the other wheel. They should agree within a few percent — if
   not, you have a wiring or mounting asymmetry worth finding before you go
   further.

**Direction check, do it here.** `firmware/sensors/src/encoders.rs`'s module
doc says forward rotation must count **up**, but the sign was chosen from the
quadrature state sequence alone, with "no way to check it against the
physical mounting from source alone" — it has never been bench-verified.
While you're turning each wheel forward in step 5, confirm that wheel's
count is increasing, not decreasing. **If a wheel counts backwards**, the
firmware's own fix is to swap that encoder's two channel wires (cheaper than
patching the decode table) — do that at the connector, not in software, and
re-run this procedure for that wheel afterward.

**Expected: ~15,000 counts for the ten turns** (~1500 ticks/rev, this
firmware's 4× decoding — see `docs/HARDWARE.md` §3 for exactly where that
number comes from: a recollection of the ROS 2 system, offered in
conversation, not a measurement. Treat it as something to agree or disagree
with, not a value to type into `config/rover.toml` directly).

- **~15,000** → decoding and wiring look right; take the measured number.
- **~7,500** → the board is still decoding at 2×. **Stop.** That is exactly
  the failure this whole document exists to catch — go back to
  `firmware/sensors/src/encoders.rs` and `docs/HARDWARE.md` §5's pin-mux
  argument before doing anything else. Do not proceed to §6 with a 2×
  reading.
- **Anything else** → find out why before driving; don't average it away.

---

## 6. Procedure B — metres per tick (field, ~10 min, authoritative)

Procedure A gives you `ticks_per_rev`. This gives you `metres_per_tick`
directly, off the real field surface — it absorbs tyre compression and true
rolling radius, which A can't. **B is what goes into `config/rover.toml`; A
is the sanity check.**

1. Mark a **10.00 m** straight line on the actual field surface.
2. Drive the rover open-loop along it.
3. `metres_per_tick = 10.00 / mean(total_ticks_left, total_ticks_right)`.
4. Three trials, average.

**The data-path problem, honestly stated:** B needs the rover actually
moving, so you can't do this at a bench with a fixed ST-Link cable the way
you did for A. The workable answer is the same ST-Link tether, just walking:
10 m at roughly 0.15 m/s is about a minute, and a USB cable plus a laptop
walking alongside the rover is genuinely fine for that duration. Keep
`probe-rs run` attached exactly as in §5 and read the same `encoders:` log
line before and after the run.

If disagreement between A and B exceeds **~5%**, suspect wheel slip on the
field surface rather than a decoding problem — A and B measuring different
things by more than a few percent is itself useful information.

> **A possible future change, not an instruction:** if walking the tether
> becomes the limiting factor, `firmware/sensors/src/config.rs`'s own
> comment on `WHEEL_SENSORS_DEST` anticipates recording a calibration run
> over the network instead — add `"telemetry"` to `[routes] WheelSensors` in
> `config/rover.toml` **together with** a subscriber in `rover-telemetry`
> and a CSV column. That file is explicit that the two must land together;
> don't add the route without the subscriber, or you'll have a board quietly
> emitting a datagram nothing reads, which is the exact defect it already
> documents having found once for this same message type.

---

## 7. Track width

`track_width_m` is a plain tape measurement: centre-to-centre between the two
drive wheels' contact patches. Measure it in the same session as A/B —
there's no separate procedure. It buys a second, independent yaw-rate
estimate from the difference between left/right tick rates, which is a
useful cross-check against the gyro and costs nothing once you already have
per-wheel tick data.

---

## 8. Writing the results back

Fill in `[drivetrain]` in `config/rover.toml`. The numbers below are
**placeholders to show the shape of the block — do not copy them**; use
what you actually measured in §5–§7.

```toml
[drivetrain]
wheel_diameter_m = 0.125          # measured
decoding         = "quadrature_4x" # MUST match the firmware — see RUST_REWRITE_PLAN.md §2.6
ticks_per_rev    = 1487            # from Procedure A — placeholder, not a real measurement
metres_per_tick  = 0.0002641       # from Procedure B — placeholder, authoritative once real
track_width_m    = 0.398           # measured — placeholder
wheelbase_m      = 0.4875          # measured, carried over from the ROS 2 system
```

`decoding` must keep saying `"quadrature_4x"` — that's what this firmware
actually does (`firmware/sensors/src/encoders.rs`), and it's what your
Procedure A/B numbers are calibrated against. Changing the decoding mode
without recalibrating silently doubles or halves every `speed_mps` in the
system.

### Verify

```sh
cargo run -p rover-doctor -- --config config/rover.toml
```

Before calibration, its drivetrain check is NO-GO. Run against this repo's
checked-in `config/rover.toml` (still `metres_per_tick = 0.0` at the time of
writing), it prints:

```
[NO-GO   ] drivetrain calibrated -- metres_per_tick = 0.0 in [drivetrain] -- the drivetrain has never been calibrated. Run the procedures in docs/CALIBRATION.md before trusting speed_mps or driving autonomously (the reasoning behind them is in docs/RUST_REWRITE_PLAN.md section 2.6). There is no override for this check.
```

Once `metres_per_tick` is a positive number, the same check flips to GO —
there is deliberately no flag to skip it in the meantime, and none appears
once it passes either.

`rover-doctor` also binds `[services] base` and listens ~10 s for the
boards' UDP feeds before printing anything; with no rover on the LAN (as
when running the command above from a bench PC that isn't `192.168.1.10`),
every other check reports `NOT SEEN` rather than GO or NO-GO — that's
expected and is a statement about the network, not about calibration.
