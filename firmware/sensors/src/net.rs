//! Ethernet + UDP bring-up. Identical structure to
//! `firmware/chassis/src/net.rs` - same Nucleo-144 board, same RMII wiring,
//! same PHY, same hardware-verification caveat.
//!
//! # Hardware verification needed
//!
//! Per the rewrite plan's §11 risk table and its "hard gate" in §9 step 4:
//! **the Nucleo-F767ZI's PHY is an LAN8742A**, and this code drives it with
//! `embassy_stm32::eth::GenericPhy`, which only assumes standard clause-22
//! MDIO register behaviour. This is a bog-standard clause-22 part and the
//! same PHY embassy's own upstream STM32F7 example targets, so this should
//! work - but it has not been run against real silicon as part of this
//! change. Confirm this on a bench before trusting anything downstream of it,
//! same as chassis.
//!
//! The RMII pin assignment below is the standard ST Nucleo-144 wiring, shared
//! by every Nucleo-144 board with on-board Ethernet - identical to chassis's,
//! because it's the same board.
//!
//! # What [`Lan8742a`] does and does not change
//!
//! Bring-up still runs `GenericPhy` exactly as described above. [`Lan8742a`]
//! wraps it to add *read-only* diagnostics - PHY identity, resolved speed and
//! duplex, symbol errors - and cannot affect link establishment. Same wrapper,
//! same reasoning as chassis; see `docs/RUST_REWRITE_PLAN.md` §13.4a for why a
//! full part-specific driver was considered and rejected.
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};
use core::task::Context;

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config as NetConfig, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_stm32::eth::{
    Ethernet, GenericPhy, InterruptHandler as EthInterruptHandler, PacketQueue, Phy, Sma,
    StationManagement,
};
use embassy_stm32::peripherals::{ETH, ETH_SMA, RNG};
use embassy_stm32::rng::{InterruptHandler as RngInterruptHandler, Rng};
use embassy_stm32::{bind_interrupts, Peri};
use static_cell::StaticCell;

use crate::config::{MAC_ADDR, SELF_IP};

bind_interrupts!(pub struct Irqs {
    ETH => EthInterruptHandler;
    RNG => RngInterruptHandler<RNG>;
});

/// Concrete device type: RMII Ethernet driven by [`Lan8742a`], a read-only
/// diagnostics wrapper around the standard clause-22 PHY driver, over the
/// on-chip station management (SMA/MDIO) block. See `Lan8742a`'s doc comment
/// for why this is a wrapper around `GenericPhy` rather than a change to it.
pub type Device = Ethernet<'static, ETH, Lan8742a<Sma<'static, ETH_SMA>>>;

/// Number of UDP sockets this board needs: one bound receive socket (the link
/// watchdog's inbound traffic, `config::SELF_PORT`) plus one transmit socket
/// each for `WheelSensors`, `PowerSample`, and `BoardDiagnostics` (via
/// [`tx_socket!`]). Raise this when adding another socket - see
/// [`tx_socket!`]'s doc comment.
const SOCKET_COUNT: usize = 4;

// --- LAN8742A read-only diagnostics ----------------------------------------
//
// Module-level atomics, not a field on `Ethernet` that some other task could
// borrow: `Ethernet` is moved into the `embassy_net::Runner` and then into
// `net_task`, so nothing outside this module can ever reach
// `Ethernet::phy_mut()` again after `init()` returns. `Lan8742a::poll_link`
// (called by the runner on its own schedule, ~500ms by default per
// `GenericPhy`) is the only writer; `phy_status()` is the only reader, from
// `diag::publish_task`. `Ordering::Relaxed` throughout: these are
// independent scalars with no ordering relationship to protect.
static PHY_ID: AtomicU32 = AtomicU32::new(0);
static LINK_SPEED_MBPS: AtomicU8 = AtomicU8::new(0);
static LINK_FULL_DUPLEX: AtomicBool = AtomicBool::new(false);
static PHY_SYMBOL_ERRORS: AtomicU16 = AtomicU16::new(0);

/// PHY Identifier 1 - datasheet default `0x0007`.
const REG_ID1: u8 = 0x02;
/// PHY Identifier 2 - datasheet default `0xC130`; bits [3:0] are a part
/// revision that varies between parts, see `check_phy_id`'s doc comment.
const REG_ID2: u8 = 0x03;
/// PHY Special Control/Status Register. Bits [4:2] are HCDSPEED, decoded in
/// `refresh_diagnostics`.
const REG_SPECIAL_CTRL_STATUS: u8 = 0x1F;
/// Symbol Error Counter. See `refresh_diagnostics` for why this is the
/// correct register (an earlier handoff note said `0x1E`, which is wrong).
const REG_SYMBOL_ERROR_COUNTER: u8 = 0x1A;

// --- ANAR / strap-config diagnostics (defmt-only, read-only) --------------
//
// Auto-Negotiation Advertisement Register and Special Modes register. Both
// are read once, logged once, and never written - see `log_strap_config`'s
// doc comment for the "why once" and this wrapper's top-of-file doc comment
// for the "why read-only at all".
//
// # Why this exists
//
// Embassy's `GenericPhy::phy_init` brings up auto-negotiation by writing
// BCR (Basic Control Register), but it never writes ANAR - embassy's source
// declares a `PHY_REG_ANTX` constant for register 0x04 and never uses it.
// So whatever the LAN8742A actually advertises to the far end is determined
// entirely by the `MODE[2:0]` hardware straps latched into the Special
// Modes register at reset (Microchip LAN8742A/LAN8742Ai datasheet, Revision
// 1.1, 05-21-13, Table 3.4):
//
//   MODE[2:0] | Meaning                                       | ANAR[8,7,6,5]
//   ----------|-----------------------------------------------|--------------
//   000       | 10BASE-T half, auto-neg disabled               | N/A
//   001       | 10BASE-T full, auto-neg disabled               | N/A
//   010       | 100BASE-TX half, auto-neg disabled             | N/A
//   011       | 100BASE-TX full, auto-neg disabled             | N/A
//   100       | 100BASE-TX half advertised, auto-neg enabled   | 0100
//   101       | Repeater mode, auto-neg enabled, 100 half adv. | 0100
//   110       | Power-Down mode                                | N/A
//   111       | All capable, auto-neg enabled                  | 1111
//
// (ANAR bits [8,7,6,5] are 100-full, 100-half, 10-full, 10-half.)
//
// If the straps land on `100` or `101` - both real, plausible strap
// configurations, not exotic ones - the PHY only ever advertises 100BASE-TX
// *half* duplex. Auto-negotiation still completes, `poll_link` still
// returns `true`, and the link comes up - just capped below its intended
// rate, with nothing in the existing diagnostics pointing at why. That is
// the same silent failure `Lan8742a` as a whole exists to catch, but with a
// far more likely root cause than a marginal cable: on this board
// `MODE[2:0]` is multiplexed onto RXD0/RXD1/CRS_DV (datasheet Table 3.5),
// which per the RMII pin list at the top of this file are STM32 pins
// PC4/PC5/PA7 - so the strap value literally depends on the state of those
// GPIOs at the instant the PHY comes out of reset.
//
// # Why `defmt` only, never the wire
//
// This is a bring-up-bench problem, not a field problem: at first flash a
// debugger is attached and `defmt`/RTT is available, so a bad strap can be
// read straight off the log. In a field run there is no probe, but by then
// `PostBits::LINK` and `BoardDiagnostics.link_speed_mbps` already report the
// *symptom* (link down, or link up at the wrong speed) and send you back to
// the bench, where the probe - and this log line - are available again.
// Adding these two registers to `BoardDiagnostics` would be a breaking wire
// change (new `WIRE_LEN`, a regenerated golden fixture, and a Python mirror
// update) purchased for information that is only ever useful with a
// debugger already attached. Do not "fix" that by promoting this to the
// wire - it would be paying a permanent protocol cost for a one-time
// bring-up question the wire was never going to answer anyway.
//
// # What this does NOT do
//
// It does not write ANAR, does not write BCR, and does not reset the PHY.
// It reads two registers and logs them - nothing else. If a bench run shows
// a bad strap, writing ANAR to override it is a separate, evidence-driven
// change, not something to bundle in here.

/// Auto-Negotiation Advertisement Register - what this PHY tells the far
/// end it can do. Read-only here; see the block comment above for why
/// nothing in this file ever writes it.
const REG_ANAR: u8 = 0x04;
/// Special Modes register. Bits [7:5] latch `MODE[2:0]` from the
/// RXD0/RXD1/CRS_DV strap pins at PHY reset - see the block comment above.
const REG_SPECIAL_MODES: u8 = 0x12;

/// Set once [`Lan8742a::log_strap_config`] has logged its one-time ANAR /
/// MODE[2:0] report. `poll_link` runs on `GenericPhy`'s own schedule
/// (~500ms by default) for the life of the program, so without this latch
/// the same line would repeat forever instead of appearing once at
/// bring-up - same reasoning as the existing address-scan latch below.
static STRAP_CONFIG_LOGGED: AtomicBool = AtomicBool::new(false);

/// Decode the latched `MODE[2:0]` strap value (Special Modes register bits
/// [7:5], always in `0..=7`) to the meaning given by the datasheet's
/// Table 3.4 (reproduced in the block comment above). `mode` must already
/// be masked to 3 bits; the fallback arm is unreachable in practice but
/// returns "unknown" rather than a guess, consistent with this driver's
/// rule of never presenting an unconfirmed encoding as a confident answer.
fn decode_mode_straps(mode: u8) -> &'static str {
    match mode {
        0b000 => "10BASE-T half, auto-neg disabled",
        0b001 => "10BASE-T full, auto-neg disabled",
        0b010 => "100BASE-TX half, auto-neg disabled",
        0b011 => "100BASE-TX full, auto-neg disabled",
        0b100 => "100BASE-TX half advertised, auto-neg enabled",
        0b101 => "repeater mode, auto-neg enabled, 100 half advertised",
        0b110 => "power-down mode",
        0b111 => "all capable, auto-neg enabled",
        _ => "unknown",
    }
}

/// Number of address-scan attempts `Lan8742a` will make over its lifetime.
/// One, per the task spec: if nothing answers on the first scan, later scans
/// are no more likely to find anything (address discovery is a boot-time
/// property of the wiring, not something that changes at runtime), so
/// repeating it every 500ms forever would only waste MDIO transactions.
const MAX_ADDR_SCAN_ATTEMPTS: u8 = 1;

/// Snapshot of what [`Lan8742a`] has learned about the PHY, for
/// `diag::publish_task`. See the module doc comment above for why this is
/// atomics-plus-accessor rather than a shared reference.
#[derive(Debug, Clone, Copy, Default)]
pub struct PhyStatus {
    /// `(ID1 << 16) | ID2`. `0` = address not yet found, or found but the
    /// read never happened - see `Lan8742a::scan_for_address`.
    pub phy_id: u32,
    /// `0` = link down, or HCDSPEED read as a value not in the LAN8742A's
    /// defined set (`Lan8742a::refresh_diagnostics` never guesses).
    pub link_speed_mbps: u8,
    pub link_full_duplex: bool,
    /// Raw Symbol Error Counter register value, NOT accumulated across
    /// polls. See `Lan8742a::refresh_diagnostics` for why accumulating a
    /// free-running, not-cleared-on-read counter would be actively
    /// misleading, and why it does not increment at all at 10BASE-T.
    pub phy_symbol_errors: u16,
}

/// Read the PHY diagnostics `Lan8742a` has collected so far. Safe to call
/// from any task at any time; returns every field at its "unknown" default
/// (`0`/`false`) until the network task's runner has polled the link at
/// least once.
pub fn phy_status() -> PhyStatus {
    PhyStatus {
        phy_id: PHY_ID.load(Ordering::Relaxed),
        link_speed_mbps: LINK_SPEED_MBPS.load(Ordering::Relaxed),
        link_full_duplex: LINK_FULL_DUPLEX.load(Ordering::Relaxed),
        phy_symbol_errors: PHY_SYMBOL_ERRORS.load(Ordering::Relaxed),
    }
}

/// Read-only diagnostics wrapper around [`GenericPhy`] for the Nucleo's
/// LAN8742A.
///
/// # Why a wrapper, and why it must stay this thin
///
/// Ethernet bring-up is this project's top hardware risk (see this module's
/// top-of-file doc comment) and has never been run against real silicon.
/// The task that added this wrapper was explicit that the bring-up write
/// sequence on the wire must stay byte-for-byte identical to what Embassy
/// ships. `phy_reset` and `phy_init` below are therefore pure delegation to
/// the inner `GenericPhy` - not so much as a log line added - and
/// `poll_link` delegates first and unconditionally, so the link-state
/// result `embassy-net` sees is exactly what `GenericPhy` would have
/// produced on its own. Everything this type adds happens *around* that
/// delegated call: read-only MDIO register reads whose only effect is to
/// populate the atomics above.
pub struct Lan8742a<SM: StationManagement> {
    inner: GenericPhy<SM>,
    /// Cached PHY address, found by `scan_for_address` the first time
    /// `poll_link` runs after `phy_init`. `GenericPhy::new_auto` discovers
    /// its own address the same way during `phy_reset`, but keeps it
    /// private - there is no accessor - so this type has to rediscover it
    /// independently, read-only, rather than share the answer.
    addr: Option<u8>,
    /// How many times `scan_for_address` has run. Bounded by
    /// `MAX_ADDR_SCAN_ATTEMPTS` so a PHY that never answers doesn't cost an
    /// extra 32-address MDIO sweep on every single poll forever.
    scan_attempts: u8,
}

impl<SM: StationManagement> Lan8742a<SM> {
    pub fn new(inner: GenericPhy<SM>) -> Self {
        Self {
            inner,
            addr: None,
            scan_attempts: 0,
        }
    }

    /// Scan MDIO addresses `0..32`, reading `REG_ID1` at each, and cache the
    /// first address that answers with neither `0x0000` nor `0xFFFF` (both
    /// mean "nobody home" on MDIO - an idle bus reads as all-ones or
    /// all-zeros depending on the transceiver, never a real PHY ID).
    /// Read-only: this issues the same kind of MDIO read `GenericPhy` itself
    /// already relies on for its own `smi_read`-based address probe, just
    /// against a different register, so it cannot perturb PHY state.
    fn scan_for_address(&mut self) {
        self.scan_attempts += 1;
        for addr in 0..32u8 {
            let id1 = self.inner.station_management().smi_read(addr, REG_ID1);
            if id1 != 0x0000 && id1 != 0xFFFF {
                self.addr = Some(addr);
                return;
            }
        }
    }

    /// Refresh every atomic from a PHY known to be at `addr`.
    fn refresh_diagnostics(&mut self, addr: u8) {
        let sm = self.inner.station_management();

        let id1 = u32::from(sm.smi_read(addr, REG_ID1));
        let id2 = u32::from(sm.smi_read(addr, REG_ID2));
        PHY_ID.store((id1 << 16) | id2, Ordering::Relaxed);

        // PHY Special Control/Status Register, bits [4:2] = HCDSPEED. Any
        // value outside this table is left as "unknown" (0 Mbit/s, half
        // duplex) rather than guessed - see the task's own instruction not
        // to guess at an undefined encoding.
        let special = sm.smi_read(addr, REG_SPECIAL_CTRL_STATUS);
        let speed_bits = (special >> 2) & 0b111;
        let (speed_mbps, full_duplex) = match speed_bits {
            0b001 => (10, false),
            0b101 => (10, true),
            0b010 => (100, false),
            0b110 => (100, true),
            _ => (0, false),
        };
        LINK_SPEED_MBPS.store(speed_mbps, Ordering::Relaxed);
        LINK_FULL_DUPLEX.store(full_duplex, Ordering::Relaxed);

        // Symbol Error Counter (register 0x1A), per the Microchip
        // LAN8742A/LAN8742Ai datasheet, Revision 1.1 (05-21-13). A prior
        // handoff note that seeded this task named register 0x1E (the
        // Interrupt Mask Register - reading it here would report a
        // meaningless number as a link-quality metric) and claimed the
        // counter is read-to-clear; both are wrong. The datasheet's own
        // words on 0x1A: "This field counts up to 65,536 and rolls over to
        // 0 if incremented beyond it's maximum value. Note: This register
        // is cleared on reset, but is not cleared by reading the register.
        // It does not increment in 10BASE-T mode."
        //
        // Two consequences of that text, both load-bearing here:
        //   1. This is a free-running counter, NOT read-to-clear. Reporting
        //      the raw value every poll (as done below) is correct;
        //      accumulating deltas across polls would count every error
        //      once per remaining poll interval instead of once, multiplying
        //      the true error count by however many times this function
        //      has run since boot - exactly the kind of lying diagnostic
        //      this whole change exists to avoid.
        //   2. The counter does not increment at 10BASE-T. So
        //      `phy_symbol_errors == 0` is only evidence of a healthy
        //      physical layer when `link_speed_mbps == 100`; at 10 Mbit/s
        //      it is the register's permanent resting value and proves
        //      nothing either way.
        let symbol_errors = sm.smi_read(addr, REG_SYMBOL_ERROR_COUNTER);
        PHY_SYMBOL_ERRORS.store(symbol_errors, Ordering::Relaxed);
    }

    /// Read ANAR and the Special Modes register once, and log what the PHY
    /// is actually advertising to the far end - see the block comment above
    /// `REG_ANAR` for why that can silently differ from "auto-negotiation
    /// enabled", and for why this is `defmt`-only rather than added to the
    /// wire.
    ///
    /// One-shot, guarded by `STRAP_CONFIG_LOGGED`: `poll_link` calls this on
    /// every poll (~500ms per `GenericPhy`'s default) for the life of the
    /// program, but the strap configuration is latched once at PHY reset
    /// and never changes afterwards, so logging it more than once would
    /// only spam the log at 2 Hz forever.
    ///
    /// Logs at `info!` when the PHY advertises 100BASE-TX full duplex (ANAR
    /// bit 8 set); at `warn!` when it does not, since the link can then
    /// never negotiate above half duplex no matter what the far end offers,
    /// naming the decoded `MODE[2:0]` strap value as the likely cause.
    ///
    /// If either register reads back as `0x0000` or `0xFFFF` - the same
    /// "nobody answered" sentinel `scan_for_address` checks for on
    /// `REG_ID1` - this is treated as a bad poll and skipped entirely: no
    /// log is emitted and the latch is left unset so a later, successful
    /// poll can still report. That keeps a flaky read from ever producing a
    /// confident warning, or a false "healthy" claim, about strap wiring.
    fn log_strap_config(&mut self, addr: u8) {
        if STRAP_CONFIG_LOGGED.load(Ordering::Relaxed) {
            return;
        }

        let sm = self.inner.station_management();
        let anar = sm.smi_read(addr, REG_ANAR);
        let special_modes = sm.smi_read(addr, REG_SPECIAL_MODES);

        if anar == 0x0000 || anar == 0xFFFF || special_modes == 0x0000 || special_modes == 0xFFFF {
            // Bad read - see this fn's doc comment. Leave the latch unset
            // and try again on the next poll.
            return;
        }

        // Special Modes register bits [7:5] = latched MODE[2:0].
        let mode = (special_modes >> 5) & 0b111;
        let mode_desc = decode_mode_straps(mode as u8);

        // ANAR bit 8 = 100BASE-TX full duplex advertised.
        let advertises_100_full = anar & (1 << 8) != 0;

        if advertises_100_full {
            info!(
                "phy: ANAR=0x{:04x} advertises 100BASE-TX full duplex; MODE[2:0]={:03b} ({})",
                anar, mode, mode_desc
            );
        } else {
            warn!(
                "phy: ANAR=0x{:04x} does not advertise 100BASE-TX full duplex, so the link \
                will negotiate at 100 half or below no matter what the far end offers; \
                likely cause: MODE[2:0]={:03b} ({}) latched from the RXD0/RXD1/CRS_DV \
                straps (PC4/PC5/PA7 on this board) at PHY reset",
                anar, mode, mode_desc
            );
        }

        STRAP_CONFIG_LOGGED.store(true, Ordering::Relaxed);
    }
}

impl<SM: StationManagement> Phy for Lan8742a<SM> {
    fn phy_reset(&mut self) {
        self.inner.phy_reset();
    }

    fn phy_init(&mut self) {
        self.inner.phy_init();
    }

    fn poll_link(&mut self, cx: &mut Context) -> bool {
        // Delegate first, unconditionally, and use its result unchanged -
        // see this type's doc comment for why the bring-up/link-state path
        // must be untouched by anything below this line.
        let up = self.inner.poll_link(cx);

        if self.addr.is_none() && self.scan_attempts < MAX_ADDR_SCAN_ATTEMPTS {
            self.scan_for_address();
        }

        if let Some(addr) = self.addr {
            self.refresh_diagnostics(addr);
            self.log_strap_config(addr);
        }

        up
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

/// Bring up RMII Ethernet + the embassy-net stack with this board's static
/// IP, and spawn the background task that drives the interface.
///
/// `eth`, `eth_sma`, and the eleven listed GPIOs are consumed here and never
/// given back - identical contract to `firmware/chassis::net::init`.
#[allow(clippy::too_many_arguments)]
pub fn init(
    spawner: Spawner,
    eth: Peri<'static, ETH>,
    eth_sma: Peri<'static, ETH_SMA>,
    rng: Peri<'static, RNG>,
    ref_clk: Peri<'static, embassy_stm32::peripherals::PA1>,
    mdio: Peri<'static, embassy_stm32::peripherals::PA2>,
    crs: Peri<'static, embassy_stm32::peripherals::PA7>,
    mdc: Peri<'static, embassy_stm32::peripherals::PC1>,
    rx_d0: Peri<'static, embassy_stm32::peripherals::PC4>,
    rx_d1: Peri<'static, embassy_stm32::peripherals::PC5>,
    tx_d1: Peri<'static, embassy_stm32::peripherals::PB13>,
    tx_en: Peri<'static, embassy_stm32::peripherals::PG11>,
    tx_d0: Peri<'static, embassy_stm32::peripherals::PG13>,
) -> Stack<'static> {
    // embassy-net wants a random seed for the TCP ISN / DHCP xid generator.
    // We run neither, but the API still asks for one; the chip's true RNG is
    // free and this is the one place it is needed.
    let mut rng = Rng::new(rng, Irqs);
    let mut seed_bytes = [0u8; 8];
    rng.fill_bytes(&mut seed_bytes);
    let seed = u64::from_le_bytes(seed_bytes);

    // Reproduces `Ethernet::new`'s own internal sequence exactly (see this
    // module's doc comment on `Device` and on `Lan8742a`): construct the
    // same `Sma` and the same auto-addressed `GenericPhy` that `new` would
    // have built internally, wrap the latter in `Lan8742a` for read-only
    // diagnostics, and hand it to `new_with_phy` instead of `new`. The MDIO
    // write sequence this produces is byte-for-byte what `new` would have
    // produced on its own.
    let sma = Sma::new(eth_sma, mdio, mdc);
    let phy = Lan8742a::new(GenericPhy::new_auto(sma));

    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let device: Device = Ethernet::new_with_phy(
        PACKETS.init(PacketQueue::<4, 4>::new()),
        eth,
        Irqs,
        ref_clk,
        crs,
        rx_d0,
        rx_d1,
        tx_d0,
        tx_d1,
        tx_en,
        MAC_ADDR,
        phy,
    );

    let net_config = NetConfig::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(SELF_IP, 24),
        gateway: None,
        dns_servers: Default::default(),
    });

    static RESOURCES: StaticCell<StackResources<SOCKET_COUNT>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        net_config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(unwrap!(net_task(runner)));
    stack
}

/// Wait for the link and static-IP configuration to come up, then log it.
pub async fn wait_up(stack: Stack<'static>) {
    stack.wait_config_up().await;
    info!("net: link up, ip={}", SELF_IP.octets());
}

/// The one socket bound to `config::SELF_PORT` — used by the link watchdog to
/// notice inbound traffic from the RPi. Transmit-only tasks use
/// [`tx_socket!`] instead.
pub fn make_rx_socket(stack: Stack<'static>) -> UdpSocket<'static> {
    const BUF_LEN: usize = 512;
    const META_LEN: usize = 8;

    static RX_META: StaticCell<[PacketMetadata; META_LEN]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; BUF_LEN]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; META_LEN]> = StaticCell::new();
    static TX_BUF: StaticCell<[u8; BUF_LEN]> = StaticCell::new();

    let mut socket = UdpSocket::new(
        stack,
        RX_META.init([PacketMetadata::EMPTY; META_LEN]),
        RX_BUF.init([0; BUF_LEN]),
        TX_META.init([PacketMetadata::EMPTY; META_LEN]),
        TX_BUF.init([0; BUF_LEN]),
    );
    unwrap!(socket.bind(crate::config::SELF_PORT));
    socket
}

/// Allocate a transmit-only UDP socket, bound to an ephemeral port. Identical
/// macro to `firmware/chassis::net::tx_socket!` - see its doc comment for why
/// this is a macro (each call site needs its own `StaticCell` storage) rather
/// than a function.
///
/// Each use costs one slot in [`SOCKET_COUNT`] - raise it when you add one.
#[macro_export]
macro_rules! tx_socket {
    ($stack:expr) => {{
        use ::embassy_net::udp::{PacketMetadata, UdpSocket};
        use ::static_cell::StaticCell;

        const BUF_LEN: usize = 128;
        const META_LEN: usize = 4;

        static RX_META: StaticCell<[PacketMetadata; META_LEN]> = StaticCell::new();
        static RX_BUF: StaticCell<[u8; BUF_LEN]> = StaticCell::new();
        static TX_META: StaticCell<[PacketMetadata; META_LEN]> = StaticCell::new();
        static TX_BUF: StaticCell<[u8; BUF_LEN]> = StaticCell::new();

        let mut socket = UdpSocket::new(
            $stack,
            RX_META.init([PacketMetadata::EMPTY; META_LEN]),
            RX_BUF.init([0; BUF_LEN]),
            TX_META.init([PacketMetadata::EMPTY; META_LEN]),
            TX_BUF.init([0; BUF_LEN]),
        );
        ::defmt::unwrap!(socket.bind(0));
        socket
    }};
}

/// RCC configuration for 216 MHz sysclk. Identical to
/// `firmware/chassis::net::clock_config` - same board, same crystal, same
/// Ethernet MAC clocking requirement.
pub fn clock_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: embassy_stm32::time::Hertz(8_000_000),
            mode: HseMode::Bypass,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL216,
            divp: Some(PllPDiv::DIV2),
            divq: None,
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
    }
    config
}
