//! Ethernet + UDP bring-up.
//!
//! # Hardware verification needed
//!
//! This is the one thing in the whole firmware image that cannot be proven
//! from source alone, per the rewrite plan's §11 risk table and its
//! "hard gate" in §9 step 4: **the Nucleo-F767ZI's PHY is an LAN8742A**, and
//! this code drives it with `embassy_stm32::eth::GenericPhy`, which only
//! assumes standard clause-22 MDIO register behaviour (`BMCR`/`BMSR`/
//! autonegotiation). The LAN8742A is a bog-standard clause-22 part and this
//! is exactly the PHY embassy's own upstream STM32F7 example targets, so
//! this should work - but it has not been run against real silicon as part
//! of this change, and the plan is explicit that this is the first thing to
//! confirm on a bench before anything else here is trusted.
//!
//! The RMII pin assignment below (`PA1/PA2/PA7/PC1/PC4/PC5/PB13/PG11/PG13`)
//! is the standard ST Nucleo-144 RMII wiring, shared by every Nucleo-144
//! board with on-board Ethernet (F429ZI/F767ZI/H743ZI/...) - not something
//! specific to this rover - so it is a much safer bet than the PHY driver
//! choice above.
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Config as NetConfig, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_stm32::eth::{Ethernet, GenericPhy, InterruptHandler as EthInterruptHandler, PacketQueue, Sma};
use embassy_stm32::peripherals::{ETH, ETH_SMA, RNG};
use embassy_stm32::rng::{InterruptHandler as RngInterruptHandler, Rng};
use embassy_stm32::{bind_interrupts, Peri};
use static_cell::StaticCell;

use crate::config::{MAC_ADDR, SELF_IP};

bind_interrupts!(pub struct Irqs {
    ETH => EthInterruptHandler;
    RNG => RngInterruptHandler<RNG>;
});

/// Concrete device type: RMII Ethernet driven by the standard clause-22 PHY
/// driver, over the on-chip station management (SMA/MDIO) block.
pub type Device = Ethernet<'static, ETH, GenericPhy<Sma<'static, ETH_SMA>>>;

/// Number of UDP sockets this board needs. One is enough - everything this
/// board sends or receives is one datagram stream on one bound port
/// (`config::SELF_PORT`); `[routes]` in `config/rover.toml` is unicast
/// fan-out from the RPi's side, not multiple sockets on ours.
const SOCKET_COUNT: usize = 3;

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

/// Bring up RMII Ethernet + the embassy-net stack with this board's static
/// IP, and spawn the background task that drives the interface. Returns the
/// `Stack` handle, which is `Copy` and cheap to pass into every task that
/// needs to open a socket.
///
/// `eth`, `eth_sma`, and the eleven listed GPIOs are consumed here and never
/// given back - Ethernet on this board is not optional or reconfigurable at
/// runtime.
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

    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let device: Device = Ethernet::new(
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
        eth_sma,
        mdio,
        mdc,
    );

    let net_config = NetConfig::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(SELF_IP, 24),
        gateway: None,
        dns_servers: Default::default(),
    });

    static RESOURCES: StaticCell<StackResources<SOCKET_COUNT>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(device, net_config, RESOURCES.init(StackResources::new()), seed);

    spawner.spawn(unwrap!(net_task(runner)));
    stack
}

/// Wait for the link and static-IP configuration to come up, then log it.
/// Since there is no DHCP handshake to wait for, this mostly waits for the
/// PHY to report a carrier - i.e. the Ethernet cable being plugged in.
pub async fn wait_up(stack: Stack<'static>) {
    stack.wait_config_up().await;
    info!("net: link up, ip={}", SELF_IP.octets());
}

/// The one socket bound to `config::SELF_PORT` — inbound commands.
/// Transmit-only tasks use [`tx_socket!`] instead.
///
/// Allocate one UDP socket bound to `config::SELF_PORT`, backed by
/// `'static` buffers (required because embassy-net sockets borrow their
/// packet buffers for their whole lifetime, and this socket lives for the
/// life of the program).
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

/// Allocate a transmit-only UDP socket, bound to an ephemeral port.
///
/// A macro rather than a function because each socket needs its own
/// `StaticCell` buffers: a function would hold exactly one set of statics
/// and panic on `StaticCell::init` the second time it was called — at boot,
/// on hardware, with no obvious cause. Expanding at the call site gives
/// every caller its own storage, so adding a transmit task is a one-line
/// change that cannot collide with an existing one.
///
/// `embassy_net::UdpSocket` is not `Sync`, so tasks cannot share one without
/// a mutex on the path that would least tolerate waiting. Port 0 lets the
/// stack choose: nothing ever originates a datagram *to* these sockets, so
/// their port numbers are irrelevant, and a fixed one would be another
/// value to keep in step with `config/rover.toml` for no benefit.
///
/// Each use costs one slot in [`SOCKET_COUNT`] — raise it when you add one.
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

/// RCC configuration for 216 MHz sysclk from the Nucleo's 8 MHz HSE (fed by
/// the on-board ST-Link's MCO, same as every Nucleo-144 board).
///
/// Copied from embassy's own `examples/stm32f7/src/bin/eth.rs` essentially
/// unchanged: `8 MHz / 4 * 216 / 2 = 216 MHz`. 216 MHz is the documented
/// maximum sysclk for the F76x/F77x line, and matches what the Ethernet MAC
/// needs for its 50 MHz RMII reference divide to come out exact.
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
