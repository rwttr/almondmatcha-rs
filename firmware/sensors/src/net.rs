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

/// Number of UDP sockets this board needs: one bound receive socket (the link
/// watchdog's inbound traffic, `config::SELF_PORT`) plus one transmit socket
/// each for `WheelSensors` and `PowerSample` (via [`tx_socket!`]). Raise this
/// when adding another socket - see [`tx_socket!`]'s doc comment.
const SOCKET_COUNT: usize = 3;

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
