//! End-to-end test over real sockets: two `UdpLink`s on localhost, playing
//! the rover and the base station.
//!
//! This is the scenario the whole crate exists for — plan §4: a stream
//! flowing one way (`ImuSample`, standing in for any telemetry-ish signal)
//! and the command protocol flowing the other way, with no shared
//! connection state and no TCP anywhere. Ephemeral ports (`:0`, resolved
//! after bind) mean this can never collide with a real deployment or another
//! test running at the same time.

use rover_bus::{Bus, BusConfig, CommandReceiver, CommandSender};
use rover_link::{PeerId, UdpLink};
use rover_msgs::{
    Command, CommandFrame, GnssFix, HealthBits, ImuSample, MissionStatus, PowerSample, RoverState,
    Telemetry,
};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Only the routes this test exercises. Real `config/rover.toml` has more,
/// but `BusConfig` treats an unrouted type as "no destinations" rather than
/// an error, so a minimal config is a legitimate one, not a stand-in.
const CONFIG: &str = r#"
    [hosts]
    rpi  = "127.0.0.1"
    base = "127.0.0.1"

    [ports]
    rpi  = 7001
    base = 7005

    [routes]
    ImuSample    = ["base"]
    Telemetry    = ["base"]
    CommandFrame = ["rpi"]
"#;

#[test]
fn command_handshake_and_stream_complete_over_real_udp_sockets() {
    let config = BusConfig::parse(CONFIG).expect("test config must parse");

    let mut rover_link = UdpLink::bind(PeerId::Rpi, "127.0.0.1:0", HashMap::new()).unwrap();
    let mut base_link = UdpLink::bind(PeerId::Base, "127.0.0.1:0", HashMap::new()).unwrap();
    let rover_addr = rover_link.local_addr().unwrap();
    let base_addr = base_link.local_addr().unwrap();
    // The config's own [ports] (7001/7005) are never dialled: peer addresses
    // for a `Link` come from its own table, which is exactly what lets this
    // test bind to ephemeral ports instead. See rover-link's own tests for
    // the same pattern in isolation.
    rover_link.set_peer(PeerId::Base, base_addr);
    base_link.set_peer(PeerId::Rpi, rover_addr);

    let mut rover_bus = Bus::new(rover_link, config.clone());
    let mut base_bus = Bus::new(base_link, config);

    // Base: queue a command. It will retransmit this at its configured
    // interval until `Telemetry::last_cmd_seq` echoes it back.
    let mut cmd_tx = CommandSender::new(Duration::from_millis(20));
    cmd_tx.set(Command::SetSpeedLimit(42));

    // Rover: applies whatever command frame shows up, newest-by-cmd_seq-wins.
    let mut cmd_rx = CommandReceiver::new();

    let mut stream_seen = false;
    let mut acked_at = None;

    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        let now = Instant::now();

        // --- base tick -----------------------------------------------
        if let Some(frame) = cmd_tx.poll(now) {
            base_bus.publish(&frame).expect("base -> rpi send");
        }
        base_bus.poll();
        if let Some(telemetry) = base_bus.latest::<Telemetry>() {
            cmd_tx.on_telemetry(telemetry.last_cmd_seq);
        }
        if base_bus.latest::<ImuSample>().is_some() {
            stream_seen = true;
        }

        // --- rover tick ------------------------------------------------
        // The one-way stream: published every tick regardless of the
        // command side, exactly like an IMU sample would be.
        rover_bus
            .publish(&ImuSample {
                accel_mps2: [0.0, 0.0, 9.81],
                gyro_radps: [0.0, 0.0, 0.0],
                t_us: 1,
            })
            .expect("rpi -> base send");

        rover_bus.poll();
        if let Some(frame) = rover_bus.latest::<CommandFrame>() {
            cmd_rx.apply(frame);
        }

        // Telemetry echoes back whatever the rover has applied so far,
        // acknowledging the command once it has actually seen it.
        rover_bus
            .publish(&Telemetry {
                seq: 0,
                t_us: 0,
                state: RoverState::default(),
                mission: MissionStatus::default(),
                power: PowerSample::default(),
                rtk: GnssFix::default(),
                backup: GnssFix::default(),
                last_cmd_seq: cmd_rx.last_applied(),
                health: HealthBits::default(),
            })
            .expect("rpi -> base send");

        if cmd_tx.is_acked() && stream_seen && acked_at.is_none() {
            acked_at = Some(Instant::now());
        }
        // Keep the loop running a little past the ack so the base has a
        // chance to actually observe the stop-retransmitting behaviour, not
        // just the moment it happens.
        if acked_at.is_some_and(|t| Instant::now() > t + Duration::from_millis(50)) {
            break;
        }

        std::thread::sleep(Duration::from_millis(2));
    }

    assert!(
        stream_seen,
        "the one-way ImuSample stream never reached the base"
    );
    assert!(
        cmd_tx.is_acked(),
        "command handshake never completed within the deadline"
    );
    assert_eq!(
        cmd_rx.last_applied(),
        cmd_tx.cmd_seq(),
        "rover applied a different cmd_seq than the base thinks it acked"
    );

    // And the base really did stop retransmitting: one more poll produces
    // nothing to send.
    assert!(cmd_tx
        .poll(Instant::now() + Duration::from_secs(10))
        .is_none());
}
