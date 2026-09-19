//! Runtime dispatch from a wire `type_id` to a decoder and a name.
//!
//! Every other crate in the workspace knows which `Wire` type it wants at
//! compile time (`Bus::publish::<ImuSample>`, `Bus::latest::<RoverState>`,
//! ...) and never needs this. `rover-tap` is the one place that has to go
//! the other way — a raw frame off the wire, with only a `u16` to say what it
//! is — because a sniffer that only understood one hard-coded message type
//! would not replace `ros2 topic echo`.
//!
//! This list is hand-kept in step with `rover_msgs::types` (the same way
//! `rover-msgs/tests/roundtrip.rs` keeps its own `all_types!` list in step);
//! there is no way to enumerate `impl Wire` at runtime in Rust, so a new
//! message type needs one line added here to show up in `rover-tap`.

use rover_msgs::*;

/// A decoded message, ready to print, plus the wire name it came from.
pub struct Decoded {
    pub name: &'static str,
    pub rendered: String,
}

/// Decode `body` as whatever `type_id` says it is, for display.
///
/// `None` covers two cases `rover-tap` treats identically: an unrecognised
/// `type_id` (a message type this build doesn't know about) and a `type_id`
/// this build *does* know but whose body failed to decode (a truncated or
/// corrupt frame). Either way there is nothing sensible to print.
pub fn decode(type_id: u16, body: &[u8]) -> Option<Decoded> {
    macro_rules! try_type {
        ($t:ty) => {
            if type_id == <$t as Wire>::TYPE_ID {
                return <$t as Wire>::decode(body).ok().map(|v| Decoded {
                    name: <$t as Wire>::NAME,
                    rendered: format!("{v:?}"),
                });
            }
        };
    }
    try_type!(ImuSample);
    try_type!(MagSample);
    try_type!(ChassisStatus);
    try_type!(ChassisCommand);
    try_type!(WheelSensors);
    try_type!(PowerSample);
    try_type!(GnssFix);
    try_type!(LaneMeasurement);
    try_type!(RoverState);
    try_type!(MotionSetpoint);
    try_type!(MissionGoal);
    try_type!(MissionStatus);
    try_type!(CommandFrame);
    try_type!(Telemetry);
    try_type!(SpeedLoopDebug);
    try_type!(EkfDebug);
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_a_known_type() {
        let msg = PowerSample {
            bus_volts: 12.6,
            current_amps: 3.25,
        };
        let mut buf = [0u8; PowerSample::WIRE_LEN];
        msg.encode(&mut buf);
        let decoded = decode(PowerSample::TYPE_ID, &buf).expect("PowerSample must be registered");
        assert_eq!(decoded.name, "PowerSample");
        assert!(decoded.rendered.contains("12.6"));
    }

    #[test]
    fn unknown_type_id_decodes_to_none() {
        assert!(decode(0xFFFF, &[0u8; 64]).is_none());
    }

    #[test]
    fn truncated_body_decodes_to_none() {
        // PowerSample needs 8 bytes; give it 2.
        assert!(decode(PowerSample::TYPE_ID, &[0u8; 2]).is_none());
    }
}
