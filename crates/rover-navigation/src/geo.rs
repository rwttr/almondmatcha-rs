//! Great-circle distance, in metres, always.
//!
//! `gnss_mission_monitor_node.cpp`'s `haversine_distance` returned
//! **kilometres**, and `DESTINATION_THRESHOLD_KM` compared against it in
//! kilometres too — but `docs/CSV_LOGGING.md`'s `mission_state.csv` schema
//! documents `Distance_Remaining_m` in **metres** for the exact same
//! quantity relayed from the same node. One code path, two units, depending
//! on which downstream file you read the number from. `rover_msgs::MissionStatus`
//! fixes this by being metres unconditionally, so this function only ever
//! returns metres and nothing here should ever get multiplied or divided by
//! 1000 again.

/// Mean Earth radius, metres (IUGG value). The ROS 2 node used the same
/// constant in kilometres (`6371.0`).
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Great-circle (haversine) distance between two WGS84 points, in metres.
///
/// Haversine over a sphere, not a full geodesic over the WGS84 ellipsoid —
/// exactly what the ROS 2 node used. The error from ignoring ellipsoidal
/// flattening is well under 0.5% at any distance this rover will ever drive,
/// which is negligible next to the GNSS accuracy itself.
pub fn haversine_distance_m(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) -> f64 {
    let phi1 = lat1_deg.to_radians();
    let phi2 = lat2_deg.to_radians();
    let dphi = (lat2_deg - lat1_deg).to_radians();
    let dlambda = (lon2_deg - lon1_deg).to_radians();

    let a = (dphi / 2.0).sin().powi(2) + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    EARTH_RADIUS_M * c
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reference values computed independently in Python with the same
    /// spherical-haversine formula and `EARTH_RADIUS_M = 6_371_000.0`, so
    /// these check the formula's correctness (algebra, trig, unit
    /// conversion), not just self-consistency with this file.
    #[test]
    fn matches_an_independently_computed_reference_a() {
        // Nashville, TN -> Los Angeles, CA. A commonly cited haversine test
        // pair (movable-type.co.uk-style), computed here in Python.
        let d = haversine_distance_m(36.12, -86.67, 33.94, -118.40);
        assert!((d - 2_886_444.44).abs() < 1.0, "got {d}");
    }

    #[test]
    fn matches_an_independently_computed_reference_b() {
        // One degree of longitude at the equator.
        let d = haversine_distance_m(0.0, 0.0, 0.0, 1.0);
        assert!((d - 111_194.93).abs() < 1.0, "got {d}");
    }

    #[test]
    fn same_point_is_zero() {
        let d = haversine_distance_m(13.0, 100.0, 13.0, 100.0);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn small_offset_is_meaningful_at_arrival_radius_scale() {
        // ~2 m apart -- the scale mission arrival-radius checks operate at.
        // Independently computed reference: 2.0015086794867805 m.
        let d = haversine_distance_m(13.7367, 100.5232, 13.736718, 100.5232);
        assert!((d - 2.0015).abs() < 1e-3, "got {d}");
    }

    #[test]
    fn is_symmetric() {
        let a = haversine_distance_m(13.0, 100.0, 14.0, 101.0);
        let b = haversine_distance_m(14.0, 101.0, 13.0, 100.0);
        assert!((a - b).abs() < 1e-9);
    }

    #[test]
    fn antipodal_points_are_half_the_circumference() {
        let d = haversine_distance_m(0.0, 0.0, 0.0, 180.0);
        let half_circumference = std::f64::consts::PI * EARTH_RADIUS_M;
        assert!((d - half_circumference).abs() < 1.0);
    }
}
