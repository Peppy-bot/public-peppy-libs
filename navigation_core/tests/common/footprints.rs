//! The padded footprints the suites build, and where a point lies against one.

use control_core::footprint::Footprint;
use navigation_core::PaddedFootprint;

use super::finite::non_negative;

/// The footprint of `points`, about the base's center, padded by `padding_m`.
pub fn padded(points: &[[f64; 2]], padding_m: f64) -> PaddedFootprint {
    let footprint = Footprint::parse(points).expect("a footprint around the base's center");
    PaddedFootprint::new(&footprint, non_negative(padding_m)).expect("a padded footprint")
}

/// Whether `point` lies inside the convex polygon `polygon`, counterclockwise, by more
/// than `margin` from each edge's line; a `margin` below 0 reaches that far past them.
pub fn inside_by(polygon: &[[f64; 2]], point: [f64; 2], margin: f64) -> bool {
    (0..polygon.len()).all(|i| {
        let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
        let cross = (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0]);
        cross / (b[0] - a[0]).hypot(b[1] - a[1]) > margin
    })
}
