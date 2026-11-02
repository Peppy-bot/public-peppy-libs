//! A ball for a scan to cut out, as MoveIt's robot filter holds a sphere body.

use occupancy_map::{Cutout, Frame};

/// A ball in `frame`: a point lies inside when it is nearer the center than
/// the radius, the strict test of the robot filter's bounding sphere.
pub struct Ball {
    pub frame: Frame,
    pub center: [f64; 3],
    pub radius: f64,
}

impl Cutout for Ball {
    fn frame(&self) -> &Frame {
        &self.frame
    }

    fn contains(&self, point: [f64; 3]) -> bool {
        (0..3)
            .map(|i| (point[i] - self.center[i]).powi(2))
            .sum::<f64>()
            < self.radius * self.radius
    }
}
