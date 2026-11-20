//! A body velocity on the floor plane in the robot frame: a twist with no vertical and no
//! tilting part, as the limiter and Nav2's controller read one.

use crate::not_finite::NotFinite;

/// A body velocity in the robot frame: m/s along +X (forward) and +Y (left),
/// rad/s about +Z, counterclockwise seen from above. Every value is finite.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarTwist {
    linear_x: f64,
    linear_y: f64,
    angular_z: f64,
}

impl PlanarTwist {
    /// At rest.
    pub const ZERO: PlanarTwist = PlanarTwist {
        linear_x: 0.0,
        linear_y: 0.0,
        angular_z: 0.0,
    };

    /// The velocity of `linear_x` and `linear_y` in m/s and `angular_z` in
    /// rad/s; a value that is not finite is refused, naming the three.
    pub fn new(linear_x: f64, linear_y: f64, angular_z: f64) -> Result<Self, NotFinite<3>> {
        NotFinite::check(
            "a velocity",
            [
                ("linear_x", linear_x),
                ("linear_y", linear_y),
                ("angular_z", angular_z),
            ],
            "a finite number on each axis, m/s along x and y and rad/s about z",
        )?;
        Ok(Self {
            linear_x,
            linear_y,
            angular_z,
        })
    }

    pub fn linear_x(self) -> f64 {
        self.linear_x
    }

    pub fn linear_y(self) -> f64 {
        self.linear_y
    }

    pub fn angular_z(self) -> f64 {
        self.angular_z
    }

    /// The three axes in the order linear_x, linear_y, angular_z.
    pub fn axes(self) -> [f64; 3] {
        [self.linear_x, self.linear_y, self.angular_z]
    }

    /// Each axis replaced by `change` of its index, in the order of
    /// [`PlanarTwist::axes`], and its value; refused when `change` gives a value
    /// that is not finite, naming the three values it gave.
    pub fn map_axes(self, change: impl Fn(usize, f64) -> f64) -> Result<Self, NotFinite<3>> {
        let [linear_x, linear_y, angular_z] = self.axes();
        Self::new(
            change(0, linear_x),
            change(1, linear_y),
            change(2, angular_z),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_velocity_with_a_number_that_is_not_finite_is_refused_naming_its_axes() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for axes in [[bad, 0.0, 0.0], [0.0, bad, 0.0], [0.0, 0.0, bad]] {
                let [linear_x, linear_y, angular_z] = axes;
                let refused = PlanarTwist::new(linear_x, linear_y, angular_z).unwrap_err();
                assert_eq!(
                    refused.values.map(|(_, value)| value.to_bits()),
                    axes.map(f64::to_bits),
                    "{axes:?}"
                );
                assert_eq!(
                    refused.values.map(|(name, _)| name),
                    ["linear_x", "linear_y", "angular_z"]
                );
            }
        }
        let twist = PlanarTwist::new(0.5, 0.0, -1.0).expect("finite");
        assert_eq!(twist.axes(), [0.5, 0.0, -1.0]);
    }

    // Each axis changes by its index and value, and a change that overflows
    // on any one axis is refused, naming the three values it gave.
    #[test]
    fn a_change_of_the_axes_is_refused_where_it_is_not_finite() {
        let twist = PlanarTwist::new(0.5, -0.25, 2.0).expect("finite");
        let changed = twist
            .map_axes(|axis, value| if axis == 2 { value + 1.0 } else { value * 2.0 })
            .expect("finite");
        assert_eq!(changed.axes(), [1.0, -0.5, 3.0]);
        for overflowing in 0..3 {
            let refused = twist
                .map_axes(|axis, value| {
                    if axis == overflowing {
                        value * 1e308 * 1e308
                    } else {
                        value
                    }
                })
                .unwrap_err();
            let expected: [f64; 3] = std::array::from_fn(|axis| {
                let value = twist.axes()[axis];
                if axis == overflowing {
                    value.signum() * f64::INFINITY
                } else {
                    value
                }
            });
            assert_eq!(
                refused.values.map(|(_, value)| value),
                expected,
                "axis {overflowing}"
            );
        }
    }
}
