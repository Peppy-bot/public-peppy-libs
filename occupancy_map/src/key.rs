//! Where a point lies in the voxel grid, as OctoMap keys it: per axis the
//! index of the voxel, the floor of the coordinate times the reciprocal of
//! the voxel size, so the voxels lie at multiples of the voxel size, and the
//! voxel's center half a voxel above its floor.

use control_core::positive_finite::PositiveFinite;

/// The voxel a point lies in: one index per axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Key(pub [i32; 3]);

/// The keys an OctoMap tree of 16 levels holds on each side of each axis, the
/// tree MoveIt's occupancy map is: from -32768 up to, and not including, 32768.
pub(crate) const TREE_VOXELS: i32 = 32768;

/// The edge of a voxel, in meters.
#[derive(Clone, Copy, Debug)]
pub(crate) struct VoxelSize {
    size: f64,
    /// The reciprocal OctoMap multiplies a coordinate by.
    factor: f64,
}

impl VoxelSize {
    pub(crate) fn new(size: PositiveFinite) -> Self {
        Self {
            size: size.get(),
            factor: 1.0 / size.get(),
        }
    }

    pub(crate) fn get(self) -> f64 {
        self.size
    }

    /// How far the tree's voxels reach from the origin on each side of each
    /// axis, in meters.
    pub(crate) fn tree_reach_m(self) -> f64 {
        f64::from(TREE_VOXELS) * self.size
    }

    /// The index of the voxel holding `coordinate` on one axis, whether or not
    /// the tree reaches it: the floor of the coordinate times the reciprocal
    /// of the voxel size, NaN for a coordinate that is not a number.
    pub(crate) fn axis_index(self, coordinate: f64) -> f64 {
        (self.factor * coordinate).floor()
    }

    /// The index of the voxel holding `coordinate` on one axis, when it lies
    /// in the tree's range (OctoMap's `coordToKeyChecked`).
    fn axis_key(self, coordinate: f64) -> Option<i32> {
        let index = self.axis_index(coordinate);
        let half_width = f64::from(TREE_VOXELS);
        (-half_width..half_width)
            .contains(&index)
            .then_some(index as i32)
    }

    /// The voxel holding `point`, when it lies in the tree's range.
    pub(crate) fn key(self, point: [f64; 3]) -> Option<Key> {
        Some(Key([
            self.axis_key(point[0])?,
            self.axis_key(point[1])?,
            self.axis_key(point[2])?,
        ]))
    }

    /// The center of voxel `index` on one axis (OctoMap's `keyToCoord`).
    pub(crate) fn axis_center(self, index: i32) -> f64 {
        (f64::from(index) + 0.5) * self.size
    }

    /// The center of the voxel `key`.
    pub(crate) fn center(self, key: Key) -> [f64; 3] {
        key.0.map(|index| self.axis_center(index))
    }

    /// The center of the voxel `key` in single precision, as OctoMap hands a
    /// voxel's center to a ray.
    pub(crate) fn center_f32(self, key: Key) -> [f32; 3] {
        self.center(key).map(|c| c as f32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    fn size(tc: &TestCase) -> VoxelSize {
        VoxelSize::new(
            PositiveFinite::parse(tc.draw(gs::floats().min_value(0.005).max_value(0.2)))
                .expect("drawn positive"),
        )
    }

    // A voxel holds the points from its floor up to its next floor, so its
    // center lies inside it, half a voxel above its floor.
    #[hegel::test]
    fn a_voxel_holds_its_own_center(tc: TestCase) {
        let size = size(&tc);
        let point: [f64; 3] = tc.draw(gs::arrays(gs::floats().min_value(-5.0).max_value(5.0)));
        let key = size.key(point).expect("in range");
        assert_eq!(size.key(size.center(key)), Some(key));
        for (coordinate, index) in point.into_iter().zip(key.0) {
            let floor = f64::from(index) * size.get();
            assert!(coordinate >= floor - 1e-9 * size.get());
            assert!(coordinate < floor + size.get() * (1.0 + 1e-9));
        }
    }

    #[test]
    fn a_point_beyond_the_tree_has_no_voxel() {
        let size = VoxelSize::new(PositiveFinite::parse(0.01).expect("positive"));
        assert_eq!(size.key([327.675, 0.0, 0.0]).map(|k| k.0[0]), Some(32767));
        assert_eq!(size.key([327.685, 0.0, 0.0]), None);
        assert_eq!(size.key([-327.675, 0.0, 0.0]).map(|k| k.0[0]), Some(-32768));
        assert_eq!(size.key([-327.685, 0.0, 0.0]), None);
        assert_eq!(size.key([f64::NAN, 0.0, 0.0]), None);
    }
}
