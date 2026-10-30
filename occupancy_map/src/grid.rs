//! The bounded grid of voxels a map holds: its voxel size, the key of its
//! lowest voxel and its extent, and where each voxel lies in its storage, x
//! slowest and z fastest.

use crate::key::{Key, VoxelSize};

#[derive(Debug, Clone, Copy)]
pub(crate) struct Grid {
    pub size: VoxelSize,
    /// The key of the voxel at the grid's lowest corner.
    pub lo: Key,
    pub dims: [usize; 3],
}

impl Grid {
    pub fn len(&self) -> usize {
        self.dims.iter().product()
    }

    /// Where voxel `key` lies in the grid's storage, when the grid holds it.
    pub fn index(&self, key: Key) -> Option<usize> {
        let mut index = 0;
        for i in 0..3 {
            let offset = i64::from(key.0[i]) - i64::from(self.lo.0[i]);
            if offset < 0 || offset >= self.dims[i] as i64 {
                return None;
            }
            index = index * self.dims[i] + offset as usize;
        }
        Some(index)
    }

    /// The key of the voxel at storage index `index`.
    pub fn key_of(&self, index: usize) -> Key {
        let z = index % self.dims[2];
        let y = (index / self.dims[2]) % self.dims[1];
        let x = index / (self.dims[2] * self.dims[1]);
        Key([
            self.lo.0[0] + x as i32,
            self.lo.0[1] + y as i32,
            self.lo.0[2] + z as i32,
        ])
    }

    /// The voxel holding `point`, when the grid holds it.
    pub fn key(&self, point: [f64; 3]) -> Option<Key> {
        self.size
            .key(point)
            .filter(|key| self.index(*key).is_some())
    }

    /// The grid's voxel nearest `point`: the one holding it, or the one on
    /// the grid's boundary each axis is held to.
    pub fn nearest(&self, point: [f64; 3]) -> Key {
        Key(std::array::from_fn(|i| {
            self.held(i, self.size.axis_index(point[i]))
        }))
    }

    /// The index `index` of a voxel on axis `axis`, held to the grid's extent on it;
    /// the grid's lowest for a NaN.
    fn held(&self, axis: usize, index: f64) -> i32 {
        if index.is_nan() {
            return self.lo.0[axis];
        }
        index.clamp(f64::from(self.lo.0[axis]), f64::from(self.hi().0[axis])) as i32
    }

    /// Every voxel of the grid whose box meets the box from `min` to `max`, faces
    /// included, x slowest and z fastest.
    pub fn keys_between(&self, min: [f64; 3], max: [f64; 3]) -> impl Iterator<Item = Key> + use<> {
        let within = self.within(min, max);
        (0..within.len()).map(move |index| within.key_of(index))
    }

    /// The voxels of the grid whose boxes meet the box from `min` to `max`, faces
    /// included, as a grid of their own: on each axis from the voxel whose upper face
    /// reaches `min` to the one whose lower face reaches `max`.
    pub fn within(&self, min: [f64; 3], max: [f64; 3]) -> Grid {
        let size = self.size.get();
        let lo = Key(std::array::from_fn(|i| {
            self.held(i, (min[i] / size).ceil() - 1.0)
        }));
        let hi = Key(std::array::from_fn(|i| {
            self.held(i, (max[i] / size).floor())
        }));
        Grid {
            size: self.size,
            lo,
            dims: std::array::from_fn(|i| (hi.0[i] - lo.0[i] + 1).max(0) as usize),
        }
    }

    /// The storage indices of the grid's voxels that share a face with the voxel at
    /// storage index `index`.
    pub fn face_neighbors(&self, index: usize) -> impl Iterator<Item = usize> + use<> {
        let grid = *self;
        let key = grid.key_of(index);
        (0..3)
            .flat_map(|axis| [(axis, -1), (axis, 1)])
            .filter_map(move |(axis, step)| {
                let mut neighbor = key;
                neighbor.0[axis] += step;
                grid.index(neighbor)
            })
    }

    /// The center of voxel `key`.
    pub fn center(&self, key: Key) -> [f64; 3] {
        self.size.center(key)
    }

    /// The key of the voxel at the grid's highest corner.
    pub fn hi(&self) -> Key {
        Key(std::array::from_fn(|i| {
            self.lo.0[i] + (self.dims[i] - 1) as i32
        }))
    }

    /// The storage strides of the three axes.
    pub fn strides(&self) -> [usize; 3] {
        [self.dims[1] * self.dims[2], self.dims[2], 1]
    }
}
