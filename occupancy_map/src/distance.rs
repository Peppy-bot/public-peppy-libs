//! The distance from any point to the nearest occupied voxel of a map: the
//! exact Euclidean distance transform of the grid (Felzenszwalb and
//! Huttenlocher, "Distance Transforms of Sampled Functions", 2012), taken
//! between voxel centers, one axis at a time, and its gradient by central
//! differences, as MoveIt's distance field gives it to CHOMP.

use control_core::positive_finite::PositiveFinite;

use crate::grid::Grid;
use crate::key::Key;
use crate::map::VoxelMap;

/// A snapshot of a map's distances: for each voxel, the distance from its
/// center to the center of the nearest occupied voxel.
pub struct DistanceField {
    grid: Grid,
    /// Squared distances in voxels, in the grid's order; infinite in a map
    /// with nothing occupied.
    squared: Vec<f64>,
}

impl DistanceField {
    /// The distances of `map` as it stands.
    pub fn of(map: &VoxelMap) -> Self {
        let grid = map.grid();
        let dims = grid.dims;
        let mut squared: Vec<f64> = (0..grid.len())
            .map(|index| {
                if map.is_occupied(grid.key_of(index)) {
                    0.0
                } else {
                    f64::INFINITY
                }
            })
            .collect();
        let strides = grid.strides();
        for axis in 0..3 {
            let others: Vec<usize> = (0..3).filter(|&a| a != axis).collect();
            for i in 0..dims[others[0]] {
                for j in 0..dims[others[1]] {
                    let start = i * strides[others[0]] + j * strides[others[1]];
                    let line: Vec<f64> = (0..dims[axis])
                        .map(|k| squared[start + k * strides[axis]])
                        .collect();
                    for (k, value) in transform_line(&line).into_iter().enumerate() {
                        squared[start + k * strides[axis]] = value;
                    }
                }
            }
        }
        Self { grid, squared }
    }

    /// The distance from `point`, in the map's frame, to the center of the
    /// nearest occupied voxel, infinite where nothing is occupied. Inside the
    /// map it is the distance from the center of the point's own voxel,
    /// within half a voxel's diagonal of the point's own distance. Outside
    /// the map it is a lower bound of the point's own distance: its distance
    /// to `p`, the point nearest it in the box of the map's voxel centers,
    /// joined at a right angle to a lower bound of `p`'s distance, that of
    /// `p`'s voxel less `p`'s distance to its center.
    pub fn distance(&self, point: [f64; 3]) -> Result<f64, NotANumber> {
        if point.iter().any(|c| c.is_nan()) {
            return Err(NotANumber(point));
        }
        if let Some(key) = self.grid.key(point) {
            return Ok(self.at(key));
        }
        // Every occupied center lies in the box, so the step from the point to
        // `p` meets the step from `p` to any of them at a right angle or
        // wider, and the squares of the two steps sum to at most the square
        // of the point's distance to that center.
        let (low, high) = (
            self.grid.center(self.grid.lo),
            self.grid.center(self.grid.hi()),
        );
        let projected: [f64; 3] = std::array::from_fn(|i| point[i].clamp(low[i], high[i]));
        let key = self.grid.nearest(projected);
        let from_projected = (self.at(key) - between(projected, self.grid.center(key))).max(0.0);
        Ok((between(point, projected).powi(2) + from_projected.powi(2)).sqrt())
    }

    /// The distance at the center of `point`'s voxel and its gradient there,
    /// every distance first capped at `cap` rounded up to whole voxels, as
    /// MoveIt's `DistanceField::getDistanceGradient` reads a propagation field
    /// whose distances stop at `ceil(cap / resolution)` voxels
    /// (propagation_distance_field.cpp:88): along each axis, the capped
    /// distance at the voxel's neighbor ahead less the capped distance at its
    /// neighbor behind, over twice the voxel size. None outside the map and on
    /// its outermost voxels, which lack a neighbor on one side, where MoveIt
    /// reports the point out of bounds.
    pub fn gradient(
        &self,
        point: [f64; 3],
        cap: PositiveFinite,
    ) -> Result<Option<DistanceGradient>, NotANumber> {
        if point.iter().any(|c| c.is_nan()) {
            return Err(NotANumber(point));
        }
        let Some(key) = self.grid.key(point) else {
            return Ok(None);
        };
        let size = self.grid.size.get();
        let limit = (cap.get() / size).ceil() * size;
        let capped = |key: Key| {
            self.grid
                .index(key)
                .map(|index| self.distance_at(index).min(limit))
        };
        let neighbor = |axis: usize, step: i32| {
            capped(Key(std::array::from_fn(|i| {
                key.0[i] + if i == axis { step } else { 0 }
            })))
        };
        let differences = [0, 1, 2].map(|axis| Some(neighbor(axis, 1)? - neighbor(axis, -1)?));
        let [Some(x), Some(y), Some(z)] = differences else {
            return Ok(None);
        };
        let twice = 2.0 * size;
        Ok(Some(DistanceGradient {
            distance: capped(key).expect("the grid holds the point's voxel"),
            gradient: [x / twice, y / twice, z / twice],
        }))
    }

    /// The distance from the center of voxel `key` of the grid to the center
    /// of the nearest occupied voxel.
    fn at(&self, key: Key) -> f64 {
        self.distance_at(self.grid.index(key).expect("the grid holds the voxel"))
    }

    /// The distance from the center of the voxel at `index` of the grid to the
    /// center of the nearest occupied voxel.
    fn distance_at(&self, index: usize) -> f64 {
        self.squared[index].sqrt() * self.grid.size.get()
    }
}

/// The capped distance at the center of a voxel, and its gradient there, a
/// vector in the map's frame of meters per meter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceGradient {
    pub distance: f64,
    pub gradient: [f64; 3],
}

/// A distance asked at a point that is not a number on every axis.
#[derive(Debug, thiserror::Error, PartialEq)]
#[error(
    "the point {0:?} is not a number on every axis: ask for the distance at a point of numbers"
)]
pub struct NotANumber(pub [f64; 3]);

/// The distance between points `a` and `b`.
fn between(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt()
}

/// The squared distance transform of one line of the grid: for each sample,
/// the least `f(q) + (p - q)^2` over the samples `q`, by the lower envelope
/// of the parabolas rooted at the samples.
fn transform_line(f: &[f64]) -> Vec<f64> {
    let n = f.len();
    let mut out = vec![f64::INFINITY; n];
    let finite: Vec<usize> = (0..n).filter(|&q| f[q].is_finite()).collect();
    if finite.is_empty() {
        return out;
    }
    // The parabolas of the lower envelope, by their roots, and where each
    // takes over from the one before.
    let mut roots: Vec<usize> = Vec::with_capacity(finite.len());
    let mut starts: Vec<f64> = Vec::with_capacity(finite.len() + 1);
    let intersection = |q: usize, r: usize| {
        let (q_f, r_f) = (q as f64, r as f64);
        ((f[q] + q_f * q_f) - (f[r] + r_f * r_f)) / (2.0 * q_f - 2.0 * r_f)
    };
    for &q in &finite {
        while let Some(&r) = roots.last() {
            let s = intersection(q, r);
            if s <= *starts.last().expect("a start per root") {
                roots.pop();
                starts.pop();
            } else {
                break;
            }
        }
        starts.push(match roots.last() {
            Some(&r) => intersection(q, r),
            None => f64::NEG_INFINITY,
        });
        roots.push(q);
    }
    let mut k = 0;
    for (p, value) in out.iter_mut().enumerate() {
        while k + 1 < roots.len() && starts[k + 1] < p as f64 {
            k += 1;
        }
        let r = roots[k];
        let d = p as f64 - r as f64;
        *value = d * d + f[r];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

    // A line's transform is the least squared distance to a finite sample
    // plus that sample's value, by brute force.
    #[hegel::test]
    fn a_line_transform_is_the_least_parabola(tc: TestCase) {
        let f: Vec<f64> = tc
            .draw(
                gs::vecs(gs::integers::<i32>().min_value(-1).max_value(9))
                    .min_size(1)
                    .max_size(40),
            )
            .into_iter()
            .map(|v| if v < 0 { f64::INFINITY } else { f64::from(v) })
            .collect();
        let out = transform_line(&f);
        for (p, value) in out.iter().enumerate() {
            let want = (0..f.len())
                .map(|q| f[q] + (p as f64 - q as f64).powi(2))
                .fold(f64::INFINITY, f64::min);
            assert_eq!(*value, want, "sample {p} of {f:?}");
        }
    }
}
