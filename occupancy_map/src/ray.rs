//! The voxels a ray crosses, as OctoMap's `computeRayKeys` walks them: the
//! 3D digital differential analyzer of Amanatides and Woo ("A Fast Voxel
//! Traversal Algorithm for Ray Tracing", 1987), in OctoMap's own mix of
//! single and double precision (octomap 1.10), so a ray crosses the voxels
//! OctoMap's crosses, bit for bit.

use crate::key::{Key, VoxelSize};

/// The voxels a ray from `origin` to `end` crosses: the origin's voxel, then
/// every voxel the ray enters, up to and not including the end's voxel, or
/// the voxel where the ray has run its length by the walk's own sums. Empty
/// where both lie in one voxel; `None` where either lies outside the tree.
pub(crate) fn ray_keys(size: VoxelSize, origin: [f32; 3], end: [f32; 3]) -> Option<Vec<Key>> {
    let key_origin = size.key(origin.map(f64::from))?;
    let key_end = size.key(end.map(f64::from))?;
    if key_origin == key_end {
        return Some(Vec::new());
    }
    let mut ray = vec![key_origin];
    let direction: [f32; 3] = std::array::from_fn(|i| end[i] - origin[i]);
    // octomath's norm: the squared norm summed in single precision, its
    // square root taken in double, the length kept in single.
    let norm_sq =
        direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2];
    let length = f64::from(norm_sq).sqrt() as f32;
    // A squared norm that underflows leaves no direction to walk: the ray has
    // run its length in its origin's voxel. OctoMap's walk has no exit here.
    if length == 0.0 {
        return Some(ray);
    }
    let direction = direction.map(|d| d / length);
    let mut step = [0_i32; 3];
    let mut t_max = [f64::MAX; 3];
    let mut t_delta = [f64::MAX; 3];
    let mut current = key_origin;
    for i in 0..3 {
        step[i] = if direction[i] > 0.0 {
            1
        } else if direction[i] < 0.0 {
            -1
        } else {
            0
        };
        if step[i] != 0 {
            // The corner of the voxel the ray leaves through on this axis.
            let mut border = size.axis_center(current.0[i]);
            border += f64::from((f64::from(step[i]) * size.get() * 0.5) as f32);
            t_max[i] = (border - f64::from(origin[i])) / f64::from(direction[i]);
            t_delta[i] = size.get() / f64::from(direction[i].abs());
        }
    }
    loop {
        let dim = if t_max[0] < t_max[1] {
            if t_max[0] < t_max[2] { 0 } else { 2 }
        } else if t_max[1] < t_max[2] {
            1
        } else {
            2
        };
        current.0[dim] += step[dim];
        t_max[dim] += t_delta[dim];
        if current == key_end {
            return Some(ray);
        }
        // How far the ray has run to the border of the voxel it is in: past
        // its length, it missed the end's voxel by the sums' rounding.
        let run = min(min(t_max[0], t_max[1]), t_max[2]);
        if run > f64::from(length) {
            return Some(ray);
        }
        ray.push(current);
    }
}

/// `std::min`: the second when it is less than the first, else the first.
fn min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

#[cfg(test)]
mod tests {
    use super::*;
    use control_core::positive_finite::PositiveFinite;
    use hegel::TestCase;
    use hegel::generators as gs;

    fn size() -> VoxelSize {
        VoxelSize::new(PositiveFinite::parse(0.05).expect("positive"))
    }

    fn point(tc: &TestCase) -> [f32; 3] {
        tc.draw(gs::arrays(
            gs::floats::<f32>().min_value(-2.0).max_value(2.0),
        ))
    }

    // A ray starts in the origin's voxel, each voxel after it shares a face
    // with the one before, no voxel comes twice, and it stops before the
    // end's voxel, a face away from it or short of it by the rounding of the
    // walk.
    #[hegel::test]
    fn a_ray_steps_face_to_face_from_the_origin_toward_the_end(tc: TestCase) {
        let (origin, end) = (point(&tc), point(&tc));
        let size = size();
        let ray = ray_keys(size, origin, end).expect("in range");
        let key_end = size.key(end.map(f64::from)).expect("in range");
        let key_origin = size.key(origin.map(f64::from)).expect("in range");
        if key_origin == key_end {
            assert!(ray.is_empty());
            return;
        }
        assert_eq!(ray[0], key_origin);
        for w in ray.windows(2) {
            let steps: i32 = (0..3).map(|i| (w[1].0[i] - w[0].0[i]).abs()).sum();
            assert_eq!(steps, 1, "{:?} to {:?}", w[0], w[1]);
        }
        let mut seen = ray.clone();
        seen.sort();
        seen.dedup();
        assert_eq!(seen.len(), ray.len(), "a voxel comes twice");
        assert!(!ray.contains(&key_end));
        let last = ray[ray.len() - 1];
        let to_end: i32 = (0..3).map(|i| (key_end.0[i] - last.0[i]).abs()).sum();
        let span: i32 = (0..3).map(|i| (key_end.0[i] - key_origin.0[i]).abs()).sum();
        assert!(
            to_end <= span,
            "the ray ends farther from the end than it began"
        );
    }

    // A ray along an axis crosses every voxel between its ends' voxels.
    #[test]
    fn a_ray_along_an_axis_crosses_every_voxel_between() {
        let ray = ray_keys(size(), [0.01, 0.01, 0.01], [0.26, 0.01, 0.01]).expect("in range");
        assert_eq!(
            ray,
            (0..5).map(|x| Key([x, 0, 0])).collect::<Vec<_>>(),
            "the end's voxel, 5, is left out"
        );
    }

    // A ray whose squared norm underflows to 0, its ends either side of zero
    // on two axes, ends at its origin's voxel.
    #[test]
    fn a_ray_of_a_length_that_rounds_to_zero_ends_at_its_origin() {
        let ray = ray_keys(size(), [-1e-30, -1e-30, 0.0], [1e-30, 1e-30, 0.0]).expect("in range");
        assert_eq!(ray, [Key([-1, -1, 0])]);
    }
}
