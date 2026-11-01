//! Where a ray first meets a box whose faces lie square to the axes, by the
//! slab method.

/// How far along `direction` from `origin` the ray first meets the box from
/// `min` to `max`, in lengths of `direction`; none for a ray that misses the
/// box or starts inside it.
pub fn box_entry(
    origin: [f64; 3],
    direction: [f64; 3],
    min: [f64; 3],
    max: [f64; 3],
) -> Option<f64> {
    let (mut near, mut far) = (0.0_f64, f64::INFINITY);
    for i in 0..3 {
        let (a, b) = (
            (min[i] - origin[i]) / direction[i],
            (max[i] - origin[i]) / direction[i],
        );
        near = near.max(a.min(b));
        far = far.min(a.max(b));
    }
    (near <= far && near > 0.0).then_some(near)
}
