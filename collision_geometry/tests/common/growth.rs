//! The growth of a primitive collision element by a padding, as MoveIt's robot filter
//! grows its body.

use collision_geometry::chain::Padding;
use collision_geometry::nalgebra::Point3;
use collision_geometry::urdf_collision::{Collision, CollisionGeometry};

/// How far `point`, in the frame of its link, lies outside the primitive `element` grown
/// by `padding` about its origin, along the axis it lies farthest out on, negative
/// inside; `None` for a mesh. A box's half extents, a cylinder's radius and half length,
/// and a sphere's radius each grow to `scale` times their own, plus `offset`.
pub fn grown_primitive_excess(
    element: &Collision,
    padding: Padding,
    point: &Point3<f64>,
) -> Option<f64> {
    let (scale, offset) = (padding.scale(), padding.offset());
    let local = element.origin.inverse() * point;
    match element.geometry {
        CollisionGeometry::Mesh { .. } => None,
        CollisionGeometry::Box { size } => Some(
            (local.coords.abs() - size * (scale / 2.0))
                .add_scalar(-offset)
                .max(),
        ),
        CollisionGeometry::Cylinder { radius, length } => Some(
            (local.z.abs() - (length * scale / 2.0 + offset))
                .max(local.coords.xy().norm() - (radius * scale + offset)),
        ),
        CollisionGeometry::Sphere { radius } => {
            Some(local.coords.norm() - (radius * scale + offset))
        }
    }
}
