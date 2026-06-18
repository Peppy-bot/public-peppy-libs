//! OpenArm v10 deployment geometry shared by the integration test and the
//! visualizer (included via `#[path]` from both, so it is one source of truth).
//!
//! The torso (`openarm_body_link0`) is concave: a wide base plate, a flared
//! shoulder mount, a rear brace, a thin central column, and a wide head block.
//! Its auto-fit single hull bridges those into one bulging solid that clips the
//! grippers at rest. These five axis-aligned boxes, verified at build to contain
//! the whole torso mesh surface, replace it with a tight conservative proxy.
//! Supply them via [`Builder::hulls`] for the torso body.
//!
//! Each piece bounds one feature of the collision mesh (`body_link0_symp.stl`),
//! found by reading the mesh's faces (not just its vertex extents: a coarse
//! simplified mesh has large faces, including a rear brace that slopes from the
//! base pad up to the column, which a per-z-band box set would leak).
//!
//! ```text
//!  plate        box, the flat base, z [0.00, 0.03], full footprint
//!  flare        frustum, shoulder mount tapering y +/-0.085 -> column width
//!  rear brace   box, the diagonal support, x [-0.156, -0.029], z [0.01, 0.23]
//!  column       box, the thin central beam, x/y +/-0.032, z [0.06, 0.60]
//!  head         box, the wide top block, z [0.60, 0.78]
//! ```
//!
//! The flare is a frustum, not a box: the mesh flares to y +/-0.08 at its base
//! and necks back to the column by z 0.08, so a rectangular box would claim a
//! phantom slab up top that reads false proximity against a gripper at rest. The
//! plate's top rises to z 0.026 to enclose the flare's widest ring.

use bimanual_collision_model::ConvexPiece;
use bimanual_collision_model::Point3;

/// The collision body whose pieces replace its auto-fit hull.
pub const TORSO_BODY: &str = "openarm_body_link0";

/// Base plate, shoulder flare, rear brace, central column, and head block
/// bounding the OpenArm v10 torso mesh in the root frame.
pub fn torso() -> Vec<ConvexPiece> {
    vec![
        ConvexPiece::aabb(Point3::new(-0.157, -0.097, -0.002), Point3::new(0.097, 0.097, 0.026)),
        flare(),
        ConvexPiece::aabb(Point3::new(-0.156, -0.034, 0.006), Point3::new(-0.029, 0.034, 0.226)),
        ConvexPiece::aabb(Point3::new(-0.032, -0.032, 0.058), Point3::new(0.032, 0.032, 0.604)),
        ConvexPiece::aabb(Point3::new(-0.087, -0.082, 0.598), Point3::new(0.067, 0.082, 0.775)),
    ]
}

/// The shoulder flare as a frustum: a wide bottom ring at the flare's full width
/// and a narrow top ring at the column width, eight corners hulled at build.
fn flare() -> ConvexPiece {
    let ring = |y: f64, z: f64| {
        [
            Point3::new(-0.033, -y, z),
            Point3::new(0.033, -y, z),
            Point3::new(0.033, y, z),
            Point3::new(-0.033, y, z),
        ]
    };
    ConvexPiece::from_points(ring(0.085, 0.016).into_iter().chain(ring(0.034, 0.080)).collect())
}
