//! Shared scaffolding for the GJK eval binaries: fit one convex hull per
//! collision body (mirroring the capsule bodies in assemble.rs) and place them
//! by forward kinematics. Lives beside the binaries, not in the library, so it
//! drops with the eval. Included by each bin with `#[path]`.
#![allow(dead_code)]

use std::collections::{HashMap, HashSet};

use bimanual_collision_model::hull::{ConvexHull, decompose};
use bimanual_collision_model::nalgebra::{Isometry3, Point3};
use bimanual_collision_model::urdf_collision::UrdfCollisions;
use srs_model::{ARM_DOF, Arm, JointVec};

pub const URDF: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/openarm_v10.urdf");
pub const MESHES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/meshes");
pub const FIXED: [&str; 3] = ["openarm_body_link0", "openarm_left_link0", "openarm_right_link0"];

/// Grid cell for hull simplification: points are welded onto this grid before
/// hulling, shrinking both the cloud (fast construction) and the hull (cheap
/// support), recovered by each hull's inflation radius.
pub const SIMPLIFY_TOL: f64 = 0.004;

/// Decomposition budget: up to this many convex pieces per body, taken only
/// while a split still cuts total volume by at least `MIN_GAIN` of the body.
pub const MAX_PIECES: usize = 5;
pub const MIN_GAIN: f64 = 0.04;

/// Moving-link names per arm, from walking each chain at the zero pose.
pub fn chains() -> Vec<Vec<String>> {
    [FIXED[1], FIXED[2]]
        .iter()
        .map(|base| {
            let mut arm = Arm::from_urdf_file(URDF, base).expect("arm from urdf");
            let posed = arm.at(&[0.0; ARM_DOF]);
            (0..ARM_DOF).map(|i| posed.link_name(i)).collect()
        })
        .collect()
}

/// The local triangle soup of every collision body: fixed bodies in root frame,
/// moving links in link frame with attached fingers baked over their full
/// travel (the same bodies assemble.rs fits capsules to).
pub fn body_vertices() -> HashMap<String, Vec<Point3<f64>>> {
    let urdf = UrdfCollisions::from_file(URDF).expect("urdf");
    let chains = chains();
    let chain_set: HashSet<String> = chains.iter().flatten().cloned().chain(FIXED.iter().map(|s| s.to_string())).collect();

    let mut bodies = HashMap::new();
    for name in FIXED {
        bodies.insert(name.to_string(), urdf.fixed_vertices_in_root(name, MESHES).expect("fixed vertices"));
    }
    for name in chains.iter().flatten() {
        let mut v = urdf.link_vertices(name, MESHES).expect("link vertices");
        for child in urdf.children_of(name) {
            if chain_set.contains(&child) || urdf.collisions_of(&child).is_empty() {
                continue;
            }
            let j = urdf.parent_joint(&child).expect("child joint");
            v.extend(urdf.child_vertices_in_parent(&child, j.lower_limit, MESHES).expect("child lo"));
            if !j.is_fixed() {
                v.extend(urdf.child_vertices_in_parent(&child, j.upper_limit, MESHES).expect("child hi"));
            }
        }
        bodies.insert(name.clone(), v);
    }
    bodies
}

/// One decomposition (up to `MAX_PIECES` simplified hulls) per collision body.
pub fn fit_hulls() -> HashMap<String, Vec<(ConvexHull, f64)>> {
    body_vertices().iter().map(|(k, v)| (k.clone(), decompose(v, SIMPLIFY_TOL, MAX_PIECES, MIN_GAIN))).collect()
}

/// World placement of every body at a configuration.
pub fn body_isometries(ql: &JointVec, qr: &JointVec) -> HashMap<String, Isometry3<f64>> {
    let mut iso: HashMap<String, Isometry3<f64>> = FIXED.iter().map(|n| (n.to_string(), Isometry3::identity())).collect();
    for (base, q) in [(FIXED[1], ql), (FIXED[2], qr)] {
        let mut arm = Arm::from_urdf_file(URDF, base).expect("arm from urdf");
        let posed = arm.at(q);
        for i in 0..ARM_DOF {
            iso.insert(posed.link_name(i), posed.link_pose_world(i));
        }
    }
    iso
}

/// Side bucket for colouring: left, right, or fixed.
pub fn side(name: &str) -> &'static str {
    if name.contains("_left_") {
        "left"
    } else if name.contains("_right_") {
        "right"
    } else {
        "fixed"
    }
}
