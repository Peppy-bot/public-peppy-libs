//! Fit the collision bodies for a bimanual model directly from the URDF and
//! its meshes, at construction time. There is no intermediate artifact: the
//! hulls can never go stale against the geometry.

use std::collections::{HashMap, HashSet};

use srs_model::nalgebra::Point3;

use crate::gjk::Hull;
use crate::hull::decompose;
use crate::urdf_collision::UrdfCollisions;

/// Grid cell for hull simplification: points are welded onto this grid before
/// hulling, recovered by each hull's inflation radius, so a 68k-vertex link
/// builds fast and its support stays cheap.
const SIMPLIFY_CELL: f64 = 0.004;
/// Up to this many convex pieces per body, taken only while a split still cuts
/// at least `MIN_GAIN` cubic metres off the body (absolute, so only a large
/// concave body like the torso splits; small ones stay one hull).
const MAX_PIECES: usize = 5;
const MIN_GAIN: f64 = 0.001;

/// Convex-hull pieces for every collision body: world-fixed bodies (in root
/// frame) and chain links (in link frame, with attached collision-bearing
/// children baked in across their travel). Each body decomposes into one or
/// more convex hulls; only a large concave body (the torso) takes more than one.
pub(crate) struct FittedBodies {
    pub fixed: Vec<(String, Vec<Hull>)>,
    pub links: HashMap<String, Vec<Hull>>,
}

/// Fit all collision bodies. `chains` are the moving-link names per arm (from
/// FK); every other collision-bearing link must be world-fixed or an attached
/// child of a chain link, anything else is an error rather than a silently
/// unmodeled body.
pub(crate) fn fit_bodies(urdf: &UrdfCollisions, chains: &[Vec<String>], meshes_dir: &str) -> Result<FittedBodies, String> {
    let chain_set: HashSet<&str> = chains.iter().flatten().map(String::as_str).collect();
    let fit = |verts: &[Point3<f64>]| -> Result<Vec<Hull>, String> {
        decompose(verts, SIMPLIFY_CELL, MAX_PIECES, MIN_GAIN)?.iter().map(|rh| Hull::new(&rh.hull, rh.radius)).collect()
    };

    let mut links = HashMap::new();
    let mut attached: HashSet<String> = HashSet::new();
    for name in chains.iter().flatten() {
        // The link mesh plus any attached collision-bearing child (a gripper
        // finger) over its full travel, baked into one vertex cloud: travel is
        // a joint-space line, so the extremes bound every intermediate pose.
        // Only direct children are baked; a deeper collision-bearing descendant
        // (a multi-link end-effector) is not, and falls through to the fixed-body
        // pass below, which errors if it is not actually world-fixed. So such a
        // URDF fails construction loudly rather than being modeled wrong. OpenArm
        // has only single-link attachments; widen this to a recursive walk if a
        // multi-link end-effector is added. A mimic joint's own declared limits
        // are used as its travel; the mimic multiplier/offset are not resolved,
        // which is sound only while those limits already cover the true travel
        // (true on OpenArm, where the fingers mirror 1:1).
        let mut verts = urdf.link_vertices(name, meshes_dir)?;
        for child in urdf.children_of(name) {
            if chain_set.contains(child.as_str()) || urdf.collisions_of(&child).is_empty() {
                continue;
            }
            let joint = urdf.parent_joint(&child).expect("children_of implies a parent joint");
            verts.extend(urdf.child_vertices_in_parent(&child, joint.lower_limit, meshes_dir)?);
            if !joint.is_fixed() {
                verts.extend(urdf.child_vertices_in_parent(&child, joint.upper_limit, meshes_dir)?);
            }
            attached.insert(child);
        }
        links.insert(name.clone(), fit(&verts)?);
    }

    let mut fixed = Vec::new();
    for name in urdf.collision_link_names() {
        if chain_set.contains(name.as_str()) || attached.contains(&name) {
            continue;
        }
        let verts = urdf
            .fixed_vertices_in_root(&name, meshes_dir)
            .map_err(|e| format!("collision link '{name}' is neither a chain link, an attached child, nor world-fixed: {e}"))?;
        fixed.push((name, fit(&verts)?));
    }

    Ok(FittedBodies { fixed, links })
}
