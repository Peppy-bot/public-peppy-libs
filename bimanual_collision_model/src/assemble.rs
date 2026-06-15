//! Fit the collision bodies for a bimanual model directly from the URDF and
//! its meshes, at construction time. There is no intermediate artifact: the
//! capsules can never go stale against the geometry.

use std::collections::{HashMap, HashSet};

use srs_model::nalgebra::Point3;

use crate::fit::fit_capsules_adaptive;
use crate::geometry::Capsule;
use crate::urdf_collision::UrdfCollisions;

/// Adaptive band search ceilings: compound fixed bodies (a torso) are worth
/// many bands; limb links taper at most a little, and extra capsules cost
/// pairwise checks.
const MAX_BANDS_FIXED: usize = 8;
const MAX_BANDS_LINK: usize = 3;

/// Capsules for every collision body: world-fixed bodies (in root frame) and
/// chain links (in link frame, with attached collision-bearing children
/// baked in across their travel).
pub(crate) struct FittedBodies {
    pub fixed: Vec<(String, Vec<Capsule>)>,
    pub links: HashMap<String, Vec<Capsule>>,
}

/// Fit all collision bodies. `chains` are the moving-link names per arm
/// (from FK); every other collision-bearing link must be world-fixed or an
/// attached child of a chain link, anything else is an error rather than a
/// silently unmodeled body.
pub(crate) fn fit_bodies(
    urdf: &UrdfCollisions,
    chains: &[Vec<String>],
    meshes_dir: &str,
) -> Result<FittedBodies, String> {
    let chain_set: HashSet<&str> = chains.iter().flatten().map(String::as_str).collect();

    let mut links = HashMap::new();
    let mut attached: HashSet<String> = HashSet::new();
    for name in chains.iter().flatten() {
        let mut capsules = fit_capsules_adaptive(&urdf.link_vertices(name, meshes_dir)?, MAX_BANDS_LINK)?;
        for child in urdf.children_of(name) {
            if chain_set.contains(child.as_str()) || urdf.collisions_of(&child).is_empty() {
                continue;
            }
            capsules.push(attached_child_capsule(urdf, &child, meshes_dir)?);
            attached.insert(child);
        }
        links.insert(name.clone(), capsules);
    }

    let mut fixed = Vec::new();
    for name in urdf.collision_link_names() {
        if chain_set.contains(name.as_str()) || attached.contains(&name) {
            continue;
        }
        let vertices = urdf
            .fixed_vertices_in_root(&name, meshes_dir)
            .map_err(|e| format!("collision link '{name}' is neither a chain link, an attached child, nor world-fixed: {e}"))?;
        fixed.push((name, fit_capsules_adaptive(&vertices, MAX_BANDS_FIXED)?));
    }

    Ok(FittedBodies { fixed, links })
}

/// One capsule containing an attached child (e.g. a gripper finger) across
/// its full joint travel, in the parent link's frame. Travel is a
/// joint-space line, so containing both extremes contains every
/// intermediate position.
fn attached_child_capsule(urdf: &UrdfCollisions, child: &str, meshes_dir: &str) -> Result<Capsule, String> {
    let joint = urdf.parent_joint(child).expect("children_of implies a parent joint");
    let mut vertices: Vec<Point3<f64>> = urdf.child_vertices_in_parent(child, joint.lower_limit, meshes_dir)?;
    if !joint.is_fixed() {
        vertices.extend(urdf.child_vertices_in_parent(child, joint.upper_limit, meshes_dir)?);
    }
    Ok(fit_capsules_adaptive(&vertices, 1)?.remove(0))
}
