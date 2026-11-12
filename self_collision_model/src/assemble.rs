//! Fit the collision bodies of the self-collision model directly from the URDF and
//! its meshes, at construction time. There is no intermediate artifact: the
//! hulls can never go stale against the geometry.

use std::collections::{HashMap, HashSet};

use collision_geometry::GeometryError;
use collision_geometry::fit::{fit_fixed_link, fit_pieces};
use collision_geometry::gjk::Hull;
use collision_geometry::urdf_collision::{ParentJoint, UrdfCollisions};
use robot_description::Finger;

use crate::BuildError;

/// Convex-hull pieces for every collision body: world-fixed bodies (in root
/// frame), chain links (in link frame, with any *fixed* collision-bearing child
/// baked in), and the fingers, each fit as its own body in its finger-link frame
/// and placed live from its chain's opening. A body has one piece per collision
/// element of its links.
pub(crate) struct FittedBodies {
    pub fixed: Vec<(String, Vec<Hull>)>,
    pub links: HashMap<String, Vec<Hull>>,
    pub fingers: Vec<FittedFinger>,
}

/// A finger: a collision-bearing link on a movable joint off a chain link, fit as its
/// own hull and placed at the live opening. `parent_link` is the chain link it hangs
/// off (its FK segment host); `joint` places its hull in that link's frame at a
/// given opening ([`ParentJoint::offset`]). `closed` and `open` are the joint's
/// positions where the gripper's `closed` and `open` states put it.
pub(crate) struct FittedFinger {
    pub name: String,
    pub parent_link: String,
    pub hulls: Vec<Hull>,
    pub joint: ParentJoint,
    pub closed: f64,
    pub open: f64,
}

/// Fit all collision bodies. `chains` are the moving-link names per chain (from
/// FK); every other collision-bearing link must be world-fixed or an attached
/// child of a chain link, anything else is an error rather than a silently
/// unmodeled body. A child on a movable joint is a finger, which `fingers` must name
/// with its closed and open positions.
pub(crate) fn fit_bodies(
    urdf: &UrdfCollisions,
    chains: &[Vec<String>],
    fingers: &[&Finger],
    meshes_dir: &str,
) -> Result<FittedBodies, BuildError> {
    let chain_set: HashSet<&str> = chains.iter().flatten().map(String::as_str).collect();

    let mut links = HashMap::new();
    let mut fitted_fingers: Vec<FittedFinger> = Vec::new();
    let mut attached: HashSet<String> = HashSet::new();
    for name in chains.iter().flatten() {
        // A chain link's surfaces are its own plus those of any *fixed*
        // collision-bearing child (a fixed sensor or hand base rides rigidly with
        // the link). A *movable* collision-bearing child is a finger: its own body,
        // fit from its own surfaces and placed live at the current opening; the
        // link's pieces cover only the link and its fixed children. Only direct
        // children are handled; a deeper collision-bearing descendant (a multi-link
        // end-effector) falls through to the fixed-body pass below, which errors if
        // it is not actually world-fixed, so such a URDF fails construction. A mimic
        // finger joint's own declared limits are its travel; the mimic
        // multiplier/offset are not resolved, sound while those limits cover the true
        // travel.
        let mut surfaces = urdf.link_surfaces(name, meshes_dir)?;
        for child in urdf.children_of(name) {
            if chain_set.contains(child.as_str()) || urdf.collisions_of(&child).is_empty() {
                continue;
            }
            let joint = urdf
                .parent_joint(&child)
                .expect("children_of implies a parent joint")
                .clone();
            attached.insert(child.clone());
            if joint.is_fixed() {
                surfaces.extend(urdf.child_surfaces_in_parent(&child, 0.0, meshes_dir)?);
                continue;
            }
            // A movable finger placed live per opening. A finger joint that is not
            // prismatic or revolute (continuous, planar or floating) is refused when
            // the model parses it into its runtime placer.
            let finger = fingers
                .iter()
                .find(|finger| finger.link == child)
                .ok_or_else(|| BuildError::UnnamedFinger {
                    link: child.clone(),
                })?;
            let (closed, open) = (finger.closed, finger.open);
            let finger_surfaces = urdf.link_surfaces(&child, meshes_dir)?;
            let hulls = fit_pieces(&child, &finger_surfaces)?;
            fitted_fingers.push(FittedFinger {
                name: child,
                parent_link: name.clone(),
                hulls,
                joint,
                closed,
                open,
            });
        }
        links.insert(name.clone(), fit_pieces(name, &surfaces)?);
    }
    if let Some(finger) = fingers.iter().find(|finger| {
        !urdf.collisions_of(&finger.link).is_empty()
            && !fitted_fingers
                .iter()
                .any(|fitted| fitted.name == finger.link)
    }) {
        return Err(BuildError::NoSuchFinger {
            link: finger.link.to_string(),
        });
    }

    let mut fixed = Vec::new();
    for name in urdf.collision_link_names() {
        if chain_set.contains(name.as_str()) || attached.contains(&name) {
            continue;
        }
        let hulls = fit_fixed_link(urdf, &name, meshes_dir)
            .map_err(|error| match error {
                GeometryError::NotFixedToTheRoot { .. } => BuildError::Geometry(format!(
                    "collision link '{name}' is neither a chain link, an attached child, nor world-fixed: {error}"
                )),
                fit => BuildError::Fit(fit),
            })?;
        fixed.push((name, hulls));
    }

    Ok(FittedBodies {
        fixed,
        links,
        fingers: fitted_fingers,
    })
}
