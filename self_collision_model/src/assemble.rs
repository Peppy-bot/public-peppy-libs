//! Fit the collision bodies of the self-collision model directly from the URDF and
//! its meshes, at construction time. There is no intermediate artifact: the
//! hulls can never go stale against the geometry.

use std::collections::{HashMap, HashSet};

use crate::nalgebra::Point3;
use collision_geometry::clip::ClipRegion;
use collision_geometry::fit::fit_pieces;
use collision_geometry::gjk::Hull;
use collision_geometry::urdf_collision::{ParentJoint, UrdfCollisions};

use crate::BuildError;

/// Convex-hull pieces for every collision body: world-fixed bodies (in root
/// frame), chain links (in link frame, with any *fixed* collision-bearing child
/// baked in), and the movable end-effector fingers, each fit as its own body in
/// its finger-link frame and placed live from the gripper opening. A body has one
/// piece per collision element of its links, unless the caller supplied clip
/// regions for it.
pub(crate) struct FittedBodies {
    pub fixed: Vec<(String, Vec<Hull>)>,
    pub links: HashMap<String, Vec<Hull>>,
    pub fingers: Vec<FittedFinger>,
}

/// A gripper finger: a collision-bearing link on a movable joint off a chain
/// link. Fit as its own hull (not baked into the parent) so it can be placed at
/// the live opening rather than swept over its whole travel. `parent_link` is the
/// chain link it hangs off (its FK segment host); `joint` places its hull in that
/// link's frame at a given opening ([`ParentJoint::offset`]). `closed` and `open`
/// are the joint positions of the two travel extremes, oriented geometrically by
/// [`orient_finger_pair`], not by the URDF limit order.
pub(crate) struct FittedFinger {
    pub name: String,
    pub parent_link: String,
    pub hulls: Vec<Hull>,
    pub joint: ParentJoint,
    pub closed: f64,
    pub open: f64,
}

/// Fit all collision bodies. `chains` are the moving-link names per arm (from
/// FK); every other collision-bearing link must be world-fixed or an attached
/// child of a chain link, anything else is an error rather than a silently
/// unmodeled body. `supplied` overrides the auto-fit for named bodies with a
/// clip-region decomposition, whose fitted pieces are verified here to contain
/// that body's mesh.
pub(crate) fn fit_bodies(
    urdf: &UrdfCollisions,
    chains: &[Vec<String>],
    meshes_dir: &str,
    supplied: &HashMap<String, Vec<ClipRegion>>,
) -> Result<FittedBodies, BuildError> {
    let chain_set: HashSet<&str> = chains.iter().flatten().map(String::as_str).collect();
    let mut body_names: HashSet<String> = HashSet::new();

    let mut links = HashMap::new();
    let mut fingers: Vec<FittedFinger> = Vec::new();
    let mut attached: HashSet<String> = HashSet::new();
    for name in chains.iter().flatten() {
        // A chain link's surfaces are its own plus those of any *fixed*
        // collision-bearing child (a fixed sensor or hand base rides rigidly with
        // the link). A *movable* collision-bearing child is a gripper finger: its
        // own body, fit from its own surfaces and placed live at the current
        // opening; the link's pieces cover only the link and its fixed children. Only
        // direct children are handled; a deeper collision-bearing descendant
        // (a multi-link end-effector) falls through to the fixed-body pass
        // below, which errors if it is not actually world-fixed, so such a
        // URDF fails construction loudly rather than being modeled wrong.
        // OpenArm has only single-link attachments; widen this to a recursive
        // walk if a multi-link end-effector is added. A mimic finger joint's
        // own declared limits are its travel; the mimic multiplier/offset are
        // not resolved, sound while those limits cover the true travel
        // (OpenArm's fingers mirror 1:1).
        let mut surfaces = urdf.link_surfaces(name, meshes_dir)?;
        let mut link_fingers: Vec<(FittedFinger, Point3<f64>)> = Vec::new();
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
            // the model parses it into its runtime placer; the mesh centroid feeds
            // the pair orientation below.
            let finger_surfaces = urdf.link_surfaces(&child, meshes_dir)?;
            let centroid = vertex_centroid(&finger_surfaces.concat());
            let hulls = fit_pieces(&child, &finger_surfaces, regions_of(supplied, &child))?;
            body_names.insert(child.clone());
            let (closed, open) = (joint.lower_limit, joint.upper_limit);
            link_fingers.push((
                FittedFinger {
                    name: child,
                    parent_link: name.clone(),
                    hulls,
                    joint,
                    closed,
                    open,
                },
                centroid,
            ));
        }
        orient_finger_pair(&mut link_fingers)?;
        fingers.extend(link_fingers.into_iter().map(|(finger, _)| finger));
        links.insert(
            name.clone(),
            fit_pieces(name, &surfaces, regions_of(supplied, name))?,
        );
        body_names.insert(name.clone());
    }

    let mut fixed = Vec::new();
    for name in urdf.collision_link_names() {
        if chain_set.contains(name.as_str()) || attached.contains(&name) {
            continue;
        }
        let surfaces = urdf
            .fixed_surfaces_in_root(&name, meshes_dir)
            .map_err(|e| format!("collision link '{name}' is neither a chain link, an attached child, nor world-fixed: {e}"))?;
        let hulls = fit_pieces(&name, &surfaces, regions_of(supplied, &name))?;
        body_names.insert(name.clone());
        fixed.push((name, hulls));
    }

    if let Some(unknown) = supplied.keys().find(|k| !body_names.contains(*k)) {
        return Err(BuildError::UnknownSuppliedBody {
            name: unknown.clone(),
        });
    }

    Ok(FittedBodies {
        fixed,
        links,
        fingers,
    })
}

/// Orient a two-finger gripper's `closed`/`open` joint positions from its
/// meshes. URDF convention puts closed at the lower limit, but a mirrored
/// gripper (OpenArm v2's right hand) mirrors by flipping the limit range
/// instead of the axis, so its lower limit is the open end; trusting the
/// convention there would invert the live placement, and "fully open" (the
/// conservative default) would park the collision fingers closed. The meshes
/// decide instead: the travel extreme with the larger between-finger centroid
/// separation is open. The limit order is untrustworthy in general, so a link
/// with any other movable-child count has no oriented travel and errors,
/// keeping an unorientable gripper a loud build failure instead of a silently
/// inverted model.
fn orient_finger_pair(pair: &mut [(FittedFinger, Point3<f64>)]) -> Result<(), BuildError> {
    let [a, b] = pair else {
        if pair.is_empty() {
            return Ok(());
        }
        return Err(BuildError::Geometry(format!(
            "link '{}' has {} movable collision-bearing children; orienting finger travel \
             needs exactly two (a two-finger gripper)",
            pair[0].0.parent_link,
            pair.len()
        )));
    };
    let posed = |f: &(FittedFinger, Point3<f64>), q: f64| -> Result<Point3<f64>, BuildError> {
        let offset =
            f.0.joint
                .offset(q)
                .map_err(|e| BuildError::Geometry(format!("finger '{}': {e}", f.0.name)))?;
        Ok(offset * f.1)
    };
    let separation = |qa: f64, qb: f64| -> Result<f64, BuildError> {
        Ok((posed(a, qa)? - posed(b, qb)?).norm())
    };
    let at_lower = separation(a.0.joint.lower_limit, b.0.joint.lower_limit)?;
    let at_upper = separation(a.0.joint.upper_limit, b.0.joint.upper_limit)?;
    // A real gripper's tip separation differs by centimeters between its limit
    // extremes; near-equal separations mean the two joints' limit ranges point
    // opposite ways (e.g. a half-mirrored export), where a strict comparison
    // would pick an orientation by floating-point noise. Refuse to guess.
    const ORIENT_MARGIN_M: f64 = 1e-6;
    if (at_lower - at_upper).abs() <= ORIENT_MARGIN_M {
        return Err(BuildError::Geometry(format!(
            "link '{}': finger travel orientation is ambiguous (tip separation \
             {at_lower:.6} m at the lower limits vs {at_upper:.6} m at the upper); \
             the two finger joints' limit ranges do not agree on a closed-to-open \
             direction",
            a.0.parent_link
        )));
    }
    if at_lower > at_upper {
        for (finger, _) in pair.iter_mut() {
            (finger.closed, finger.open) = (finger.joint.upper_limit, finger.joint.lower_limit);
        }
    }
    Ok(())
}

/// Centroid of a vertex cloud (the meshes here are dense enough that the
/// vertex mean is a stable stand-in for the surface centroid).
fn vertex_centroid(verts: &[Point3<f64>]) -> Point3<f64> {
    let sum = verts
        .iter()
        .fold(crate::nalgebra::Vector3::zeros(), |acc, p| acc + p.coords);
    Point3::from(sum / verts.len().max(1) as f64)
}

/// The clip regions the caller supplied for `body`, if any.
fn regions_of<'a>(
    supplied: &'a HashMap<String, Vec<ClipRegion>>,
    body: &str,
) -> Option<&'a [ClipRegion]> {
    supplied.get(body).map(Vec::as_slice)
}
