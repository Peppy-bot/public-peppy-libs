//! Fit the collision bodies for a bimanual model directly from the URDF and
//! its meshes, at construction time. There is no intermediate artifact: the
//! hulls can never go stale against the geometry.

use std::collections::{HashMap, HashSet};

use srs_model::nalgebra::Point3;

use crate::gjk::{self, Hull};
use crate::{BuildError, ContainmentFailure};
use crate::hull::{ConvexHull, ConvexPiece, convex_hull, simplified_hull};
use crate::urdf_collision::UrdfCollisions;

/// Grid cell for hull simplification: points are welded onto this grid before
/// hulling, recovered by each hull's inflation radius, so a 68k-vertex link
/// builds fast and its support stays cheap.
const SIMPLIFY_CELL: f64 = 0.004;
/// A mesh point counts as inside a supplied hull when its signed distance is
/// within this of the surface (metres), so on-boundary points pass.
const CONTAIN_TOL: f64 = 1e-6;
/// Subdivision floor for the face-coverage check (metres, longest edge): a
/// sub-triangle smaller than this that still sits in no single piece is reported
/// as escaping. Well below the centimetre-scale proximity bands the model feeds.
const FACE_FLOOR: f64 = 1e-3;
/// A face whose smallest altitude is below this (metres) is a degenerate sliver
/// from mesh simplification: it bounds no surface, so the face check skips it.
/// Its corners are still checked by the per-vertex pass.
const SLIVER_ALTITUDE: f64 = 1e-4;

/// Convex-hull pieces for every collision body: world-fixed bodies (in root
/// frame) and chain links (in link frame, with attached collision-bearing
/// children baked in across their travel). Each body is one auto-fit hull unless
/// the caller supplied its own pieces.
pub(crate) struct FittedBodies {
    pub fixed: Vec<(String, Vec<Hull>)>,
    pub links: HashMap<String, Vec<Hull>>,
}

/// Fit all collision bodies. `chains` are the moving-link names per arm (from
/// FK); every other collision-bearing link must be world-fixed or an attached
/// child of a chain link, anything else is an error rather than a silently
/// unmodeled body. `supplied` overrides the auto-fit for named bodies with the
/// caller's own convex pieces, verified here to contain that body's mesh.
pub(crate) fn fit_bodies(
    urdf: &UrdfCollisions,
    chains: &[Vec<String>],
    meshes_dir: &str,
    supplied: &HashMap<String, Vec<ConvexPiece>>,
) -> Result<FittedBodies, BuildError> {
    let chain_set: HashSet<&str> = chains.iter().flatten().map(String::as_str).collect();
    let mut body_names: HashSet<String> = HashSet::new();

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
            let joint = urdf
                .parent_joint(&child)
                .expect("children_of implies a parent joint");
            verts.extend(urdf.child_vertices_in_parent(&child, joint.lower_limit, meshes_dir)?);
            if !joint.is_fixed() {
                verts.extend(urdf.child_vertices_in_parent(
                    &child,
                    joint.upper_limit,
                    meshes_dir,
                )?);
            }
            attached.insert(child);
        }
        links.insert(name.clone(), fit_body(name, &verts, supplied)?);
        body_names.insert(name.clone());
    }

    let mut fixed = Vec::new();
    for name in urdf.collision_link_names() {
        if chain_set.contains(name.as_str()) || attached.contains(&name) {
            continue;
        }
        let verts = urdf
            .fixed_vertices_in_root(&name, meshes_dir)
            .map_err(|e| format!("collision link '{name}' is neither a chain link, an attached child, nor world-fixed: {e}"))?;
        let hulls = fit_body(&name, &verts, supplied)?;
        body_names.insert(name.clone());
        fixed.push((name, hulls));
    }

    if let Some(unknown) = supplied.keys().find(|k| !body_names.contains(*k)) {
        return Err(BuildError::UnknownSuppliedBody { name: unknown.clone() });
    }

    Ok(FittedBodies { fixed, links })
}

/// The hulls for one body: the caller's supplied pieces if any (verified to
/// contain the mesh), otherwise a single simplified hull of the vertex cloud.
fn fit_body(
    name: &str,
    verts: &[Point3<f64>],
    supplied: &HashMap<String, Vec<ConvexPiece>>,
) -> Result<Vec<Hull>, BuildError> {
    let Some(pieces) = supplied.get(name) else {
        let rounded = simplified_hull(verts, SIMPLIFY_CELL)?;
        return Ok(vec![Hull::new(&rounded.hull, rounded.radius)?]);
    };
    if pieces.is_empty() {
        return Err(BuildError::EmptyHulls { body: name.to_string() });
    }
    let mut hulls = Vec::with_capacity(pieces.len());
    for piece in pieces {
        hulls.push(Hull::new(&convex_hull(piece.points())?, 0.0)?);
    }
    verify_contains(name, &hulls, verts)?;
    Ok(hulls)
}

/// Error unless the supplied hulls conservatively contain the body's whole mesh
/// surface, not merely its vertices. `verts` is the mesh triangle soup (every
/// three a triangle). A vertex-only check passes geometry that leaks: a face can
/// slope through the gap between two pieces while its three corners land inside
/// them (a tapering shoulder against stacked boxes). So every real face is
/// checked for surface coverage, and every vertex is checked outright (the
/// latter also covers the corners of degenerate sliver faces, which the face
/// pass skips because they bound no surface).
fn verify_contains(name: &str, hulls: &[Hull], verts: &[Point3<f64>]) -> Result<(), BuildError> {
    for v in verts {
        if !inside_union(hulls, v)? {
            return Err(BuildError::HullMissesMesh {
                body: name.to_string(),
                kind: ContainmentFailure::VertexOutside,
            });
        }
    }
    for tri in verts.chunks_exact(3) {
        let t = [tri[0], tri[1], tri[2]];
        if min_altitude(&t) < SLIVER_ALTITUDE {
            continue;
        }
        if !face_in_union(hulls, &t)? {
            return Err(BuildError::HullMissesMesh {
                body: name.to_string(),
                kind: ContainmentFailure::FaceEscapes,
            });
        }
    }
    Ok(())
}

/// Whether `v` lies inside any single hull (within [`CONTAIN_TOL`]).
fn inside_union(hulls: &[Hull], v: &Point3<f64>) -> Result<bool, String> {
    for h in hulls {
        if inside(h, v)? {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Whether every point of triangle `t` lies in the union of the hulls. A
/// triangle whose three corners share one convex hull lies wholly in it (a
/// convex set contains every convex combination of its points). One that
/// straddles pieces is split into four and each part re-checked, down to a
/// [`FACE_FLOOR`] edge length below which an uncovered remnant is reported as
/// escaping. The recursion only deepens near piece boundaries: a sub-triangle
/// that falls entirely in one piece stops immediately.
fn face_in_union(hulls: &[Hull], t: &[Point3<f64>; 3]) -> Result<bool, String> {
    for h in hulls {
        if t.iter()
            .try_fold(true, |acc, v| Ok::<_, String>(acc && inside(h, v)?))?
        {
            return Ok(true);
        }
    }
    if longest_edge(t) < FACE_FLOOR {
        return Ok(false);
    }
    let mid = |a: &Point3<f64>, b: &Point3<f64>| Point3::from((a.coords + b.coords) / 2.0);
    let (m01, m12, m20) = (mid(&t[0], &t[1]), mid(&t[1], &t[2]), mid(&t[2], &t[0]));
    let subs = [
        [t[0], m01, m20],
        [m01, t[1], m12],
        [m20, m12, t[2]],
        [m01, m12, m20],
    ];
    for s in &subs {
        if !face_in_union(hulls, s)? {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Whether point `v` lies inside hull `h` (within [`CONTAIN_TOL`]).
fn inside(h: &Hull, v: &Point3<f64>) -> Result<bool, String> {
    let point = Hull::new(
        &ConvexHull {
            vertices: vec![*v],
            faces: Vec::new(),
        },
        0.0,
    )?;
    Ok(gjk::distance(&point, h).distance <= CONTAIN_TOL)
}

/// The longest of the triangle's three edges (metres).
fn longest_edge(t: &[Point3<f64>; 3]) -> f64 {
    [
        (t[1] - t[0]).norm(),
        (t[2] - t[1]).norm(),
        (t[0] - t[2]).norm(),
    ]
    .into_iter()
    .fold(0.0, f64::max)
}

/// The triangle's smallest altitude (`2 * area / longest edge`): near zero for a
/// degenerate sliver, which has no surface to contain.
fn min_altitude(t: &[Point3<f64>; 3]) -> f64 {
    let area = 0.5 * (t[1] - t[0]).cross(&(t[2] - t[0])).norm();
    let longest = longest_edge(t);
    if longest <= 0.0 {
        0.0
    } else {
        2.0 * area / longest
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hull::convex_hull;

    /// A solid box hull (radius 0) from its min/max corner.
    fn box_hull(min: [f64; 3], max: [f64; 3]) -> Hull {
        let corners: Vec<Point3<f64>> = (0..8)
            .map(|i| {
                let pick = |bit: usize, c: usize| if i & (1 << bit) == 0 { min[c] } else { max[c] };
                Point3::new(pick(0, 0), pick(1, 1), pick(2, 2))
            })
            .collect();
        Hull::new(&convex_hull(&corners).expect("box hull"), 0.0).expect("hull")
    }

    /// Two boxes with an empty gap in z, the case a vertex-only check gets wrong.
    fn split_pieces() -> [Hull; 2] {
        [
            box_hull([-0.05, -0.05, 0.0], [0.05, 0.05, 0.1]),
            box_hull([-0.05, -0.05, 0.2], [0.05, 0.05, 0.3]),
        ]
    }

    #[test]
    fn a_real_face_bridging_the_gap_is_rejected() {
        // Corners land in the two pieces, but the face slopes through the gap.
        let verts = vec![
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.04, 0.0, 0.25),
            Point3::new(-0.04, 0.0, 0.25),
        ];
        let e = verify_contains("t", &split_pieces(), &verts)
            .expect_err("bridging face must be rejected");
        assert!(matches!(&e, BuildError::HullMissesMesh { kind: ContainmentFailure::FaceEscapes, .. }), "{e}");
    }

    #[test]
    fn a_degenerate_sliver_across_the_gap_is_skipped() {
        // Collinear: no surface to contain, only its (contained) corners matter.
        let verts = vec![
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.0, 0.0, 0.25),
            Point3::new(0.0, 0.0, 0.28),
        ];
        verify_contains("t", &split_pieces(), &verts)
            .expect("a sliver is skipped when its corners are contained");
    }

    #[test]
    fn a_vertex_outside_every_piece_is_rejected() {
        let verts = vec![
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.0, 0.0, 0.15),
            Point3::new(0.0, 0.0, 0.25),
        ];
        let e = verify_contains("t", &split_pieces(), &verts)
            .expect_err("a gap vertex must be rejected");
        assert!(matches!(&e, BuildError::HullMissesMesh { kind: ContainmentFailure::VertexOutside, .. }), "{e}");
    }

    #[test]
    fn a_face_inside_one_piece_passes() {
        let tall = [box_hull([-0.05, -0.05, 0.0], [0.05, 0.05, 0.3])];
        let verts = vec![
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.04, 0.0, 0.25),
            Point3::new(-0.04, 0.0, 0.25),
        ];
        verify_contains("t", &tall, &verts).expect("a face within one piece is contained");
    }
}
