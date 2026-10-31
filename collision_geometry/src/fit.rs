//! Fitting the convex pieces of one body from its collision surfaces: one
//! circumscribing fit per surface, which holds it by construction, or, for a concave
//! body, one fit per clip region, whose union the fit checks holds the surfaces.

use nalgebra::Point3;

use crate::clip::ClipRegion;
use crate::gjk::{self, Hull, Vertex};
use crate::simplify::circumscribe;
use crate::{ContainmentFailure, GeometryError};

/// How much slack a fitted piece may carry: the widest gap (meters) from its own
/// surface inward to the convex hull of its surface.
///
/// A piece encloses its surface at any budget, because every plane of the fit is a
/// supporting plane of the surface's points. So the budget bounds by how much a
/// piece is *larger* than the geometry it stands in for: a distance to the piece
/// reads at most this much under the distance to the geometry. The face count
/// needed to hold it follows from how much the surface curves.
const MAX_DEVIATION_M: f64 = 0.001;
/// A surface point counts as inside a hull when its signed distance is within this of
/// the hull's surface (meters), so on-boundary points pass.
const CONTAIN_TOL: f64 = 1e-6;
/// Subdivision floor for the face-coverage check (meters, longest edge): a
/// sub-triangle smaller than this that still sits in no single piece is reported
/// as escaping.
const FACE_FLOOR: f64 = 1e-3;
/// A face whose smallest altitude is below this (meters) is a degenerate sliver
/// from mesh simplification: it bounds no surface, so the face check skips it.
/// Its corners are still checked by the per-vertex pass.
const SLIVER_ALTITUDE: f64 = 1e-4;

/// The pieces of one body whose collision elements have `surfaces` (each a triangle
/// soup, every three points a triangle). Without clip regions: one circumscribing fit
/// of each surface. With clip regions (a concave body): the surfaces together are
/// clipped to each region and every clipped slice gets the same fit, then the union is
/// verified to contain the surfaces.
///
/// The fit's planes support the surface, so each piece contains its own surface by
/// construction and the pieces carry no inflation radius: a distance to a piece is the
/// distance to the surface less at most 1 mm.
pub fn fit_pieces(
    name: &str,
    surfaces: &[Vec<Point3<f64>>],
    regions: Option<&[ClipRegion]>,
) -> Result<Vec<Hull>, GeometryError> {
    let Some(regions) = regions else {
        return surfaces
            .iter()
            .map(|surface| {
                let fit = circumscribe(surface, MAX_DEVIATION_M).map_err(|reason| {
                    GeometryError::DegenerateBody {
                        body: name.to_string(),
                        reason,
                    }
                })?;
                Ok(Hull::new(&fit.hull, 0.0)?)
            })
            .collect();
    };
    if regions.is_empty() {
        return Err(GeometryError::EmptyRegions {
            body: name.to_string(),
        });
    }
    let soup = surfaces.concat();
    let mut hulls = Vec::with_capacity(regions.len());
    for (index, region) in regions.iter().enumerate() {
        let points = region.clip_triangles(&soup);
        let fit = circumscribe(&points, MAX_DEVIATION_M).map_err(|reason| {
            GeometryError::DegenerateRegion {
                body: name.to_string(),
                index,
                reason,
            }
        })?;
        hulls.push(Hull::new(&fit.hull, 0.0)?);
    }
    verify_contains(name, &hulls, &soup)?;
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
fn verify_contains(name: &str, hulls: &[Hull], verts: &[Point3<f64>]) -> Result<(), GeometryError> {
    for v in verts {
        if !inside_union(hulls, v) {
            return Err(GeometryError::HullMissesMesh {
                body: name.to_string(),
                kind: ContainmentFailure::VertexOutside,
            });
        }
    }
    let (triangles, _) = verts.as_chunks::<3>();
    for t in triangles {
        if min_altitude(t) < SLIVER_ALTITUDE {
            continue;
        }
        if !face_in_union(hulls, t) {
            return Err(GeometryError::HullMissesMesh {
                body: name.to_string(),
                kind: ContainmentFailure::FaceEscapes,
            });
        }
    }
    Ok(())
}

/// Whether `v` lies inside any single hull (within [`CONTAIN_TOL`]).
fn inside_union(hulls: &[Hull], v: &Point3<f64>) -> bool {
    hulls.iter().any(|h| inside(h, v))
}

/// Whether every point of triangle `t` lies in the union of the hulls. A
/// triangle whose three corners share one convex hull lies wholly in it (a
/// convex set contains every convex combination of its points). One that
/// straddles pieces is split into four and each part re-checked, down to a
/// [`FACE_FLOOR`] edge length below which an uncovered remnant is reported as
/// escaping. The recursion only deepens near piece boundaries: a sub-triangle
/// that falls entirely in one piece stops immediately.
fn face_in_union(hulls: &[Hull], t: &[Point3<f64>; 3]) -> bool {
    if hulls.iter().any(|h| t.iter().all(|v| inside(h, v))) {
        return true;
    }
    if longest_edge(t) < FACE_FLOOR {
        return false;
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
        if !face_in_union(hulls, s) {
            return false;
        }
    }
    true
}

/// Whether point `v` lies inside hull `h` (within [`CONTAIN_TOL`]).
fn inside(h: &Hull, v: &Point3<f64>) -> bool {
    gjk::distance(&Vertex(*v), h).distance <= CONTAIN_TOL
}

/// The longest of the triangle's three edges (meters).
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
    use crate::hull::exact_hull;

    /// A solid box hull (radius 0) from its min/max corner.
    fn box_hull(min: [f64; 3], max: [f64; 3]) -> Hull {
        let corners: Vec<Point3<f64>> = (0..8)
            .map(|i| {
                let pick = |bit: usize, c: usize| if i & (1 << bit) == 0 { min[c] } else { max[c] };
                Point3::new(pick(0, 0), pick(1, 1), pick(2, 2))
            })
            .collect();
        Hull::new(&exact_hull(&corners).expect("box hull"), 0.0).expect("hull")
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
        assert!(
            matches!(
                &e,
                GeometryError::HullMissesMesh {
                    kind: ContainmentFailure::FaceEscapes,
                    ..
                }
            ),
            "{e}"
        );
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
        assert!(
            matches!(
                &e,
                GeometryError::HullMissesMesh {
                    kind: ContainmentFailure::VertexOutside,
                    ..
                }
            ),
            "{e}"
        );
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
