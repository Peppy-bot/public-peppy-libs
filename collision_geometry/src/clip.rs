//! Axis-aligned clip regions, which cut a concave mesh into slices. A single convex
//! hull of a concave mesh bridges its features into one bulging solid, so a URDF names
//! each slice of such a mesh as a collision element of its own: each slice gets the fit
//! every element gets, and the union of the pieces bounds the body tightly.

use nalgebra::Point3;

/// An axis-aligned region `[min, max]` a mesh is clipped to, in the mesh's frame. A
/// bound may be infinite to leave that side open. Construction rejects NaN bounds and
/// empty spans, so a held `ClipRegion` always spans a volume.
///
/// The pieces fitted to the slices of a mesh hold its whole surface only where
/// adjacent regions overlap: [`verify_contains`](crate::fit::verify_contains)
/// certifies each ~1 mm face patch inside a single piece, so adjacent regions overlap by
/// a few millimeters. A bound resting exactly on a large flat mesh face captures that face
/// into the region's piece; sit bounds ~1 mm off such planes.
#[derive(Clone, Copy, Debug)]
pub struct ClipRegion {
    min: Point3<f64>,
    max: Point3<f64>,
}

impl ClipRegion {
    /// A region spanning `[min, max]` per axis; infinite bounds leave that side
    /// open. Errors on a NaN bound or an empty span (`min >= max` on any axis).
    pub fn new(min: Point3<f64>, max: Point3<f64>) -> Result<ClipRegion, String> {
        for axis in 0..3 {
            let (lo, hi) = (min[axis], max[axis]);
            if lo.is_nan() || hi.is_nan() {
                return Err(format!("clip region has a NaN bound on axis {axis}"));
            }
            if lo >= hi {
                return Err(format!(
                    "clip region spans nothing on axis {axis}: min {lo} >= max {hi}"
                ));
            }
        }
        Ok(ClipRegion { min, max })
    }

    /// The region's slice of the mesh surface `soup` (each three points one triangle),
    /// as a triangle soup: each triangle clipped to the region's planes, the polygon a
    /// clip leaves a fan of triangles from its first corner. A clip that leaves fewer
    /// than three corners (a triangle that touches a bound plane, or a degenerate
    /// sliver) gives one triangle of its corners repeated, so the slice holds every
    /// point the clip keeps and a fit of the slice spans what the clip reaches.
    /// Boundary-inclusive, so a triangle lying in a bound plane is kept and adjoining
    /// regions both hold it.
    pub fn slice(&self, soup: &[Point3<f64>]) -> Vec<Point3<f64>> {
        let (triangles, _) = soup.as_chunks::<3>();
        triangles
            .iter()
            .flat_map(|tri| fan(&self.clip_triangle(tri)))
            .collect()
    }

    /// Sutherland-Hodgman: the triangle successively clipped against each
    /// finite bound plane. Empty once the polygon falls entirely outside.
    fn clip_triangle(&self, tri: &[Point3<f64>; 3]) -> Vec<Point3<f64>> {
        let mut poly = tri.to_vec();
        for axis in 0..3 {
            for (bound, keep_below) in [(self.min[axis], false), (self.max[axis], true)] {
                if bound.is_infinite() {
                    continue;
                }
                if poly.is_empty() {
                    return poly;
                }
                poly = clip_polygon(&poly, axis, bound, keep_below);
            }
        }
        poly
    }
}

/// The triangles of `polygon`, a fan from its first corner; a polygon of fewer than
/// three corners is one triangle of its corners repeated.
fn fan(polygon: &[Point3<f64>]) -> Vec<Point3<f64>> {
    match polygon {
        [] => Vec::new(),
        [a] => vec![*a, *a, *a],
        [a, b] => vec![*a, *b, *b],
        [first, rest @ ..] => rest
            .windows(2)
            .flat_map(|edge| [*first, edge[0], edge[1]])
            .collect(),
    }
}

/// One polygon against one axis-aligned half-space: vertices on the kept side
/// (boundary inclusive) survive, and each edge crossing the plane contributes
/// its intersection point. The crossing interpolation never divides by zero:
/// an edge with equal `axis` coordinates is entirely on one side.
fn clip_polygon(
    poly: &[Point3<f64>],
    axis: usize,
    bound: f64,
    keep_below: bool,
) -> Vec<Point3<f64>> {
    let inside = |p: &Point3<f64>| {
        if keep_below {
            p[axis] <= bound
        } else {
            p[axis] >= bound
        }
    };
    let mut out = Vec::with_capacity(poly.len() + 1);
    for (i, a) in poly.iter().enumerate() {
        let b = &poly[(i + 1) % poly.len()];
        if inside(a) {
            out.push(*a);
        }
        if inside(a) != inside(b) {
            let t = (bound - a[axis]) / (b[axis] - a[axis]);
            out.push(Point3::from(a.coords + t * (b.coords - a.coords)));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const INF: f64 = f64::INFINITY;

    fn pt(x: f64, y: f64, z: f64) -> Point3<f64> {
        Point3::new(x, y, z)
    }

    fn region(min: [f64; 3], max: [f64; 3]) -> ClipRegion {
        ClipRegion::new(pt(min[0], min[1], min[2]), pt(max[0], max[1], max[2]))
            .expect("test region")
    }

    fn contains(points: &[Point3<f64>], p: Point3<f64>) -> bool {
        points.iter().any(|q| (q - p).norm() < 1e-12)
    }

    /// The distinct points of a slice.
    fn corners(slice: &[Point3<f64>]) -> Vec<Point3<f64>> {
        let mut points = slice.to_vec();
        points.sort_by(|p, q| {
            p.x.total_cmp(&q.x)
                .then_with(|| p.y.total_cmp(&q.y))
                .then_with(|| p.z.total_cmp(&q.z))
        });
        points.dedup();
        points
    }

    #[test]
    fn rejects_nan_and_empty_spans() {
        assert!(ClipRegion::new(pt(0.0, 0.0, f64::NAN), pt(1.0, 1.0, 1.0)).is_err());
        assert!(ClipRegion::new(pt(0.0, 0.0, 0.0), pt(1.0, 1.0, 0.0)).is_err());
        assert!(ClipRegion::new(pt(0.0, 2.0, 0.0), pt(1.0, 1.0, 1.0)).is_err());
        assert!(ClipRegion::new(pt(-INF, -INF, -INF), pt(INF, INF, INF)).is_ok());
    }

    #[test]
    fn a_fan_repeats_the_corners_of_a_polygon_of_fewer_than_three() {
        let [a, b, c, d] = [
            pt(0., 0., 0.),
            pt(1., 0., 0.),
            pt(1., 1., 0.),
            pt(0., 1., 0.),
        ];
        assert!(fan(&[]).is_empty());
        assert_eq!(fan(&[a]), [a, a, a]);
        assert_eq!(fan(&[a, b]), [a, b, b]);
        assert_eq!(fan(&[a, b, c, d]), [a, b, c, a, c, d]);
    }

    #[test]
    fn a_triangle_inside_passes_unchanged() {
        let tri = [pt(0.1, 0.1, 0.1), pt(0.4, 0.1, 0.1), pt(0.1, 0.4, 0.1)];
        let out = region([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]).slice(&tri);
        assert_eq!(out, tri);
    }

    #[test]
    fn a_triangle_outside_clips_to_nothing() {
        let tri = [pt(2.0, 0.0, 0.0), pt(3.0, 0.0, 0.0), pt(2.0, 1.0, 0.0)];
        let out = region([0.0, -1.0, -1.0], [1.0, 1.0, 1.0]).slice(&tri);
        assert!(out.is_empty());
    }

    #[test]
    fn a_straddling_triangle_gains_the_cut_points() {
        // One corner beyond x = 1: the clip keeps the two inside corners and
        // adds the two edge crossings, exactly on the plane, as two triangles.
        let tri = [pt(0.0, 0.0, 0.0), pt(2.0, 0.0, 0.0), pt(0.0, 2.0, 0.0)];
        let slice = region([-INF, -INF, -INF], [1.0, INF, INF]).slice(&tri);
        assert_eq!(slice.len(), 6);
        let out = corners(&slice);
        assert_eq!(out.len(), 4);
        assert!(contains(&out, pt(0.0, 0.0, 0.0)));
        assert!(contains(&out, pt(0.0, 2.0, 0.0)));
        assert!(contains(&out, pt(1.0, 0.0, 0.0)));
        assert!(contains(&out, pt(1.0, 1.0, 0.0)));
    }

    #[test]
    fn both_neighbors_keep_a_triangle_on_their_shared_plane() {
        let tri = [pt(0.5, 0.0, 0.0), pt(0.5, 1.0, 0.0), pt(0.5, 0.0, 1.0)];
        let below = region([-INF, -INF, -INF], [0.5, INF, INF]).slice(&tri);
        let above = region([0.5, -INF, -INF], [INF, INF, INF]).slice(&tri);
        assert_eq!(below, tri, "coplanar triangle kept below the cut");
        assert_eq!(above, tri, "coplanar triangle kept above the cut");
    }

    #[test]
    fn a_degenerate_sliver_keeps_its_boundary_points() {
        // Collinear soup: no area to clip, but the surviving points still feed
        // the fit so the sliver's extent is not lost.
        let tri = [pt(0.0, 0.0, 0.0), pt(2.0, 0.0, 0.0), pt(1.0, 0.0, 0.0)];
        let out = region([-INF, -INF, -INF], [1.5, INF, INF]).slice(&tri);
        assert!(contains(&out, pt(0.0, 0.0, 0.0)));
        assert!(contains(&out, pt(1.0, 0.0, 0.0)));
        assert!(contains(&out, pt(1.5, 0.0, 0.0)), "cut point on the sliver");
    }

    #[test]
    fn every_clipped_point_lies_in_the_region_and_extremes_survive() {
        // A cube surface clipped to its upper half: all output within bounds,
        // and the cut ring at z = 0.5 present.
        let mut soup = Vec::new();
        let faces = [
            [
                pt(0., 0., 0.),
                pt(1., 0., 0.),
                pt(1., 1., 0.),
                pt(0., 1., 0.),
            ],
            [
                pt(0., 0., 1.),
                pt(1., 0., 1.),
                pt(1., 1., 1.),
                pt(0., 1., 1.),
            ],
            [
                pt(0., 0., 0.),
                pt(1., 0., 0.),
                pt(1., 0., 1.),
                pt(0., 0., 1.),
            ],
            [
                pt(0., 1., 0.),
                pt(1., 1., 0.),
                pt(1., 1., 1.),
                pt(0., 1., 1.),
            ],
            [
                pt(0., 0., 0.),
                pt(0., 1., 0.),
                pt(0., 1., 1.),
                pt(0., 0., 1.),
            ],
            [
                pt(1., 0., 0.),
                pt(1., 1., 0.),
                pt(1., 1., 1.),
                pt(1., 0., 1.),
            ],
        ];
        for [a, b, c, d] in faces {
            soup.extend([a, b, c, a, c, d]);
        }
        let out = region([-INF, -INF, 0.5], [INF, INF, INF]).slice(&soup);
        assert_eq!(out.len() % 3, 0, "a slice is a triangle soup");
        assert!(out.iter().all(|p| p.z >= 0.5), "a point escaped the clip");
        for corner in [
            pt(0., 0., 1.),
            pt(1., 1., 1.),
            pt(0., 0., 0.5),
            pt(1., 1., 0.5),
        ] {
            assert!(contains(&out, corner), "extreme {corner:?} missing");
        }
    }
}
