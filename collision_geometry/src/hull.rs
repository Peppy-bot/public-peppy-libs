//! Exact convex hull of a point cloud, computed once at construction, by Qhull
//! (Barber, Dobkin & Huhdanpaa 1996), the library MoveIt builds its convex meshes
//! with. The hull is the geometric reference everything else is measured against: it
//! is the tightest convex body containing the mesh, so a proxy's deviation is its
//! distance from this hull.
//!
//! Qhull merges the facets that rounding leaves coplanar or concave, and its
//! triangulated output (`Qt`) splits each merged facet into triangles that keep the
//! merged facet's plane, so the plane of a face is as exact as the facet's however thin
//! the triangle. Qhull then checks that every point lies inside the hull. A cloud that
//! fails this pass on rounding (a precision, topology or wide-facet error) is hulled
//! again with joggled input (`QJ`): Qhull moves each point by a rounding-sized amount,
//! with a fixed seed, and builds a simplicial hull, whose planes then move out to the
//! farthest of the points as given. A cloud that fails the joggled pass too, or fails
//! the first pass otherwise, has no hull, and [`exact_hull`] returns Qhull's error: the
//! kind of failure and Qhull's first error message.
//!
//! Qhull writes its messages, the check of the points and the report of a failed pass,
//! to a stream in memory that holds no file descriptor, so a hull builds with no file
//! descriptor free and prints nothing to stdout or stderr, with one exception. The
//! `qhull` crate creates a temporary file when a pass fails, and panics when it cannot
//! create one (no writable temporary directory, or no free file descriptor): the
//! process's panic hook, by default, prints that panic to stderr before [`exact_hull`]
//! catches it. A caught panic counts as the error that Qhull's report names: singular
//! input, when its first error message is that of an initial simplex that is flat or not
//! convex, and a precision error otherwise, so the cloud is hulled again joggled, and a
//! panic of the joggled pass is the error.

use std::panic::AssertUnwindSafe;

use nalgebra::{Point3, Vector3};
use qhull::{Qh, sys};

#[allow(unsafe_code)]
mod report;

use report::Report;

/// The convex hull as triangles over a deduplicated vertex list, each with the plane
/// of the facet it triangulates. `vertices` is what GJK needs; `faces` index into it
/// for rendering and for the vertex adjacency the support hill-climb walks.
#[derive(Debug, Clone)]
pub struct ConvexHull {
    pub vertices: Vec<Point3<f64>>,
    pub faces: Vec<[usize; 3]>,
    /// The plane of each face, in the order of `faces`: its outward unit normal and
    /// its offset along it, `normal . x = offset`.
    pub planes: Vec<(Vector3<f64>, f64)>,
}

/// The fewest points that span a solid.
const SOLID_POINTS: usize = 4;

/// Exact convex hull of `points`: every point lies on or inside it, and every
/// returned vertex is a point of the cloud. Errors on a point that is not finite, and
/// on a cloud that spans fewer than three dimensions (collinear or coplanar), which no
/// solid collision mesh is.
pub fn exact_hull(points: &[Point3<f64>]) -> Result<ConvexHull, String> {
    if let Some(point) = points
        .iter()
        .find(|p| !p.coords.iter().all(|c| c.is_finite()))
    {
        return Err(format!("a convex hull needs finite points, got {point}"));
    }
    // Sorted, the hull does not depend on how the caller ordered the cloud, and
    // deduplicated, the repeated corners of a triangle soup are one point.
    let mut sorted = points.to_vec();
    sorted.sort_by(|p, q| {
        p.x.total_cmp(&q.x)
            .then_with(|| p.y.total_cmp(&q.y))
            .then_with(|| p.z.total_cmp(&q.z))
    });
    sorted.dedup();
    if sorted.len() < SOLID_POINTS {
        return Err(format!(
            "a convex hull needs {SOLID_POINTS} distinct points, got {}",
            sorted.len()
        ));
    }
    match qhull(&sorted, Input::Exact) {
        Err(failure) if failure.precision => qhull(&sorted, Input::Joggled),
        hull => hull,
    }
    .map_err(|failure| failure.message)
}

/// Why Qhull built no hull.
struct Failure {
    /// Rounding defeated it: a precision, topology or wide-facet error, which joggled
    /// input resolves. A flat or collinear cloud fails otherwise.
    precision: bool,
    message: String,
}

impl Failure {
    fn of(message: &str) -> Self {
        Failure {
            precision: false,
            message: message.to_owned(),
        }
    }
}

/// The points Qhull builds a hull of.
#[derive(Clone, Copy)]
enum Input {
    /// The points as given, facets merged and triangulated (`Qt`).
    Exact,
    /// The points joggled by a rounding-sized amount (`QJ`). Each facet's plane then
    /// moves out to the farthest of the points as given, so it holds them all.
    Joggled,
}

/// The numbers of Qhull's error messages of singular input, with which Qhull exits as
/// `qh_ERRsingular`: an initial simplex that is not convex (6114, 6379) or is flat (6154).
const SINGULAR_INPUT: [&str; 3] = ["QH6114", "QH6154", "QH6379"];

/// The verified Qhull hull of distinct `points`, as a [`ConvexHull`] over them. A panic
/// of the `qhull` crate counts as singular input when Qhull's report names it, and as a
/// precision error otherwise.
fn qhull(points: &[Point3<f64>], input: Input) -> Result<ConvexHull, Failure> {
    let report = Report::open().map_err(|e| {
        Failure::of(&format!(
            "convex hull: no stream in memory for Qhull's messages: {e}"
        ))
    })?;
    std::panic::catch_unwind(AssertUnwindSafe(|| qhull_build(points, input, &report)))
        .unwrap_or_else(|payload| {
            let reason = payload
                .downcast_ref::<String>()
                .map(String::as_str)
                .or_else(|| payload.downcast_ref::<&str>().copied())
                .unwrap_or("no message");
            let singular = report.error().is_some_and(|error| {
                SINGULAR_INPUT
                    .iter()
                    .any(|number| error.starts_with(number))
            });
            Err(Failure {
                precision: !singular,
                message: with_report(
                    format!("convex hull: the qhull crate panicked: {reason}"),
                    &report,
                ),
            })
        })
}

/// `message`, then Qhull's first error message in `report`, if it wrote one.
fn with_report(message: String, report: &Report) -> String {
    match report.error() {
        Some(error) => format!("{message}; {error}"),
        None => message,
    }
}

/// What Qhull's exit `code` names.
fn failure_kind(code: u32) -> String {
    match code {
        sys::qh_ERRinput => "an input error".to_owned(),
        sys::qh_ERRsingular => "singular input".to_owned(),
        sys::qh_ERRprec => "a precision error".to_owned(),
        sys::qh_ERRmem => "a memory error".to_owned(),
        sys::qh_ERRqhull => "an internal error".to_owned(),
        sys::qh_ERRtopology => "a topology error".to_owned(),
        sys::qh_ERRwide => "a wide facet".to_owned(),
        other => format!("error {other}"),
    }
}

/// [`qhull`], with no guard against a panic, its Qhull writing its messages to `report`.
fn qhull_build(
    points: &[Point3<f64>],
    input: Input,
    report: &Report,
) -> Result<ConvexHull, Failure> {
    let builder = Qh::builder().capture_stderr(false).check_points(true);
    let builder = match input {
        Input::Exact => builder.triangulate(true),
        Input::Joggled => builder
            .qhull_args(["QJ"])
            .expect("a static option has no interior nul"),
    };
    report
        .build(builder, points.iter().map(|p| [p.x, p.y, p.z]), |qh| {
            hull_of(qh, points, input)
        })
        .map_err(|e| {
            let code = e.kind.error_code() as u32;
            Failure {
                precision: [sys::qh_ERRprec, sys::qh_ERRtopology, sys::qh_ERRwide].contains(&code),
                message: with_report(
                    format!("convex hull: Qhull failed on {}", failure_kind(code)),
                    report,
                ),
            }
        })?
}

/// The [`ConvexHull`] over `points` that `qh` built of them from `input`.
fn hull_of(qh: &Qh, points: &[Point3<f64>], input: Input) -> Result<ConvexHull, Failure> {
    let facets = qh
        .facets()
        .map(|facet| {
            let corners = facet
                .vertices()
                .ok_or_else(|| Failure::of("a hull facet has no vertices"))?
                .iter()
                .map(|v| {
                    v.point_id(qh)
                        .ok()
                        .and_then(|id| usize::try_from(id).ok())
                        .filter(|&id| id < points.len())
                        .ok_or_else(|| Failure::of("a hull vertex is no point of the cloud"))
                })
                .collect::<Result<Vec<usize>, _>>()?;
            let corners: [usize; 3] = corners.try_into().map_err(|corners: Vec<usize>| {
                Failure::of(&format!(
                    "a triangulated facet has {} vertices",
                    corners.len()
                ))
            })?;
            let normal = facet
                .normal()
                .ok_or_else(|| Failure::of("a hull facet has no normal"))?;
            let normal = Vector3::new(normal[0], normal[1], normal[2]);
            let offset = match input {
                // Qhull's plane is `normal . x + offset = 0`, negative inside.
                Input::Exact => -facet.offset(),
                Input::Joggled => points
                    .iter()
                    .map(|p| normal.dot(&p.coords))
                    .fold(f64::NEG_INFINITY, f64::max),
            };
            Ok((corners, (normal, offset)))
        })
        .collect::<Result<Vec<_>, Failure>>()?;
    let mut kept: Vec<usize> = facets.iter().flat_map(|(corners, _)| *corners).collect();
    kept.sort_unstable();
    kept.dedup();
    let compact = |point: usize| kept.binary_search(&point).expect("a facet corner is kept");
    Ok(ConvexHull {
        vertices: kept.iter().map(|&point| points[point]).collect(),
        faces: facets
            .iter()
            .map(|(corners, _)| corners.map(compact))
            .collect(),
        planes: facets.iter().map(|&(_, plane)| plane).collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{RngExt, SeedableRng};

    fn pt(x: f64, y: f64, z: f64) -> Point3<f64> {
        Point3::new(x, y, z)
    }

    /// Worst signed distance of `p` past any face plane of `hull` (negative when
    /// strictly inside). The plane metric understates true distance near an edge,
    /// but for "is this point inside a convex hull" it is exact.
    fn worst_violation(hull: &ConvexHull, p: &Point3<f64>) -> f64 {
        hull.planes
            .iter()
            .map(|(normal, offset)| normal.dot(&p.coords) - offset)
            .fold(f64::NEG_INFINITY, f64::max)
    }

    fn contains_every(hull: &ConvexHull, points: &[Point3<f64>]) -> f64 {
        points
            .iter()
            .map(|p| worst_violation(hull, p))
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// From just the eight corners the hull is the clean box: eight vertices,
    /// twelve triangles, all corners inside.
    #[test]
    fn cube_corners_make_a_clean_box() {
        let mut verts = Vec::new();
        for sx in [0.0, 1.0] {
            for sy in [0.0, 1.0] {
                for sz in [0.0, 1.0] {
                    verts.push(pt(sx, sy, sz));
                }
            }
        }
        let hull = exact_hull(&verts).expect("box hull");
        assert_eq!(hull.vertices.len(), 8, "a box has eight hull vertices");
        assert_eq!(hull.faces.len(), 12, "a box triangulates to twelve faces");
        assert!(contains_every(&hull, &verts) <= 1e-12);
    }

    /// A dense grid contains every point, and its hull is the cube of its eight
    /// corners: the points on its faces and edges are no vertices.
    #[test]
    fn a_dense_grid_hull_contains_every_point_and_keeps_the_corners() {
        let mut pts = Vec::new();
        for i in 0..5 {
            for j in 0..5 {
                for k in 0..5 {
                    pts.push(pt(i as f64 / 4.0, j as f64 / 4.0, k as f64 / 4.0));
                }
            }
        }
        let hull = exact_hull(&pts).expect("grid hull");
        assert!(contains_every(&hull, &pts) <= 1e-12, "a grid point escaped");
        for corner in [
            pt(0.0, 0.0, 0.0),
            pt(1.0, 1.0, 1.0),
            pt(1.0, 0.0, 1.0),
            pt(0.0, 1.0, 0.0),
        ] {
            assert!(
                hull.vertices.iter().any(|v| (v - corner).norm() < 1e-9),
                "corner {corner:?} missing"
            );
        }
        assert_eq!(hull.vertices.len(), 8, "only the corners are vertices");
    }

    #[test]
    fn interior_points_are_dropped() {
        let mut pts = vec![
            pt(0.0, 0.0, 0.0),
            pt(1.0, 0.0, 0.0),
            pt(0.0, 1.0, 0.0),
            pt(0.0, 0.0, 1.0),
            pt(1.0, 1.0, 1.0),
        ];
        let mut rng = rand::rngs::StdRng::seed_from_u64(5);
        for _ in 0..200 {
            pts.push(pt(
                rng.random_range(0.05..0.3),
                rng.random_range(0.05..0.3),
                rng.random_range(0.05..0.3),
            ));
        }
        let hull = exact_hull(&pts).expect("hull");
        assert_eq!(
            hull.vertices.len(),
            5,
            "only the five extreme points are vertices"
        );
    }

    #[test]
    fn every_cloud_point_lies_inside_its_hull() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(9);
        for _ in 0..20 {
            let pts: Vec<_> = (0..300)
                .map(|_| {
                    pt(
                        rng.random_range(-1.0..1.0),
                        rng.random_range(-1.0..1.0),
                        rng.random_range(-1.0..1.0),
                    )
                })
                .collect();
            let hull = exact_hull(&pts).expect("hull");
            assert!(
                contains_every(&hull, &pts) <= 1e-9,
                "a cloud point escaped its own hull"
            );
            assert!(
                hull.vertices.len() < pts.len(),
                "hull should compress the cloud"
            );
        }
    }

    /// Every vertex the hull keeps is genuinely extreme: it lies on the surface,
    /// not strictly inside. This is what a welded or incremental fit fails and
    /// what makes the vertex count meaningful as a query-cost proxy.
    #[test]
    fn every_kept_vertex_lies_on_the_surface() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(11);
        let pts: Vec<_> = (0..2000)
            .map(|_| {
                pt(
                    rng.random_range(-1.0..1.0),
                    rng.random_range(-1.0..1.0),
                    rng.random_range(-1.0..1.0),
                )
            })
            .collect();
        let hull = exact_hull(&pts).expect("hull");
        for v in &hull.vertices {
            assert!(
                worst_violation(&hull, v).abs() < 1e-9,
                "vertex {v:?} is not on the surface"
            );
        }
    }

    #[test]
    fn sphere_surface_points_are_all_vertices() {
        // Points on a sphere are all extreme, so all survive as hull vertices.
        let mut pts = Vec::new();
        for i in 0..80 {
            let phi = i as f64 * 0.618 * std::f64::consts::TAU;
            let z = -1.0 + 2.0 * (i as f64 + 0.5) / 80.0;
            let r = (1.0f64 - z * z).sqrt();
            pts.push(pt(r * phi.cos(), r * phi.sin(), z));
        }
        let hull = exact_hull(&pts).expect("sphere hull");
        assert_eq!(
            hull.vertices.len(),
            pts.len(),
            "all sphere points are extreme"
        );
    }

    #[test]
    fn rejects_a_point_that_is_not_finite() {
        let mut corners: Vec<_> = (0..8)
            .map(|i| pt((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64))
            .collect();
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            corners[3].y = bad;
            let error = exact_hull(&corners).expect_err("a point that is not finite");
            assert!(error.contains("finite"), "{error}");
        }
    }

    #[test]
    fn rejects_degenerate_clouds() {
        let flat: Vec<_> = (0..10)
            .flat_map(|i| (0..10).map(move |j| pt(i as f64, j as f64, 0.0)))
            .collect();
        assert!(exact_hull(&flat).is_err(), "a flat cloud has no volume");
        assert!(exact_hull(&[pt(0.0, 0.0, 0.0)]).is_err(), "one point");
        let line: Vec<_> = (0..10).map(|i| pt(i as f64, 0.0, 0.0)).collect();
        assert!(exact_hull(&line).is_err(), "a collinear cloud has no hull");
    }

    /// The error of a cloud Qhull cannot hull names the kind of failure and carries
    /// Qhull's first error message.
    #[test]
    fn a_failure_names_its_kind_and_qhulls_message() {
        let flat: Vec<_> = (0..3)
            .flat_map(|i| (0..3).map(move |j| pt(i as f64, j as f64, 0.0)))
            .collect();
        let error = exact_hull(&flat).expect_err("a flat cloud has no hull");
        assert!(
            error.starts_with("convex hull: Qhull failed on singular input; QH6"),
            "{error}"
        );
    }

    /// A real collision mesh in the production (meter) frame, with coplanar machined
    /// faces: every mesh point ends up inside.
    #[test]
    fn a_real_collision_mesh_hulls_exactly() {
        let raw = crate::stl::load_stl(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../openarm_description/assets/meshes/link6_symp.stl"
        ))
        .expect("mesh");
        let pts: Vec<Point3<f64>> = raw
            .iter()
            .map(|v| pt(v.x * 0.001, v.y * 0.001, v.z * 0.001))
            .collect();
        let hull = exact_hull(&pts).expect("hull");
        assert!(
            contains_every(&hull, &pts) <= 1e-9,
            "a mesh point escaped the exact hull"
        );
        assert!(
            hull.vertices.len() < pts.len() / 4,
            "the hull should be a real compression of {} mesh points, got {}",
            pts.len(),
            hull.vertices.len()
        );
        // Euler on a closed genus-0 triangulation: V - E + F = 2 with E = 3F/2,
        // so F = 2V - 4 exactly, and a hole would show as a facet deficit.
        assert_eq!(
            hull.faces.len(),
            2 * hull.vertices.len() - 4,
            "{} faces over {} vertices is not a closed triangulation",
            hull.faces.len(),
            hull.vertices.len()
        );
    }

    include!("../tests/fixtures/thin_plate.rs");

    /// The dual points of a thin plate's fit, shrunk from a drawn chain: Qhull's merged
    /// hull leaves a point 0.095 outside a facet, so the points are joggled, and the
    /// hull's planes hold every one.
    #[test]
    fn a_cloud_merging_cannot_verify_is_hulled_joggled() {
        let pts: Vec<Point3<f64>> = THIN_PLATE_DUAL_POINTS
            .iter()
            .map(|&[x, y, z]| pt(x, y, z))
            .collect();
        let hull = exact_hull(&pts).expect("hull");
        let escape = contains_every(&hull, &pts);
        assert!(escape <= 0.0, "a point escaped its hull by {escape}");
    }

    /// The hull does not depend on the caller's point order.
    #[test]
    fn construction_is_order_independent() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(21);
        let pts: Vec<_> = (0..500)
            .map(|_| {
                pt(
                    rng.random_range(-1.0..1.0),
                    rng.random_range(-1.0..1.0),
                    rng.random_range(-1.0..1.0),
                )
            })
            .collect();
        let a = exact_hull(&pts).expect("hull");
        let mut shuffled = pts.clone();
        shuffled.reverse();
        let b = exact_hull(&shuffled).expect("hull");
        assert_eq!(a.vertices, b.vertices);
        assert_eq!(a.faces, b.faces);
    }
}
