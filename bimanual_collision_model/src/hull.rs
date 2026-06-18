//! Convex hull of a point cloud, computed once at construction to turn a mesh
//! of tens of thousands of triangles into the handful of vertices GJK needs for
//! its support function. The hull strictly contains the mesh (every mesh point
//! is a convex combination of the cloud, so it lies inside the cloud's hull),
//! so its distance is a true lower bound on the mesh distance while the proxy
//! follows the shape far more tightly than a single bounding primitive.
//!
//! Incremental construction (the classic Clarkson-Shor / "Quickhull"-family
//! online insertion): seed a tetrahedron, then fold each remaining point in,
//! deleting the faces it can see and stitching new faces across the horizon.
//! Face orientation is kept robust by pointing every normal away from a fixed
//! interior point instead of tracking winding, so the horizon is found purely
//! by counting shared edges.

use std::collections::HashMap;

use srs_model::nalgebra::{Point3, Vector3};

/// A point in front of a face by more than this (metres) sees it. At 1e-9 a
/// point is classified inside only when it is inside to within a nanometre, far
/// under any millimetre-scale safety threshold.
const FRONT_EPS: f64 = 1e-9;

/// Degenerate area/length guard: a face or spanning direction below this in
/// squared magnitude is treated as collapsed.
const DEGEN_EPS2: f64 = 1e-20;

/// Most repair rounds [`build_hull`] will run re-inserting protruding points
/// before giving up. Real meshes converge in a handful; exceeding this means a
/// pathological cloud, and erroring beats silently returning a hull that does
/// not contain its own points.
const REPAIR_ROUNDS: usize = 16;

/// The convex hull as outward-oriented triangles over a deduplicated vertex
/// list. `vertices` is what GJK needs; `faces` index into it for rendering.
#[derive(Debug, Clone)]
pub struct ConvexHull {
    pub vertices: Vec<Point3<f64>>,
    pub faces: Vec<[usize; 3]>,
}

/// A convex hull paired with the inflation radius that, swept around it,
/// re-contains the source mesh the hull was simplified from. The radius is the
/// containment guarantee, so it travels with the hull rather than as a loose
/// scalar a caller could forget to apply.
#[derive(Debug, Clone)]
pub struct RoundedHull {
    pub hull: ConvexHull,
    pub radius: f64,
}

/// A working face: vertex indices into the source cloud and an outward normal
/// (pointing away from the hull interior) with its plane offset.
#[derive(Clone, Copy)]
struct Face {
    v: [usize; 3],
    normal: Vector3<f64>,
    offset: f64,
}

impl Face {
    /// Signed distance of a point from the face plane, positive outward.
    fn signed_distance(&self, p: &Point3<f64>) -> f64 {
        self.normal.dot(&p.coords) - self.offset
    }
}

/// Exact convex hull of `points` (minimal tolerance, every point on or inside).
/// Errors only on a cloud that spans fewer than three dimensions (collinear or
/// coplanar), which no solid collision mesh is.
pub fn convex_hull(points: &[Point3<f64>]) -> Result<ConvexHull, String> {
    build_hull(points, FRONT_EPS)
}

/// A simplified hull: points are welded onto a grid of cell size `cell` before
/// hulling, and the returned `radius` re-contains the mesh. Welding both shrinks
/// the cloud (tens of thousands of vertices fall to a few thousand, so the hull
/// builds fast) and thins near-duplicate hull vertices (cheap support). Every
/// welded-away point sits within `radius` of its cell's kept representative, so
/// the radius is the exact worst weld displacement, tracked in one pass with no
/// scan, and the rounded hull strictly contains the mesh (the standard
/// shrink-then-inflate margin trick).
pub fn simplified_hull(points: &[Point3<f64>], cell: f64) -> Result<RoundedHull, String> {
    if !(cell.is_finite() && cell > 0.0) {
        return Err(format!("simplification cell must be finite and positive, got {cell}"));
    }
    let mut reps: HashMap<[i64; 3], Point3<f64>> = HashMap::new();
    let mut weld = 0.0_f64;
    for p in points {
        let key = [(p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64];
        match reps.entry(key) {
            std::collections::hash_map::Entry::Vacant(e) => {
                e.insert(*p);
            }
            std::collections::hash_map::Entry::Occupied(e) => {
                weld = weld.max((p - e.get()).norm());
            }
        }
    }
    let rep_points: Vec<Point3<f64>> = reps.into_values().collect();
    let hull = build_hull(&rep_points, FRONT_EPS)?;
    // The inflation must cover the weld displacement AND any residual hull build
    // defect (a rep left a hair outside its own faces by the float predicates),
    // so the rounded hull provably contains every original point: point to rep
    // within `weld`, rep to hull within its protrusion, hence point to hull
    // within the sum.
    let radius = weld + max_protrusion(&hull, &rep_points);
    Ok(RoundedHull { hull, radius })
}

/// Outward face planes (unit normal, plane offset) of a hull, oriented away from
/// the vertex centroid (interior to a convex hull).
fn face_planes(hull: &ConvexHull) -> Vec<(Vector3<f64>, f64)> {
    let interior = Point3::from(hull.vertices.iter().fold(Vector3::zeros(), |a, v| a + v.coords) / hull.vertices.len() as f64);
    hull.faces
        .iter()
        .filter_map(|f| {
            let (a, b, c) = (hull.vertices[f[0]], hull.vertices[f[1]], hull.vertices[f[2]]);
            let n = (b - a).cross(&(c - a));
            if n.norm_squared() <= DEGEN_EPS2 {
                return None;
            }
            let mut normal = n.normalize();
            if normal.dot(&(interior - a)) > 0.0 {
                normal = -normal;
            }
            Some((normal, normal.dot(&a.coords)))
        })
        .collect()
}

/// Farthest any of `points` protrudes past the hull's face planes (zero if all
/// inside): how far the inflation must reach to re-contain them.
fn max_protrusion(hull: &ConvexHull, points: &[Point3<f64>]) -> f64 {
    let planes = face_planes(hull);
    points
        .iter()
        .map(|p| planes.iter().map(|(n, off)| n.dot(&p.coords) - off).fold(f64::NEG_INFINITY, f64::max).max(0.0))
        .fold(0.0, f64::max)
}

/// A convex collision piece a caller supplies for a body in place of the
/// auto-fit hull: the convex hull of its points is taken as the piece. A body's
/// supplied pieces must together contain that body's mesh, which is checked at
/// build time. [`aabb`](Self::aabb) gives the eight corners of an axis-aligned
/// box, usually all a blocky concave body like a torso needs.
#[derive(Clone, Debug)]
pub struct ConvexPiece {
    points: Vec<Point3<f64>>,
}

impl ConvexPiece {
    /// A piece spanning the convex hull of `points`.
    pub fn from_points(points: Vec<Point3<f64>>) -> ConvexPiece {
        ConvexPiece { points }
    }

    /// The axis-aligned box between `min` and `max`, as its eight corners.
    pub fn aabb(min: Point3<f64>, max: Point3<f64>) -> ConvexPiece {
        let mut points = Vec::with_capacity(8);
        for x in [min.x, max.x] {
            for y in [min.y, max.y] {
                for z in [min.z, max.z] {
                    points.push(Point3::new(x, y, z));
                }
            }
        }
        ConvexPiece { points }
    }

    pub(crate) fn points(&self) -> &[Point3<f64>] {
        &self.points
    }
}

/// Incremental hull with a tunable visibility tolerance: a point in front of a
/// face by more than `front_eps` sees it (and so becomes a vertex), so a larger
/// `front_eps` keeps fewer, more separated vertices.
fn build_hull(points: &[Point3<f64>], front_eps: f64) -> Result<ConvexHull, String> {
    // Sort the cloud so the insertion order, and thus the exact hull and any
    // residual the repair pass leaves, is independent of how the caller (a
    // HashMap of grid cells) happened to order it. Construction is then
    // reproducible run to run.
    let mut sorted = points.to_vec();
    sorted.sort_by(|p, q| p.x.total_cmp(&q.x).then_with(|| p.y.total_cmp(&q.y)).then_with(|| p.z.total_cmp(&q.z)));
    let points: &[Point3<f64>] = &sorted;

    let seed = initial_tetrahedron(points)?;
    let interior = seed.iter().fold(Vector3::zeros(), |acc, &i| acc + points[i].coords) / 4.0;
    let interior = Point3::from(interior);

    let mut faces: Vec<Face> = Vec::new();
    for &[i, j, k] in &[[seed[0], seed[1], seed[2]], [seed[0], seed[1], seed[3]], [seed[0], seed[2], seed[3]], [seed[1], seed[2], seed[3]]] {
        if let Some(f) = make_face(i, j, k, points, &interior) {
            faces.push(f);
        }
    }

    for idx in 0..points.len() {
        insert_point(idx, points, &mut faces, &interior, front_eps);
    }
    // Repair: a single insertion pass is not robust on real meshes. Float
    // predicates can leave a point a hair outside a near-parallel face, so a
    // vertex can end up protruding past its own hull. Re-insert every point
    // still outside until none remain, which makes the hull genuinely convex and
    // (since the points are what it is built from) guarantees it contains them
    // to within `front_eps`. Erroring on non-convergence keeps a bad hull from
    // silently under-containing the mesh downstream.
    for round in 0.. {
        let outside: Vec<usize> = (0..points.len()).filter(|&i| faces.iter().any(|f| f.signed_distance(&points[i]) > front_eps)).collect();
        if outside.is_empty() {
            break;
        }
        if round == REPAIR_ROUNDS {
            return Err(format!("convex hull did not converge in {REPAIR_ROUNDS} repair rounds, {} points still outside", outside.len()));
        }
        for idx in outside {
            insert_point(idx, points, &mut faces, &interior, front_eps);
        }
    }

    Ok(reindex(points, &faces))
}

/// Fold one point into the hull: delete the faces it can see, then stitch it to
/// the horizon they leave behind. A no-op if the point is already inside.
fn insert_point(idx: usize, points: &[Point3<f64>], faces: &mut Vec<Face>, interior: &Point3<f64>, front_eps: f64) {
    let p = &points[idx];
    let visible: Vec<usize> = faces.iter().enumerate().filter(|(_, f)| f.signed_distance(p) > front_eps).map(|(i, _)| i).collect();
    if visible.is_empty() {
        return;
    }
    let horizon = horizon_edges(faces, &visible);
    let dropped: std::collections::HashSet<usize> = visible.into_iter().collect();
    *faces = std::mem::take(faces).into_iter().enumerate().filter(|(i, _)| !dropped.contains(i)).map(|(_, f)| f).collect();
    for (a, b) in horizon {
        if let Some(f) = make_face(a, b, idx, points, interior) {
            faces.push(f);
        }
    }
}

/// Undirected edges that border exactly one visible face: the boundary between
/// the deleted cap and the rest of the hull, where the new faces attach.
fn horizon_edges(faces: &[Face], visible: &[usize]) -> Vec<(usize, usize)> {
    let mut count: HashMap<(usize, usize), u32> = HashMap::new();
    for &fi in visible {
        let v = faces[fi].v;
        for (a, b) in [(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            *count.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0) += 1;
        }
    }
    count.into_iter().filter(|&(_, c)| c == 1).map(|(e, _)| e).collect()
}

/// A face on `i, j, k`, its normal flipped to point away from `interior`.
/// `None` if the three points are collinear (zero area).
fn make_face(i: usize, j: usize, k: usize, points: &[Point3<f64>], interior: &Point3<f64>) -> Option<Face> {
    let n = (points[j] - points[i]).cross(&(points[k] - points[i]));
    if n.norm_squared() <= DEGEN_EPS2 {
        return None;
    }
    let mut normal = n.normalize();
    if normal.dot(&(interior - points[i])) > 0.0 {
        normal = -normal;
    }
    Some(Face { v: [i, j, k], normal, offset: normal.dot(&points[i].coords) })
}

/// Four affinely independent seed points: an extreme point, the farthest from
/// it, the farthest from that line, the farthest from that plane.
fn initial_tetrahedron(points: &[Point3<f64>]) -> Result<[usize; 4], String> {
    if points.len() < 4 {
        return Err(format!("a hull needs at least four points, got {}", points.len()));
    }
    let i0 = (0..points.len()).max_by(|&a, &b| points[a].x.total_cmp(&points[b].x)).expect("nonempty");
    let i1 = farthest(points, |p| (p - points[i0]).norm_squared()).ok_or("cloud is a single point")?;
    let axis = points[i1] - points[i0];
    if axis.norm_squared() <= DEGEN_EPS2 {
        return Err("cloud is a single point".into());
    }
    let i2 = farthest(points, |p| (p - points[i0]).cross(&axis).norm_squared()).ok_or("collinear cloud")?;
    let normal = (points[i1] - points[i0]).cross(&(points[i2] - points[i0]));
    if normal.norm_squared() <= DEGEN_EPS2 {
        return Err("collinear cloud has no hull".into());
    }
    let i3 = farthest(points, |p| (p - points[i0]).dot(&normal).abs()).ok_or("coplanar cloud")?;
    if (points[i3] - points[i0]).dot(&normal).abs() <= FRONT_EPS {
        return Err("coplanar cloud has no volume".into());
    }
    Ok([i0, i1, i2, i3])
}

/// Index of the point maximizing `score`, or `None` if every score is zero
/// (the cloud is degenerate along this measure).
fn farthest(points: &[Point3<f64>], score: impl Fn(&Point3<f64>) -> f64) -> Option<usize> {
    let (idx, best) = (0..points.len()).map(|i| (i, score(&points[i]))).max_by(|a, b| a.1.total_cmp(&b.1))?;
    if best <= DEGEN_EPS2 { None } else { Some(idx) }
}

/// Compact the surviving faces to a deduplicated vertex list.
fn reindex(points: &[Point3<f64>], faces: &[Face]) -> ConvexHull {
    let mut remap: HashMap<usize, usize> = HashMap::new();
    let mut vertices = Vec::new();
    let mut out_faces = Vec::new();
    for f in faces {
        let tri = f.v.map(|old| {
            *remap.entry(old).or_insert_with(|| {
                vertices.push(points[old]);
                vertices.len() - 1
            })
        });
        out_faces.push(tri);
    }
    ConvexHull { vertices, faces: out_faces }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{Rng, SeedableRng};

    fn pt(x: f64, y: f64, z: f64) -> Point3<f64> {
        Point3::new(x, y, z)
    }

    fn contains(hull: &ConvexHull, p: &Point3<f64>, interior: &Point3<f64>) -> bool {
        // Inside iff behind every face plane (normals point outward).
        hull.faces.iter().all(|tri| {
            let (a, b, c) = (hull.vertices[tri[0]], hull.vertices[tri[1]], hull.vertices[tri[2]]);
            let mut n = (b - a).cross(&(c - a));
            if n.dot(&(interior - a)) > 0.0 {
                n = -n;
            }
            n.dot(&(p - a)) <= 1e-9 * n.norm().max(1.0)
        })
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
        let hull = convex_hull(&verts).expect("box hull");
        assert_eq!(hull.vertices.len(), 8, "a box has eight hull vertices");
        assert_eq!(hull.faces.len(), 12, "a box triangulates to twelve faces");
        let center = pt(0.5, 0.5, 0.5);
        for p in &verts {
            assert!(contains(&hull, p, &center));
        }
    }

    /// A dense grid still contains every point and keeps the corners. Coplanar
    /// face points may survive as vertices (a known property of plain
    /// incremental insertion), so the count is not asserted minimal, only that
    /// the hull is correct and a real compression.
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
        let hull = convex_hull(&pts).expect("grid hull");
        let center = pt(0.5, 0.5, 0.5);
        for p in &pts {
            assert!(contains(&hull, p, &center), "grid point {p:?} escaped the hull");
        }
        for corner in [pt(0.0, 0.0, 0.0), pt(1.0, 1.0, 1.0), pt(1.0, 0.0, 1.0), pt(0.0, 1.0, 0.0)] {
            assert!(hull.vertices.iter().any(|v| (v - corner).norm() < 1e-9), "corner {corner:?} missing");
        }
        assert!(hull.vertices.len() < pts.len(), "hull should compress the cloud");
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
        // Pile of interior points that must not become vertices.
        let mut rng = rand::rngs::StdRng::seed_from_u64(5);
        for _ in 0..200 {
            pts.push(pt(rng.gen_range(0.05..0.3), rng.gen_range(0.05..0.3), rng.gen_range(0.05..0.3)));
        }
        let hull = convex_hull(&pts).expect("hull");
        assert_eq!(hull.vertices.len(), 5, "only the five extreme points are vertices");
    }

    #[test]
    fn every_cloud_point_lies_inside_its_hull() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(9);
        for _ in 0..20 {
            let pts: Vec<_> = (0..300)
                .map(|_| pt(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0)))
                .collect();
            let hull = convex_hull(&pts).expect("hull");
            let center = Point3::from(pts.iter().fold(Vector3::zeros(), |a, p| a + p.coords) / pts.len() as f64);
            for p in &pts {
                assert!(contains(&hull, p, &center), "a cloud point escaped its own hull");
            }
            // A random cloud hull is far smaller than the cloud.
            assert!(hull.vertices.len() < pts.len(), "hull should compress the cloud");
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
        let hull = convex_hull(&pts).expect("sphere hull");
        assert_eq!(hull.vertices.len(), pts.len(), "all sphere points are extreme");
    }

    #[test]
    fn rejects_a_coplanar_cloud() {
        let flat: Vec<_> = (0..10).flat_map(|i| (0..10).map(move |j| pt(i as f64, j as f64, 0.0))).collect();
        assert!(convex_hull(&flat).is_err(), "a flat cloud has no volume");
    }

    #[test]
    fn simplified_hull_welds_points_and_its_radius_recontains() {
        // Dense fibonacci sphere: welding at 0.12 collapses neighbours, and the
        // radius (bounded by a cell diagonal) re-contains every original point.
        let mut pts = Vec::new();
        for i in 0..4000 {
            let phi = i as f64 * 0.618 * std::f64::consts::TAU;
            let z = -1.0 + 2.0 * (i as f64 + 0.5) / 4000.0;
            let r = (1.0f64 - z * z).sqrt();
            pts.push(pt(r * phi.cos(), r * phi.sin(), z));
        }
        let exact = convex_hull(&pts).expect("exact hull");
        let RoundedHull { hull: simp, radius } = simplified_hull(&pts, 0.12).expect("simplified hull");
        assert!(simp.vertices.len() < exact.vertices.len(), "welding should drop vertices");
        assert!(radius > 0.0 && radius <= 0.12 * 3.0f64.sqrt() + 1e-9, "radius {radius} within a cell diagonal");
        // Every original point within `radius` of the simplified hull by the
        // per-face metric. That metric is a lower bound on the true distance, so
        // this is a necessary check; the true-distance (GJK) containment test
        // below is the sufficient one.
        let interior = Point3::from(simp.vertices.iter().fold(Vector3::zeros(), |a, v| a + v.coords) / simp.vertices.len() as f64);
        for p in &pts {
            let protrusion = simp
                .faces
                .iter()
                .map(|f| {
                    let (a, b, c) = (simp.vertices[f[0]], simp.vertices[f[1]], simp.vertices[f[2]]);
                    let mut n = (b - a).cross(&(c - a)).normalize();
                    if n.dot(&(interior - a)) > 0.0 {
                        n = -n;
                    }
                    n.dot(&(p - a))
                })
                .fold(f64::NEG_INFINITY, f64::max);
            assert!(protrusion <= radius + 1e-9, "point protrudes {protrusion} past radius {radius}");
        }
    }

    #[test]
    fn simplified_hull_contains_a_real_collision_mesh() {
        // A real collision mesh in the production (metre) frame: the incremental
        // hull is prone to a float-predicate dent on data like this, so this
        // pins the repair + inflation that guarantee containment regardless.
        let raw = crate::stl::load_stl(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/meshes/link6_symp.stl")).expect("mesh");
        let pts: Vec<Point3<f64>> = raw.iter().map(|v| pt(v.x * 0.001, v.y * 0.001, v.z * 0.001)).collect();
        let RoundedHull { hull, radius } = simplified_hull(&pts, 0.004).expect("hull");
        let planes = face_planes(&hull);
        for p in &pts {
            let protrusion = planes.iter().map(|(n, off)| n.dot(&p.coords) - off).fold(f64::NEG_INFINITY, f64::max);
            assert!(protrusion <= radius + 1e-9, "mesh point protrudes {protrusion} past inflation {radius}");
        }
        // The repair leaves the hull convex: its own vertices do not protrude.
        assert!(max_protrusion(&hull, &hull.vertices) < 1e-6, "hull not convex after repair");
    }

    #[test]
    fn rounded_hull_contains_a_real_mesh_by_true_distance() {
        // The checks above use the per-face plane metric, which is only a lower
        // bound on the true distance to the hull. Verify containment with an
        // independent, exact metric: the crate's own GJK, which gives the true
        // distance from each mesh point to the rounded hull. A point inside the
        // rounded hull reads a non-positive signed distance.
        let raw = crate::stl::load_stl(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/meshes/link6_symp.stl")).expect("mesh");
        let pts: Vec<Point3<f64>> = raw.iter().map(|v| pt(v.x * 0.001, v.y * 0.001, v.z * 0.001)).collect();
        let RoundedHull { hull, radius } = simplified_hull(&pts, 0.004).expect("hull");
        let body = crate::gjk::Hull::new(&hull, radius).expect("gjk hull");
        for p in &pts {
            let point = crate::gjk::Hull::new(&ConvexHull { vertices: vec![*p], faces: vec![] }, 0.0).expect("point");
            let d = crate::gjk::distance(&point, &body).distance;
            assert!(d <= 1e-9, "mesh point at true distance {d:+} lies outside the rounded hull (radius {radius})");
        }
    }

    #[test]
    fn aabb_piece_has_the_eight_box_corners() {
        let piece = ConvexPiece::aabb(pt(-1.0, -2.0, -3.0), pt(1.0, 2.0, 3.0));
        let hull = convex_hull(piece.points()).expect("box hull");
        assert_eq!(hull.vertices.len(), 8, "a box has eight hull vertices");
        for c in piece.points() {
            assert!(hull.vertices.iter().any(|v| (v - c).norm() < 1e-12), "corner {c:?} missing");
        }
    }
}
