//! Gilbert-Johnson-Keerthi distance between convex bodies.
//!
//! GJK computes the distance between two convex shapes from nothing but their
//! support functions (the farthest point in a direction). It works on the
//! Minkowski difference `A - B`: the shapes are disjoint exactly when the
//! origin lies outside it, and their separation is the distance from the origin
//! to it. The algorithm walks a simplex (1 to 4 points of the difference)
//! toward the origin, each step adding the support point in the direction of
//! the origin and dropping the vertices that no longer carry the closest point,
//! until no support point lies closer (Gilbert, Johnson & Keerthi 1988; van den
//! Bergen, "Collision Detection in Interactive 3D Environments"; the simplex
//! sub-distance follows Ericson, "Real-Time Collision Detection", ch. 5).
//!
//! Distance is computed on the unrounded cores, then the radii are subtracted
//! (the standard margin trick): a [`Capsule`] is a segment of radius `r`, so
//! GJK on the two segments minus the two radii reproduces
//! [`Capsule::distance_to`] exactly, into mild penetration. Penetration deeper
//! than the cores (cores overlapping) returns zero core distance; recovering
//! depth there needs EPA and is out of scope for a proximity guardrail, which
//! treats any overlap as a full stop.

use std::cell::Cell;

use srs_model::nalgebra::{Point3, Vector3};

use crate::geometry::Capsule;
use crate::hull::ConvexHull;

/// Relative duality-gap tolerance: stop when the support point beyond the
/// current closest point `v` improves the origin distance by less than this
/// fraction of `v . v`. At 1e-10 the distance is tight to roughly float
/// precision without spinning on shapes whose surfaces are nearly flat.
const REL_TOL: f64 = 1e-10;

/// Squared length below which a vector is treated as the origin: cores touch
/// (1e-10 m)^2, the same surface round-off scale the geometry module uses.
const ORIGIN_EPS2: f64 = 1e-20;

/// Iteration cap. GJK on simple primitives converges in well under ten steps;
/// this only backstops a numerical stall, never a normal query.
const MAX_ITERS: u32 = 32;

/// A convex body usable by [`distance`]: its core support function (the
/// farthest core point along a direction) and an optional rounding radius
/// swept around that core.
pub trait Support {
    /// Farthest point of the unrounded core in direction `dir` (need not be
    /// normalized).
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64>;

    /// Radius swept around the core (zero for a polytope).
    fn radius(&self) -> f64 {
        0.0
    }
}

/// A convex polytope GJK primitive: hull vertices, their edge adjacency, and an
/// optional rounding `radius`. Support is found by hill-climbing the edge graph
/// (a linear objective on a convex polytope has no local maxima but the global
/// one), warm-started from the previous query through `hint`, so support costs
/// amortized O(1) instead of O(vertices). The `hint` makes a `Hull` not `Sync`;
/// share one per thread.
#[derive(Debug)]
pub struct Hull {
    vertices: Vec<Point3<f64>>,
    neighbors: Vec<Vec<u32>>,
    radius: f64,
    hint: Cell<u32>,
}

/// Result of a GJK query: signed surface distance (negative is penetration of
/// the rounded shapes), the closest point on each surface, and the iteration
/// count for profiling.
#[derive(Debug, Clone)]
pub struct GjkDistance {
    pub distance: f64,
    pub on_a: Point3<f64>,
    pub on_b: Point3<f64>,
    pub iterations: u32,
}

/// A vertex of the Minkowski difference, carrying the two core support points
/// it came from so the witnesses can be recovered by the same barycentric
/// weights that locate the closest point.
#[derive(Debug, Clone, Copy)]
struct SupportPoint {
    v: Vector3<f64>,
    a: Point3<f64>,
    b: Point3<f64>,
}

/// The closest point of a sub-simplex to the origin: the point as a vector from
/// the origin, the carrying sub-simplex, and its barycentric weights.
type Closest = (Vector3<f64>, Vec<SupportPoint>, Vec<f64>);

impl Hull {
    /// A GJK primitive from a computed [`ConvexHull`], building the vertex edge
    /// adjacency its hill-climbing support walks. Errors on an empty hull.
    pub fn new(hull: &ConvexHull, radius: f64) -> Result<Hull, String> {
        if hull.vertices.is_empty() {
            return Err("cannot build a hull from zero vertices".into());
        }
        let mut neighbors = vec![Vec::new(); hull.vertices.len()];
        for f in &hull.faces {
            for (a, b) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
                if !neighbors[a].contains(&(b as u32)) {
                    neighbors[a].push(b as u32);
                }
                if !neighbors[b].contains(&(a as u32)) {
                    neighbors[b].push(a as u32);
                }
            }
        }
        Ok(Hull { vertices: hull.vertices.clone(), neighbors, radius, hint: Cell::new(0) })
    }

    /// The hull vertices, for placement and rendering.
    pub fn vertices(&self) -> &[Point3<f64>] {
        &self.vertices
    }
}

impl Support for Hull {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        let mut best = self.hint.get() as usize;
        let mut best_dot = self.vertices[best].coords.dot(dir);
        loop {
            let mut improved = false;
            for &n in &self.neighbors[best] {
                let dot = self.vertices[n as usize].coords.dot(dir);
                if dot > best_dot {
                    (best, best_dot, improved) = (n as usize, dot, true);
                }
            }
            if !improved {
                break;
            }
        }
        self.hint.set(best as u32);
        self.vertices[best]
    }

    fn radius(&self) -> f64 {
        self.radius
    }
}

impl Support for Capsule {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        if self.a.coords.dot(dir) >= self.b.coords.dot(dir) { self.a } else { self.b }
    }

    fn radius(&self) -> f64 {
        self.radius
    }
}

/// Signed surface distance between two convex bodies and the closest points.
/// Runs GJK on the cores, then subtracts the radii and pushes the witnesses out
/// to the rounded surfaces along the separating direction.
pub fn distance(a: &impl Support, b: &impl Support) -> GjkDistance {
    let support = |dir: &Vector3<f64>| -> SupportPoint {
        let pa = a.core_support(dir);
        let pb = b.core_support(&(-dir));
        SupportPoint { v: pa.coords - pb.coords, a: pa, b: pb }
    };

    let mut simplex = vec![support(&Vector3::x())];
    let mut weights = vec![1.0];
    let mut v = simplex[0].v;
    let mut iterations = 0;

    while v.norm_squared() > ORIGIN_EPS2 {
        let w = support(&(-v));
        let vv = v.norm_squared();
        // Duality gap: ||v|| - (v . w)/||v|| as a fraction of ||v||. Once the
        // farthest point toward the origin is no closer than v's plane, v is
        // the answer.
        if vv - v.dot(&w.v) <= REL_TOL * vv {
            break;
        }
        // A repeated support direction means no new vertex is reachable.
        if simplex.iter().any(|s| (s.v - w.v).norm_squared() <= ORIGIN_EPS2) {
            break;
        }
        simplex.push(w);
        let (next_v, kept, kept_weights) = closest_to_origin(&simplex);
        simplex = kept;
        weights = kept_weights;
        v = next_v;
        iterations += 1;
        if iterations >= MAX_ITERS {
            break;
        }
    }

    let core_a = weighted_point(&simplex, &weights, |s| s.a);
    let core_b = weighted_point(&simplex, &weights, |s| s.b);
    let core_dist = v.norm();
    let (ra, rb) = (a.radius(), b.radius());
    let (on_a, on_b) = if core_dist > ORIGIN_EPS2.sqrt() {
        let n = v / core_dist;
        (core_a - n * ra, core_b + n * rb)
    } else {
        (core_a, core_b)
    };
    GjkDistance { distance: core_dist - ra - rb, on_a, on_b, iterations }
}

/// Barycentric blend of a chosen core point over the simplex.
fn weighted_point(simplex: &[SupportPoint], weights: &[f64], pick: impl Fn(&SupportPoint) -> Point3<f64>) -> Point3<f64> {
    let blended = simplex.iter().zip(weights).fold(Vector3::zeros(), |acc, (s, &w)| acc + pick(s).coords * w);
    Point3::from(blended)
}

/// Closest point of the simplex's convex hull to the origin, reduced to the
/// sub-simplex that carries it with its barycentric weights.
fn closest_to_origin(simplex: &[SupportPoint]) -> Closest {
    match simplex {
        [a] => (a.v, vec![*a], vec![1.0]),
        [a, b] => closest_segment(*a, *b),
        [a, b, c] => closest_triangle(*a, *b, *c),
        [a, b, c, d] => closest_tetrahedron(*a, *b, *c, *d),
        _ => unreachable!("GJK simplex holds one to four points"),
    }
}

fn closest_segment(a: SupportPoint, b: SupportPoint) -> Closest {
    let ab = b.v - a.v;
    let len2 = ab.norm_squared();
    if len2 <= ORIGIN_EPS2 {
        return (a.v, vec![a], vec![1.0]);
    }
    let t = (-a.v.dot(&ab) / len2).clamp(0.0, 1.0);
    if t <= 0.0 {
        (a.v, vec![a], vec![1.0])
    } else if t >= 1.0 {
        (b.v, vec![b], vec![1.0])
    } else {
        (a.v + ab * t, vec![a, b], vec![1.0 - t, t])
    }
}

/// Closest point on triangle `abc` to the origin (Ericson 5.1.5, query point at
/// the origin), returned as the carrying sub-simplex and weights.
fn closest_triangle(a: SupportPoint, b: SupportPoint, c: SupportPoint) -> Closest {
    let (pa, pb, pc) = (a.v, b.v, c.v);
    let ab = pb - pa;
    let ac = pc - pa;

    // Vertex regions. `ap = origin - pa = -pa`, and so on.
    let d1 = ab.dot(&-pa);
    let d2 = ac.dot(&-pa);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (pa, vec![a], vec![1.0]);
    }
    let d3 = ab.dot(&-pb);
    let d4 = ac.dot(&-pb);
    if d3 >= 0.0 && d4 <= d3 {
        return (pb, vec![b], vec![1.0]);
    }
    let d5 = ab.dot(&-pc);
    let d6 = ac.dot(&-pc);
    if d6 >= 0.0 && d5 <= d6 {
        return (pc, vec![c], vec![1.0]);
    }

    // Edge regions.
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        return (pa + ab * t, vec![a, b], vec![1.0 - t, t]);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        return (pa + ac * t, vec![a, c], vec![1.0 - t, t]);
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (pb + (pc - pb) * t, vec![b, c], vec![1.0 - t, t]);
    }

    // Face interior.
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (pa + ab * v + ac * w, vec![a, b, c], vec![1.0 - v - w, v, w])
}

/// Closest point on tetrahedron `abcd` to the origin (Ericson 5.1.6): the
/// closest point over whichever faces the origin lies outside of, or the origin
/// itself (zero, overlap) when it lies inside all four.
fn closest_tetrahedron(
    a: SupportPoint,
    b: SupportPoint,
    c: SupportPoint,
    d: SupportPoint,
) -> Closest {
    let mut best: Option<(f64, Closest)> = None;
    // Each face listed with the opposite vertex, which fixes the inward side.
    for (p, q, r, opp) in [(a, b, c, d), (a, c, d, b), (a, d, b, c), (b, d, c, a)] {
        if !origin_outside_plane(p.v, q.v, r.v, opp.v) {
            continue;
        }
        let face = closest_triangle(p, q, r);
        let d2 = face.0.norm_squared();
        if best.as_ref().is_none_or(|(bd2, _)| d2 < *bd2) {
            best = Some((d2, face));
        }
    }
    // Inside all faces: the origin is enclosed, the cores overlap.
    best.map_or_else(|| (Vector3::zeros(), vec![a, b, c, d], vec![0.25; 4]), |(_, face)| face)
}

/// Whether the origin lies on the far side of plane `pqr` from `opp` (so the
/// face can carry the closest point). A degenerate face is never outside.
fn origin_outside_plane(p: Vector3<f64>, q: Vector3<f64>, r: Vector3<f64>, opp: Vector3<f64>) -> bool {
    let n = (q - p).cross(&(r - p));
    let origin_side = -p.dot(&n);
    let opp_side = (opp - p).dot(&n);
    origin_side * opp_side < 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::segment_segment_closest;
    use rand::{Rng, SeedableRng};
    use srs_model::nalgebra::{Isometry3, Translation3, UnitQuaternion};

    fn pt(x: f64, y: f64, z: f64) -> Point3<f64> {
        Point3::new(x, y, z)
    }

    /// Axis-aligned box of half-extent `h` centered at `c`, as eight vertices.
    fn box_hull(c: Point3<f64>, h: f64, radius: f64) -> Hull {
        let mut verts = Vec::new();
        for sx in [-1.0, 1.0] {
            for sy in [-1.0, 1.0] {
                for sz in [-1.0, 1.0] {
                    verts.push(pt(c.x + sx * h, c.y + sy * h, c.z + sz * h));
                }
            }
        }
        Hull::new(&crate::hull::convex_hull(&verts).expect("box hull"), radius).expect("box")
    }

    #[test]
    fn matches_closed_form_capsule_distance() {
        // The anchor: GJK on two segments minus the radii must equal the
        // closed-form capsule distance, separated and into mild penetration.
        let mut rng = rand::rngs::StdRng::seed_from_u64(11);
        let rand_cap = |rng: &mut rand::rngs::StdRng| Capsule {
            a: pt(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0)),
            b: pt(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0)),
            radius: rng.gen_range(0.02..0.3),
        };
        for _ in 0..2000 {
            let c1 = rand_cap(&mut rng);
            let c2 = rand_cap(&mut rng);
            let closed = c1.distance_to(&c2).distance;
            let gjk = distance(&c1, &c2);
            // Only compare where the cores (segments) do not cross; past that
            // GJK reports zero core distance by design.
            let core = segment_segment_closest(c1.a, c1.b, c2.a, c2.b).0;
            if core > 1e-6 {
                assert!((gjk.distance - closed).abs() < 1e-9, "gjk {} vs closed {}", gjk.distance, closed);
            }
        }
    }

    #[test]
    fn witnesses_lie_on_surfaces_and_span_the_distance() {
        let c1 = Capsule { a: pt(0.0, 0.0, 0.0), b: pt(1.0, 0.0, 0.0), radius: 0.2 };
        let c2 = Capsule { a: pt(0.0, 1.0, 0.0), b: pt(1.0, 1.0, 0.0), radius: 0.3 };
        let r = distance(&c1, &c2);
        assert!((r.distance - 0.5).abs() < 1e-9);
        // Witnesses are the surface gap and span exactly the reported distance.
        assert!(((r.on_b - r.on_a).norm() - r.distance).abs() < 1e-9);
        assert!((r.on_a.y - 0.2).abs() < 1e-9, "on_a on c1 surface, got {}", r.on_a.y);
        assert!((r.on_b.y - 0.7).abs() < 1e-9, "on_b on c2 surface, got {}", r.on_b.y);
    }

    #[test]
    fn sphere_sphere_is_center_distance_minus_radii() {
        let s1 = Capsule { a: pt(0.0, 0.0, 0.0), b: pt(0.0, 0.0, 0.0), radius: 0.5 };
        let s2 = Capsule { a: pt(2.0, 0.0, 0.0), b: pt(2.0, 0.0, 0.0), radius: 0.7 };
        assert!((distance(&s1, &s2).distance - 0.8).abs() < 1e-9);
    }

    #[test]
    fn separated_boxes_report_the_gap() {
        let a = box_hull(pt(0.0, 0.0, 0.0), 0.5, 0.0);
        let b = box_hull(pt(2.0, 0.0, 0.0), 0.5, 0.0);
        // Faces one apart at x = 0.5 and x = 1.5.
        assert!((distance(&a, &b).distance - 1.0).abs() < 1e-9);
        // Diagonal offset: nearest features are edges/corners.
        let c = box_hull(pt(2.0, 2.0, 0.0), 0.5, 0.0);
        let expected = ((2.0f64 - 1.0).powi(2) * 2.0).sqrt();
        assert!((distance(&a, &c).distance - expected).abs() < 1e-9);
    }

    #[test]
    fn rounded_boxes_subtract_radius() {
        let a = box_hull(pt(0.0, 0.0, 0.0), 0.5, 0.1);
        let b = box_hull(pt(2.0, 0.0, 0.0), 0.5, 0.15);
        assert!((distance(&a, &b).distance - (1.0 - 0.25)).abs() < 1e-9);
    }

    #[test]
    fn overlapping_cores_report_zero_or_penetration() {
        // Cores overlap (origin inside the Minkowski difference): core distance
        // zero, so rounded shapes report negative depth -ra-rb.
        let a = box_hull(pt(0.0, 0.0, 0.0), 0.5, 0.1);
        let b = box_hull(pt(0.3, 0.0, 0.0), 0.5, 0.2);
        let r = distance(&a, &b);
        assert!(r.distance <= 0.0, "overlap should be non-positive, got {}", r.distance);
    }

    #[test]
    fn agrees_with_capsule_under_isometry() {
        let c1 = Capsule { a: pt(0.2, -1.0, 0.4), b: pt(1.3, 0.7, -0.2), radius: 0.15 };
        let c2 = Capsule { a: pt(-0.5, 0.9, 1.1), b: pt(0.8, 1.4, 0.3), radius: 0.25 };
        let iso = Isometry3::from_parts(
            Translation3::new(0.3, -2.0, 1.7),
            UnitQuaternion::from_euler_angles(0.4, -0.9, 1.3),
        );
        let before = distance(&c1, &c2).distance;
        let after = distance(&c1.transformed(&iso), &c2.transformed(&iso)).distance;
        assert!((before - after).abs() < 1e-9);
    }

    #[test]
    fn is_symmetric() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(3);
        for _ in 0..500 {
            let a = box_hull(pt(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0), 0.0), 0.4, 0.05);
            let b = box_hull(pt(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0), 1.0), 0.4, 0.05);
            let ab = distance(&a, &b).distance;
            let ba = distance(&b, &a).distance;
            assert!((ab - ba).abs() < 1e-9, "asymmetric: {ab} vs {ba}");
        }
    }

    #[test]
    fn converges_quickly_on_simple_primitives() {
        // A handful of iterations, never the cap, for well-separated convexes.
        let a = box_hull(pt(0.0, 0.0, 0.0), 0.5, 0.0);
        let b = box_hull(pt(3.0, 1.0, 0.5), 0.5, 0.0);
        assert!(distance(&a, &b).iterations < 10);
    }
}
