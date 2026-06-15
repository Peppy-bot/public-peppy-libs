//! Capsule geometry: the collision primitive and its closed-form distance.
//!
//! A capsule (sphere-swept segment) conservatively over-approximates a link
//! mesh while keeping the pairwise distance closed-form and light, fast
//! enough to evaluate every control tick. Distances can be negative, meaning
//! the surfaces interpenetrate by that depth.

use srs_model::nalgebra::{Isometry3, Point3};

/// Below this squared segment direction norm a segment degenerates to a point
/// (its capsule to a sphere) and direction-dependent math switches to the
/// point form. The threshold is a 1e-12 m segment: collapsing it to a point
/// perturbs any distance by at most that length, six orders of magnitude
/// under any millimeter-scale threshold, while staying three orders above
/// f64 coordinate noise at meter scale (~1e-15), so no real segment is ever
/// misclassified. Tested at both sides of the boundary.
const DEGENERATE_EPS2: f64 = 1e-24;

/// Relative threshold on the segment-pair denominator `a*e - b*b`, whose
/// scale is `a*e` and whose value is `a*e*sin^2(theta)` for angle `theta`
/// between the segments: below it (`sin(theta) <= 1e-6`) the pair is
/// treated as parallel. Near-parallel distance is flat in the free
/// parameter, so taking the parallel branch perturbs the result by at most
/// `length * sin(theta)`, sub-micrometer for meter-scale links; taking the
/// general branch below the threshold would divide rounding noise by a
/// vanishing denominator instead. Tested at both sides of the boundary.
const PARALLEL_EPS: f64 = 1e-12;

/// A capsule: the set of points within `radius` of the segment `a..b`.
/// Frame-agnostic; the segment endpoints are in whatever frame the caller
/// works in (link-local at fit time, world after placement).
#[derive(Debug, Clone, PartialEq)]
pub struct Capsule {
    pub a: Point3<f64>,
    pub b: Point3<f64>,
    pub radius: f64,
}

/// Result of a capsule-capsule query: the signed surface distance and the
/// closest points. `distance <= 0.0` means the capsules touch or overlap.
#[derive(Debug, Clone)]
pub struct CapsuleDistance {
    pub distance: f64,
    /// Closest point on the first capsule's surface (on its axis when the
    /// axes themselves intersect, where no outward direction exists).
    pub on_a: Point3<f64>,
    /// Closest point on the second capsule's surface (same caveat).
    pub on_b: Point3<f64>,
}

impl Capsule {
    /// The capsule mapped through `pose` (radius is isometry-invariant).
    pub fn transformed(&self, pose: &Isometry3<f64>) -> Capsule {
        Capsule { a: pose * self.a, b: pose * self.b, radius: self.radius }
    }

    /// Signed surface distance to `other`, with witness points.
    pub fn distance_to(&self, other: &Capsule) -> CapsuleDistance {
        let (axis_gap, p, q) = segment_segment_closest(self.a, self.b, other.a, other.b);
        let distance = axis_gap - self.radius - other.radius;
        let dir = q - p;
        if dir.norm_squared() <= DEGENERATE_EPS2 {
            // Axes intersect: no outward direction, report the axis points.
            return CapsuleDistance { distance, on_a: p, on_b: q };
        }
        let dir = dir.normalize();
        CapsuleDistance { distance, on_a: p + dir * self.radius, on_b: q - dir * other.radius }
    }

    /// True if `point` lies inside the capsule (within `radius` of the axis).
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        point_segment_distance(point, &self.a, &self.b) <= self.radius
    }
}

/// Distance from `point` to the segment `a..b`.
pub fn point_segment_distance(point: &Point3<f64>, a: &Point3<f64>, b: &Point3<f64>) -> f64 {
    let ab = b - a;
    let len2 = ab.norm_squared();
    if len2 <= DEGENERATE_EPS2 {
        return (point - a).norm();
    }
    let t = ((point - a).dot(&ab) / len2).clamp(0.0, 1.0);
    (point - (a + ab * t)).norm()
}

/// Closest points between segments `p1..q1` and `p2..q2`, returning
/// `(distance, on_first, on_second)`. Closed form (Ericson, "Real-Time
/// Collision Detection", 5.1.9), covering parallel and degenerate
/// (zero-length) segments.
pub fn segment_segment_closest(
    p1: Point3<f64>,
    q1: Point3<f64>,
    p2: Point3<f64>,
    q2: Point3<f64>,
) -> (f64, Point3<f64>, Point3<f64>) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.norm_squared();
    let e = d2.norm_squared();
    let f = d2.dot(&r);

    let (s, t) = if a <= DEGENERATE_EPS2 && e <= DEGENERATE_EPS2 {
        (0.0, 0.0)
    } else if a <= DEGENERATE_EPS2 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(&r);
        if e <= DEGENERATE_EPS2 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(&d2);
            let denom = a * e - b * b;
            // Parallel segments have denom == 0. The threshold is relative to
            // a*e (denom's own scale, length^4) so it is dimensionless, unlike
            // the absolute squared-length epsilon above. Any s on the overlap
            // works for parallels; start at 0 and let the t/s clamps find it.
            let s = if denom > PARALLEL_EPS * a * e { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
            let t = (b * s + f) / e;
            // Re-clamp s against the clamped t (the standard second pass).
            if t < 0.0 {
                ((-c / a).clamp(0.0, 1.0), 0.0)
            } else if t > 1.0 {
                (((b - c) / a).clamp(0.0, 1.0), 1.0)
            } else {
                (s, t)
            }
        }
    };

    let c1 = p1 + d1 * s;
    let c2 = p2 + d2 * t;
    ((c1 - c2).norm(), c1, c2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use srs_model::nalgebra::{Translation3, UnitQuaternion};

    fn pt(x: f64, y: f64, z: f64) -> Point3<f64> {
        Point3::new(x, y, z)
    }

    fn seg_dist(p1: Point3<f64>, q1: Point3<f64>, p2: Point3<f64>, q2: Point3<f64>) -> f64 {
        segment_segment_closest(p1, q1, p2, q2).0
    }

    #[test]
    fn parallel_segments_at_unit_offset() {
        let d = seg_dist(pt(0., 0., 0.), pt(1., 0., 0.), pt(0., 1., 0.), pt(1., 1., 0.));
        assert!((d - 1.0).abs() < 1e-12);
    }

    #[test]
    fn parallel_segments_offset_along_axis() {
        // Collinear with a 2-unit gap between the nearest endpoints.
        let d = seg_dist(pt(0., 0., 0.), pt(1., 0., 0.), pt(3., 0., 0.), pt(4., 0., 0.));
        assert!((d - 2.0).abs() < 1e-12);
    }

    #[test]
    fn skew_perpendicular_segments() {
        // Crossing X and Y axes separated by 1 in Z: line distance is 1.
        let d = seg_dist(pt(-1., 0., 0.), pt(1., 0., 0.), pt(0., -1., 1.), pt(0., 1., 1.));
        assert!((d - 1.0).abs() < 1e-12);
    }

    #[test]
    fn intersecting_segments_have_zero_distance() {
        let d = seg_dist(pt(-1., 0., 0.), pt(1., 0., 0.), pt(0., -1., 0.), pt(0., 1., 0.));
        assert!(d.abs() < 1e-12);
    }

    #[test]
    fn endpoint_to_endpoint_region() {
        // Closest approach is between two endpoints, not interior points.
        let d = seg_dist(pt(0., 0., 0.), pt(1., 0., 0.), pt(2., 1., 0.), pt(3., 2., 0.));
        let expected = ((2.0f64 - 1.0).powi(2) + 1.0).sqrt();
        assert!((d - expected).abs() < 1e-12);
    }

    #[test]
    fn zero_length_segments_are_points() {
        let d = seg_dist(pt(0., 0., 0.), pt(0., 0., 0.), pt(3., 4., 0.), pt(3., 4., 0.));
        assert!((d - 5.0).abs() < 1e-12);
        let d = seg_dist(pt(0., 0., 0.), pt(0., 0., 0.), pt(-1., 3., 0.), pt(1., 3., 0.));
        assert!((d - 3.0).abs() < 1e-12);
    }

    #[test]
    fn degenerate_second_segment_clamps_onto_first() {
        // First segment real, second a point: exercises the e ~ 0 branch.
        // Point beside the interior: perpendicular distance.
        let d = seg_dist(pt(0., 0., 0.), pt(2., 0., 0.), pt(1., 0.5, 0.), pt(1., 0.5, 0.));
        assert!((d - 0.5).abs() < 1e-12);
        // Point past the end: clamps to the endpoint, not the infinite line.
        let d = seg_dist(pt(0., 0., 0.), pt(2., 0., 0.), pt(3., 1., 0.), pt(3., 1., 0.));
        assert!((d - 2.0f64.sqrt()).abs() < 1e-12);
        // Symmetric: degenerate FIRST segment takes the a ~ 0 branch.
        let d = seg_dist(pt(3., 1., 0.), pt(3., 1., 0.), pt(0., 0., 0.), pt(2., 0., 0.));
        assert!((d - 2.0f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn segment_distance_is_symmetric() {
        let (a, b) = (pt(0.2, -1.0, 0.4), pt(1.3, 0.7, -0.2));
        let (c, d) = (pt(-0.5, 0.9, 1.1), pt(0.8, 1.4, 0.3));
        let d1 = seg_dist(a, b, c, d);
        let d2 = seg_dist(c, d, a, b);
        assert!((d1 - d2).abs() < 1e-12);
    }

    #[test]
    fn segment_distance_is_isometry_invariant() {
        let (a, b) = (pt(0.2, -1.0, 0.4), pt(1.3, 0.7, -0.2));
        let (c, d) = (pt(-0.5, 0.9, 1.1), pt(0.8, 1.4, 0.3));
        let before = seg_dist(a, b, c, d);
        let iso = Isometry3::from_parts(
            Translation3::new(0.3, -2.0, 1.7),
            UnitQuaternion::from_euler_angles(0.4, -0.9, 1.3),
        );
        let after = seg_dist(iso * a, iso * b, iso * c, iso * d);
        assert!((before - after).abs() < 1e-12);
    }

    #[test]
    fn near_parallel_segments_agree_across_the_branch_threshold() {
        // Two unit segments at perpendicular offset 0.5, tilted by theta.
        // Closest approach is at the near ends, distance 0.5, whichever
        // branch computes it; crossing the parallel threshold must not jump.
        let tilted = |theta: f64| {
            seg_dist(
                pt(0., 0., 0.),
                pt(1., 0., 0.),
                pt(0., 0.5, 0.),
                pt(theta.cos(), 0.5 + theta.sin(), 0.),
            )
        };
        // sin(theta) = 1e-5 takes the general branch, 1e-7 the parallel one.
        let above = tilted(1e-5);
        let below = tilted(1e-7);
        assert!((above - 0.5).abs() < 1e-9, "general branch: {above}");
        assert!((below - 0.5).abs() < 1e-9, "parallel branch: {below}");
        assert!((above - below).abs() < 1e-7, "branch discontinuity: {above} vs {below}");
        let exactly_parallel = tilted(0.0);
        assert!((exactly_parallel - 0.5).abs() < 1e-12);
    }

    #[test]
    fn near_degenerate_segments_agree_across_the_point_threshold() {
        // A short second segment beside a unit segment: just above the
        // degeneracy threshold it is treated as a segment, just below as a
        // point. Either way the distance is the perpendicular offset.
        let shorty = |len: f64| {
            seg_dist(pt(0., 0., 0.), pt(1., 0., 0.), pt(0.5, 0.3, 0.), pt(0.5 + len, 0.3, 0.))
        };
        let above = shorty(1e-10);
        let below = shorty(1e-13);
        assert!((above - 0.3).abs() < 1e-9, "segment branch: {above}");
        assert!((below - 0.3).abs() < 1e-9, "point branch: {below}");
        assert!((above - below).abs() < 1e-9, "threshold discontinuity");
    }

    #[test]
    fn capsule_distance_subtracts_radii() {
        let c1 = Capsule { a: pt(0., 0., 0.), b: pt(1., 0., 0.), radius: 0.2 };
        let c2 = Capsule { a: pt(0., 1., 0.), b: pt(1., 1., 0.), radius: 0.3 };
        let d = c1.distance_to(&c2);
        assert!((d.distance - 0.5).abs() < 1e-12);
        // Witnesses sit on the surfaces along the gap direction.
        assert!((d.on_a.y - 0.2).abs() < 1e-12);
        assert!((d.on_b.y - 0.7).abs() < 1e-12);
    }

    #[test]
    fn overlapping_capsules_report_penetration_depth() {
        let c1 = Capsule { a: pt(0., 0., 0.), b: pt(1., 0., 0.), radius: 0.4 };
        let c2 = Capsule { a: pt(0., 0.5, 0.), b: pt(1., 0.5, 0.), radius: 0.4 };
        let d = c1.distance_to(&c2);
        assert!((d.distance + 0.3).abs() < 1e-12, "expected -0.3, got {}", d.distance);
    }

    #[test]
    fn intersecting_axes_fall_back_to_axis_witnesses() {
        let c1 = Capsule { a: pt(-1., 0., 0.), b: pt(1., 0., 0.), radius: 0.1 };
        let c2 = Capsule { a: pt(0., -1., 0.), b: pt(0., 1., 0.), radius: 0.1 };
        let d = c1.distance_to(&c2);
        assert!((d.distance + 0.2).abs() < 1e-12);
        assert!((d.on_a - d.on_b).norm() < 1e-12);
    }

    #[test]
    fn capsule_distance_is_symmetric() {
        let c1 = Capsule { a: pt(0.2, -1.0, 0.4), b: pt(1.3, 0.7, -0.2), radius: 0.15 };
        let c2 = Capsule { a: pt(-0.5, 0.9, 1.1), b: pt(0.8, 1.4, 0.3), radius: 0.25 };
        let d12 = c1.distance_to(&c2).distance;
        let d21 = c2.distance_to(&c1).distance;
        assert!((d12 - d21).abs() < 1e-12);
    }

    #[test]
    fn transformed_preserves_radius_and_maps_endpoints() {
        let c = Capsule { a: pt(0.1, 0.2, 0.3), b: pt(1.0, -0.5, 0.7), radius: 0.12 };
        let iso = Isometry3::from_parts(
            Translation3::new(-0.4, 0.9, 2.0),
            UnitQuaternion::from_euler_angles(1.0, 0.2, -0.7),
        );
        let t = c.transformed(&iso);
        assert!((t.a - iso * c.a).norm() < 1e-15);
        assert!((t.b - iso * c.b).norm() < 1e-15);
        assert_eq!(t.radius, c.radius);
    }

    #[test]
    fn contains_accepts_axis_and_surface_points() {
        let c = Capsule { a: pt(0., 0., 0.), b: pt(1., 0., 0.), radius: 0.5 };
        assert!(c.contains(&pt(0.5, 0., 0.)));
        assert!(c.contains(&pt(0.5, 0.5, 0.)));
        assert!(c.contains(&pt(-0.4, 0., 0.))); // inside the end cap
        assert!(!c.contains(&pt(-0.6, 0., 0.)));
        assert!(!c.contains(&pt(0.5, 0.51, 0.)));
    }

    #[test]
    fn point_segment_handles_zero_length() {
        let d = point_segment_distance(&pt(3., 4., 0.), &pt(0., 0., 0.), &pt(0., 0., 0.));
        assert!((d - 5.0).abs() < 1e-12);
    }
}
