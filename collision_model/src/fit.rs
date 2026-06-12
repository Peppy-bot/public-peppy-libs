//! Capsule fitting: the smallest-reasonable capsule containing a vertex
//! cloud, used offline to turn each URDF collision mesh into its runtime
//! proxy.
//!
//! Containment is exact by construction: the radius is the maximum distance
//! of any vertex to the chosen axis segment, so every vertex (and therefore
//! the convex hull of the mesh) lies inside the capsule. The fit is not the
//! globally minimal capsule; PCA picks the axis and one shrink pass trades
//! cap length against radius, which is tight enough for link-shaped meshes.

use srs_model::nalgebra::{Matrix3, Point3, Vector3};

use crate::geometry::{Capsule, point_segment_distance};

/// Fit a capsule containing every point of `points`.
pub fn fit_capsule(points: &[Point3<f64>]) -> Result<Capsule, String> {
    if points.is_empty() {
        return Err("cannot fit a capsule to zero points".into());
    }
    if points.iter().any(|p| !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())) {
        return Err("cannot fit a capsule to non-finite points".into());
    }

    let axis = principal_axis(points);
    let centroid = centroid(points);

    // Extent along the axis and the largest perpendicular distance.
    let mut t_min = f64::INFINITY;
    let mut t_max = f64::NEG_INFINITY;
    let mut r_perp: f64 = 0.0;
    for p in points {
        let d = p.coords - centroid;
        let t = d.dot(&axis);
        t_min = t_min.min(t);
        t_max = t_max.max(t);
        r_perp = r_perp.max((d - axis * t).norm());
    }

    // How far to pull the endpoints inward so the spherical caps cover the
    // ends is shape-dependent: zero is best for flat-ended cylinders (any
    // shrink pays a corner penalty on the rim), the full perpendicular radius
    // for rounded ends. Try a few candidates and keep the smallest radius;
    // containment is exact for each because the radius is recomputed as the
    // worst vertex's distance to the candidate segment.
    let mid = (t_min + t_max) / 2.0;
    let max_shrink = r_perp.min((t_max - t_min) / 2.0);
    let candidates: Vec<Capsule> = [0.0, 0.25, 0.5, 0.75, 1.0]
        .iter()
        .map(|frac| {
            let half = (t_max - t_min) / 2.0 - max_shrink * frac;
            let a = Point3::from(centroid + axis * (mid - half));
            let b = Point3::from(centroid + axis * (mid + half));
            let radius = points.iter().map(|p| point_segment_distance(p, &a, &b)).fold(0.0, f64::max);
            Capsule { a, b, radius }
        })
        .collect();

    // Smallest radius wins; among near-ties (e.g. blob clouds, where every
    // shrink gives the same radius up to sampling noise) prefer the shortest
    // segment, which adds the least phantom volume. The tie window is
    // relative: a radius within 0.1% is not worth a longer segment.
    let r_min = candidates.iter().map(|c| c.radius).fold(f64::INFINITY, f64::min);
    Ok(candidates
        .into_iter()
        .filter(|c| c.radius <= r_min * 1.001 + 1e-9)
        .min_by(|x, y| (x.b - x.a).norm().total_cmp(&(y.b - y.a).norm()))
        .expect("the minimum-radius candidate always survives its own tie window"))
}

/// Fit up to `max_bands` capsules: partition the cloud into equal-width
/// slabs along its principal axis and fit each slab independently (each with
/// its own axis), trying every band count from 1 to `max_bands` and keeping
/// the one with the least total capsule volume.
///
/// Slab assignment is per point, and a capsule union is not convex, so a
/// face spanning two bands could escape between them; when the input is a
/// triangle soup (length a multiple of 3, as every STL path here produces),
/// the repair pass grows leaking bands until sampled faces are contained
/// too. A plain point cloud gets point containment only. Volume is the
/// phantom space a proxy adds (the cause of false proximity), and it
/// self-regularizes: every extra band brings its own end caps, so a uniform
/// shape stays one capsule while tapered or compound shapes (a torso: wide
/// base, slim column; an elbow link) band where it genuinely helps. More
/// capsules also cost pairwise checks, so a higher count must earn a
/// material (5%) volume reduction.
pub fn fit_capsules_adaptive(points: &[Point3<f64>], max_bands: usize) -> Result<Vec<Capsule>, String> {
    let single = fit_capsule(points)?;
    let mut best = vec![single];
    let mut best_volume = volume(&best);

    let axis = principal_axis(points);
    let ts: Vec<f64> = points.iter().map(|p| p.coords.dot(&axis)).collect();
    let t_min = ts.iter().cloned().fold(f64::INFINITY, f64::min);
    let t_max = ts.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if t_max - t_min <= 0.0 {
        return Ok(best);
    }

    for bands in 2..=max_bands.max(1) {
        let width = (t_max - t_min) / bands as f64;
        let slab_of = |t: f64| (((t - t_min) / width) as usize).min(bands - 1);
        let mut slabs: Vec<Vec<Point3<f64>>> = vec![Vec::new(); bands];
        for (p, t) in points.iter().zip(&ts) {
            slabs[slab_of(*t)].push(*p);
        }
        let mut fitted = slabs
            .iter()
            .filter(|s| !s.is_empty())
            .map(|s| fit_capsule(s))
            .collect::<Result<Vec<_>, _>>()?;
        repair_face_coverage(points, &mut fitted)?;
        // Volume is scored after repair so candidates compete on what they
        // actually cost once sound.
        let v = volume(&fitted);
        if v < best_volume * 0.95 {
            best_volume = v;
            best = fitted;
        }
    }
    Ok(best)
}

/// Barycentric sample weights covering a triangle's interior and edges.
const FACE_SAMPLES: [[f64; 3]; 12] = [
    [0.5, 0.5, 0.0], [0.5, 0.0, 0.5], [0.0, 0.5, 0.5],
    [0.75, 0.25, 0.0], [0.25, 0.75, 0.0], [0.75, 0.0, 0.25],
    [0.25, 0.0, 0.75], [0.0, 0.75, 0.25], [0.0, 0.25, 0.75],
    [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0], [0.5, 0.25, 0.25], [0.25, 0.5, 0.25],
];

/// Make a capsule union cover the FACES of a triangle soup, not only its
/// vertices: a union is not convex, so a face spanning two bands can escape
/// between them. Sample each face and grow the nearest band's radius by the
/// worst escape, repeating until nothing escapes. Vertices are exact by
/// construction, faces by sampled bound; the final containment tests sample
/// independently. No-op for non-triangle clouds and single capsules.
fn repair_face_coverage(points: &[Point3<f64>], capsules: &mut [Capsule]) -> Result<(), String> {
    if !points.len().is_multiple_of(3) || capsules.len() < 2 {
        return Ok(());
    }
    for _ in 0..8 {
        let mut worst_escape = vec![0.0_f64; capsules.len()];
        for tri in points.chunks_exact(3) {
            for w in &FACE_SAMPLES {
                let p = Point3::from(tri[0].coords * w[0] + tri[1].coords * w[1] + tri[2].coords * w[2]);
                let (mut nearest, mut escape) = (0usize, f64::INFINITY);
                for (i, c) in capsules.iter().enumerate() {
                    let d = point_segment_distance(&p, &c.a, &c.b) - c.radius;
                    if d < escape {
                        escape = d;
                        nearest = i;
                    }
                }
                if escape > 0.0 {
                    worst_escape[nearest] = worst_escape[nearest].max(escape);
                }
            }
        }
        if worst_escape.iter().all(|&e| e <= 0.0) {
            return Ok(());
        }
        for (c, e) in capsules.iter_mut().zip(&worst_escape) {
            c.radius += e + 1e-9;
        }
    }
    Err("face coverage repair did not converge".into())
}

/// Total volume of a capsule set (overlaps double-counted: a proxy for the
/// union, exact enough to rank band candidates).
fn volume(capsules: &[Capsule]) -> f64 {
    capsules
        .iter()
        .map(|c| {
            let len = (c.b - c.a).norm();
            std::f64::consts::PI * c.radius * c.radius * (len + 4.0 / 3.0 * c.radius)
        })
        .sum()
}

/// Dominant eigenvector of the point covariance: the direction of largest
/// spread. The Z fallback guards a numerically degenerate eigenvector, not a
/// reachable input class (a single point yields unit eigenvectors already).
fn principal_axis(points: &[Point3<f64>]) -> Vector3<f64> {
    let centroid = centroid(points);
    let cov = points.iter().fold(Matrix3::zeros(), |acc, p| {
        let d = p.coords - centroid;
        acc + d * d.transpose()
    }) / points.len() as f64;

    let eigen = cov.symmetric_eigen();
    let axis = eigen.eigenvectors.column(eigen.eigenvalues.imax()).into_owned();
    if axis.norm_squared() < 1e-12 { Vector3::z() } else { axis.normalize() }
}

fn centroid(points: &[Point3<f64>]) -> Vector3<f64> {
    points.iter().fold(Vector3::zeros(), |acc, p| acc + p.coords) / points.len() as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contained(points: &[Point3<f64>], c: &Capsule) -> bool {
        // Tolerate float round-off at the surface.
        points.iter().all(|p| point_segment_distance(p, &c.a, &c.b) <= c.radius + 1e-9)
    }

    /// Points on a cylinder of radius `r` around the segment `a..b`.
    fn cylinder_cloud(a: Point3<f64>, b: Point3<f64>, r: f64) -> Vec<Point3<f64>> {
        let axis = (b - a).normalize();
        let u = axis.cross(&Vector3::new(0.3, 0.7, -0.2)).normalize();
        let v = axis.cross(&u);
        let mut pts = Vec::new();
        for i in 0..40 {
            let t = i as f64 / 39.0;
            let center = a + (b - a) * t;
            for k in 0..12 {
                let ang = k as f64 * std::f64::consts::TAU / 12.0;
                pts.push(center + (u * ang.cos() + v * ang.sin()) * r);
            }
        }
        pts
    }

    #[test]
    fn fits_a_cylinder_tightly() {
        let (a, b) = (Point3::new(0.1, -0.2, 0.3), Point3::new(1.4, 0.5, -0.1));
        let cloud = cylinder_cloud(a, b, 0.05);
        let c = fit_capsule(&cloud).expect("fit");
        assert!(contained(&cloud, &c));
        // Tight: radius within 20% of the true cylinder radius.
        assert!(c.radius < 0.06, "radius {} too loose", c.radius);
        // Axis aligned with the cylinder axis.
        let fitted_axis = (c.b - c.a).normalize();
        let true_axis = (b - a).normalize();
        assert!(fitted_axis.dot(&true_axis).abs() > 0.999);
    }

    #[test]
    fn collapses_to_a_sphere_for_blob_clouds() {
        // Points on a sphere: the segment should collapse to ~the center.
        let mut pts = Vec::new();
        for i in 0..100 {
            let phi = i as f64 * 0.618 * std::f64::consts::TAU;
            let z = -1.0 + 2.0 * (i as f64 + 0.5) / 100.0;
            let r = (1.0f64 - z * z).sqrt();
            pts.push(Point3::new(r * phi.cos(), r * phi.sin(), z));
        }
        let c = fit_capsule(&pts).expect("fit");
        assert!(contained(&pts, &c));
        assert!((c.b - c.a).norm() < 0.2, "segment {} should be near-degenerate", (c.b - c.a).norm());
        assert!(c.radius < 1.1);
    }

    #[test]
    fn handles_collinear_and_single_points() {
        let line: Vec<_> = (0..10).map(|i| Point3::new(i as f64 * 0.1, 0.0, 0.0)).collect();
        let c = fit_capsule(&line).expect("fit line");
        assert!(contained(&line, &c));

        let single = [Point3::new(0.3, 0.4, 0.5)];
        let c = fit_capsule(&single).expect("fit point");
        assert!(contained(&single, &c));

        assert!(fit_capsule(&[]).is_err());
        assert!(fit_capsule(&[Point3::new(f64::NAN, 0.0, 0.0)]).is_err());
    }

    #[test]
    fn banded_union_contains_faces_spanning_band_boundaries() {
        // Long thin triangles crossing the band boundary of a dumbbell: with
        // per-point slabs their midspans can escape the union; per-triangle
        // slabs must keep every sampled face point inside.
        let mut pts = Vec::new();
        for k in 0..24 {
            let ang = k as f64 * std::f64::consts::TAU / 24.0;
            let (c, s) = (ang.cos(), ang.sin());
            // Wide-base ring vertex paired with two slim far-column vertices.
            pts.push(Point3::new(0.5 * c, 0.5 * s, 0.0));
            pts.push(Point3::new(0.05 * c, 0.05 * s, 2.9));
            pts.push(Point3::new(0.05 * s, 0.05 * c, 3.0));
        }
        let banded = fit_capsules_adaptive(&pts, 5).expect("banded");
        for tri in pts.chunks_exact(3) {
            for (w0, w1, w2) in [(0.5, 0.5, 0.0), (0.5, 0.0, 0.5), (0.0, 0.5, 0.5), (1. / 3., 1. / 3., 1. / 3.)] {
                let p = Point3::from(tri[0].coords * w0 + tri[1].coords * w1 + tri[2].coords * w2);
                let inside =
                    banded.iter().any(|c| point_segment_distance(&p, &c.a, &c.b) <= c.radius + 1e-9);
                assert!(inside, "face point {p:?} escapes the banded union");
            }
        }
    }

    #[test]
    fn adaptive_fit_bands_a_dumbbell_and_contains_it() {
        // A slim column with a wide base: one capsule needs the base radius
        // everywhere, bands give the column its own slim capsule.
        let mut pts = Vec::new();
        for i in 0..200 {
            let z = i as f64 / 199.0;
            let r = if z < 0.2 { 0.5 } else { 0.05 };
            for k in 0..8 {
                let ang = k as f64 * std::f64::consts::TAU / 8.0;
                pts.push(Point3::new(r * ang.cos(), r * ang.sin(), z * 3.0));
            }
        }
        let banded = fit_capsules_adaptive(&pts, 5).expect("banded");
        assert!(banded.len() >= 2, "dumbbell should band, got {} capsule(s)", banded.len());
        for p in &pts {
            let inside = banded.iter().any(|c| point_segment_distance(p, &c.a, &c.b) <= c.radius + 1e-9);
            assert!(inside, "point {p:?} escapes the banded union");
        }
        let slimmest = banded.iter().map(|c| c.radius).fold(f64::INFINITY, f64::min);
        assert!(slimmest < 0.2, "no slim column band: {slimmest}");
    }

    #[test]
    fn adaptive_fit_keeps_one_capsule_when_banding_does_not_help() {
        // A plain cylinder: every band count gives the same radius, so the
        // adaptive choice must keep the single capsule.
        let cloud = cylinder_cloud(Point3::new(0., 0., 0.), Point3::new(2., 0., 0.), 0.1);
        let fitted = fit_capsules_adaptive(&cloud, 4).expect("fit");
        assert_eq!(fitted.len(), 1, "cylinder must not band");
        let single = fit_capsule(&cloud).expect("single");
        assert_eq!(fitted[0], single);
    }

    #[test]
    fn adaptive_fit_with_max_one_matches_single() {
        let pts: Vec<_> = (0..50).map(|i| Point3::new(i as f64 * 0.01, 0.0, 0.0)).collect();
        let single = fit_capsule(&pts).expect("single");
        let banded = fit_capsules_adaptive(&pts, 1).expect("banded");
        assert_eq!(banded, vec![single]);
    }

    #[test]
    fn contains_random_clouds() {
        use rand::{Rng, SeedableRng};
        let mut rng = rand::rngs::StdRng::seed_from_u64(7);
        for _ in 0..20 {
            let pts: Vec<_> = (0..200)
                .map(|_| {
                    Point3::new(
                        rng.gen_range(-1.0..1.0),
                        rng.gen_range(-0.2..0.2) + 2.0 * rng.gen_range(-1.0..1.0f64).powi(3),
                        rng.gen_range(-0.5..0.5),
                    )
                })
                .collect();
            let c = fit_capsule(&pts).expect("fit");
            assert!(contained(&pts, &c));
        }
    }
}
