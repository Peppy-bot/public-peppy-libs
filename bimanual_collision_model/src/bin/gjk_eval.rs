//! Evaluation: convex-hull (decomposed) + GJK against the capsule baseline, on
//! the fixture robot. Reports setup cost, hot per-query cost, fit tightness
//! (false clearance a capsule eats versus the hulls), and how many pairs
//! intersect at each pose, so the GJK route can be judged on numbers.
//!
//! ```sh
//! cargo run --release --bin gjk_eval
//! ```
use std::collections::HashMap;

use bimanual_collision_model::gjk::{self, Hull, Support};
use bimanual_collision_model::nalgebra::{Isometry3, Point3, Vector3};
use bimanual_collision_model::{BimanualCollisionModel, Capsule, GovernorBand, MarginPolicy};

#[path = "shared/eval_common.rs"]
mod common;

/// A hull piece placed by an isometry, supporting GJK without transforming
/// vertices: the query direction is rotated into the hull's frame instead.
struct PosedHull<'a> {
    hull: &'a Hull,
    iso: Isometry3<f64>,
}

impl Support for PosedHull<'_> {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        self.iso * self.hull.core_support(&self.iso.inverse_transform_vector(dir))
    }
}

/// Closest signed distance between two bodies: the minimum over their pieces.
fn min_gjk(a: &[PosedHull], b: &[PosedHull]) -> f64 {
    a.iter().flat_map(|x| b.iter().map(move |y| gjk::distance(x, y).distance)).fold(f64::INFINITY, f64::min)
}

fn place<'a>(hulls: &'a HashMap<String, Vec<Hull>>, iso: &HashMap<String, Isometry3<f64>>) -> HashMap<String, Vec<PosedHull<'a>>> {
    hulls.iter().map(|(k, hs)| (k.clone(), hs.iter().map(|h| PosedHull { hull: h, iso: iso[k] }).collect())).collect()
}

fn main() {
    let policy = MarginPolicy { band: GovernorBand::new(0.01, 0.03).unwrap(), references: vec![[0.0; 7]] };
    let mut model =
        BimanualCollisionModel::from_urdf_file(common::URDF, common::MESHES, common::FIXED[1], common::FIXED[2], &policy).unwrap();
    let pairs: Vec<(String, String)> = model.checked_pairs().iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();

    let t = std::time::Instant::now();
    let convex = common::fit_hulls();
    let hulls: HashMap<String, Vec<Hull>> =
        convex.iter().map(|(k, pieces)| (k.clone(), pieces.iter().map(|(ch, r)| Hull::new(ch, *r).unwrap()).collect())).collect();
    let build_ms = t.elapsed().as_secs_f64() * 1e3;
    let hull_verts: usize = convex.values().flatten().map(|(h, _)| h.vertices.len()).sum();
    let max_radius = convex.values().flatten().map(|(_, r)| *r).fold(0.0, f64::max);
    let pieces: usize = convex.values().map(Vec::len).sum();
    let mut split: Vec<String> = convex.iter().filter(|(_, v)| v.len() > 1).map(|(k, v)| format!("{k} x{}", v.len())).collect();
    split.sort();

    // Cold query: the first full sweep, hill-climb hints unwarmed, as a setup cost.
    let iso0 = common::body_isometries(&[0.0; 7], &[0.0; 7]);
    let posed0 = place(&hulls, &iso0);
    let t = std::time::Instant::now();
    for (a, b) in &pairs {
        std::hint::black_box(min_gjk(&posed0[a], &posed0[b]));
    }
    let cold_us = t.elapsed().as_secs_f64() * 1e6;

    println!("setup: hull construction {build_ms:.0} ms + one cold query {cold_us:.0} us");
    println!("       {pieces} pieces over {} bodies, {hull_verts} hull verts, max inflation radius {:.1} mm", convex.len(), max_radius * 1e3);
    println!("       decomposed: {}\n", if split.is_empty() { "none".into() } else { split.join(", ") });

    let poses = [
        ("rest", [0.0; 7], [0.0; 7]),
        ("wrapped", [0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0], [0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0]),
        ("arms-out", [-0.45, -0.1, 0.0, 0.5, 0.0, -0.3, 0.0], [0.4, 0.1, 0.0, 0.7, 0.0, -0.2, 0.0]),
    ];

    for (label, ql, qr) in poses {
        let placed: HashMap<String, Vec<Capsule>> =
            model.world_capsules(&ql, &qr).unwrap().into_iter().map(|(n, c)| (n.to_string(), c)).collect();
        let iso = common::body_isometries(&ql, &qr);
        let posed = place(&hulls, &iso);

        let cap_dist = |a: &str, b: &str| -> f64 {
            placed[a].iter().flat_map(|ca| placed[b].iter().map(move |cb| ca.distance_to(cb).distance)).fold(f64::INFINITY, f64::min)
        };
        let gjk_dist = |a: &str, b: &str| min_gjk(&posed[a], &posed[b]);

        let (mut cap_min, mut hull_min) = (f64::INFINITY, f64::INFINITY);
        let (mut cap_pair, mut hull_pair) = (("", ""), ("", ""));
        let (mut worst_gain, mut worst_gain_pair) = (0.0, ("", ""));
        let (mut cap_overlaps, mut hull_overlaps) = (0, 0);
        for (a, b) in &pairs {
            let c = cap_dist(a, b);
            let h = gjk_dist(a, b);
            if c <= 0.0 {
                cap_overlaps += 1;
            }
            if h <= 0.0 {
                hull_overlaps += 1;
            }
            if c < cap_min {
                (cap_min, cap_pair) = (c, (a.as_str(), b.as_str()));
            }
            if h < hull_min {
                (hull_min, hull_pair) = (h, (a.as_str(), b.as_str()));
            }
            if h - c > worst_gain {
                (worst_gain, worst_gain_pair) = (h - c, (a.as_str(), b.as_str()));
            }
        }

        let reps = 50;
        let t = std::time::Instant::now();
        for _ in 0..reps {
            for (a, b) in &pairs {
                std::hint::black_box(cap_dist(a, b));
            }
        }
        let cap_us = t.elapsed().as_secs_f64() / reps as f64 * 1e6;
        let t = std::time::Instant::now();
        for _ in 0..reps {
            for (a, b) in &pairs {
                std::hint::black_box(gjk_dist(a, b));
            }
        }
        let gjk_us = t.elapsed().as_secs_f64() / reps as f64 * 1e6;

        println!("[{label}]  ({cap_overlaps} capsule / {hull_overlaps} hull pairs overlapping of {})", pairs.len());
        println!("  capsule  min {cap_min:+.4} m  ({} vs {})", cap_pair.0, cap_pair.1);
        println!("  hull+GJK min {hull_min:+.4} m  ({} vs {})", hull_pair.0, hull_pair.1);
        println!("  worst capsule false clearance vs hull: {worst_gain:.4} m on ({} vs {})", worst_gain_pair.0, worst_gain_pair.1);
        println!("  hot query: capsule {cap_us:.1} us, hull+GJK {gjk_us:.1} us ({:.0}x)", gjk_us / cap_us);
    }
}
