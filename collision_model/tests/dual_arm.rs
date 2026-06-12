//! Integration tests: the OpenArm fixture model driven through two-arm
//! scenarios. Distances assert against the classified pair set, so these
//! also pin the checked-in fixture margins; regenerate with the documented
//! fit_capsules/classify_pairs invocations after changing geometry and
//! re-baseline deliberately if values move.

use collision_model::config::CollisionConfig;
use collision_model::DualArmCollisionModel;
use srs_model::JointVec;

const FIXTURES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures");

/// The classifier's reference-pose headroom: margined pairs read exactly
/// this at home/ready.
const FLOOR: f64 = 0.04;

/// In-limit home: the elbow's one-sided lower limit is 0.05, and the
/// classifier clamps its reference poses into limits the same way.
const HOME: JointVec = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];
const READY: JointVec = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0];

fn model() -> DualArmCollisionModel {
    let config = CollisionConfig::from_file(&format!("{FIXTURES}/openarm_v10_capsules.json"))
        .expect("fixture config")
        .parse()
        .expect("valid config");
    DualArmCollisionModel::from_urdf_file(
        &format!("{FIXTURES}/openarm_v10.urdf"),
        "openarm_left_link0",
        "openarm_right_link0",
        &config,
    )
    .expect("fixture model")
}

/// Both arms elbow-bent, j3 wrapping the wrists toward the centerline.
fn wrists_inward(t: f64) -> (JointVec, JointVec) {
    let mut ql: JointVec = [0.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0];
    let mut qr = ql;
    ql[2] = t;
    qr[2] = -t;
    (ql, qr)
}

#[test]
fn home_and_ready_rest_at_the_margin_floor() {
    let mut m = model();
    for q in [HOME, READY] {
        let p = m.min_distance(&q, &q).expect("clear pose");
        assert!(
            (p.distance - FLOOR).abs() < 1e-3,
            "rest pose should sit at the classified headroom floor, got {:+.4} ({} vs {})",
            p.distance,
            p.link_a,
            p.link_b,
        );
    }
}

#[test]
fn wrists_converging_monotonically_reach_collision() {
    let mut m = model();
    let mut prev = f64::INFINITY;
    let mut last = (String::new(), String::new(), 0.0);
    for i in 0..=12 {
        let t = i as f64 * 0.1;
        let (ql, qr) = wrists_inward(t);
        let p = m.min_distance(&ql, &qr).expect("query");
        assert!(
            p.distance <= prev + 1e-3,
            "approach not monotone at t={t}: {:+.4} after {prev:+.4}",
            p.distance,
        );
        prev = p.distance;
        last = (p.link_a.to_string(), p.link_b.to_string(), p.distance);
    }
    let (a, b, d) = last;
    assert!(d < -0.06, "fully wrapped wrists should interpenetrate, got {d:+.4}");
    assert!(
        a.contains("link7") && b.contains("link7"),
        "deepest pair should be the two wrists, got {a} vs {b}",
    );
}

#[test]
fn elbows_collide_when_folded_inward() {
    let mut m = model();
    // Mirrored j2 folds both arms toward the centerline.
    let ql: JointVec = [0.0, 0.6, 0.0, 0.4, 0.0, 0.0, 0.0];
    let qr: JointVec = [0.0, -0.6, 0.0, 0.4, 0.0, 0.0, 0.0];
    let p = m.min_distance(&ql, &qr).expect("query");
    assert!(p.distance < -0.08, "folded elbows should interpenetrate deeply, got {:+.4}", p.distance);
    for link in [p.link_a, p.link_b] {
        assert!(
            link.contains("link3") || link.contains("link4"),
            "witness should be an upper-arm/elbow link, got {link}",
        );
    }
}

#[test]
fn separating_sweep_never_alarms() {
    let mut m = model();
    for i in 0..=12 {
        let t = i as f64 * 0.1;
        let ql: JointVec = [0.0, -t, 0.0, 0.4, 0.0, 0.0, 0.0];
        let qr: JointVec = [0.0, t, 0.0, 0.4, 0.0, 0.0, 0.0];
        let p = m.min_distance(&ql, &qr).expect("query");
        assert!(p.distance > FLOOR - 0.005, "outward sweep dipped to {:+.4} at t={t}", p.distance);
    }
}

#[test]
fn witnesses_are_finite_world_points_consistent_with_distance() {
    let mut m = model();
    let (ql, qr) = wrists_inward(0.9);
    let p = m.min_distance(&ql, &qr).expect("query");
    for w in [p.on_a, p.on_b] {
        assert!(w.coords.iter().all(|c| c.is_finite() && c.abs() < 2.0), "witness {w:?} not plausible");
    }
    // The witness gap equals the raw (margin-free) distance for the winning
    // pair; the wrist pair is unmargined, so it matches `distance` exactly.
    let gap = (p.on_a - p.on_b).norm();
    assert!(
        (gap - p.distance.abs()).abs() < 1e-9,
        "witness gap {gap:.4} vs reported distance {:+.4}",
        p.distance,
    );
}

#[test]
fn in_collision_threshold_semantics() {
    let mut m = model();
    assert!(m.in_collision(&HOME, &HOME, FLOOR + 0.005).expect("query"));
    assert!(!m.in_collision(&HOME, &HOME, FLOOR - 0.005).expect("query"));
    let (ql, qr) = wrists_inward(1.2);
    assert!(m.in_collision(&ql, &qr, 0.0).expect("query"));
}

#[test]
fn non_finite_configurations_are_rejected() {
    let mut m = model();
    let mut bad = HOME;
    bad[3] = f64::NAN;
    assert!(m.min_distance(&bad, &HOME).is_err());
    assert!(m.min_distance(&HOME, &bad).is_err());
    bad[3] = f64::INFINITY;
    assert!(m.in_collision(&HOME, &bad, 0.0).is_err());
}

/// Wall-clock budget: a full dual-arm query must stay far inside a control
/// tick. Debug builds are several times slower and not what runs on the
/// robot, so the budget is asserted in release only.
#[test]
fn query_stays_inside_the_control_tick() {
    let mut m = model();
    let configs: Vec<(JointVec, JointVec)> = (0..200).map(|i| wrists_inward(i as f64 * 0.005)).collect();

    let start = std::time::Instant::now();
    let mut acc = 0.0;
    for (ql, qr) in &configs {
        acc += m.min_distance(ql, qr).expect("query").distance;
    }
    let per_query = start.elapsed().as_secs_f64() / configs.len() as f64;
    assert!(acc.is_finite());
    println!("per-query: {:.1} us", per_query * 1e6);
    if !cfg!(debug_assertions) {
        assert!(per_query < 1e-3, "query took {:.1} us, budget is 1 ms", per_query * 1e6);
    }
}

#[test]
fn governor_halts_an_approach_before_contact() {
    let mut m = model();
    let band = collision_model::GovernorBand::new(0.01, 0.03).expect("band");
    let mut halted_at = None;
    let mut d_now = m.min_distance(&wrists_inward(0.0).0, &wrists_inward(0.0).1).expect("d0").distance;
    for i in 1..=120 {
        let t = i as f64 * 0.01;
        let (ql, qr) = wrists_inward(t);
        let d_next = m.min_distance(&ql, &qr).expect("query").distance;
        if band.scale(d_now, d_next) == 0.0 {
            halted_at = Some((t, d_now));
            break;
        }
        d_now = d_next;
    }
    let (t, d) = halted_at.expect("the approach must trip the governor");
    assert!(d > 0.0, "governor halted at t={t} with clearance {d:+.4}, after contact");
}
