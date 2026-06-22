//! Integration tests: the OpenArm fixture model driven through two-arm
//! scenarios. Distances are signed hull distances (GJK, EPA on overlap), so a
//! change to the fixture geometry or the fit moves these numbers; the
//! assertions are kept qualitative (clear vs colliding, monotone, which links)
//! so they survive a re-fit.

use bimanual_collision_model::{BimanualCollisionModel, GovernorBand};
use srs_model::JointVec;

#[path = "fixtures/openarm.rs"]
mod openarm;

const FIXTURES: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures");

/// In-limit home: the elbow's one-sided lower limit is 0.05.
const HOME: JointVec = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0];

/// The band the governor-halt test gates with.
fn band() -> GovernorBand {
    GovernorBand::new(0.005, 0.02).expect("valid band")
}

/// The fixture model under the tight torso boxes we actually ship; the auto-fit
/// torso hull bulges across the chest and clips the grippers at rest, so the
/// integration scenarios run against the supplied proxy instead.
fn model() -> BimanualCollisionModel {
    BimanualCollisionModel::builder_from_file(
        &format!("{FIXTURES}/openarm_v10.urdf"),
        &format!("{FIXTURES}/meshes"),
        "openarm_left_link0",
        "openarm_right_link0",
    )
    .expect("read fixture urdf")
    .hulls(openarm::TORSO_BODY, openarm::torso())
    .build()
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
fn wrists_converging_monotonically_reach_collision() {
    let mut m = model();
    let mut prev = f64::INFINITY;
    let mut last = (String::new(), String::new(), 0.0);
    for i in 0..=12 {
        let (ql, qr) = wrists_inward(i as f64 * 0.1);
        let p = m.min_distance(&ql, &qr).expect("query");
        assert!(
            p.distance <= prev + 1e-3,
            "approach not monotone: {:+.4} after {prev:+.4}",
            p.distance
        );
        prev = p.distance;
        last = (p.link_a.to_string(), p.link_b.to_string(), p.distance);
    }
    let (a, b, d) = last;
    assert!(
        d < 0.0,
        "fully wrapped wrists should interpenetrate, got {d:+.4}"
    );
    assert!(
        a.contains("link7") || b.contains("link7"),
        "deepest pair should involve a wrist, got {a} vs {b}"
    );
}

#[test]
fn folding_the_arms_inward_drives_a_collision() {
    let mut m = model();
    // Mirrored j2 folds both arms toward the centerline, into the torso.
    let ql: JointVec = [0.0, 0.6, 0.0, 0.4, 0.0, 0.0, 0.0];
    let qr: JointVec = [0.0, -0.6, 0.0, 0.4, 0.0, 0.0, 0.0];
    let p = m.min_distance(&ql, &qr).expect("query");
    assert!(
        p.distance < 0.0,
        "folded arms should interpenetrate, got {:+.4}",
        p.distance
    );
    let touches_arm = [p.link_a, p.link_b]
        .iter()
        .any(|l| l.contains("link3") || l.contains("link4") || l.contains("body"));
    assert!(
        touches_arm,
        "expected an upper-arm or torso witness, got {} vs {}",
        p.link_a, p.link_b
    );
}

#[test]
fn rest_pose_clears_d_safe() {
    // The auto-fit torso bulges a phantom slab that reads a false near-contact
    // against the grippers at rest; the tight shipped boxes leave the true
    // clearance of several centimetres, well clear of the band.
    let mut m = model();
    let p = m.min_distance(&HOME, &HOME).expect("query");
    assert!(
        p.distance > band().d_safe(),
        "rest min {:+.4} should clear d_safe",
        p.distance
    );
}

#[test]
fn separating_sweep_never_alarms() {
    let mut m = model();
    for i in 0..=12 {
        let t = i as f64 * 0.1;
        let p = m
            .min_distance(
                &[0.0, -t, 0.0, 0.4, 0.0, 0.0, 0.0],
                &[0.0, t, 0.0, 0.4, 0.0, 0.0, 0.0],
            )
            .expect("query");
        assert!(
            p.distance > band().d_safe(),
            "outward sweep dipped to {:+.4} at t={t}",
            p.distance
        );
    }
}

#[test]
fn witnesses_are_finite_and_span_the_gap_when_clear() {
    let mut m = model();
    // A clearly separated pose: the witnesses lie on the surfaces and their gap
    // equals the (positive) reported distance.
    let (ql, qr) = wrists_inward(0.3);
    let p = m.min_distance(&ql, &qr).expect("query");
    assert!(
        p.distance > 0.0,
        "pose should be clear, got {:+.4}",
        p.distance
    );
    for w in [p.on_a, p.on_b] {
        assert!(
            w.coords.iter().all(|c| c.is_finite() && c.abs() < 2.0),
            "witness {w:?} not plausible"
        );
    }
    assert!(
        ((p.on_a - p.on_b).norm() - p.distance).abs() < 1e-6,
        "witness gap vs distance {:+.4}",
        p.distance
    );
}

#[test]
fn in_collision_threshold_semantics() {
    let mut m = model();
    let rest = m.min_distance(&HOME, &HOME).expect("query").distance;
    assert!(m.in_collision(&HOME, &HOME, rest + 0.005).expect("query"));
    assert!(!m.in_collision(&HOME, &HOME, rest - 0.005).expect("query"));
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
/// tick. Debug builds are several times slower, so assert in release only.
#[test]
fn query_stays_inside_the_control_tick() {
    let mut m = model();
    let configs: Vec<(JointVec, JointVec)> =
        (0..200).map(|i| wrists_inward(i as f64 * 0.005)).collect();

    let start = std::time::Instant::now();
    let mut acc = 0.0;
    for (ql, qr) in &configs {
        acc += m.min_distance(ql, qr).expect("query").distance;
    }
    let per_query = start.elapsed().as_secs_f64() / configs.len() as f64;
    assert!(acc.is_finite());
    println!("per-query: {:.1} us", per_query * 1e6);
    if !cfg!(debug_assertions) {
        assert!(
            per_query < 1e-3,
            "query took {:.1} us, budget is 1 ms",
            per_query * 1e6
        );
    }
}

#[test]
fn governor_halts_an_approach_before_contact() {
    let mut m = model();
    let band = band();
    let mut halted_at = None;
    let mut d_now = m
        .min_distance(&wrists_inward(0.0).0, &wrists_inward(0.0).1)
        .expect("d0")
        .distance;
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
    assert!(
        d > 0.0,
        "governor halted at t={t} with clearance {d:+.4}, after contact"
    );
}
