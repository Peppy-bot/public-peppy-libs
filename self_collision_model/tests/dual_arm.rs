//! Integration tests: the OpenArm fixture model driven through two-arm
//! scenarios. Distances are signed hull distances (GJK, EPA on overlap), so a
//! change to the fixture geometry or the fit moves these numbers. Most assertions
//! are qualitative (clear vs colliding, monotone, which links) so they survive a
//! re-fit; `home_clearance_is_stable` is the deliberate exception, pinning the
//! clearance at home as a regression guard (update its constant on an intended re-fit).

use robot_description::{RobotDescription, states};
use self_collision_model::nalgebra::Point3;
use self_collision_model::{Joints, Placed};

#[path = "fixtures/models.rs"]
mod fixture;

use fixture::openarm::{ARM_DOF, MODEL, model};
use fixture::{description, fingers, joint, parent_link, posture, torso};

/// One arm's joints.
type Q = [f64; ARM_DOF];

/// The bodies of every gripper: each finger and the hand it hangs off.
fn gripper_bodies(description: &RobotDescription) -> Vec<String> {
    description
        .arms()
        .iter()
        .flat_map(|arm| &arm.gripper.fingers)
        .flat_map(|finger| [finger.link.clone(), parent_link(description, &finger.joint)])
        .collect()
}

/// Both arms elbow-bent, j3 wrapping the wrists toward the centerline.
fn wrists_inward(t: f64) -> Joints<ARM_DOF, 2> {
    let mut ql: Q = [0.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0];
    let mut qr = ql;
    ql[2] = t;
    qr[2] = -t;
    [ql, qr]
}

#[test]
fn wrists_converging_monotonically_reach_collision() {
    let mut m = model();
    let mut prev = f64::INFINITY;
    let mut last = (String::new(), String::new(), 0.0);
    for i in 0..=12 {
        let p = m
            .min_distance(&wrists_inward(i as f64 * 0.1))
            .expect("query");
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
    // The gripper region: a hand or one of its finger bodies. A wrapped wrist drives
    // its outstretched finger into contact, so the deepest witness is typically a
    // finger.
    let grippers = gripper_bodies(&description(MODEL));
    assert!(
        grippers.contains(&a) || grippers.contains(&b),
        "deepest pair should involve a hand or its finger, got {a} vs {b}"
    );
}

#[test]
fn closing_the_gripper_recovers_clearance() {
    // The core reason per-finger live hulls exist: when the two grippers approach,
    // closing the fingers pulls them back from the fat fully-open envelope, so the
    // reported clearance grows and the arms can come closer before the governor
    // stops them. Same pose, only the opening changes.
    let mut m = model();
    let q = wrists_inward(0.9);

    m.set_gripper_openings(&[1.0, 1.0]); // fully open: widest finger envelope
    let (open, open_a, open_b) = {
        let p = m.min_distance(&q).expect("query");
        (p.distance, p.link_a.to_string(), p.link_b.to_string())
    };
    let fingers = fingers(&description(MODEL));
    assert!(
        fingers.contains(&open_a) || fingers.contains(&open_b),
        "setup: a finger should be the nearest body when open, got {open_a} vs {open_b}"
    );

    m.set_gripper_openings(&[0.0, 0.0]); // fully closed: fingers retract
    let closed = m.min_distance(&q).expect("query").distance;

    assert!(
        closed > open + 1e-4,
        "closing the fingers should recover clearance: open {open:+.4}, closed {closed:+.4}"
    );

    // Out-of-range openings clamp to the travel: same placement as the extremes.
    m.set_gripper_openings(&[-0.5, -0.5]);
    let clamped = m.min_distance(&q).expect("query").distance;
    assert!(
        (clamped - closed).abs() < 1e-12,
        "an out-of-range opening should clamp to the nearest extreme"
    );
}

#[test]
fn folding_the_arms_inward_drives_a_collision() {
    let description = description(MODEL);
    let mut m = model();
    // Mirrored j2 folds both arms toward the centerline, into the torso.
    let ql: Q = [0.0, 0.6, 0.0, 0.4, 0.0, 0.0, 0.0];
    let qr: Q = [0.0, -0.6, 0.0, 0.4, 0.0, 0.0, 0.0];
    let p = m.min_distance(&[ql, qr]).expect("query");
    assert!(
        p.distance < 0.0,
        "folded arms should interpenetrate, got {:+.4}",
        p.distance
    );
    // The upper arms: the links of each arm's third and fourth joints.
    let upper_arms_and_torso: Vec<String> = description
        .arms()
        .iter()
        .flat_map(|arm| &arm.joints[2..4])
        .map(|name| joint(&description, name).child.link.clone())
        .chain([torso(&description)])
        .collect();
    let touches_arm = [p.link_a, p.link_b]
        .iter()
        .any(|link| upper_arms_and_torso.iter().any(|body| body == link));
    assert!(
        touches_arm,
        "expected an upper-arm or torso witness, got {} vs {}",
        p.link_a, p.link_b
    );
}

#[test]
fn home_clearance_is_stable() {
    // The clearance at home, pinned to the measured value as a two-sided regression
    // guard: a re-fit that moves the clearance at home in either direction trips this, so
    // update the constant when the geometry deliberately changes.
    const HOME_CLEARANCE_M: f64 = 0.0872;
    const TOLERANCE_M: f64 = 0.001;
    let mut m = model();
    let p = m
        .min_distance(&posture(&description(MODEL), states::HOME))
        .expect("query");
    assert!(
        (p.distance - HOME_CLEARANCE_M).abs() < TOLERANCE_M,
        "home clearance {:+.5} drifted from {HOME_CLEARANCE_M} (tol {TOLERANCE_M})",
        p.distance
    );
}

#[test]
fn queries_are_order_independent() {
    // Support is an exact scan, so a query's answer does not depend on the queries
    // before it. At a near-home pose on the wrists' converging path the clearance reads
    // clear, and a sweep of the model between two reads of it leaves the answer
    // unchanged.
    let mut m = model();
    let q = [
        [0.0, 0.0, 0.1075, 0.1575, 0.0, 0.0, 0.0],
        [0.0, 0.0, -0.1075, 0.1575, 0.0, 0.0, 0.0],
    ];
    let first = m.min_distance(&q).expect("query").distance;
    assert!(
        first > 0.02,
        "near-home converging pose must read clear (~+0.033 true), got {first:+.5}"
    );
    m.min_distance(&wrists_inward(1.2)).expect("query");
    let again = m.min_distance(&q).expect("query").distance;
    assert!(
        (first - again).abs() < 1e-12,
        "query order changed the answer: {first:+.5} then {again:+.5}"
    );
}

#[test]
fn separating_sweep_increases_clearance() {
    // Sweeping j2 outward moves the arms apart, so the nearest-pair clearance only
    // grows (and plateaus once the binding pair stops closing). Stronger than a
    // fixed floor: it asserts the separation actually shows up as monotone,
    // positive clearance.
    let mut m = model();
    let mut prev = f64::NEG_INFINITY;
    for i in 0..=12 {
        let t = i as f64 * 0.1;
        let p = m
            .min_distance(&[
                [0.0, -t, 0.0, 0.4, 0.0, 0.0, 0.0],
                [0.0, t, 0.0, 0.4, 0.0, 0.0, 0.0],
            ])
            .expect("query");
        assert!(
            p.distance > 0.0,
            "outward sweep should stay clear at t={t}, got {:+.4}",
            p.distance
        );
        assert!(
            p.distance >= prev - 1e-6,
            "outward sweep reduced clearance to {:+.4} at t={t}",
            p.distance
        );
        prev = p.distance;
    }
}

#[test]
fn witnesses_are_finite_and_span_the_gap_when_clear() {
    let mut m = model();
    // A clearly separated pose: the witnesses lie on the surfaces and their gap
    // equals the (positive) reported distance.
    let p = m.min_distance(&wrists_inward(0.3)).expect("query");
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
fn non_finite_configurations_are_rejected() {
    let mut m = model();
    let [left, right] = posture(&description(MODEL), states::HOME);
    let mut bad = left;
    bad[3] = f64::NAN;
    assert!(m.min_distance(&[bad, right]).is_err());
    assert!(m.min_distance(&[left, bad]).is_err());
    bad[3] = f64::INFINITY;
    assert!(m.min_distance(&[left, bad]).is_err());
}

/// A conservative lower bound on the distance from `p` to a placed piece's
/// rounded surface: the deepest face-plane violation of the core hull (each
/// half-space distance under-estimates the hull distance) minus the inflation
/// radius. Positive means provably outside the piece by at least that much.
fn outside_by_at_least(piece: &Placed, p: &Point3<f64>) -> f64 {
    let local = piece.pose().inverse_transform_point(p);
    piece
        .hull()
        .planes()
        .iter()
        .map(|(normal, offset)| normal.dot(&local.coords) - offset)
        .fold(f64::NEG_INFINITY, f64::max)
        - piece.hull().inflation()
}

#[test]
fn points_the_torso_mesh_leaves_empty_are_outside_its_pieces() {
    // Points by the torso's concave features that its mesh leaves empty: behind the
    // gusset's hypotenuse, beside the tapered top of the head, and above the 8 mm base
    // plate. Each is provably clear of every torso piece by its margin, so a body
    // approaching one reads its real clearance.
    let probes: [(Point3<f64>, f64); 3] = [
        (Point3::new(-0.10, 0.0, 0.15), 0.015),
        (Point3::new(-0.083, 0.075, 0.760), 0.020),
        (Point3::new(0.05, 0.05, 0.022), 0.003),
    ];
    let torso_body = torso(&description(MODEL));
    let mut m = model();
    let pieces = m
        .world_pieces(&posture(&description(MODEL), states::HOME))
        .expect("pieces");
    let torso: &Vec<Placed> = &pieces
        .iter()
        .find(|(name, _)| *name == torso_body)
        .expect("torso body present")
        .1;
    for (probe, margin) in probes {
        let clearance = torso
            .iter()
            .map(|piece| outside_by_at_least(piece, &probe))
            .fold(f64::INFINITY, f64::min);
        assert!(
            clearance >= margin,
            "phantom probe {probe:?} clears the pieces by {clearance:+.4}, need {margin}"
        );
    }
}

/// What the fit costs and buys, per piece and in total. Not an assertion: the
/// numbers are the deliverable, so run it deliberately.
///
/// ```sh
/// cargo test --release --test dual_arm fit_cost_report -- --ignored --nocapture
/// ```
#[test]
#[ignore = "measurement probe, not a pass/fail check"]
fn fit_cost_report() {
    let home = posture(&description(MODEL), states::HOME);
    let build_start = std::time::Instant::now();
    let mut m = model();
    let build = build_start.elapsed();

    let mut rows: Vec<(String, usize, usize, f64)> = m
        .world_pieces(&home)
        .expect("pieces")
        .into_iter()
        .map(|(name, ps)| {
            let vertices = ps.iter().map(|p| p.hull().vertices().len()).sum();
            let radius = ps.iter().map(|p| p.hull().inflation()).fold(0.0, f64::max);
            (name.to_string(), ps.len(), vertices, radius)
        })
        .collect();
    rows.sort_by_key(|r| std::cmp::Reverse(r.2));
    let (pieces, vertices) = rows.iter().fold((0, 0), |(p, v), r| (p + r.1, v + r.2));
    let radius = rows.iter().map(|r| r.3).fold(0.0, f64::max);

    println!("\nbody                             pieces  vertices  radius (mm)");
    for (name, p, v, r) in &rows {
        println!("{name:32} {p:6} {v:9} {:12.3}", r * 1000.0);
    }
    println!(
        "{:32} {pieces:6} {vertices:9} {:12.3}",
        "TOTAL",
        radius * 1000.0
    );

    const ITERATIONS: usize = 2000;
    let mut acc = 0.0;

    // Home first, at the openings the model defaults to, so the clearance
    // printed here is the same quantity `home_clearance_is_stable` pins and
    // can be read straight into its constant.
    let home_clearance = m.min_distance(&home).expect("query").distance;
    let at_home = std::time::Instant::now();
    for _ in 0..ITERATIONS {
        acc += m.min_distance(&home).expect("query").distance;
    }
    let per_home = at_home.elapsed().as_secs_f64() / ITERATIONS as f64;

    // An in-band pose with the jaws closed, the regime the governor actually
    // queries in. This overrides the openings, so it runs last.
    let q = [
        [0.0, 0.0, 1.2, 0.4, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.2, 0.4, 0.0, 0.0, 0.0],
    ];
    m.set_gripper_openings(&[0.0, 0.0]);
    let d = m.min_distance(&q).expect("query").distance;
    let start = std::time::Instant::now();
    for _ in 0..ITERATIONS {
        acc += m.min_distance(&q).expect("query").distance;
    }
    let per_query = start.elapsed().as_secs_f64() / ITERATIONS as f64;
    assert!(acc.is_finite());

    println!("separated (home) query {:.1} us", per_home * 1e6);
    println!(
        "\nbuild {:.0} ms | in-band query {:.1} us (d={d:+.5} m) | home clearance {home_clearance:+.5} m\n",
        build.as_secs_f64() * 1e3,
        per_query * 1e6,
    );
}

/// Wall-clock budget: a full dual-arm query must stay far inside a control
/// tick. Debug builds are several times slower, so assert in release only.
#[test]
fn query_stays_inside_the_control_tick() {
    let mut m = model();
    let configs: Vec<Joints<ARM_DOF, 2>> =
        (0..200).map(|i| wrists_inward(i as f64 * 0.005)).collect();

    let start = std::time::Instant::now();
    let mut acc = 0.0;
    for q in &configs {
        acc += m.min_distance(q).expect("query").distance;
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
