//! The walk along a line by inverse kinematics from a seed, on one-joint
//! solvers whose every answer is known: each step solved from the state
//! before, the walk ending at the first step it cannot take, and the
//! precision's bisection of a step whose joint midpoint leaves the line.

use std::cell::RefCell;
use std::f64::consts::PI;

use chain_kinematics::nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use chain_kinematics::{LinePrecision, LineSolver, LineState, SolveError, WalkEnd, walk_line};
use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{TOLERANCE, poses, unit_vectors};

/// MoveIt's CartesianPrecision defaults: 1 mm, 0.01 rad, down to 1e-5 of a
/// line.
fn moveits_precision() -> LinePrecision {
    LinePrecision::new(1e-3, 1e-2, 1e-5).expect("MoveIt's precision is finite and positive")
}

/// One prismatic joint whose frame stands at `warp(q)` along x, solved exactly
/// for the frame's x, recording each solve's state before and blend, and
/// answering none at solve number `fails_at`.
struct Slider {
    warp: fn(f64) -> f64,
    unwarp: fn(f64) -> f64,
    fails_at: Option<usize>,
    solves: RefCell<Vec<(LineState<1>, f64)>>,
}

impl Slider {
    fn new(warp: fn(f64) -> f64, unwarp: fn(f64) -> f64, fails_at: Option<usize>) -> Self {
        Self {
            warp,
            unwarp,
            fails_at,
            solves: RefCell::new(Vec::new()),
        }
    }
}

impl LineSolver<1> for Slider {
    fn solve(&self, pose: &Isometry3<f64>, from: &LineState<1>, to: f64) -> Option<[f64; 1]> {
        let mut solves = self.solves.borrow_mut();
        solves.push((*from, to));
        (self.fails_at != Some(solves.len() - 1)).then(|| [(self.unwarp)(pose.translation.x)])
    }

    fn pose(&self, q: &[f64; 1]) -> Isometry3<f64> {
        Isometry3::translation((self.warp)(q[0]), 0.0, 0.0)
    }
}

/// The line along x from `x0` to `x1`.
fn along_x(x0: f64, x1: f64) -> impl Fn(f64) -> Isometry3<f64> {
    move |s| Isometry3::translation(x0 + s * (x1 - x0), 0.0, 0.0)
}

// A walk solves each blend in turn, in any order, from the state before it,
// at the line's pose there, and hands each step, its own state alone where
// the line keeps a step whole, to the caller with the state before. It ends
// at the first step whose pose has no solution or that the caller refuses:
// every step before it kept, none after it solved.
#[hegel::test]
fn a_walk_solves_each_blend_from_the_state_before_and_ends_at_the_first_step_it_cannot_take(
    tc: TestCase,
) {
    let x1: f64 = tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
    let line = along_x(0.0, x1);
    let on_line = |blend: f64| LineState {
        blend,
        q: [line(blend).translation.x],
    };
    let blends: Vec<f64> =
        tc.draw(gs::vecs(gs::floats().min_value(0.0).max_value(1.0)).max_size(10));
    let from = on_line(tc.draw(gs::floats().min_value(0.0).max_value(1.0)));
    let stop: usize = tc.draw(gs::integers().min_value(0).max_value(blends.len()));
    let refused = tc.draw(gs::booleans());
    let precision = tc.draw(gs::booleans()).then(moveits_precision);
    let solver = Slider::new(|q| q, |x| x, (!refused).then_some(stop));
    let mut steps = Vec::new();
    let end = walk_line(
        &solver,
        &line,
        from,
        blends.iter().copied(),
        precision.as_ref(),
        |before, step| {
            steps.push((*before, step.to_vec()));
            !(refused && steps.len() - 1 == stop)
        },
    );

    let expected_end = match blends.get(stop) {
        None => WalkEnd::Whole,
        Some(&blend) if refused => WalkEnd::Refused { blend },
        Some(&blend) => WalkEnd::NoSolution { blend },
    };
    assert_eq!(end, expected_end);
    let states: Vec<LineState<1>> = std::iter::once(from)
        .chain(blends.iter().map(|&blend| on_line(blend)))
        .collect();
    let solved = blends.len().min(stop + 1);
    let expected_solves: Vec<(LineState<1>, f64)> =
        (0..solved).map(|j| (states[j], blends[j])).collect();
    assert_eq!(*solver.solves.borrow(), expected_solves);
    let taken = if refused {
        solved
    } else {
        stop.min(blends.len())
    };
    let expected_steps: Vec<(LineState<1>, Vec<LineState<1>>)> = (0..taken)
        .map(|j| (states[j], vec![states[j + 1]]))
        .collect();
    assert_eq!(steps, expected_steps);
}

// A step whose solution turns a whole turn further than the line, where the
// solver jumps branch, keeps its pose at both ends while its joint midpoint
// turns half a turn off the line, and every half of it holding the jump
// does too: with a precision the walk ends imprecise at that step, every step
// before it kept; without one it walks the whole line.
#[hegel::test]
fn a_walk_whose_solution_jumps_a_turn_ends_before_the_jump(tc: TestCase) {
    struct Turntable {
        turn: f64,
        jump: f64,
    }
    impl LineSolver<1> for Turntable {
        fn solve(&self, _: &Isometry3<f64>, _: &LineState<1>, to: f64) -> Option<[f64; 1]> {
            let branch = if to >= self.jump { 2.0 * PI } else { 0.0 };
            Some([to * self.turn + branch])
        }

        fn pose(&self, q: &[f64; 1]) -> Isometry3<f64> {
            Isometry3::rotation(Vector3::z() * q[0])
        }
    }
    let solver = Turntable {
        turn: tc.draw(gs::floats().min_value(0.0).max_value(PI)),
        jump: tc.draw(gs::floats().min_value_exclusive(0.0).max_value(1.0)),
    };
    let steps: usize = tc.draw(gs::integers().min_value(1).max_value(20));
    let blends = (1..=steps).map(|k| k as f64 / steps as f64);
    let line = |s: f64| Isometry3::rotation(Vector3::z() * (s * solver.turn));
    let from = LineState {
        blend: 0.0,
        q: [0.0],
    };
    let jumping = blends
        .clone()
        .find(|&blend| blend >= solver.jump)
        .expect("the last blend is 1, at or past every jump");

    let mut kept = Vec::new();
    let end = walk_line(
        &solver,
        line,
        from,
        blends.clone(),
        Some(&moveits_precision()),
        |_, step| {
            kept.extend_from_slice(step);
            true
        },
    );
    assert_eq!(end, WalkEnd::Imprecise { blend: jumping });
    let before: Vec<LineState<1>> = blends
        .clone()
        .take_while(|&blend| blend < jumping)
        .map(|blend| LineState {
            blend,
            q: [blend * solver.turn],
        })
        .collect();
    assert_eq!(kept, before);

    let unchecked = walk_line(&solver, line, from, blends, None, |_, _| true);
    assert_eq!(unchecked, WalkEnd::Whole);
}

// With a precision, a step whose joint midpoint leaves the line by more than
// it is halved: the midpoint's pose is solved from the joint midpoint between
// the interval's two states, the states solved next to each other so far, and
// each half refined in turn. Every state kept stands on the line at its own
// blend, in order, each step's states end at its blend, and the joint
// midpoint of every two states kept next to each other keeps the precision.
#[hegel::test]
fn a_step_whose_joint_midpoint_leaves_the_line_is_halved_until_every_half_keeps_it(tc: TestCase) {
    let (x0, x1): (f64, f64) = (
        tc.draw(gs::floats().min_value(-1.0).max_value(1.0)),
        tc.draw(gs::floats().min_value(-1.0).max_value(1.0)),
    );
    let line = along_x(x0, x1);
    let steps: usize = tc.draw(gs::integers().min_value(1).max_value(4));
    let blends: Vec<f64> = (1..=steps).map(|k| k as f64 / steps as f64).collect();
    let solver = Slider::new(|q| q * q * q, f64::cbrt, None);
    let from = LineState {
        blend: 0.0,
        q: [x0.cbrt()],
    };
    let precision = moveits_precision();
    let mut kept = vec![from];
    let end = walk_line(
        &solver,
        &line,
        from,
        blends.iter().copied(),
        Some(&precision),
        |before, step| {
            assert_eq!(*before, kept[kept.len() - 1]);
            kept.extend_from_slice(step);
            true
        },
    );
    assert_eq!(end, WalkEnd::Whole);
    if kept.len() > blends.len() + 1 {
        tc.event("a step halved");
    }

    for state in &kept {
        let miss = solver.pose(&state.q).translation.x - line(state.blend).translation.x;
        assert!(miss.abs() <= TOLERANCE, "{state:?} off the line by {miss}");
    }
    for blend in &blends {
        assert!(
            kept.iter().any(|state| state.blend == *blend),
            "no state at blend {blend}"
        );
    }
    for pair in kept.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        assert!(a.blend < b.blend, "{a:?} kept before {b:?}");
        let midway = solver.pose(&[0.5 * (a.q[0] + b.q[0])]);
        let off = (midway.translation.x - line(0.5 * (a.blend + b.blend)).translation.x).abs();
        assert!(
            off <= 1e-3,
            "the joint midpoint of {a:?} and {b:?} leaves the line by {off}"
        );
    }

    let solves = solver.solves.borrow();
    let mut solved = vec![from];
    for (seed, to) in solves.iter() {
        if !blends.contains(to) {
            let right = solved
                .iter()
                .filter(|state| state.blend > *to)
                .min_by(|a, b| a.blend.total_cmp(&b.blend))
                .expect("a midpoint lies between two states solved");
            let left = solved
                .iter()
                .filter(|state| state.blend < *to)
                .max_by(|a, b| a.blend.total_cmp(&b.blend))
                .expect("a midpoint lies between two states solved");
            assert!(
                (seed.q[0] - 0.5 * (left.q[0] + right.q[0])).abs() <= TOLERANCE
                    && (seed.blend - to).abs() <= TOLERANCE
                    && (to - 0.5 * (left.blend + right.blend)).abs() <= TOLERANCE,
                "the midpoint at {to} solved from {seed:?}, between {left:?} and {right:?}"
            );
        }
        solved.push(LineState {
            blend: *to,
            q: [line(*to).translation.x.cbrt()],
        });
    }
}

// A precision holds a pose exactly when it stands within the translational
// bound of the target and turned within the rotational one, either way.
#[hegel::test]
fn a_precision_holds_a_pose_within_both_of_its_bounds(tc: TestCase) {
    let translational: f64 = tc.draw(gs::floats().min_value(1e-3).max_value(0.1));
    let rotational: f64 = tc.draw(gs::floats().min_value(1e-3).max_value(1.0));
    let precision =
        LinePrecision::new(translational, rotational, 1e-5).expect("a finite positive precision");
    let target = tc.draw(poses());
    let distance: f64 = tc.draw(gs::floats().min_value(0.0).max_value(2.0 * translational));
    let angle: f64 = tc.draw(gs::floats().min_value(0.0).max_value(2.0 * rotational));
    tc.assume((distance - translational).abs() > TOLERANCE);
    tc.assume((angle - rotational).abs() > TOLERANCE);
    let turned =
        UnitQuaternion::from_axis_angle(&Unit::new_normalize(tc.draw(unit_vectors())), angle);
    let actual = Isometry3::from_parts(
        Translation3::from(target.translation.vector + tc.draw(unit_vectors()) * distance),
        turned * target.rotation,
    );
    assert_eq!(
        precision.holds(&actual, &target),
        distance <= translational && angle <= rotational
    );
}

// A precision is built from three finite positive values; the first of them
// that is zero, negative or not finite is refused, by its value.
#[hegel::test]
fn a_precision_that_is_not_finite_and_positive_is_refused(tc: TestCase) {
    let value = || {
        hegel::one_of!(
            gs::floats().min_value(f64::MIN_POSITIVE).max_value(1.0),
            gs::sampled_from(vec![0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY]),
        )
    };
    let values: [f64; 3] = [tc.draw(value()), tc.draw(value()), tc.draw(value())];
    let refused = values
        .iter()
        .copied()
        .find(|v| !(v.is_finite() && *v > 0.0));
    match (LinePrecision::new(values[0], values[1], values[2]), refused) {
        (Ok(_), None) => {}
        (Err(SolveError::Tolerance { value, .. }), Some(bad)) => {
            assert_eq!(value.to_bits(), bad.to_bits());
        }
        (built, refused) => panic!(
            "{values:?} built {:?} with {refused:?} refusable",
            built.is_ok()
        ),
    }
}
