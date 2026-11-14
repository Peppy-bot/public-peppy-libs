//! The orientations a plan's path holds: what its states, its motions and the rounded path
//! the robot runs keep of each [`PathOrientation`], and the function a plan corrects its
//! states with, OMPL's projected state space as MoveIt's constrained planning uses it for
//! one orientation path constraint (moveit2 2.15.2 ompl_constraints.cpp), here for each of
//! them, one row per orientation, so their order carries no meaning.
//!
//! An orientation's error is its contract's measure ([`OrientationTarget::error`]). So:
//!
//! - The function is an orientation's error above the angle the plan allows it, 0 inside
//!   it: `HELD_SHARE` of its tolerance, or the start's error where that is more. A
//!   corrected state stands within the allowed angle to the projection's tolerance.
//! - A joint turning by an angle turns a frame it moves by at most that angle, so a frame
//!   turns along a motion at most the sum of the changes of its joints, and a step that
//!   turns it no farther than its spare keeps every state of the step within its
//!   tolerance. [`PathOrientations::walk`] walks a run or an arc so, and a state nearer its
//!   tolerance than `FLOOR_SHARE` of it ends the walk.
//! - A straight joint motion `a` turns a frame away from the shortest rotation between its
//!   ends by at most `|a|_1^2 / 16`: the frame's angular velocity changes along it at most
//!   `|a|_1^2 / 2`. A free axis takes as much again: along the shortest rotation it follows
//!   a small circle, at most `phi^2 / 16` off the great circle between its ends, `phi` the
//!   rotation's angle, at most `|a|_1`. The plan's spacing keeps the first under half of what
//!   lies between the allowed angle and the tolerance, and the two together under all of it,
//!   so a run between a plan's states rarely reaches the floor. The walk of each run judges
//!   it either way.

use std::num::NonZeroU32;

use chain_kinematics::{Chain, ChainFrame, JointKind};
use control_core::positive_finite::PositiveFinite;
use ompl_bridge::{CONSTRAINED_SPACING, CheckError, Constraint, Projection};

use crate::arm::Moving;
use crate::arms::Arms;
use crate::goal::{Frame, Goal, OrientationTarget, frame_pose_at};
use crate::plan::PlanFailure;

/// The share of an orientation's tolerance a plan's corrected states stand within, so the
/// straight runs and the rounded corners between them stay within all of it. At 0.9 a plan
/// keeps OMPL's default spacing from a tolerance of about 0.09 rad on a frame seven joints turn
/// (`Held::spacing`). On 200 drawn scenes at a tolerance of 0.1 rad, 0.8, 0.9 and 0.95 plan the
/// same scenes at the same median length, 0.9 with the fewest waypoints: a mean of 15.9 against
/// 17.4 and 20.6 holding the whole rotation, and 27.5 against 30.5 and 35.1 with a free axis.
const HELD_SHARE: f64 = 0.9;

/// The share of an orientation's tolerance that ends a walk: a state nearer the tolerance
/// than this takes no step. Each step then turns the frame at least this share of the
/// tolerance, so a walk checks at most `1 / (0.01 * tolerance)` states per radian the frame
/// may turn: 1000 at a tolerance of 0.1 rad. The answered paths of the drawn sets, from starts
/// under 0.4 of the tolerance, reach at most 0.904 of it, so a floor at 0.99 of it ends none of
/// their walks.
const FLOOR_SHARE: f64 = 0.01;

/// The share of what lies between the allowed angle and the tolerance that a projected
/// state may stand past the allowed angle, which leaves nine tenths of it for what a run strays.
/// It sets the projection's tolerance where that span lies under 1 mrad, a tolerance of 0.01 rad
/// at `HELD_SHARE`, and `PROJECTION_TOLERANCE` sets it above. A start past 0.9889 of its tolerance
/// allows a corrected state past `FLOOR_SHARE`'s bound, where the plan's checks refuse it.
const PROJECTION_SHARE: f64 = 0.1;

/// The most a projected state stands off its orientation's allowed angle: OMPL's default
/// (OMPL 2.0.2 `Constraint.h:55`), which MoveIt plans with.
const PROJECTION_TOLERANCE: f64 = 1e-4;

/// The Newton steps of one projection at most: OMPL's default (`Constraint.h:59`).
const PROJECTION_ITERATIONS: NonZeroU32 = NonZeroU32::new(50).expect("50 is above 0");

/// An orientation that a frame of an arm of a plan holds at every state of the path.
#[derive(Debug, Clone, Copy)]
pub struct PathOrientation<'a> {
    /// The arm's place in the plan's group.
    pub arm: usize,
    /// The name of the frame's link, as a plan's messages name it.
    pub link_name: &'a str,
    pub target: OrientationTarget,
}

/// An orientation of a plan with what the plan keeps of it.
struct Held<'a, const N: usize> {
    chain: &'a Chain<N>,
    arm: usize,
    link_name: &'a str,
    target: OrientationTarget,
    /// The joints of the arm that turn the frame.
    turning: [bool; N],
    /// The error of a state the plan corrects, at most.
    allowed: f64,
}

impl<const N: usize> Held<'_, N> {
    fn tolerance(&self) -> f64 {
        self.target.tolerance_rad.get()
    }

    /// The target with its allowed angle as its tolerance: what a goal of the frame meets.
    fn at_allowed(&self) -> OrientationTarget {
        OrientationTarget {
            tolerance_rad: PositiveFinite::parse(self.allowed)
                .expect("a tolerance above 0 allows an angle above 0"),
            ..self.target
        }
    }

    /// The largest error a state of a plan's walk may hold.
    fn largest(&self) -> f64 {
        largest_of(self.tolerance())
    }

    fn error<const D: usize>(&self, q: &[f64; D]) -> f64 {
        self.target
            .error_at(self.chain, &Arms::<N, D>::slice(q, self.arm))
    }

    /// The farthest the frame turns per unit of a position along a motion on which each
    /// joint of the state changes at most its `motion` per unit.
    fn turn_rate<const D: usize>(&self, motion: &[f64; D]) -> f64 {
        let joints = Arms::<N, D>::slice(motion, self.arm);
        (0..N).filter(|&j| self.turning[j]).map(|j| joints[j]).sum()
    }

    /// The spacing of a plan's states that keeps its runs within the tolerance, as the
    /// module says. Unbounded for a frame no joint turns.
    fn spacing(&self) -> f64 {
        let joints = self.turning.iter().filter(|turning| **turning).count();
        if joints == 0 {
            return f64::INFINITY;
        }
        (8.0 * (self.tolerance() - self.allowed) / joints as f64).sqrt()
    }

    /// Writes the row of the function's Jacobian at `q` into `row`, which holds 0: the
    /// frame's error axis against each joint's axis, outside the allowed angle.
    fn write_row<const D: usize>(&self, q: &[f64; D], row: &mut [f64]) {
        let posed = self.chain.at(&Arms::<N, D>::slice(q, self.arm));
        let actual = frame_pose_at(&posed, self.target.frame).rotation;
        if self.target.error(&actual) <= self.allowed {
            return;
        }
        let Some(axis) = self.target.error_axis(&actual) else {
            return;
        };
        for j in (0..N).filter(|&j| self.turning[j]) {
            row[self.arm * N + j] = axis.dot(&posed.axis_world(j));
        }
    }
}

/// An orientation a walk finds broken, or a goal sample past its allowed angle.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Turned {
    /// The orientation's place in the plan's list.
    pub(crate) orientation: usize,
    pub(crate) error: f64,
}

/// The orientations a plan's path holds.
pub(crate) struct PathOrientations<'a, const N: usize, const D: usize> {
    held: Vec<Held<'a, N>>,
}

impl<'a, const N: usize, const D: usize> PathOrientations<'a, N, D> {
    /// The checks of a path that holds no orientation.
    pub(crate) fn none() -> Self {
        Self { held: Vec::new() }
    }

    /// What a plan of `arms` from `start` keeps of `orientations`. A list as long as the
    /// state's joints fails with `PlanFailure::Request`: OMPL needs fewer rows than
    /// dimensions. A start that holds an orientation past what a walk may leave fails with
    /// `PlanFailure::NoPath`.
    ///
    /// # Panics
    ///
    /// When an orientation names an arm outside `arms`.
    pub(crate) fn new(
        arms: &'a Arms<'a, N, D>,
        orientations: &[PathOrientation<'a>],
        start: &[f64; D],
    ) -> Result<Self, PlanFailure> {
        if orientations.len() >= D {
            return Err(PlanFailure::Request(format!(
                "path_orientation_constraints holds {} constraints, and a plan of {D} joints takes at most {}: send fewer",
                orientations.len(),
                D - 1
            )));
        }
        let held = orientations
            .iter()
            .map(|orientation| Self::held_from(arms, orientation, start))
            .collect::<Result<_, _>>()?;
        Ok(Self { held })
    }

    fn held_from(
        arms: &'a Arms<'a, N, D>,
        orientation: &PathOrientation<'a>,
        start: &[f64; D],
    ) -> Result<Held<'a, N>, PlanFailure> {
        let PathOrientation {
            arm,
            link_name,
            target,
        } = *orientation;
        let chain = arms.arms()[arm].geometry().chain();
        let error = target.error_at(chain, &Arms::<N, D>::slice(start, arm));
        let tolerance = target.tolerance_rad.get();
        let largest = largest_of(tolerance);
        if error > largest {
            return Err(PlanFailure::NoPath(format!(
                "the start holds {link_name} {error:.4} rad from its path orientation, and a path \
                 keeps within {largest:.4} rad of its tolerance_rad {tolerance:.4}; send a start \
                 within it or a wider tolerance_rad"
            )));
        }
        Ok(Held {
            chain,
            arm,
            link_name,
            target,
            turning: turning_joints(chain, target.frame),
            allowed: (tolerance * HELD_SHARE).max(error),
        })
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.held.is_empty()
    }

    /// The first orientation that `moving` judges whose error at `q` lies past its allowed
    /// angle: a goal the projected space does not hold.
    pub(crate) fn past_allowed(&self, q: &[f64; D], moving: Moving) -> Option<Turned> {
        self.held
            .iter()
            .enumerate()
            .filter(|(_, held)| moving.judges(held.arm, held.arm))
            .find_map(|(orientation, held)| {
                let error = held.error(q);
                (error > held.allowed).then_some(Turned { orientation, error })
            })
    }

    /// The orientations of the arm at `arm`, each at its allowed angle: what a goal of that
    /// arm draws and meets ([`crate::goal::Goal::holding`]).
    pub(crate) fn targets_of(&self, arm: usize) -> Vec<OrientationTarget> {
        self.held
            .iter()
            .filter(|held| held.arm == arm)
            .map(Held::at_allowed)
            .collect()
    }

    /// Why the goal `goal` of the arm at `arm` shares no state with an orientation of its frame,
    /// as a message says it; none where it may.
    pub(crate) fn goal_apart(&self, arm: usize, goal: &Goal) -> Option<String> {
        self.held.iter().filter(|held| held.arm == arm).find_map(|held| {
            let angle = goal.orientation_apart_from(&held.at_allowed())?;
            Some(format!(
                "the goal's orientation of {} stands {angle:.4} rad from its path orientation, past \
                 its own tolerance_rad and the {:.4} rad a goal keeps of the path's tolerance_rad \
                 {:.4}; send a goal orientation nearer the path's or a wider tolerance_rad",
                held.link_name,
                held.allowed,
                held.tolerance()
            ))
        })
    }

    /// Walks the positions from `from` to `to` of a motion, its state at `at`, on which each
    /// joint changes at most its `motion` per unit of position, as the module says. Where
    /// an orientation breaks, the position and which.
    pub(crate) fn walk(
        &self,
        from: f64,
        to: f64,
        motion: &[f64; D],
        at: impl Fn(f64) -> [f64; D],
    ) -> Result<(), (f64, Turned)> {
        for (orientation, held) in self.held.iter().enumerate() {
            let rate = held.turn_rate(motion);
            let mut position = from;
            loop {
                let error = held.error(&at(position));
                if error > held.largest() {
                    return Err((position, Turned { orientation, error }));
                }
                if position >= to || rate <= 0.0 {
                    break;
                }
                position = (position + (held.tolerance() - error) / rate).min(to);
            }
        }
        Ok(())
    }

    /// The name of the link of orientation `k` and its tolerance.
    pub(crate) fn named(&self, k: usize) -> (&str, f64) {
        let held = &self.held[k];
        (held.link_name, held.tolerance())
    }

    /// The angle a goal of orientation `k` stands within.
    pub(crate) fn allowed(&self, k: usize) -> f64 {
        self.held[k].allowed
    }

    /// The links whose orientations the path holds, as a message names them.
    pub(crate) fn links(&self) -> String {
        let links: Vec<&str> = self.held.iter().map(|held| held.link_name).collect();
        links.join(", ")
    }

    /// What a message adds to the clearances a path keeps: the links whose orientations it
    /// holds, or nothing.
    pub(crate) fn kept(&self) -> String {
        if self.held.is_empty() {
            return String::new();
        }
        format!(" and the path orientations of {}", self.links())
    }

    /// The projection of a plan that holds orientations, as the module says.
    pub(crate) fn projection(&self) -> Option<Projection<'_>> {
        if self.held.is_empty() {
            return None;
        }
        let margin = self
            .held
            .iter()
            .map(|held| held.tolerance() - held.allowed)
            .fold(f64::INFINITY, f64::min);
        Some(Projection {
            constraint: self,
            tolerance: (margin * PROJECTION_SHARE).min(PROJECTION_TOLERANCE),
            iterations: PROJECTION_ITERATIONS,
            spacing: self
                .held
                .iter()
                .map(Held::spacing)
                .fold(CONSTRAINED_SPACING, f64::min),
        })
    }
}

impl<const N: usize, const D: usize> Constraint for PathOrientations<'_, N, D> {
    fn codimension(&self) -> usize {
        self.held.len()
    }

    fn function(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        let q: [f64; D] = state.try_into()?;
        for (value, held) in out.iter_mut().zip(&self.held) {
            let error = held.error(&q);
            *value = if error <= held.allowed {
                0.0
            } else {
                error - held.allowed
            };
        }
        Ok(())
    }

    fn jacobian(&self, state: &[f64], out: &mut [f64]) -> Result<(), CheckError> {
        let q: [f64; D] = state.try_into()?;
        out.fill(0.0);
        for (row, held) in out.chunks_exact_mut(D).zip(&self.held) {
            held.write_row(&q, row);
        }
        Ok(())
    }
}

/// The joints of `chain` that turn `frame`: the revolute joints that move it.
fn turning_joints<const N: usize>(chain: &Chain<N>, frame: Frame) -> [bool; N] {
    let moving = ChainFrame::from(frame).moving_joints(chain);
    let posed = chain.at(&[0.0; N]);
    std::array::from_fn(|j| moving[j] && matches!(posed.kind(j), JointKind::Revolute { .. }))
}

/// The largest error a state of a plan's walk may hold of a tolerance.
fn largest_of(tolerance: f64) -> f64 {
    tolerance * (1.0 - FLOOR_SHARE)
}

#[cfg(test)]
mod tests {
    use hegel::TestCase;
    use hegel::generators as gs;
    use nalgebra::{UnitQuaternion, Vector3};

    use super::*;
    use crate::goal::Axis;
    use crate::goal::tests::left_arm;

    const JOINTS: usize = 7;
    /// The step of the central differences of an error, in radians.
    const DIFFERENCE_STEP_RAD: f64 = 1e-6;
    /// How far a row may lie from the central differences of the error: their rounding.
    const DIFFERENCE_BOUND: f64 = 1e-6;
    /// The arithmetic's rounding in the bounds on a motion's turn.
    const ROUNDING: f64 = 1e-9;

    fn drawn_joints(tc: &TestCase, chain: &Chain<JOINTS>) -> [f64; JOINTS] {
        let limits = chain.limits();
        std::array::from_fn(|i| {
            tc.draw(gs::floats().min_value(limits[i].lo).max_value(limits[i].hi))
        })
    }

    fn drawn_rotation(tc: &TestCase) -> UnitQuaternion<f64> {
        let part = || tc.draw(gs::floats().min_value(-1.5_f64).max_value(1.5));
        UnitQuaternion::from_scaled_axis(Vector3::new(part(), part(), part()))
    }

    fn drawn_free_axis(tc: &TestCase) -> Option<Axis> {
        match tc.draw(gs::integers::<u8>().min_value(0).max_value(3)) {
            0 => None,
            1 => Some(Axis::X),
            2 => Some(Axis::Y),
            _ => Some(Axis::Z),
        }
    }

    fn target_of(
        frame: Frame,
        orientation: UnitQuaternion<f64>,
        tolerance: f64,
        free_axis: Option<Axis>,
    ) -> OrientationTarget {
        OrientationTarget {
            frame,
            orientation,
            tolerance_rad: PositiveFinite::parse(tolerance).expect("a tolerance above 0"),
            free_axis,
        }
    }

    fn held_of(chain: &Chain<JOINTS>, target: OrientationTarget, allowed: f64) -> Held<'_, JOINTS> {
        Held {
            chain,
            arm: 0,
            link_name: "link",
            target,
            turning: turning_joints(chain, target.frame),
            allowed,
        }
    }

    // The row of the function's Jacobian is the error's rate of change in each joint: the
    // central differences of the error, on drawn configurations and targets, at the grasp
    // point and at a link the later joints do not move.
    #[hegel::test(test_cases = 200)]
    fn a_row_is_the_rate_of_the_error_in_each_joint(tc: TestCase) {
        let chain = left_arm();
        let q = drawn_joints(&tc, &chain);
        let frame = if tc.draw(gs::booleans()) {
            Frame::GraspPoint
        } else {
            Frame::Link {
                segment: 3,
                offset: nalgebra::Isometry3::identity(),
            }
        };
        let target = target_of(frame, drawn_rotation(&tc), 1e-3, drawn_free_axis(&tc));
        let held = held_of(&chain, target, 0.0);
        let error = held.error(&q);
        tc.assume(error > 0.01 && error < std::f64::consts::PI - 0.01);
        let mut row = [0.0; JOINTS];
        held.write_row(&q, &mut row);
        for j in 0..JOINTS {
            let at = |by: f64| {
                let mut moved = q;
                moved[j] += by;
                held.error(&moved)
            };
            let difference =
                (at(DIFFERENCE_STEP_RAD) - at(-DIFFERENCE_STEP_RAD)) / (2.0 * DIFFERENCE_STEP_RAD);
            assert!(
                (row[j] - difference).abs() <= DIFFERENCE_BOUND,
                "joint {j}: the row holds {}, and the error changes at {difference}",
                row[j]
            );
        }
    }

    // On drawn straight motions, the error changes no faster than the sum of the changes of
    // the joints that turn the frame, the bound each step of a walk rests on.
    #[hegel::test(test_cases = 200)]
    fn a_frame_turns_at_most_the_sum_of_the_changes_of_its_joints(tc: TestCase) {
        let chain = left_arm();
        let (from, end) = (drawn_joints(&tc, &chain), drawn_joints(&tc, &chain));
        let share: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
        let to: [f64; JOINTS] = std::array::from_fn(|i| from[i] + (end[i] - from[i]) * share);
        let frame = Frame::Link {
            segment: tc.draw(gs::integers::<usize>().min_value(0).max_value(JOINTS - 1)),
            offset: nalgebra::Isometry3::identity(),
        };
        let target = target_of(frame, drawn_rotation(&tc), 0.5, drawn_free_axis(&tc));
        let held = held_of(&chain, target, 0.45);
        let motion: [f64; JOINTS] = std::array::from_fn(|i| (to[i] - from[i]).abs());
        assert!(
            (held.error(&to) - held.error(&from)).abs() <= held.turn_rate(&motion) + ROUNDING,
            "the error moves from {} to {}, past the bound {}",
            held.error(&from),
            held.error(&to),
            held.turn_rate(&motion)
        );
    }

    // On drawn straight motions, the frame stands off the shortest rotation between the
    // motion's ends by at most a sixteenth of the square of the sum of the joints' changes,
    // the bound the spacing of a plan's states rests on.
    #[hegel::test(test_cases = 200)]
    fn a_straight_motion_leaves_the_shortest_rotation_by_the_bound(tc: TestCase) {
        let chain = left_arm();
        let from = drawn_joints(&tc, &chain);
        let direction: [f64; JOINTS] =
            std::array::from_fn(|_| tc.draw(gs::floats().min_value(-1.0_f64).max_value(1.0)));
        let length: f64 = tc.draw(gs::floats().min_value(0.01).max_value(0.5));
        let sum: f64 = direction.iter().map(|d| d.abs()).sum();
        tc.assume(sum > 1e-3);
        let along: [f64; JOINTS] = direction.map(|d| d * length / sum);
        let rotation = |t: f64| {
            let q: [f64; JOINTS] = std::array::from_fn(|i| from[i] + along[i] * t);
            frame_pose_at(&chain.at(&q), Frame::GraspPoint).rotation
        };
        let (first, last) = (rotation(0.0), rotation(1.0));
        let bound = length * length / 16.0;
        for k in 1..16 {
            let t = f64::from(k) / 16.0;
            let on_the_line = first.slerp(&last, t);
            let off = on_the_line.angle_to(&rotation(t));
            assert!(
                off <= bound + ROUNDING,
                "at {t} the frame stands {off} rad off the shortest rotation, past {bound}"
            );
        }
    }

    // A run strays past the larger error of its ends by at most the bound the spacing rests on:
    // a sixteenth of the square of the sum of the joints' changes for the whole rotation, twice
    // that for a free axis, whose shortest rotation carries it on a small circle.
    #[hegel::test(test_cases = 200)]
    fn a_run_strays_past_the_error_of_its_ends_by_the_bound(tc: TestCase) {
        let chain = left_arm();
        let from = drawn_joints(&tc, &chain);
        let direction: [f64; JOINTS] =
            std::array::from_fn(|_| tc.draw(gs::floats().min_value(-1.0_f64).max_value(1.0)));
        let length: f64 = tc.draw(gs::floats().min_value(0.01).max_value(0.5));
        let sum: f64 = direction.iter().map(|d| d.abs()).sum();
        tc.assume(sum > 1e-3);
        let at = |t: f64| -> [f64; JOINTS] {
            std::array::from_fn(|i| from[i] + direction[i] * length / sum * t)
        };
        let free_axis = drawn_free_axis(&tc);
        let offset = UnitQuaternion::from_scaled_axis(Vector3::new(
            tc.draw(gs::floats().min_value(-0.5_f64).max_value(0.5)),
            tc.draw(gs::floats().min_value(-0.5_f64).max_value(0.5)),
            tc.draw(gs::floats().min_value(-0.5_f64).max_value(0.5)),
        ));
        let orientation = frame_pose_at(&chain.at(&from), Frame::GraspPoint).rotation * offset;
        let held = held_of(
            &chain,
            target_of(Frame::GraspPoint, orientation, 1.0, free_axis),
            0.9,
        );
        let ends = held.error(&at(0.0)).max(held.error(&at(1.0)));
        tc.assume(ends < std::f64::consts::FRAC_PI_2);
        let bound = length * length / if free_axis.is_some() { 8.0 } else { 16.0 };
        for k in 1..16 {
            let t = f64::from(k) / 16.0;
            let error = held.error(&at(t));
            assert!(
                error <= ends + bound + ROUNDING,
                "at {t} the error is {error}, past the ends' {ends} and the bound {bound}"
            );
        }
    }

    /// The configuration of the left arm with its last joint at `last`, its others at 0, and
    /// the checks of the grasp point's orientation at `last` 0 held within `tolerance`, the
    /// error `last` itself up to a half turn.
    fn on_the_last_joint(
        chain: &Chain<JOINTS>,
        tolerance: f64,
    ) -> PathOrientations<'_, JOINTS, JOINTS> {
        let orientation = frame_pose_at(&chain.at(&[0.0; JOINTS]), Frame::GraspPoint).rotation;
        let target = target_of(Frame::GraspPoint, orientation, tolerance, None);
        PathOrientations {
            held: vec![held_of(chain, target, 0.9 * tolerance)],
        }
    }

    fn last_joint_at(last: f64) -> [f64; JOINTS] {
        std::array::from_fn(|j| if j == JOINTS - 1 { last } else { 0.0 })
    }

    // A walk finds a peak between two ends inside the tolerance. The last joint turns the frame
    // exactly as fast as the bound says, up over a half turn and down, so the walk's step lands
    // where the frame stands as far from the tolerance as the error it left. A step twice as
    // long lands past the peak, back inside the tolerance, and finds nothing.
    #[test]
    fn a_walk_finds_a_peak_between_ends_inside_the_tolerance() {
        const FROM: f64 = 2.95;
        const TO: f64 = 3.3;
        let chain = left_arm();
        let walked = |tolerance: f64| {
            let held = on_the_last_joint(&chain, tolerance);
            let mut motion = [0.0; JOINTS];
            motion[JOINTS - 1] = TO - FROM;
            held.walk(0.0, 1.0, &motion, |t| last_joint_at(FROM + (TO - FROM) * t))
        };
        let (position, turned) = walked(3.1).expect_err("the peak stands past the tolerance");
        assert!(turned.error > largest_of(3.1), "{}", turned.error);
        assert!(
            FROM + (TO - FROM) * position < std::f64::consts::PI,
            "{position}"
        );
        assert!(
            walked(3.2).is_ok(),
            "a half turn lies inside a tolerance of 3.2 rad"
        );
    }

    // A run's first state and its last are walked, and a walk's first state past the floor
    // ends it at that state, the start of the run among them.
    #[test]
    fn a_walk_judges_the_ends_of_a_run() {
        let chain = left_arm();
        let held = on_the_last_joint(&chain, 0.1);
        let mut motion = [0.0; JOINTS];
        motion[JOINTS - 1] = 0.1;
        let inside = held.walk(0.0, 1.0, &motion, |t| last_joint_at(0.05 * t));
        assert!(inside.is_ok());
        let past_at_start = held.walk(0.0, 1.0, &motion, |t| last_joint_at(0.2 - 0.05 * t));
        assert_eq!(past_at_start.map_err(|(position, _)| position), Err(0.0));
        let past_at_end = held.walk(0.0, 1.0, &motion, |t| last_joint_at(0.2 * t));
        assert!(past_at_end.is_err());
    }

    // The function holds, for each orientation in its order, the error above the allowed
    // angle, 0 inside it, and each row is 0 inside it; a NaN state reads as NaN, off the
    // constraint.
    #[test]
    fn the_function_is_the_error_above_the_allowed_angle() {
        let chain = left_arm();
        let orientation = frame_pose_at(&chain.at(&[0.0; JOINTS]), Frame::GraspPoint).rotation;
        let wide = held_of(
            &chain,
            target_of(Frame::GraspPoint, orientation, 1.0, None),
            0.9,
        );
        let tight = held_of(
            &chain,
            target_of(Frame::GraspPoint, orientation, 0.1, None),
            0.09,
        );
        let both: PathOrientations<JOINTS, JOINTS> = PathOrientations {
            held: vec![wide, tight],
        };
        let q = last_joint_at(0.5);
        let mut values = [f64::NAN; 2];
        both.function(&q, &mut values).expect("a state");
        assert_eq!(values[0], 0.0);
        assert!(
            (values[1] - (0.5 - 0.09)).abs() <= ROUNDING,
            "{}",
            values[1]
        );
        let mut rows = [f64::NAN; 2 * JOINTS];
        both.jacobian(&q, &mut rows).expect("a state");
        assert_eq!(rows[..JOINTS], [0.0; JOINTS]);
        assert!(rows[JOINTS..].iter().any(|value| *value != 0.0));
        let mut unwritten = [0.0; 2];
        both.function(&[f64::NAN; JOINTS], &mut unwritten)
            .expect("a state");
        assert!(
            unwritten.iter().all(|value| value.is_nan()),
            "{unwritten:?}"
        );
    }

    // A plan takes the spacing and the projection's tolerance of its narrowest orientation, up
    // to OMPL's own: the margin of a tolerance of 0.1 rad on seven joints gives OMPL's spacing,
    // and that of 0.02 rad less than half of it.
    #[test]
    fn a_plan_spaces_its_states_by_its_narrowest_orientation() {
        let chain = left_arm();
        let within = |tolerance: f64| {
            let target = target_of(
                Frame::GraspPoint,
                UnitQuaternion::identity(),
                tolerance,
                None,
            );
            held_of(&chain, target, HELD_SHARE * tolerance)
        };
        let of = |tolerances: &[f64]| -> (f64, f64) {
            let orientations: PathOrientations<JOINTS, JOINTS> = PathOrientations {
                held: tolerances
                    .iter()
                    .map(|tolerance| within(*tolerance))
                    .collect(),
            };
            let projection = orientations.projection().expect("a projection");
            (projection.spacing, projection.tolerance)
        };
        assert_eq!(of(&[0.1]), (CONSTRAINED_SPACING, PROJECTION_TOLERANCE));
        let narrow = within(0.02).spacing();
        assert!(narrow < CONSTRAINED_SPACING / 2.0, "{narrow}");
        assert_eq!(of(&[0.1, 0.02]).0, narrow);
        assert_eq!(of(&[0.02, 0.1]).0, narrow);
        let tiny = 1e-3;
        let (_, tolerance) = of(&[0.1, tiny]);
        assert!(
            (tolerance - PROJECTION_SHARE * (1.0 - HELD_SHARE) * tiny).abs() <= ROUNDING,
            "{tolerance}"
        );
    }

    // A frame no joint turns leaves the spacing to the others, and a tolerance's margin sets
    // it for a frame the joints turn.
    #[test]
    fn the_spacing_follows_the_margin_and_the_joints_that_turn_the_frame() {
        let chain = left_arm();
        let target = target_of(Frame::GraspPoint, UnitQuaternion::identity(), 0.05, None);
        let held = held_of(&chain, target, 0.045);
        assert!((held.spacing() - (8.0_f64 * 0.005 / 7.0).sqrt()).abs() <= ROUNDING);
        let none = Held {
            turning: [false; JOINTS],
            ..held_of(&chain, target, 0.045)
        };
        assert_eq!(none.spacing(), f64::INFINITY);
    }
}
