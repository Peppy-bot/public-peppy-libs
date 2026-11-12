//! The rule of whether an arm of a robot reaches a target, which every answer of where a robot can
//! work follows: a robot's own answer from its design and a simulation's from its world. Each side
//! brings its arms toward a target with its own inverse kinematics ([`ArmSolver`]) and says what a
//! pose of an arm must stay clear of ([`Clearance`]); [`reach`] judges the tries.

use crate::{GraspDirection, GraspOrientation, Limits, Reach, angle_between};

/// The pose a solver brings an arm to: the position of each joint of its chain, in chain order,
/// and where its grasp point stands and the way its approach axis points there, in the robot
/// frame.
#[derive(Debug)]
pub struct Attempt {
    pub joints: Vec<f64>,
    pub grasp_point: [f64; 3],
    pub approach: [f64; 3],
}

/// What a solver meets of a grasp orientation.
#[derive(Clone, Copy, Debug)]
pub enum Solves {
    /// The full pose: the approach direction and the gripper's roll about it. Its arms try each
    /// orientation [`GraspOrientation::all`] gives.
    FullPoses,
    /// The approach direction alone, the gripper's roll whatever the arm takes there, as on an arm
    /// of too few joints to set it. Its arms try each [`GraspDirection`] once.
    Approaches,
}

impl Solves {
    /// The orientations an arm of this solver tries, in order.
    fn orientations(self) -> Vec<GraspOrientation> {
        match self {
            Self::FullPoses => GraspOrientation::all().collect(),
            Self::Approaches => GraspDirection::ALL
                .into_iter()
                .map(|direction| GraspOrientation {
                    direction,
                    roll: 0.0,
                })
                .collect(),
        }
    }
}

/// What brings the arms of one robot toward a target, in the robot frame.
pub trait ArmSolver {
    /// What the solver meets of a grasp orientation.
    fn solves(&self) -> Solves;

    /// The name of each arm, by the name the robot's limbs take, in the robot's order of its arms.
    fn arm_names(&self) -> Vec<String>;

    /// Where the base of arm `arm` stands.
    fn arm_base(&self, arm: usize) -> [f64; 3];

    /// How far short of `target` the solver holds that the grasp point of arm `arm` stops in every
    /// orientation: 0 where it cannot rule the target out. An arm whose bound passes the reach
    /// tolerance tries no orientation.
    fn reach_bound(&mut self, arm: usize, target: [f64; 3]) -> f64;

    /// How far short of `target` the grasp point of arm `arm` stops, in any orientation, as near
    /// as the solver finds: what the answer of a target no arm reaches reads.
    fn position_shortfall(&mut self, arm: usize, target: [f64; 3]) -> f64;

    /// The pose the solver brings arm `arm` to for its grasp frame at `target`, turned as
    /// `rotation`, a row-major rotation whose columns are the grasp frame's axes
    /// ([`GraspOrientation::rotation`]); none when it finds no joints for it.
    fn reach_pose(&mut self, arm: usize, target: [f64; 3], rotation: [f64; 9]) -> Option<Attempt>;

    /// The orientation arm `arm` tries first, by its index among the orientations of what the
    /// solver [`Solves`], the others following in order; none, or an index past them, tries them
    /// all in order. The order changes how long a target takes, never its answer.
    fn first_orientation(&self, _arm: usize) -> Option<usize> {
        None
    }

    /// A pose of arm `arm` in the orientation of index `orientation` came within the limits of a
    /// target, clear or not.
    fn came_within(&mut self, _arm: usize, _orientation: usize) {}
}

/// Whether arm `arm` at `joints`, its chain's joint positions in chain order, stands clear of what
/// it must not pass through.
pub trait Clearance {
    fn clear(&mut self, arm: usize, joints: &[f64]) -> bool;
}

/// The clearance of a world with nothing in it: every pose is clear.
pub struct Unobstructed;

impl Clearance for Unobstructed {
    fn clear(&mut self, _arm: usize, _joints: &[f64]) -> bool {
        true
    }
}

/// Whether an arm of the robot `solver` solves for reaches `target`, in a pose `clearance` finds
/// clear, within `limits`. The arms are tried nearest first, by where their bases stand, an arm
/// whose base stands as near as another's after it. An arm whose reach bound passes the limits'
/// reach tolerance tries no orientation. Else it tries the orientations of what the solver
/// [`Solves`], its first orientation first ([`ArmSolver::first_orientation`]). A pose comes within
/// the limits when its grasp point stands within the reach tolerance of the target and its
/// approach axis turns within the grasp angle tolerance of the orientation's grasp direction, and
/// reaches the target when `clearance` also finds it clear: the arm of the first such pose reaches
/// it. When no pose does, the target falls short by the least of each arm's position shortfall
/// and each pose's distance from it, which is within the tolerance when an arm comes that near
/// but points no grasp direction there, or only in poses that are not clear.
pub fn reach(
    solver: &mut impl ArmSolver,
    clearance: &mut impl Clearance,
    target: [f64; 3],
    limits: &Limits,
) -> Reach {
    let names = solver.arm_names();
    let orientations = solver.solves().orientations();
    let mut nearest = f64::INFINITY;
    let arms = arms_nearest_first(solver, names.len(), target);
    for &arm in &arms {
        if solver.reach_bound(arm, target) > limits.reach_tolerance {
            continue;
        }
        for index in first_then_rest(solver.first_orientation(arm), orientations.len()) {
            let orientation = &orientations[index];
            let Some(attempt) = solver.reach_pose(arm, target, orientation.rotation()) else {
                continue;
            };
            let short = distance(attempt.grasp_point, target);
            nearest = nearest.min(short);
            let points_its_way = angle_between(attempt.approach, orientation.direction.approach())
                <= limits.grasp_angle_tolerance;
            if short > limits.reach_tolerance || !points_its_way {
                continue;
            }
            solver.came_within(arm, index);
            if clearance.clear(arm, &attempt.joints) {
                return Reach::Reached {
                    arm: names[arm].clone(),
                };
            }
        }
    }
    let shortfall = arms
        .iter()
        .map(|&arm| solver.position_shortfall(arm, target))
        .fold(nearest, f64::min);
    Reach::Short { by: shortfall }
}

/// The arms of `solver`, `count` of them, the one whose base stands nearest `target` first; in the
/// robot's order when they stand as near.
fn arms_nearest_first(solver: &impl ArmSolver, count: usize, target: [f64; 3]) -> Vec<usize> {
    let mut arms: Vec<usize> = (0..count).collect();
    arms.sort_by(|&a, &b| {
        distance(solver.arm_base(a), target).total_cmp(&distance(solver.arm_base(b), target))
    });
    arms
}

/// The indexes of `count` orientations, `first` first when it is one of them, then the others in
/// order.
fn first_then_rest(first: Option<usize>, count: usize) -> impl Iterator<Item = usize> {
    let first = first.filter(|&index| index < count);
    first
        .into_iter()
        .chain((0..count).filter(move |&index| Some(index) != first))
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::REACH_TOLERANCE;

    /// [`reach`] within [`Limits::STANDARD`].
    fn reach_standard(
        solver: &mut impl ArmSolver,
        clearance: &mut impl Clearance,
        target: [f64; 3],
    ) -> Reach {
        reach(solver, clearance, target, &Limits::STANDARD)
    }

    /// A solver of two arms whose tries are scripted: each arm's reach bound and position
    /// shortfall are `position`, and its pose in an orientation stops where `pose` says for that
    /// orientation's index in [`GraspOrientation::all`], its approach the orientation's direction
    /// unless `turned` names the index; it finds no joints for the index `no_joints` names. It
    /// records each try.
    struct Scripted {
        solves: Solves,
        bases: [[f64; 3]; 2],
        position: [f64; 2],
        pose: [fn(usize) -> f64; 2],
        turned: Option<usize>,
        no_joints: Option<usize>,
        first: Option<usize>,
        tries: Vec<(usize, Option<usize>)>,
        shortfalls: Vec<usize>,
        came: Vec<(usize, usize)>,
    }

    fn index_of(rotation: [f64; 9]) -> usize {
        GraspOrientation::all()
            .position(|o| o.rotation() == rotation)
            .expect("a grasp orientation")
    }

    fn first_forward() -> usize {
        GraspOrientation::all()
            .position(|o| o.direction == GraspDirection::Forward)
            .expect("a forward orientation")
    }

    fn at(target: [f64; 3], off: f64) -> [f64; 3] {
        [target[0] + off, target[1], target[2]]
    }

    impl ArmSolver for Scripted {
        fn solves(&self) -> Solves {
            self.solves
        }
        fn arm_names(&self) -> Vec<String> {
            vec!["left_arm".into(), "right_arm".into()]
        }
        fn arm_base(&self, arm: usize) -> [f64; 3] {
            self.bases[arm]
        }
        fn reach_bound(&mut self, arm: usize, _target: [f64; 3]) -> f64 {
            self.tries.push((arm, None));
            self.position[arm]
        }
        fn position_shortfall(&mut self, arm: usize, _target: [f64; 3]) -> f64 {
            self.shortfalls.push(arm);
            self.position[arm]
        }
        fn reach_pose(
            &mut self,
            arm: usize,
            target: [f64; 3],
            rotation: [f64; 9],
        ) -> Option<Attempt> {
            let index = index_of(rotation);
            self.tries.push((arm, Some(index)));
            if self.no_joints == Some(index) {
                return None;
            }
            let direction = GraspOrientation::all().nth(index).unwrap().direction;
            let approach = if self.turned == Some(index) {
                [1.0, 0.0, 0.0]
            } else {
                direction.approach()
            };
            Some(Attempt {
                joints: vec![arm as f64, index as f64],
                grasp_point: at(target, self.pose[arm](index)),
                approach,
            })
        }
        fn first_orientation(&self, _arm: usize) -> Option<usize> {
            self.first
        }
        fn came_within(&mut self, arm: usize, orientation: usize) {
            self.came.push((arm, orientation));
        }
    }

    const TARGET: [f64; 3] = [0.4, 0.1, 0.8];
    const LEFT_NEARER: [[f64; 3]; 2] = [[0.0, 0.15, 1.0], [0.0, -0.15, 1.0]];

    fn exact(_: usize) -> f64 {
        0.0
    }
    fn far(_: usize) -> f64 {
        0.2
    }

    fn solver(position: [f64; 2], pose: [fn(usize) -> f64; 2]) -> Scripted {
        Scripted {
            solves: Solves::FullPoses,
            bases: LEFT_NEARER,
            position,
            pose,
            turned: None,
            no_joints: None,
            first: None,
            tries: Vec::new(),
            shortfalls: Vec::new(),
            came: Vec::new(),
        }
    }

    #[test]
    fn the_arm_whose_base_stands_nearest_is_tried_first_and_named() {
        let mut both = solver([0.0, 0.0], [exact, exact]);
        assert_eq!(
            reach_standard(&mut both, &mut Unobstructed, TARGET),
            Reach::Reached {
                arm: "left_arm".into()
            }
        );
        assert_eq!(both.tries, vec![(0, None), (0, Some(0))]);
        let mut swapped = solver([0.0, 0.0], [exact, exact]);
        swapped.bases = [LEFT_NEARER[1], LEFT_NEARER[0]];
        assert_eq!(
            reach_standard(&mut swapped, &mut Unobstructed, TARGET).arm(),
            "right_arm"
        );
    }

    #[test]
    fn the_position_shortfall_is_read_only_for_a_target_no_pose_reaches() {
        let mut reached = solver([0.0, 0.0], [exact, exact]);
        assert!(reach_standard(&mut reached, &mut Unobstructed, TARGET).reached());
        assert!(reached.shortfalls.is_empty());
        let mut short = solver([0.0, 0.0], [far, far]);
        assert!(!reach_standard(&mut short, &mut Unobstructed, TARGET).reached());
        assert_eq!(short.shortfalls, vec![0, 1]);
    }

    #[test]
    fn arms_as_near_are_tried_in_the_robots_order() {
        let mut level = solver([0.0, 0.0], [exact, exact]);
        level.bases = [[0.0, 0.1, 1.0], [0.0, 0.1, 1.0]];
        assert_eq!(
            reach_standard(&mut level, &mut Unobstructed, TARGET).arm(),
            "left_arm"
        );
    }

    #[test]
    fn an_arm_whose_position_try_stops_beyond_the_tolerance_tries_no_orientation() {
        let mut solver = solver([REACH_TOLERANCE + 0.001, 0.0], [exact, exact]);
        assert_eq!(
            reach_standard(&mut solver, &mut Unobstructed, TARGET).arm(),
            "right_arm"
        );
        assert_eq!(solver.tries, vec![(0, None), (1, None), (1, Some(0))]);
    }

    #[test]
    fn an_orientation_the_solver_finds_no_joints_for_does_not_reach() {
        let mut solver = solver([0.0, 1.0], [exact, exact]);
        solver.no_joints = Some(0);
        assert_eq!(
            reach_standard(&mut solver, &mut Unobstructed, TARGET).arm(),
            "left_arm"
        );
        assert_eq!(solver.tries, vec![(0, None), (0, Some(0)), (0, Some(1))]);
    }

    #[test]
    fn a_pose_whose_approach_turns_past_the_angle_tolerance_does_not_reach() {
        let mut solver = solver([0.0, 1.0], [exact, exact]);
        solver.turned = Some(0);
        assert_eq!(
            reach_standard(&mut solver, &mut Unobstructed, TARGET).arm(),
            "left_arm"
        );
        assert_eq!(solver.tries.last(), Some(&(0, Some(1))));
    }

    #[test]
    fn a_pose_the_clearance_refuses_does_not_reach_and_the_next_clear_one_does() {
        struct RefusesTheFirstTwo(usize);
        impl Clearance for RefusesTheFirstTwo {
            fn clear(&mut self, _arm: usize, _joints: &[f64]) -> bool {
                self.0 += 1;
                self.0 > 2
            }
        }
        let mut solver = solver([0.0, 1.0], [exact, exact]);
        let mut clearance = RefusesTheFirstTwo(0);
        assert_eq!(
            reach_standard(&mut solver, &mut clearance, TARGET).arm(),
            "left_arm"
        );
        assert_eq!(solver.tries.last(), Some(&(0, Some(2))));
    }

    #[test]
    fn a_target_reached_only_in_poses_that_are_not_clear_falls_short_by_the_least_distance() {
        struct Nothing;
        impl Clearance for Nothing {
            fn clear(&mut self, _arm: usize, _joints: &[f64]) -> bool {
                false
            }
        }
        fn near(index: usize) -> f64 {
            0.001 * (index + 1) as f64
        }
        let mut solver = solver([0.004, 0.006], [near, near]);
        let by = reach_standard(&mut solver, &mut Nothing, TARGET).short_by();
        assert!((by - 0.001).abs() < 1e-12, "{by}");
        let count = GraspOrientation::all().count();
        assert_eq!(solver.tries.len(), 2 * (1 + count));
    }

    #[test]
    fn a_target_no_arm_comes_near_falls_short_by_the_least_shortfall_of_its_arms() {
        let mut solver = solver([0.3, 0.25], [far, far]);
        assert_eq!(
            reach_standard(&mut solver, &mut Unobstructed, TARGET),
            Reach::Short { by: 0.25 }
        );
        assert_eq!(solver.tries, vec![(0, None), (1, None)]);
    }

    #[test]
    fn the_orientations_are_tried_down_first_then_forward() {
        fn only_forward(index: usize) -> f64 {
            let direction = GraspOrientation::all().nth(index).unwrap().direction;
            if direction == GraspDirection::Forward {
                0.0
            } else {
                0.05
            }
        }
        let mut solver = solver([0.0, 1.0], [only_forward, far]);
        assert!(reach_standard(&mut solver, &mut Unobstructed, TARGET).reached());
        assert_eq!(solver.tries.last(), Some(&(0, Some(first_forward()))));
    }

    #[test]
    fn a_solver_of_approaches_tries_each_direction_once_at_no_roll() {
        fn only_forward(index: usize) -> f64 {
            if index == first_forward() { 0.0 } else { 0.05 }
        }
        let mut solver = solver([0.0, 1.0], [only_forward, far]);
        solver.solves = Solves::Approaches;
        assert!(reach_standard(&mut solver, &mut Unobstructed, TARGET).reached());
        assert_eq!(
            solver.tries,
            vec![(0, None), (0, Some(0)), (0, Some(first_forward()))]
        );
    }

    #[test]
    fn an_arm_tries_its_first_orientation_first_then_the_others_in_order() {
        fn only_forward(index: usize) -> f64 {
            if index == first_forward() { 0.0 } else { 0.05 }
        }
        let mut hinted = solver([0.0, 1.0], [only_forward, far]);
        hinted.first = Some(first_forward());
        assert!(reach_standard(&mut hinted, &mut Unobstructed, TARGET).reached());
        assert_eq!(hinted.tries, vec![(0, None), (0, Some(first_forward()))]);
        let mut short = solver([0.0, 1.0], [far, far]);
        short.first = Some(3);
        reach_standard(&mut short, &mut Unobstructed, TARGET);
        let left: Vec<usize> = short
            .tries
            .iter()
            .filter_map(|&(arm, index)| (arm == 0).then_some(index).flatten())
            .collect();
        let count = GraspOrientation::all().count();
        let rest = (0..count).filter(|&index| index != 3);
        assert_eq!(left, std::iter::once(3).chain(rest).collect::<Vec<_>>());
    }

    #[test]
    fn a_first_orientation_past_those_of_the_solver_tries_them_in_order() {
        let mut solver = solver([0.0, 1.0], [far, far]);
        solver.solves = Solves::Approaches;
        solver.first = Some(2);
        reach_standard(&mut solver, &mut Unobstructed, TARGET);
        assert_eq!(
            solver.tries[..3],
            [(0, None), (0, Some(0)), (0, Some(first_forward()))]
        );
    }

    #[test]
    fn the_solver_hears_of_each_pose_that_comes_within_the_limits_clear_or_not() {
        struct RefusesTheFirst(bool);
        impl Clearance for RefusesTheFirst {
            fn clear(&mut self, _arm: usize, _joints: &[f64]) -> bool {
                std::mem::replace(&mut self.0, true)
            }
        }
        let mut solver = solver([0.0, 1.0], [exact, exact]);
        solver.turned = Some(0);
        assert!(reach_standard(&mut solver, &mut RefusesTheFirst(false), TARGET).reached());
        assert_eq!(solver.came, vec![(0, 1), (0, 2)]);
    }
}
