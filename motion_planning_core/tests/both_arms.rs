//! Plans of two arms together: two planar arms of two revolute joints face each other
//! across 0.9 m, their links in one plane, so neither passes the other while it points at
//! it. The plan samples the arms' goals together, each arm against the joints the arms
//! before it drew, from a start nearer than the clearance each pair on the turn of its
//! later arm, names an arm whose sampler finds none, and answers a path that keeps the
//! arms apart along the rounded path the robot runs, from a start nearer than the
//! clearance as the start rule says. OpenArm v2's two arms hold a goal and a path
//! orientation of the right hand in the left hand's frame against the left hand where each
//! state puts it, as MoveIt holds a constraint in a link of the robot, and a goal of the left
//! hand in the right hand's frame, the right arm sampled first.

mod common;
mod openarm_v2;

use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::sync::atomic::{AtomicUsize, Ordering};

use chain_kinematics::{ChainSpec, Draws, JointSelection};
use collision_geometry::chain::ChainGeometry;
use collision_geometry::gjk;
use control_core::path_timing::RoundedPath;
use hegel::TestCase;
use hegel::generators as gs;
use motion_planning_core::{
    Arm, ArmGoal, Arms, Body, Frame, Goal, GoalSampler, Holder, IkGoalSampler, JointTarget, Moving,
    Objects, OrientationTarget, PathOrientation, PlacedGoal, PlanFailure, PlanRequest,
    PositionTarget, Reference, Scene, SelfModel, plan_path,
};
use nalgebra::{Isometry3, Point3};
use occupancy_map::{DistanceField, VoxelMap};
use ompl_bridge::Planner;

use common::{LEFT_ARM, RIGHT_ARM, an_hour_from_now, left_alone, map, plan_arm, positive};
use openarm_v2::{JOINTS, LEFT, RIGHT};

/// The clearance the plans keep between the arms (m).
const MIN_CLEARANCE_M: f64 = 0.02;

/// How far a point of a link moves at most between two probes of a rounded path: a
/// millimeter.
const PROBE_M: f64 = 1e-3;

/// The most goals the plans keep.
const GOAL_SAMPLES: usize = 8;

/// A planar arm of two revolute joints about z, its first at `x` along x turned by `yaw`
/// about z, links of 30 cm and 25 cm, its names prefixed by `prefix`.
fn planar_arm(prefix: &str, x: f64, yaw: f64) -> ChainGeometry<2> {
    let urdf = format!(
        r#"<robot name="{prefix}arm">
            <link name="base"/>
            <link name="{prefix}link_1">
              <collision><origin xyz="0.15 0 0"/><geometry><box size="0.3 0.04 0.04"/></geometry></collision>
            </link>
            <link name="{prefix}link_2">
              <collision><origin xyz="0.125 0 0"/><geometry><box size="0.25 0.04 0.04"/></geometry></collision>
            </link>
            <joint name="{prefix}joint_1" type="revolute">
              <origin xyz="{x} 0 0" rpy="0 0 {yaw}"/>
              <parent link="base"/><child link="{prefix}link_1"/><axis xyz="0 0 1"/>
              <limit lower="-3" upper="3" effort="1" velocity="1"/>
            </joint>
            <joint name="{prefix}joint_2" type="revolute">
              <origin xyz="0.3 0 0"/>
              <parent link="{prefix}link_1"/><child link="{prefix}link_2"/><axis xyz="0 0 1"/>
              <limit lower="-2.5" upper="2.5" effort="1" velocity="1"/>
            </joint>
          </robot>"#
    );
    ChainGeometry::from_urdf(
        &urdf,
        "",
        &ChainSpec {
            base_link: None,
            tip_link: &format!("{prefix}link_2"),
            joints: JointSelection::PathOrder,
        },
    )
    .expect("a planar arm")
}

/// The two arms: the left at the origin pointing along x at 0, and the right 0.9 m along
/// x pointing back along -x at 0. A joint turning by a positive angle swings the left arm
/// toward +y and the right toward -y.
struct Facing {
    left: ChainGeometry<2>,
    right: ChainGeometry<2>,
}

impl Facing {
    fn new() -> Self {
        Self {
            left: planar_arm("left_", 0.0, 0.0),
            right: planar_arm("right_", 0.9, std::f64::consts::PI),
        }
    }

    /// The group of both arms, the left first, checked against `model`.
    fn arms<'a>(&'a self, model: &'a PiecesApart) -> Arms<'a, 2, 4> {
        Arms::new(
            vec![
                Arm::new("left_arm", &self.left),
                Arm::new("right_arm", &self.right),
            ],
            Some(model),
        )
    }

    /// The self model of the two arms.
    fn model(&self) -> PiecesApart<'_> {
        PiecesApart(self)
    }
}

/// The two arms' self model: the least distance between a piece of the left arm and a
/// piece of the right one.
struct PiecesApart<'a>(&'a Facing);

impl PiecesApart<'_> {
    /// Each pair of a piece of the left arm and one of the right at `q`, by their links'
    /// names, with its distance.
    fn pairs(&self, q: &[f64; 4]) -> Vec<(String, String, f64)> {
        let left = Arm::new("left_arm", &self.0.left).bodies(&Arms::<2, 4>::slice(q, 0));
        let right = Arm::new("right_arm", &self.0.right).bodies(&Arms::<2, 4>::slice(q, 1));
        left.iter()
            .flat_map(|(a, body_a)| {
                right.iter().map(move |(b, body_b)| {
                    let (Body::Piece(piece_a), Body::Piece(piece_b)) = (body_a, body_b) else {
                        panic!("the arms carry no object");
                    };
                    (
                        a.to_string(),
                        b.to_string(),
                        gjk::distance(piece_a, piece_b).distance,
                    )
                })
            })
            .collect()
    }
}

impl SelfModel<4> for PiecesApart<'_> {
    /// Each pair holds a body of both arms, so a turn of the left arm judges none.
    fn clearance(&self, q: &[f64; 4], moving: Moving, _: f64) -> f64 {
        if !moving.judges(0, 1) {
            return f64::INFINITY;
        }
        self.pairs(q)
            .into_iter()
            .map(|(_, _, distance)| distance)
            .fold(f64::INFINITY, f64::min)
    }

    fn nearest_pair(&self, q: &[f64; 4], _: Moving) -> (String, String) {
        let (a, b, _) = self
            .pairs(q)
            .into_iter()
            .min_by(|x, y| x.2.total_cmp(&y.2))
            .expect("a pair of the arms");
        (a, b)
    }
}

/// A sampler that tries the configurations it holds in turn, whatever the goal, one an
/// attempt, from the first again after the last: a sample answers the first it tries that
/// `valid` accepts. One that holds none finds none.
struct Drawn {
    configurations: Vec<[f64; 2]>,
    next: AtomicUsize,
}

impl Drawn {
    fn new(configurations: Vec<[f64; 2]>) -> Self {
        Self {
            configurations,
            next: AtomicUsize::new(0),
        }
    }
}

impl GoalSampler<2> for Drawn {
    fn sample_from(
        &self,
        _: &PlacedGoal,
        _: &[f64; 2],
        attempts: usize,
        _: &mut Draws,
        valid: &dyn Fn(&[f64; 2]) -> bool,
    ) -> Option<[f64; 2]> {
        if self.configurations.is_empty() {
            return None;
        }
        (0..attempts).find_map(|_| {
            let k = self.next.fetch_add(1, Ordering::Relaxed) % self.configurations.len();
            let q = self.configurations[k];
            valid(&q).then_some(q)
        })
    }
}

/// The goal of both joints within 0.01 rad of `q`.
fn at(q: [f64; 2]) -> Goal {
    let joints = (0..2)
        .map(|joint| JointTarget {
            joint,
            position: q[joint],
            above: positive(0.01),
            below: positive(0.01),
        })
        .collect();
    Goal::new(2, joints, Vec::new(), Vec::new()).expect("a valid goal")
}

/// An empty map about the arms, and its distance field.
fn empty() -> (VoxelMap, DistanceField) {
    let map = map(0.04, 1.6);
    let field = DistanceField::of(&map);
    (map, field)
}

/// The plan of `arms` in an empty map from `start`, each arm to its goal of `goals`, holding
/// `orientations`.
fn plan<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    start: [f64; D],
    goals: Vec<ArmGoal<N>>,
    orientations: &[PathOrientation],
) -> Result<Vec<[f64; D]>, PlanFailure> {
    let (map, field) = empty();
    plan_path(
        arms,
        &Scene {
            map: &map,
            field: &field,
            stamp: 1,
            objects: Objects::none(),
        },
        &PlanRequest {
            start,
            start_tolerance: [0.0; D],
            goals,
            path_orientations: orientations,
            goal_samples: NonZeroUsize::new(GOAL_SAMPLES).expect("nonzero"),
            planner: Planner::RrtConnect,
            refiner: None,
            deadline: an_hour_from_now(),
            max_iterations: NonZeroU64::new(20_000).expect("nonzero"),
            attempts: NonZeroU32::MIN,
            seed: 7,
            min_clearance_m: MIN_CLEARANCE_M,
            stop: None,
        },
    )
}

/// The states of the rounded path through `waypoints` at which no point of a link moves
/// more than `PROBE_M` from one to the next, as `arms` bounds it.
fn probes<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    waypoints: &[[f64; D]],
) -> Vec<[f64; D]> {
    let vectors: Vec<Vec<f64>> = waypoints.iter().map(|q| q.to_vec()).collect();
    let path = RoundedPath::joint_path(&vectors);
    let step = PROBE_M / arms.lever();
    let count = (path.length() / step).ceil() as usize;
    (0..=count)
        .map(|k| {
            path.at((k as f64 * step).min(path.length()))
                .as_slice()
                .try_into()
                .expect("a state of the arms")
        })
        .collect()
}

/// The arms pointing up and away from each other: the left at 1 rad, the right at -1 rad.
const UP: [f64; 4] = [1.0, 0.0, -1.0, 0.0];

/// The arms swapped across the line between their bases: the left at -1 rad, the right at
/// 1 rad. The straight joint motion from `UP` points both arms at each other at once.
const DOWN: [f64; 4] = [-1.0, 0.0, 1.0, 0.0];

// The arms cannot swap sides at once: halfway along the straight joint motion both point
// at each other and their links overlap. The plan answers a path that keeps them apart:
// every probe of its rounded path keeps the clearance, its last state meets each arm's
// goal, and the arms pass their k-th waypoints together, one state holding both.
#[test]
fn arms_that_cannot_swap_at_once_pass_each_other_apart() {
    let facing = Facing::new();
    let model = facing.model();
    let arms = facing.arms(&model);
    assert!(
        model.clearance(&[0.0, 0.0, 0.0, 0.0], Moving::All, f64::INFINITY) < 0.0,
        "the arms pointing at each other overlap"
    );
    let samplers = [
        IkGoalSampler::new(facing.left.chain().clone()),
        IkGoalSampler::new(facing.right.chain().clone()),
    ];
    let goals = [at([DOWN[0], DOWN[1]]), at([DOWN[2], DOWN[3]])];
    let waypoints = plan(
        &arms,
        UP,
        vec![
            ArmGoal {
                goal: &goals[0],
                sampler: &samplers[0],
            },
            ArmGoal {
                goal: &goals[1],
                sampler: &samplers[1],
            },
        ],
        &[],
    )
    .expect("a path that keeps the arms apart");
    assert_eq!(waypoints.first(), Some(&UP));
    let last = waypoints.last().expect("a waypoint");
    assert!(
        (0..4).all(|j| (last[j] - DOWN[j]).abs() <= 0.01 + 1e-12),
        "the path ends at {last:?}"
    );
    for q in probes(&arms, &waypoints) {
        let apart = model.clearance(&q, Moving::All, f64::INFINITY);
        assert!(
            apart >= MIN_CLEARANCE_M,
            "the arms come {apart} m apart at {q:?}"
        );
    }
}

// An arm whose sampler finds no configuration that meets its goal fails the plan with
// unreachable, naming the arm.
#[test]
fn an_arm_whose_sampler_finds_nothing_is_named() {
    let facing = Facing::new();
    let model = facing.model();
    let arms = facing.arms(&model);
    let (goal, found, none) = (
        at([0.0, 0.0]),
        Drawn::new(vec![[-1.0, 0.0]]),
        Drawn::new(Vec::new()),
    );
    let failure = plan(
        &arms,
        UP,
        vec![
            ArmGoal {
                goal: &goal,
                sampler: &found,
            },
            ArmGoal {
                goal: &goal,
                sampler: &none,
            },
        ],
        &[],
    );
    assert_eq!(
        failure,
        Err(PlanFailure::Unreachable(
            "inverse kinematics found no configuration of right_arm that meets the goal in \
             1000 goal samples or by the deadline"
                .to_string()
        ))
    );
}

// A goal sample draws the right arm against the left arm's newly drawn joints. The left arm
// draws pointing at the right arm, apart from it where the right arm stands; the right
// arm's first try, pointing at the left arm, overlaps those joints, and its second keeps
// them apart: the plan answers a path to it.
#[test]
fn a_goal_sample_draws_each_arm_against_the_arms_drawn_before_it() {
    let facing = Facing::new();
    let model = facing.model();
    let arms = facing.arms(&model);
    let goal = at([0.0, 0.0]);
    let (left, right) = (
        Drawn::new(vec![[0.0, 0.0]]),
        Drawn::new(vec![[0.0, 0.0], [1.0, 0.0]]),
    );
    assert!(
        model.clearance(&[0.0, 0.0, UP[2], UP[3]], Moving::All, f64::INFINITY) >= MIN_CLEARANCE_M
    );
    assert!(model.clearance(&[0.0, 0.0, 0.0, 0.0], Moving::All, f64::INFINITY) < MIN_CLEARANCE_M);
    let waypoints = plan(
        &arms,
        UP,
        vec![
            ArmGoal {
                goal: &goal,
                sampler: &left,
            },
            ArmGoal {
                goal: &goal,
                sampler: &right,
            },
        ],
        &[],
    )
    .expect("a path to the right arm's second try");
    let last = waypoints.last().expect("a waypoint");
    assert_eq!(Arms::<2, 4>::slice(last, 1), [1.0, 0.0], "{last:?}");
}

// Each try of the right arm in collision with the left arm's drawn joints, the plan fails
// with unreachable, counting the states the goal sampling rejected and naming the pair of
// the one that falls short least: each of the 1000 goal samples tries the right arm 4
// times.
#[test]
fn a_goal_sampling_that_rejects_every_try_of_an_arm_is_unreachable() {
    let facing = Facing::new();
    let model = facing.model();
    let arms = facing.arms(&model);
    let goal = at([0.0, 0.0]);
    let (left, right) = (Drawn::new(vec![[0.0, 0.0]]), Drawn::new(vec![[0.0, 0.0]]));
    let Err(PlanFailure::Unreachable(reason)) = plan(
        &arms,
        UP,
        vec![
            ArmGoal {
                goal: &goal,
                sampler: &left,
            },
            ArmGoal {
                goal: &goal,
                sampler: &right,
            },
        ],
        &[],
    ) else {
        panic!("every state overlaps");
    };
    assert!(
        reason.starts_with(
            "the goal sampling rejected 4000 states of the arms, each nearer than the plan's \
             clearances, "
        ) && reason.contains("; the one that falls short least has left_link_"),
        "{reason}"
    );
}

/// A state of both arms inside their limits, drawn.
fn drawn_state(tc: &TestCase, arms: &Arms<2, 4>) -> [f64; 4] {
    let limits = arms.limits();
    std::array::from_fn(|i| tc.draw(gs::floats().min_value(limits[i].lo).max_value(limits[i].hi)))
}

// From drawn starts that keep the arms apart, to drawn goals that do, every path the plan
// answers keeps the arms apart at every probe of its rounded path.
#[hegel::test(test_cases = 40)]
fn every_answered_path_of_two_arms_keeps_them_apart(tc: TestCase) {
    let facing = Facing::new();
    let model = facing.model();
    let arms = facing.arms(&model);
    let (start, end) = (drawn_state(&tc, &arms), drawn_state(&tc, &arms));
    tc.assume(model.clearance(&start, Moving::All, f64::INFINITY) >= MIN_CLEARANCE_M + 0.01);
    tc.assume(model.clearance(&end, Moving::All, f64::INFINITY) >= MIN_CLEARANCE_M + 0.01);
    let goal = at([0.0, 0.0]);
    let (left, right) = (
        Drawn::new(vec![Arms::<2, 4>::slice(&end, 0)]),
        Drawn::new(vec![Arms::<2, 4>::slice(&end, 1)]),
    );
    let Ok(waypoints) = plan(
        &arms,
        start,
        vec![
            ArmGoal {
                goal: &goal,
                sampler: &left,
            },
            ArmGoal {
                goal: &goal,
                sampler: &right,
            },
        ],
        &[],
    ) else {
        return;
    };
    for q in probes(&arms, &waypoints) {
        let apart = model.clearance(&q, Moving::All, f64::INFINITY);
        assert!(
            apart >= MIN_CLEARANCE_M,
            "the arms come {apart} m apart at {q:?}"
        );
    }
}

/// The arms 1 cm apart, the halfway of the clearance: the left arm points at the right
/// one, which swings toward it from 1 rad until they stand so.
fn nearer_than_the_clearance(model: &PiecesApart) -> [f64; 4] {
    let swung = |turn: f64| [0.0, 0.0, turn, 0.0];
    let (mut apart, mut near) = (1.0, 0.0);
    for _ in 0..60 {
        let middle = (apart + near) / 2.0;
        if model.clearance(&swung(middle), Moving::All, f64::INFINITY) > MIN_CLEARANCE_M / 2.0 {
            apart = middle;
        } else {
            near = middle;
        }
    }
    let start = swung(apart);
    let at_start = model.clearance(&start, Moving::All, f64::INFINITY);
    assert!(
        0.0 < at_start && at_start < MIN_CLEARANCE_M,
        "the arms start {at_start} m apart"
    );
    start
}

// From a start whose arms stand nearer than the clearance, the left arm's goal its own
// start and the right arm's swung away: the left arm's turn of a goal sample judges no
// pair of the two arms, which the right arm's turn judges with both where the goal holds
// them, and the plan answers a path to both goals.
#[test]
fn a_goal_sample_judges_a_pair_of_the_arms_on_the_later_arms_turn() {
    let facing = Facing::new();
    let model = facing.model();
    let arms = facing.arms(&model);
    let start = nearer_than_the_clearance(&model);
    let (stays, away) = ([start[0], start[1]], [1.0, 0.0]);
    let end = [stays[0], stays[1], away[0], away[1]];
    assert!(model.clearance(&end, Moving::All, f64::INFINITY) >= MIN_CLEARANCE_M);
    let (left_goal, right_goal) = (at(stays), at(away));
    let (left, right) = (Drawn::new(vec![stays]), Drawn::new(vec![away]));
    let waypoints = plan(
        &arms,
        start,
        vec![
            ArmGoal {
                goal: &left_goal,
                sampler: &left,
            },
            ArmGoal {
                goal: &right_goal,
                sampler: &right,
            },
        ],
        &[],
    )
    .expect("a path to the goals, apart where they hold the arms");
    assert_eq!(waypoints.last(), Some(&end));
}

// From a start whose arms stand nearer than the clearance, to a goal across the line
// between their bases, whose straight joint motion turns the left arm into the right
// one, the path departs by the start rule: at no probe of its rounded path do the arms
// stand nearer than half their distance at the start.
#[test]
fn arms_that_start_nearer_than_the_clearance_depart_by_the_start_rule() {
    let facing = Facing::new();
    let model = facing.model();
    let arms = facing.arms(&model);
    let start = nearer_than_the_clearance(&model);
    let at_start = model.clearance(&start, Moving::All, f64::INFINITY);
    let goal = at([0.0, 0.0]);
    let (left, right) = (Drawn::new(vec![[-1.0, 0.0]]), Drawn::new(vec![[-1.0, 0.0]]));
    let waypoints = plan(
        &arms,
        start,
        vec![
            ArmGoal {
                goal: &goal,
                sampler: &left,
            },
            ArmGoal {
                goal: &goal,
                sampler: &right,
            },
        ],
        &[],
    )
    .expect("a path that departs and goes around");
    for q in probes(&arms, &waypoints) {
        let apart = model.clearance(&q, Moving::All, f64::INFINITY);
        assert!(
            apart > at_start / 2.0,
            "the arms come {apart} m apart at {q:?}, from {at_start} m"
        );
    }
}

/// The joints of OpenArm v2's two arms.
const OPENARM_BOTH: usize = 2 * JOINTS;

/// How far (m) a goal of the right hand's position in the left hand's frame lets it end.
const IN_HAND_M: f64 = 0.01;

/// How far (rad) a goal or a path orientation of the right hand in the left hand's frame
/// lets it turn.
const IN_HAND_RAD: f64 = 0.05;

/// Each OpenArm v2 arm with its elbow bent half a radian and its other joints at 0.
const REST: [f64; JOINTS] = [0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0];

/// The left and the right arm at `REST` with their first joints turned the robot's way by
/// `turn` (rad): the left arm's first joint by `turn`, and the right arm's by the turn about
/// its own axis that turns it alike, so both hands turn about one axis of the robot.
fn both_turned(turn: f64) -> ([f64; JOINTS], [f64; JOINTS]) {
    let axis =
        |arm: &collision_geometry::chain::ChainGeometry<JOINTS>| arm.chain().at(&REST).axis_root(0);
    let alike = axis(&LEFT_ARM).dot(&axis(&RIGHT_ARM));
    assert!(
        (alike.abs() - 1.0).abs() < 1e-9,
        "the arms' first joints turn about one axis of the robot, {alike} apart"
    );
    let mut left = REST;
    left[0] += turn;
    let mut right = REST;
    right[0] += turn * alike;
    (left, right)
}

/// The grasp point's pose of OpenArm v2's left arm at `left` and of its right arm at `right`.
fn hands(left: &[f64; JOINTS], right: &[f64; JOINTS]) -> (Isometry3<f64>, Isometry3<f64>) {
    (
        LEFT_ARM.chain().root_ee_pose(left),
        RIGHT_ARM.chain().root_ee_pose(right),
    )
}

/// The goal of every joint of an OpenArm v2 arm within 0.01 rad of `q`.
fn openarm_at(q: [f64; JOINTS]) -> Goal {
    let joints = (0..JOINTS)
        .map(|joint| JointTarget {
            joint,
            position: q[joint],
            above: positive(0.01),
            below: positive(0.01),
        })
        .collect();
    Goal::new(JOINTS, joints, Vec::new(), Vec::new()).expect("a valid goal")
}

/// The left hand's grasp point, the frame of the left arm, the first of the group.
const LEFT_HAND: Reference = Reference::Arm {
    arm: 0,
    frame: Frame::GraspPoint,
};

/// The goal of a hand at `relative` in the frame of the hand of the arm at place `leader`: its
/// position, and its orientation where `with_orientation` holds.
fn in_the_hand_of(leader: usize, relative: &Isometry3<f64>, with_orientation: bool) -> Goal {
    let reference = Reference::Arm {
        arm: leader,
        frame: Frame::GraspPoint,
    };
    let orientation = OrientationTarget {
        frame: Frame::GraspPoint,
        reference,
        orientation: relative.rotation,
        tolerance_rad: positive(IN_HAND_RAD),
        free_axis: None,
    };
    Goal::new(
        JOINTS,
        Vec::new(),
        vec![PositionTarget {
            frame: Frame::GraspPoint,
            reference,
            position: relative.translation.vector,
            tolerance_m: positive(IN_HAND_M),
        }],
        with_orientation
            .then_some(orientation)
            .into_iter()
            .collect(),
    )
    .expect("a valid goal")
}

/// OpenArm v2's two arms, left then right, with no self model.
fn openarm_both() -> Arms<'static, JOINTS, OPENARM_BOTH> {
    Arms::new(vec![plan_arm(LEFT), plan_arm(RIGHT)], None)
}

/// The plan of OpenArm v2's two arms, left then right, with no self model, in an empty map,
/// from `start`, each arm to its goal of `goals`, holding `orientations`.
fn plan_openarm(
    start: [f64; OPENARM_BOTH],
    goals: [&Goal; 2],
    orientations: &[PathOrientation],
) -> Result<Vec<[f64; OPENARM_BOTH]>, PlanFailure> {
    let samplers = [
        IkGoalSampler::new(LEFT_ARM.chain().clone()),
        IkGoalSampler::new(RIGHT_ARM.chain().clone()),
    ];
    let goals = goals
        .iter()
        .zip(&samplers)
        .map(|(goal, sampler)| ArmGoal { goal, sampler })
        .collect();
    plan(&openarm_both(), start, goals, orientations)
}

/// Plans OpenArm v2's two arms from `REST`: the arm at place `leader` turns at its shoulder to
/// a goal of its joints, and the other arm's hand to a goal in the leader's hand frame, a
/// position and then a pose, that both hands meet where `both_turned` turns them. The other hand
/// ends at its goal against the leader's hand where the path's last state puts it, far from
/// where the leader's hand frame stood at the start.
fn a_goal_in_the_leaders_hand_frame_holds_where_that_hand_ends(leader: usize) {
    let (left_end, right_end) = both_turned(-0.5);
    let ends = [left_end, right_end];
    let follower = 1 - leader;
    let hands_at = |q: &[f64; OPENARM_BOTH]| {
        let (left, right) = hands(
            &Arms::<JOINTS, OPENARM_BOTH>::slice(q, 0),
            &Arms::<JOINTS, OPENARM_BOTH>::slice(q, 1),
        );
        [left, right]
    };
    let at_the_end = hands_at(&Arms::<JOINTS, OPENARM_BOTH>::join(&ends));
    let relative = at_the_end[leader].inverse() * at_the_end[follower];
    let leader_goal = openarm_at(ends[leader]);
    let start = Arms::<JOINTS, OPENARM_BOTH>::join(&[REST, REST]);
    for with_orientation in [false, true] {
        let follower_goal = in_the_hand_of(leader, &relative, with_orientation);
        let goals = if leader == 0 {
            [&leader_goal, &follower_goal]
        } else {
            [&follower_goal, &leader_goal]
        };
        let path = plan_openarm(start, goals, &[]).unwrap_or_else(|failure| panic!("{failure}"));
        let ended = hands_at(path.last().expect("a waypoint"));
        let held = ended[leader] * Point3::from(relative.translation.vector);
        let off = (held.coords - ended[follower].translation.vector).norm();
        assert!(off <= IN_HAND_M, "the hand ends {off} m from its goal");
        if with_orientation {
            let turned =
                (ended[leader].rotation * relative.rotation).angle_to(&ended[follower].rotation);
            assert!(
                turned <= IN_HAND_RAD,
                "the hand ends {turned} rad from its goal"
            );
        }
        let at_start = hands_at(&start)[leader] * Point3::from(relative.translation.vector);
        assert!(
            (at_start.coords - ended[follower].translation.vector).norm() > 0.1,
            "the goal held where the leader's hand started"
        );
    }
}

// The left arm turns at its shoulder to a goal of its joints, and the right hand's goal, a
// position and then a pose in the left hand's frame, holds against the left hand where the
// path's last state puts it: the right hand ends there, far from where the left hand's frame
// stood at the start.
#[test]
fn a_goal_in_the_other_hands_frame_holds_against_that_hand_where_it_ends() {
    a_goal_in_the_leaders_hand_frame_holds_where_that_hand_ends(0);
}

// The right arm, after the left in the group, turns at its shoulder to a goal of its joints,
// and the left hand's goal is in the right hand's frame: the plan samples the right arm first,
// as MoveIt orders a sampler after the samplers whose links its frames are, and the left hand
// ends at its goal against the right hand where the path's last state puts it.
#[test]
fn a_goal_in_a_frame_of_an_arm_later_in_the_group_holds_where_that_arm_ends() {
    a_goal_in_the_leaders_hand_frame_holds_where_that_hand_ends(1);
}

// Both arms turn at their shoulders about one axis of the robot while the right hand holds
// its orientation in the left hand's frame: every waypoint, and every probe of the rounded
// path the robot runs, keeps the right hand within the tolerance of it where the left hand
// stands, though each hand turns half a radian in the robot frame.
#[test]
fn a_path_orientation_in_the_other_hands_frame_holds_at_every_state() {
    let (left_end, right_end) = both_turned(-0.5);
    let (left_start, right_start) = hands(&REST, &REST);
    let relative = left_start.rotation.inverse() * right_start.rotation;
    let orientations = [PathOrientation {
        holder: Holder::Arm(1),
        link_name: &openarm_v2::arm(RIGHT).grasp_link,
        target: OrientationTarget {
            frame: Frame::GraspPoint,
            reference: LEFT_HAND,
            orientation: relative,
            tolerance_rad: positive(IN_HAND_RAD),
            free_axis: None,
        },
    }];
    let start = Arms::<JOINTS, OPENARM_BOTH>::join(&[REST, REST]);
    let path = plan_openarm(
        start,
        [&openarm_at(left_end), &openarm_at(right_end)],
        &orientations,
    )
    .unwrap_or_else(|failure| panic!("{failure}"));
    let error = |q: &[f64; OPENARM_BOTH]| {
        let (left, right) = (
            Arms::<JOINTS, OPENARM_BOTH>::slice(q, 0),
            Arms::<JOINTS, OPENARM_BOTH>::slice(q, 1),
        );
        let (left_hand, right_hand) = hands(&left, &right);
        (left_hand.rotation * relative).angle_to(&right_hand.rotation)
    };
    for q in path.iter().copied().chain(probes(&openarm_both(), &path)) {
        let turned = error(&q);
        assert!(
            turned <= IN_HAND_RAD,
            "the right hand turns {turned} rad at {q:?}"
        );
    }
    let (left_end_hand, _) = hands(
        &Arms::<JOINTS, OPENARM_BOTH>::slice(path.last().expect("a waypoint"), 0),
        &REST,
    );
    assert!(left_end_hand.rotation.angle_to(&left_start.rotation) > 0.4);
}

/// A plan of OpenArm v2's left arm alone from `start` to `goal`, holding `orientations`,
/// the right arm outside the plan, as `plan_openarm` plans both.
fn plan_left_alone(
    start: [f64; JOINTS],
    goal: &Goal,
    orientations: &[PathOrientation],
) -> Result<Vec<[f64; JOINTS]>, PlanFailure> {
    let sampler = IkGoalSampler::new(LEFT_ARM.chain().clone());
    let goals = vec![ArmGoal {
        goal,
        sampler: &sampler,
    }];
    plan(&left_alone(), start, goals, orientations)
}

/// How far (rad) the right hand, standing outside a plan of the left arm, may turn from its
/// orientation in the left hand's frame.
const STANDING_RAD: f64 = 0.6;

// The right hand stands outside a plan of the left arm and holds an orientation in the left
// hand's frame. MoveIt judges such a constraint at each state against where the state puts
// the left hand, every joint outside the planning group at the start state's
// (state_validity_checker.cpp:58, :102-107; kinematic_constraint.cpp:728-735). A turn of the
// left arm that keeps the right hand within the tolerance plans, every waypoint and every
// probe of the rounded path within it, and the right hand ends turned from the left hand's
// frame by about the left hand's turn; a turn past the tolerance fails with no_path.
#[test]
fn a_standing_hand_holds_its_orientation_in_the_moving_hands_frame() {
    let (left_start, right_standing) = hands(&REST, &REST);
    let relative = left_start.rotation.inverse() * right_standing.rotation;
    let orientations = [PathOrientation {
        holder: Holder::Standing(right_standing.rotation),
        link_name: &openarm_v2::arm(RIGHT).grasp_link,
        target: OrientationTarget {
            frame: Frame::GraspPoint,
            reference: LEFT_HAND,
            orientation: relative,
            tolerance_rad: positive(STANDING_RAD),
            free_axis: None,
        },
    }];
    let error = |left: &[f64; JOINTS]| {
        (LEFT_ARM.chain().root_ee_pose(left).rotation * relative).angle_to(&right_standing.rotation)
    };
    let (within, _) = both_turned(-0.4);
    let path = plan_left_alone(REST, &openarm_at(within), &orientations)
        .unwrap_or_else(|failure| panic!("{failure}"));
    for q in path.iter().copied().chain(probes(&left_alone(), &path)) {
        let turned = error(&q);
        assert!(
            turned <= STANDING_RAD,
            "the right hand stands {turned} rad off at {q:?}"
        );
    }
    let at_the_end = error(path.last().expect("a waypoint"));
    assert!(
        at_the_end > 0.3,
        "the left hand's turn moves the frame the right hand's orientation is held in: {at_the_end}"
    );
    let (past, _) = both_turned(-1.0);
    assert!(
        matches!(
            plan_left_alone(REST, &openarm_at(past), &orientations),
            Err(PlanFailure::NoPath(_))
        ),
        "a turn past the tolerance breaks the orientation"
    );
}
