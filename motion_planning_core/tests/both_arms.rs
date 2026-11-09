//! Plans of two arms together: two planar arms of two revolute joints face each other
//! across 0.9 m, their links in one plane, so neither passes the other while it points at
//! it. The plan samples the arms' goals together, each arm against the joints the arms
//! before it drew, from a start nearer than the clearance each pair on the turn of its
//! later arm, names an arm whose sampler finds none, and answers a path that keeps the
//! arms apart along the rounded path the robot runs, from a start nearer than the
//! clearance as the start rule says.

mod common;

use std::num::{NonZeroU32, NonZeroU64, NonZeroUsize};
use std::sync::atomic::{AtomicUsize, Ordering};

use chain_kinematics::{ChainSpec, Draws, JointSelection};
use collision_geometry::chain::ChainGeometry;
use collision_geometry::gjk;
use control_core::path_timing::RoundedPath;
use hegel::TestCase;
use hegel::generators as gs;
use motion_planning_core::{
    Arm, ArmGoal, Arms, Body, Goal, GoalSampler, IkGoalSampler, JointTarget, Moving, Objects,
    PlanFailure, PlanRequest, Scene, SelfModel, plan_path,
};
use occupancy_map::{DistanceField, VoxelMap};
use ompl_bridge::Planner;

use common::{an_hour_from_now, distance_to_map, map, occupied_at, positive};

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
    fn clearance(&self, q: &[f64; 4], moving: Moving) -> f64 {
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
        _: &Goal,
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

/// The plan of `arms` from `start`, each arm to its goal of `goals`, in an empty map.
fn plan(
    arms: &Arms<2, 4>,
    start: [f64; 4],
    goals: Vec<ArmGoal<2>>,
) -> Result<Vec<[f64; 4]>, PlanFailure> {
    let (map, field) = empty();
    plan_in(&map, &field, arms, start, goals)
}

/// The plan of `arms` on `map`, whose distances `field` holds, from `start`, each arm to
/// its goal of `goals`.
fn plan_in(
    map: &VoxelMap,
    field: &DistanceField,
    arms: &Arms<2, 4>,
    start: [f64; 4],
    goals: Vec<ArmGoal<2>>,
) -> Result<Vec<[f64; 4]>, PlanFailure> {
    plan_path(
        arms,
        &Scene {
            map,
            field,
            stamp: 1,
            objects: Objects::none(),
        },
        &PlanRequest {
            start,
            start_tolerance: [0.0; 4],
            goals,
            path_orientations: &[],
            goal_samples: NonZeroUsize::new(GOAL_SAMPLES).expect("nonzero"),
            planner: Planner::RrtConnect,
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
fn probes(arms: &Arms<2, 4>, waypoints: &[[f64; 4]]) -> Vec<[f64; 4]> {
    let vectors: Vec<Vec<f64>> = waypoints.iter().map(|q| q.to_vec()).collect();
    let path = RoundedPath::joint_path(&vectors);
    let step = PROBE_M / arms.lever();
    let count = (path.length() / step).ceil() as usize;
    (0..=count)
        .map(|k| {
            path.at((k as f64 * step).min(path.length()))
                .as_slice()
                .try_into()
                .expect("a state of both arms")
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
        model.clearance(&[0.0, 0.0, 0.0, 0.0], Moving::All) < 0.0,
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
    )
    .expect("a path that keeps the arms apart");
    assert_eq!(waypoints.first(), Some(&UP));
    let last = waypoints.last().expect("a waypoint");
    assert!(
        (0..4).all(|j| (last[j] - DOWN[j]).abs() <= 0.01 + 1e-12),
        "the path ends at {last:?}"
    );
    for q in probes(&arms, &waypoints) {
        let apart = model.clearance(&q, Moving::All);
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
    assert!(model.clearance(&[0.0, 0.0, UP[2], UP[3]], Moving::All) >= MIN_CLEARANCE_M);
    assert!(model.clearance(&[0.0, 0.0, 0.0, 0.0], Moving::All) < MIN_CLEARANCE_M);
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
    )
    .expect("a path to the right arm's second try");
    let last = waypoints.last().expect("a waypoint");
    assert_eq!(Arms::<2, 4>::slice(last, 1), [1.0, 0.0], "{last:?}");
}

/// A point just past the right arm's tip where it stands at `UP`: its voxel lies 1.6 cm
/// from the arm, inside the plan's clearance and out of contact, and far from the left
/// arm.
const BESIDE_THE_RIGHT_TIP: [f64; 3] = [0.58, 0.5, 0.02];

/// The right arm's goal: swung back from `UP`, away from `BESIDE_THE_RIGHT_TIP`.
const RIGHT_AWAY: [f64; 2] = [-0.7, 0.0];

// The right arm starts nearer the map than the plan's clearance, which the start rule
// plans from: the left arm's turn of a goal sample judges the left arm's pairs, the right
// arm standing at its start, and the right arm's turn moves it clear of the map: the plan
// answers a path to both goals.
#[test]
fn an_arm_that_starts_nearer_than_the_clearance_holds_back_no_goal_sample() {
    let facing = Facing::new();
    let model = facing.model();
    let arms = facing.arms(&model);
    let mut map = map(0.04, 1.6);
    let above = [BESIDE_THE_RIGHT_TIP[0], BESIDE_THE_RIGHT_TIP[1], 1.0];
    occupied_at(&mut map, above, &[BESIDE_THE_RIGHT_TIP]);
    let field = DistanceField::of(&map);
    let end = [UP[0], UP[1], RIGHT_AWAY[0], RIGHT_AWAY[1]];
    let at_start = distance_to_map(&arms, &map, &UP);
    assert!(0.0 < at_start && at_start < MIN_CLEARANCE_M, "{at_start}");
    assert!(distance_to_map(&arms, &map, &end) > MIN_CLEARANCE_M);
    let (left_goal, right_goal) = (at([UP[0], UP[1]]), at(RIGHT_AWAY));
    let (left, right) = (
        Drawn::new(vec![[UP[0], UP[1]]]),
        Drawn::new(vec![RIGHT_AWAY]),
    );
    let waypoints = plan_in(
        &map,
        &field,
        &arms,
        UP,
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
    )
    .expect("a path from a start the start rule plans from");
    assert_eq!(waypoints.last(), Some(&end));
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
    tc.assume(model.clearance(&start, Moving::All) >= MIN_CLEARANCE_M + 0.01);
    tc.assume(model.clearance(&end, Moving::All) >= MIN_CLEARANCE_M + 0.01);
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
    ) else {
        return;
    };
    for q in probes(&arms, &waypoints) {
        let apart = model.clearance(&q, Moving::All);
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
        if model.clearance(&swung(middle), Moving::All) > MIN_CLEARANCE_M / 2.0 {
            apart = middle;
        } else {
            near = middle;
        }
    }
    let start = swung(apart);
    let at_start = model.clearance(&start, Moving::All);
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
    assert!(model.clearance(&end, Moving::All) >= MIN_CLEARANCE_M);
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
    let at_start = model.clearance(&start, Moving::All);
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
    )
    .expect("a path that departs and goes around");
    for q in probes(&arms, &waypoints) {
        let apart = model.clearance(&q, Moving::All);
        assert!(
            apart > at_start / 2.0,
            "the arms come {apart} m apart at {q:?}, from {at_start} m"
        );
    }
}
