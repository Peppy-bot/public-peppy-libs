# motion_planning_core

Plans the paths of one or more serial chains of joints, moved together, around a voxel
map of what the robot's cameras see and the objects of its planning scene. It knows no
robot: a robot's node gives it each chain's collision geometry, the parts its links
carry, the robot's self model where the robot has one, its links, and a goal sampler for
each chain.

- `Arm` binds a chain's collision pieces (`collision_geometry`) and the bodies its links
  carry rigidly through a plan (`Rider`, `Arm::riding`): parts of the robot, such as a
  gripper finger at the gripper's opening (`ChainGeometry::carried` reads them from the
  URDF), and the shapes of the objects attached to its grasp link
  (`PlanningScene::riders`). It places each as a `Body`, a piece or a shape, and binds
  how far a joint moves any point of them. A rider on no joint of the arm is refused
  (`RiderOffTheArm`).
- `Arms` is the arms a plan moves together: a state holds their joints one arm after
  another, `N` per arm and `D` in all, and a motion moves a point of any arm at most the
  largest of the arms' bounds. It binds the `SelfModel`.
- `PlanningScene` holds the collision objects, the attached objects and the pairs
  allowed to touch, as MoveIt's planning scene holds them, and decides every rule of
  motion_planning's scene members (`apply_collision_objects`,
  `apply_attached_collision_object`, `set_allowed_collision`, `get_planning_scene`).
- `Surroundings` says whether the arms at a state keep their clearance from the map
  (`occupancy_map`) and the objects that stand still (`Objects`), within the robot's own
  body and from one another, and reads the distance of each pair it checks
  (`Surroundings::readings`).
- `Goal` holds the constraints an arm's last waypoint meets; `IkGoalSampler` finds
  configurations meeting them for any chain.
- `plan_path` plans a `PlanRequest` on a `Scene` with OMPL (`ompl_bridge`), each arm to
  its `ArmGoal`, refines OMPL's path with MoveIt's CHOMP when the request's `Refiner` says
  so, and checks the rounded path the robot runs
  (`control_core::path_timing::RoundedPath::joint_path`); `check_path` checks a caller's
  path, a `CheckRequest`, with the same checks.
- The map's intake of a depth frame: `DepthCamera` reads its points, `JointHistory`
  gives the robot's joint positions at its stamp, and `insert_cut_out` inserts the
  points with the robot there and the objects, a `RobotMask`, cut out.

## Quick start

A plan of both of a robot's arms, each to its own goal; a plan of one arm is a group of
one, `Arms<7, 7>`, and the other arm stands among the parts of the robot it keeps clear
of.

```rust
use collision_geometry::chain::{CarriedPart, Rider};
use motion_planning_core::{Arm, ArmGoal, Arms, IkGoalSampler, PlanRequest, Scene, plan_path};

// The scene as a plan of both arms meets it.
let grasp_links = ["openarm_left_tcp", "openarm_right_tcp"];
let objects = planning_scene.for_arms(&grasp_links, |grasp_link| grasp_pose(grasp_link));
// Each arm carries its fingers and the objects its grasp link holds.
let riders = |geometry: &ChainGeometry<7>, grasp_link: &str| -> Vec<Rider> {
    geometry
        .carried()
        .iter()
        .map(CarriedPart::rider)
        .chain(planning_scene.riders(grasp_link, geometry))
        .collect()
};
let (left_riders, right_riders) = (
    riders(&left_geometry, grasp_links[0]),
    riders(&right_geometry, grasp_links[1]),
);
let arms = Arms::<7, 14>::new(
    vec![
        Arm::new("left_arm", &left_geometry).riding(&left_riders)?,
        Arm::new("right_arm", &right_geometry).riding(&right_riders)?,
    ],
    Some(&self_model),
);
let samplers = [
    IkGoalSampler::new(left_geometry.chain().clone()),
    IkGoalSampler::new(right_geometry.chain().clone()),
];
let field = DistanceField::of(&map);
let waypoints = plan_path(
    &arms,
    &Scene {
        map: &map,
        field: &field,
        stamp: last_frame_ns,
        // The body where it stands.
        objects: objects.objects(&standing_parts),
    },
    &PlanRequest {
        // The left arm's joints, then the right's.
        start: measured,
        start_tolerance: [0.01; 14],
        goals: vec![
            ArmGoal { goal: &left_goal, sampler: &samplers[0] },
            ArmGoal { goal: &right_goal, sampler: &samplers[1] },
        ],
        path_orientations: &[],
        goal_samples: NonZeroUsize::new(10).unwrap(),
        planner: Planner::RrtConnect,
        deadline: Instant::now() + Duration::from_secs(1),
        max_iterations: NonZeroU64::new(20_000).unwrap(),
        attempts: NonZeroU32::new(4).unwrap(),
        seed: 1,
        min_clearance_m: 0.02,
        stop: Some(&stop),
    },
)?;
```

## The map's intake

A robot's node and a replay of its recorded frames, such as the example
`scenes_from_episodes`, build the map with the same calls:

- `DepthCamera` holds the depth stream of a camera the robot carries: its depth optical
  frame in the robot frame, the color optical frame's pose there composed with the depth
  frame's offset in it (`DepthCamera::new`), its lens, depth model and meters per count.
  `DepthCamera::points` reads a z16 frame's points in the depth optical frame, a sample
  of 0 making none (`occupancy_map::depth_points`).
- `JointHistory` keeps the robot's measured positions over a span longer than the
  camera's delay, `N` values each, such as both arms' joints and the grippers' openings,
  and reads them at any instant inside it, interpolated between the measurements around
  it; a measurement no newer than the newest is dropped.
- `RobotMask` is the robot at the frame's stamp grown by a padding: each chain at its
  joint positions (`PosedChain`), the bodies its links carry, such as a gripper's fingers
  at the gripper's opening and the shapes of the objects attached to its grasp link
  (`PlanningScene::riders`), and the body fixed around the chains, each piece grown as
  `collision_geometry` grows it for MoveIt's distance rule; `RobotMask::with_objects`
  adds the shapes of the objects no link carries where they stand
  (`PlanningScene::standing_shapes`), each shape grown by the same padding and rule
  (`PlacedShape::contains`). `insert_cut_out` inserts the scan into the voxel map with
  the mask cut out (`occupancy_map::RobotCutout` and `ShapeCutout`), the URDF root at the
  origin of the scan's frame: a point inside it marks nothing, and its ray clears the
  space before it.

## A valid state

A state keeps the plan's clearance of three terms (`TERMS`): the clearance asked of the
term (`term_clearances`) plus 1 mm for each of its bodies that moves; the next section
says why each is as large as it is:

- The map and the objects, which stand still through a plan: `min_clearance_m` plus
  1 mm. Each body of the arms, a piece of a link, a piece of a part a link carries or a
  shape of an object it carries, keeps it from every occupied voxel and from each
  obstacle, an object of the scene where it stands or where an arm the plan does not move
  holds it; and each object the arms carry keeps it from each part of the robot the plan
  does not move, such as an arm it does not move and the torso (`RobotPart`).
- The robot's own body, where both bodies of a pair move: `min_clearance_m` plus 2 mm.
  Each pair of the self model keeps it, and so does each object an arm carries from each
  piece of another arm of the plan and of the parts it carries, and from each object
  another arm of the plan carries.
- An attached object against the arm that carries it, where both bodies of a pair move: 0
  plus 2 mm. Each object an arm carries keeps it from each piece of that arm's links and
  of the parts they carry, outside its touch links: between an attached object and the
  links of the arm that carries it, outside its touch_links, the clearance is 0: they keep
  out of contact. Objects one arm carries are never checked against each other. From the
  rest of the robot an attached object keeps the clearance as from an object: from an arm
  the plan does not move, the torso and the objects such an arm holds, which stand still
  through the plan, in the first term, and from another arm of the plan and the objects
  it carries in the second.

MoveIt checks an attached object against the robot unpadded and never checks two objects
attached to one link against each other (moveit2 2.15.2 `collision_common.cpp`, the
always-allowed collision of bodies attached to one link). This crate departs from it
there: an attached object keeps 2 mm, the spare of its two moving bodies, from the arm
that carries it at each checked state, and the clearance it keeps from an object from the
parts of the robot and from the other arms, which MoveIt checks it against unpadded. An
object an arm the plan does not move holds is an obstacle here: the arms a plan moves and
the objects they carry keep `min_clearance_m` from it, where MoveIt holds it as an
attached body of that arm and checks those pairs in its self check, unpadded
(`CollisionRequest`'s `pad_self_collisions`, false by default,
moveit_core/collision_detection/include/moveit/collision_detection/collision_common.hpp:157;
planning_scene.cpp:453 runs the self check on the unpadded collision environment).

MoveIt pads no object an arm holds: an attached object takes the padding of its link
(collision_env_fcl.cpp:195-196), `setPadding` pads only the links with collision geometry
(collision_env.cpp:127-133), and a link without, such as the grasp point, has a padding of
0 (collision_env.cpp:166-176). So at a clearance MoveIt keeps a held object unpadded from
the objects and the map, where this crate keeps it the clearance. MoveIt pads a link's
mesh by moving each vertex along its normal (geometric_shapes 2.3.4 shapes.cpp:402-404,
through collision_common.cpp:977), so a face beside a right-angled edge moves out the
padding over the square root of 2, 2.8 cm at 4 cm, and one at a box's corner the padding
over the square root of 3, 2.3 cm; this crate keeps the clearance all round.

A pair allowed to touch keeps none (`PlanningScene::set_allowed_collision`), and nor does
an object an arm carries from its grasp link and its touch links, or an object an arm the
plan does not move holds from that arm's grasp link and its touch links. The messages of
`Unreachable` and `NoPath` name the plan's clearances in meters.

Each body's bounding sphere screens first: against the map's distance field, which reads
the distance between voxel centers, within half a voxel's diagonal of the sphere center's
own distance to the nearest occupied center, which lies within another half diagonal of
that voxel's box; and against an object's shape or a part's piece, by the distance
between the two spheres. A body whose sphere does not clear by the screen is measured
exactly: against the box of each occupied voxel near it, the object's shape or the part's
hull, by GJK. The screen never changes the exact verdict. A mesh is measured by its
triangles, so a body passes through the open corner of an L-shaped mesh, and a body
wholly inside a mesh, a closed solid, lies at a negative distance from it, in collision.
A voxel no ray has reached holds nothing to collide with. A distance that is not a
number, from GJK or from the self model, counts as short of every clearance, and a self
model that answers none is in contact.

## A valid motion

The planner's search checks a motion as OMPL's discrete motion check does, and MoveIt's
planners with it: at the states that divide the motion into stretches of OMPL's default
share of the space's extent at most, by the sum of the joints' changes, each keeping the
plan's clearances. The path a plan answers and a path `check_path` checks are walked
whole, every run and arc, as the rest of this section says, and the last paragraph of
"The plan and its path" says which path a plan answers.

A motion is walked by conservative advancement. A joint turning by an angle moves a point
at most that angle times the point's distance from the joint, so how far a step moves any
point of a link or of a rider, a part or an object it carries, follows from each joint's
reach over every configuration (`Arm::reach`), each body bounded by its bounding sphere; a
prismatic joint lengthens the arm past it by up to its farthest limit. A point rides one
arm, so a motion of several arms moves it at most the largest of the arms' bounds
(`Arms::displacement`), and two bodies of two moving arms come nearer by at most twice
that. At each checked state the walk measures the arms' distance from the map and the
objects, the least distance within the robot's own body, and the least between an attached
object and the arm that carries it. It measures the bodies against the map nearest screen
first, and sets aside a body whose screen, a voxel whose bounding sphere, or a pair whose
spheres lie apart past the least distance it has found: what it sets aside lies no nearer.
A body's distance from an occupied voxel or an object, which stand still, falls by at most
how far the body moves, and the distance of a pair of its own body, each of whose bodies
moves, by at most twice that. So the walk steps no farther than the least spare: the
distance from the map and the objects above `min_clearance_m`, half the own body's above
it, and half the distance between an attached object and the arm that carries it, and at
most `MAX_STEP_M`, 1.5 cm. Every state between two checked states keeps the clearance
asked of each term. A state from which the step would fall under 1 mm falls short of the
plan's clearance and ends the motion: near a surface at the clearance asked of its term
the spare, and the step with it, shrinks toward 0, and the floor bounds a motion's checks
by its length over 1 mm. The cap keeps a checked state within 1.5 cm of any point of a
run, where a corner's repair adds its waypoints, and the distance from the map is measured
exactly up to `min_clearance_m` plus 1.5 cm, past which a body's bounding sphere bounds
it, and so is the distance from an object or within the robot's own body, past which the
self model's lower bound of a pair stands for it. Each arc of the
rounded path is walked the same way along the path: a step of its length moves a point at
most that times the arc's lever, the sum over the joints of the largest share of the
path's unit tangent the joint has on the arc times its reach, and at most the joints'
reach in their L2 norm, the largest of the arms' (`Arms::lever_along`). The voxel's size
needs no inflation, for a state is measured against each occupied voxel's whole box, which
holds every point the voxel stands for.

## The start

A joint that starts outside its limits by no more than its start tolerance moves onto
the limit; one farther out refuses the plan, naming the joint by its URDF name
(`StartFault::OutsideLimits`), and a caller asks `start_inside_limits` before it takes
a request. A start in contact with an obstacle or with its own body refuses it too, and
the refusal names the pair (`StartFault::InCollision`, `Contact`). So does a start nearer
than `min_clearance_m` to the map or to an object no arm holds, as MoveIt's collision
check, padded by the clearance, refuses a start in collision (START_STATE_IN_COLLISION):
the refusal names the nearest such pair and how far apart it stands
(`StartFault::InsideClearance`). MoveIt pads no pair of the robot's own body.

The robot runs a path from its measured joints, the start. A start otherwise closer than
the plan's clearance of a term, such as one whose links stand a millimeter apart, departs
along the straight line from the start by the start rule, each term judged on its own,
until it keeps that clearance. The rule holds at the states the walk checks: each lies no nearer than the
farthest checked state before it, or than the plan's clearance where that is nearer, to
the rounding of the arithmetic, 1e-9 m. Between two checked states the term comes nearer
by at most half its room. The start sets each term's floor: 0 when the start is nearer
than the clearance asked of the term, and when it is at least that far, that clearance or
the start's distance less twice its moving bodies times the shortest step, 0.1 mm,
whichever is lower. The term's room is its farthest checked distance above the floor, and
a step moves no point farther than the room over twice its moving bodies, so between two
checked states it stays above the middle of its floor and its farthest distance: from a
start below the clearance asked above half that distance, so nothing touches; from one at
least 0.2 mm above it from the map and the objects, or 0.4 mm above it from its own body,
above that clearance; and from one nearer above it, at most 0.1 mm below it from the map
and the objects, and 0.2 mm from its own body, each. An attached object, asked to keep 0
from the arm that carries it, has a floor of 0 there and never comes nearer than 0. A
motion ends where its step falls under 0.1 mm. A term the start keeps the clearance of
keeps it as on any run. A run whose path from the start lies on the straight line from the
start to the run's end, past waypoints on it such as one a corner's repair adds to the
first run, is walked by the rule from the start, and so is a planner's motion whose first
state lies on the straight line from the start to its end; any other run is walked by its
spare from its first state, so one that leaves a state short of the plan's clearance fails
there.

Run from the plan's start, the path keeps the clearance asked of each term. Run from up to
a tolerance from the start in each joint, as a robot's controller allows, it comes nearer
anything near its start by at most the sum over the joints of the tolerance times the
joint's reach (`Arm::reach`), over the joints of both arms for a pair of two arms, as
MoveIt's execution starts within its allowed start tolerance and checks nothing more.

## The goal

A goal holds joints within tolerances above and below their targets, or holds the grasp
point, a link of the chain or one fixed below it, or a part a link carries through a plan
(`Frame`; `Frame::Part`, from a part's `Rider`, stands where the clearance checks place
the part): its origin within a distance of a position of the robot frame, and its
orientation within an angle of a target, the angle of the rotation between them, or, with
a free axis, the angle between that axis and its target direction. MoveIt holds a pose
goal's orientation by each component of the rotation vector between the two within the
tolerance (moveit2 2.15.2 `constructGoalConstraints`, `ROTATION_VECTOR`, utils.cpp:220-226
and kinematic_constraint.cpp:765-775), a box that admits a rotation of up to the square
root of 3 times the tolerance; this crate holds the rotation's angle within it, the ball
inside that box. The pose scenario's tally plans both at the same tolerance. Each
tolerance is a `control_core` `PositiveFinite`. A goal that mixes joints with poses, names
a joint or a frame twice in one list, or holds the position of one frame and the
orientation of another is refused (`GoalError`): a goal holds one frame, whose pose the
sampler reaches for. MoveIt keeps one sampler for a goal on several links, the link whose
constraints leave the least volume, and accepts a sample that meets the others by chance.

The plan samples its goals as MoveIt's `ConstrainedGoalSampler` samples a goal's states
(moveit2 2.15.2 constrained_goal_sampler.cpp; `sample_goals`). A working state starts at
the plan's start. Each goal sample sets out from it with MoveIt's 4 attempts
(`max_state_sampling_attempts`), the plan's clearance check deciding inside the sample, as
MoveIt's validity callback does. The goal a sample finds becomes the working state, so the
goals of a short pose move gather about the first one, which lies on the start's shoulder
and wrist branches where the first attempt from the start reaches the pose; a goal joins
the goals unless it equals one of them. An attempt's sampling ends at `goal_samples`
goals, as MoveIt's `max_goal_samples` (10 by default) ends it; once the samples since the
last goal found reach 1000 less the goals found so far, as MoveIt's
`max_goal_sampling_attempts` (1000) bounds the samples its sampling function makes before
it returns a goal, counting from the goals OMPL has taken; or before a sample once the
deadline passes or the stop flag is set. MoveIt samples on a thread beside its planner and
ends once the problem holds a path (constrained_goal_sampler.cpp:109-111). Here each
attempt samples its own goals on its own thread when its planner asks for one
(`ompl_bridge::Goals::Sampled`, and its README's "Sampled goals"), so a plan's goals
depend on its planner's questions alone and the plan repeats: RRTConnect waits for its
first goal, and plans once it holds one, whatever the later samples find. PRM and PRM*
plan once the sampling has ended.

Each arm of a plan has its own goal and sampler (`ArmGoal`), and a goal sample of several
arms is MoveIt's sample of a group whose constraint sampler is the union of its arms'
(`UnionConstraintSampler`, union_constraint_sampler.cpp): it draws every arm in turn
(`GoalSampler::sample_from`) from its joints of the state the sample has reached, with the
4 attempts, and from the draws of the request's seed plus the arm's place in the group;
the plan's clearance check decides on that state with the arms before it at their sampled
joints and the arms after it at the working state's, an arm the plan does not move among
the scene. From a start that keeps its clearances, the check judges the whole state, as
MoveIt's validity callback does. From a start nearer than the plan's clearance, which the
start rule plans from, a turn judges each pair that holds a body of its arm and none of an
arm after it (`Moving::Arm`), and a pair with an arm after it waits for that arm's turn,
where both stand as the goal holds them: so an arm that starts nearer the scene or the
other arm than the clearance holds back no goal, and every goal keeps every clearance. A
sample in which an arm finds none finds no goal. A plan whose sampling finds no goal fails
`Unreachable`, saying so and, for a plan of several arms, naming the first arm that met
its goal in no sample; one whose sampling rejects states and finds no goal fails
`Unreachable` and says that instead, how many states the sampling rejected, naming, of the
state that falls short least, the nearest pair the check judged: of the robot's own body,
of an attached object and the arm that carries it, or of a body of an arm and the map, an
object or a part of the robot, of each clearance it falls short of and how far apart they
stand.

`IkGoalSampler` makes a sample. For a goal of joints it draws one configuration inside the
joint limits, the targeted joints inside their targets, as MoveIt's
`JointConstraintSampler` does. For a pose, as MoveIt's `IKConstraintSampler`, each attempt
draws a pose of the goal's frame inside the goal and searches for it by MoveIt's KDL
inverse kinematics, as [`chain_kinematics`](../chain_kinematics) holds it, at the search's
budget of 9274 passes, so a seed gives the same goals on every machine: the first attempt
from the working state, each later one from a configuration drawn inside the limits. The
grasp point, and a link that the chain's every joint moves and that is fixed below the last
joint's link, are the tip of MoveIt's solver or a link fixed to it, and the search solves
for them as KDL does. A link the chain's later joints do not move, and a part a link
carries such as a finger, which hangs on a joint of its own, depart from MoveIt, which has
no inverse kinematics for them: MoveIt builds no IK sampler for a link that is neither the tip nor fixed to it
(`IKConstraintSampler::loadIKSolver`, default_constraint_samplers.cpp:386-439), so its
constraint sampler manager selects none (constraint_sampler_manager.cpp:395) and its goal
sampler draws states uniformly and keeps those that meet the goal by chance
(constrained_goal_sampler.cpp:71-72, :158-166). This sampler descends toward what the goal
names, with KDL's position-only and approximate-solution options applied
(kdl_kinematics_plugin.cpp:344-350, :389), and the goal's constraints decide.

A chain of fewer joints than a pose has freedoms, SO-101's five, reaches no pose drawn inside
a goal exactly, so a frame MoveIt solves for takes other options. A goal of a position alone
descends on the position alone, as MoveIt's KDL solver does under its `position_only_ik`
option. A goal of a position and an orientation departs from MoveIt, whose sampler offers
converged ends alone (`callIK` passes no options, default_constraint_samplers.cpp:687-689): a
descent weighs the orientation's rows at 0.01, the search offers each end, and the goal's
constraints decide. A goal of an orientation alone descends on the orientation alone, and the
goal's constraints decide on each end. Of
100 goals of SO-101's arm at 1 mm, held within 0.02 rad, or tilted 0.05 rad and held within
0.1 rad, it meets 100 of each, where `position_only_ik` with converged ends meets 45 and 94,
and the search of a longer chain 10 and 2. A Cartesian path takes the search of every chain,
so it ends where a chain this short cannot keep the precision.

On 200 drawn reachable poses of OpenArm v2's left arm, with the configurations of every
other branch invalid, one goal sample from all joints at zero reaches each shoulder and
wrist branch that `srs_model`'s closed form finds on each of them, 144 of which have two
(`tests/goal_coverage.rs`). The goal sampling reaches each of 10 drawn reachable poses of
the forearm, of the wrist link, and of a part the hand carries.

## The plan and its path

The request's deadline bounds the plan as MoveIt's allowed planning time bounds a request:
OMPL makes the request's `attempts` before it, as MoveIt makes a request's
`num_planning_attempts` (ompl_bridge's README, Attempts): each from its own seed, under
the whole of that time and the whole iteration bound, sampling up to `goal_samples` goals
as its planner asks for them, ending before a sample once the deadline passes, their paths
joined by OMPL's path hybridization, and OMPL's simplifier takes what remains after the
attempts, at most 1000 iterations, as MoveIt simplifies in the allowed planning time less
the plan's (moveit2 2.15.2 `ModelBasedPlanningContext::solve` calls `simplifySolution`
with that time, whatever is left of it). OMPL's simplifier runs its first pass however
little time it has (`PathSimplifier::simplify`, `atLeastOnce`), so a planner that plans
until its time ends, such as RRT*, has its path simplified once, past the deadline. A
corner's repair runs only before the deadline: when a corner of the simplified path rounds
short of the clearances once the deadline has passed, the plan answers its planned path if
that passes its own check, and fails `NoPath` otherwise. The hybrid of several attempts
and its checks, the check of the rounded path, and CHOMP's refinement of it (CHOMP), run to
their end past the deadline.
OMPL plans with the request's planner and iteration bound, seeded from the request's seed
and the map's stamp, so the same request on the same map gives the same path when the plan
ends before its deadline, for every planner but PRM and PRM*, with any number of attempts.
OMPL's simplifier shortens the path.
OMPL measures the joints' space as MoveIt measures a joint group, by the sum of the
joints' changes (moveit2 2.15.2 `JointModelGroup::distance`, ompl_bridge's README): the
length RRT* shortens, and the distance of which each planner's range is a fifth of the
space's extent.

The caller's flag `stop` ends a plan at any part of it, and the plan fails `Stopped`:
the goal sampling makes no further sample, OMPL's planners and simplifier end at
their next question whether to stop, and the check of the rounded path ends before its
next walk of the path.

The robot runs the path with its corners rounded as execute_joint_path rounds them
(`RoundedPath::joint_path`). The planner walks each arc of the rounded path, from where
it leaves a run to where it joins the next, as a motion is walked; a state on a planned
segment is left to the walk of the path's runs. At a corner whose rounding falls short,
it adds a waypoint on each of the corner's runs within a quarter of the run from the
corner, which shrinks the arc, and halves that share at each failure until the rounding
passes, or fails `NoPath` once the added waypoints lie within twice
`MIN_JOINT_CHANGE_RAD` of the corner or a run's walk checks no state within the share
that keeps the plan's clearance. Each added waypoint is the state the run's walk checks
there farthest from the corner that keeps the plan's clearance, the start rule's walk
included, so each run it splits walks only states the plan walked and each waypoint
after the start keeps the plan's clearance.
It then walks every run of its answer, a run that joins two waypoints past one the path
drops, one within `MIN_JOINT_CHANGE_RAD` of the waypoint before it, among them, and fails
`NoPath` on a run that falls short.

The search takes the first state of a motion as valid, as OMPL's discrete check does. A
plan from a start short of its clearances walks each motion as "A valid motion" and "The
start" say: a motion fails when its first state, the start excepted, falls short, and a
motion from the start fails when it ends before each term keeps its clearance. OMPL's
simplifier walks each motion it checks, so each shortcut it keeps passes the walk of the
plan's answer. The plan answers the simplified path when the path the robot runs through
it passes the walk above, its rounded corners included, the planned path when only that
one passes, and no path when neither does. MoveIt checks the simplified path at its checked
states alone, and answers INVALID_MOTION_PLAN when it fails.

## CHOMP

`src/chomp.rs` is MoveIt 2.15.2's CHOMP optimizer (`chomp_motion_planner`) in Rust, with the
values of its planner plugin (`chomp_interface.cpp`, `loadParams`), and keeps MoveIt's BSD
license notice. A plan whose `refiner` is `Refiner::Chomp` refines the path its planner
answers. CHOMP starts from the path the robot runs through it, its corners rounded as
execute_joint_path rounds them, its 101 points, 3 s at 0.03 s, evenly spaced along it; the
start and the end stay where they are, and the 99 points between them are free. MoveIt's
`fillInFromTrajectory` fills CHOMP's points by waypoint index through the trajectory CHOMP
is given (chomp_trajectory.cpp:188-209): through the trajectory MoveIt's time
parameterization answers, sampled every 0.1 s, the points lie at even times on the chords
between the samples. On 100 drawn scenes whose RRTConnect path turns, at a clearance of
4 cm, CHOMP from MoveIt's fill answers a path that bends more than RRTConnect's on each of
the 82 it changes, and from the start here one that bends less on 75 of those 82 and that
the robot runs in less time on 76. A
trajectory's cost is its squared acceleration, weighed by 0.1, plus, at each free point,
each collision sphere's potential times its speed. The spheres are those of each link of
each arm and of each part its links carry, such as a gripper finger
(`collision_geometry::spheres`), grown by the plan's clearance, as MoveIt grows each
link's bodies by its padding. A sphere's potential is MoveIt's of its center's distance in
CHOMP's field less its radius: 0 from a clearance of 0.2 m, quadratic within it and linear
inside an obstacle. The field holds the map's distances and every obstacle of the plan,
each voxel its shapes touch occupied (`VoxelMap::occupy_intersecting`): every object that
stands in the world, as MoveIt's CHOMP holds every object of its world, and every object
an arm the plan does not move holds, as MoveIt's CHOMP holds the bodies of the links
outside the group in a second field. Its distances and gradient stop at MoveIt's
propagation distance, 0.25 m, rounded up to whole voxels as MoveIt's field rounds it, and a
sphere 0.25 m or more from every obstacle has no potential. The field's cells are the
map's voxels; MoveIt's field has cells of 2 cm (collision_env_distance_field.hpp:53)
whatever the size of its octomap's voxels.

Each iteration descends as MoveIt's optimizer does: the smoothness increments of every
free point and the collision increments of one free point drawn from the plan's seed,
joined with the map's stamp, through the inverse of the smoothness cost, each joint's step
held to 0.1 rad, each joint pulled back inside its limits. CHOMP runs its 200 iterations
to their end, past the plan's deadline when the planner leaves it none, as MoveIt's CHOMP
plans by its own limit and reads no allowed planning time; a stop ends it. It weighs the
trajectories MoveIt's optimizer walks that cost less than the planner's path, the
trajectory of least cost at every tenth iteration and after the last, each free point held
inside the joint limits, walks them with the plan's own check of the path the robot runs,
corner repairs run to their end whatever the time, from the least cost up, and the plan
answers the first that passes, and the planner's path when none does. So a refined plan
passes every check of a plan, costs less than its planner's path in CHOMP's cost, and is
the same for the same request on the same map whatever its timing. MoveIt's optimizer
stops at its first clear walk, the walk of the planner's path itself when that path is
valid, or five iterations after its collision cost falls below 0.07, and walks nothing
after its last iteration.

A sphere's Jacobian is the derivative of its center: a revolute joint's axis crossed with
the center's offset from the joint, and a prismatic joint's axis. MoveIt's
`computeJointProperties` (chomp_optimizer.cpp:749) takes `joint_transform * axis` on an
`Eigen::Isometry3d`, which adds the joint's position to its axis. CHOMP's field holds no
link of the robot: MoveIt's optimizer also reads the distances to the robot's links outside
the group, and between the group's own links.

`tests/ros_reference_chomp_refinement.rs` hands MoveIt's CHOMP and this crate's the same
input paths, those this crate's RRTConnect plans through the scenes of the
[`ros_reference`](../ros_reference) harness's `chomp_refinement_openarm_v2` scenario:
MoveIt's collision check finds every refinement of ours free, and where MoveIt's CHOMP
answers a path of a committed scene, ours bends no more than it. On the first 200 drawn
scenes at a clearance of 4 cm, ours bends less than MoveIt's, and runs in less time, on
each of the 17 whose input path turns. Of the 156 straight input paths, ours bends 47 of
the 77 that pass within 0.2 m of an obstacle, CHOMP's clearance, where MoveIt's CHOMP
answers each straight path as it is.

## Path orientations

`PlanRequest::path_orientations` lists the orientations every state of the path holds, each
a frame of an arm of the group with an `OrientationTarget`, the measure a goal's orientation
takes: the angle of the rotation from the target, or, with a free axis, the angle between
that axis and its direction in the target. The list carries no order. It holds fewer
orientations than the state has joints, for OMPL's projection needs fewer rows than
dimensions; a longer list fails with `PlanFailure::Request`.

The plan corrects its states onto the orientations with OMPL's projected state space. Its
function holds one row for each orientation, the error above the angle the plan allows it:
`HELD_SHARE`, 0.9, of the tolerance, or the start's error where that is more. Each row's
Jacobian is the unit axis along which the error grows against each joint that turns the
frame, and 0 for every other joint, so the row of one arm leaves the other arm's joints
alone. MoveIt's constrained planning projects so for one orientation path constraint, its
default for one (moveit2 2.15.2 `ConstrainedPlanningStateSpaceFactory`, priority 200), with
each component of a rotation vector bounded by twice its tolerance (ompl_constraints.cpp).
With two or more it plans in pose space through inverse kinematics, and on a group of two
chains its Jacobian is refused (`RobotState::getJacobian`, robot_state.cpp:1516). Here the
rows of every orientation stack on one arm and on two.

Every state of the rounded path holds each orientation. A joint turning by an angle turns a
frame it moves by at most that angle, so a frame turns along a motion at most the sum of the
changes of its joints, and a step that turns it no farther than what lies between its error
and its tolerance keeps every state of the step inside. The plan walks each run and each arc
so, apart from the walks of its clearances, and a state within 1% of the tolerance ends the
walk: the planner's motions, each straight stretch of OMPL's walk along the orientations as
one motion; the runs of its answer; and each arc. A corner whose arc breaks an orientation
gains a waypoint on each of its runs, a quarter of the run from the corner and then each time
half as far, which pulls the arc in until it holds or its waypoints lie at the corner. OMPL's
walk checks no bounds when the caller checks its states, so the bridge refuses a walk with a
state outside the joints' limits, as MoveIt's validity check does (state_validity_checker.cpp).
A run of the answer stays inside between its states: a straight joint motion `a` turns a frame
off the shortest rotation between its ends by at most `|a|_1^2 / 16`, a free axis as much again
by the small circle it follows, and the plan spaces its states so that the first is at most half
of what lies between the allowed angle and the tolerance (`Projection::spacing`), which is
OMPL's default, 0.1, from a tolerance of about 0.09 rad up on a frame all seven joints turn. The
answer drops the waypoints inside a straight stretch. Run from up to a tolerance from the start
in each joint, as a robot's controller allows, a frame turns from the plan's start by at most
the sum of its joints' differences from it.

A start more than 99% of a tolerance from its orientation fails with `PlanFailure::NoPath`.
A goal stands within the angle the plan allows: a goal that names a frame's position or
orientation takes a path orientation of that frame as MoveIt merges a request's goal and path
constraints, drawing from the smaller of the two sampling volumes (constraint_sampler_manager.cpp)
and meeting both, and any other goal is judged by the orientations as it is sampled. A goal
orientation whose middle stands farther from a path orientation's than the goal's tolerance and
the allowed angle together fails with `PlanFailure::NoPath` at once, unless the two hold different
free axes, whose distance no one angle states. After each sample that misses, an attempt samples
the goal alone, until a sample of either finds a state. A plan with no goal fails with
`PlanFailure::NoPath` where a sample of the goal alone found a configuration, and with
`PlanFailure::Unreachable` where none did, as a plan that holds nothing does.

`check_path` holds no orientation, as check_joint_path takes none.

## Checking a path

`check_path` checks the path execute_joint_path runs from a caller's start through its
waypoints, each holding every arm's joints, with the checks of a plan's answer: the start
inside its limits by its tolerance and out of contact, each waypoint inside the joint
limits, a joint at most `TARGET_SLACK` (0.001) outside a limit taken at the limit as
execute_joint_path takes it, and, after the start, at the plan's clearances, each run as the plan walks the runs
of its answer, the run whose path from the start lies on the straight line from the start
by the start rule, and each arc of the rounded path as the plan walks its arcs. The walks
follow the run into the first waypoint outside a limit up to where it leaves the limits,
and go no farther. It answers the first fault along the path (`PathFault`): the start's, a
waypoint that holds a joint, by its URDF name, outside its limits, or a state short of the
plan's clearance, where it lies (`Place`: at a waypoint, on a run, or on the arc at a
corner) and how near it comes to the map and the objects, within the robot's own body, and
between an attached object and the arm that carries it. It answers too the most waypoints,
from the first, whose path, every arm run together through them with the last of them as
its end, passes: the waypoints up to the one whose run or corner fails, the start counted
as waypoint 0, or before the waypoint that fails. A corner's arc runs before the rest of
the run from its waypoint, and that run before the waypoint it ends at. Each walk of a run
or of a corner starts only before the check's deadline, and a check that reaches its
deadline fails `TimeEnded`.

## Cartesian paths

`cartesian_path` moves a frame of one arm along straight lines through poses, as MoveIt's
`CartesianInterpolator::computeCartesianPath` does for move_group's
`compute_cartesian_path` service (moveit2 2.15.2, `cartesian_interpolator.cpp:206-307`,
`cartesian_path_service_capability.cpp:186-188`). A `FollowedFrame` is the grasp point or a
link fixed to it; MoveIt has no inverse kinematics for any other link.

- Each line runs from the frame's pose at the state the path reached to the next pose, its
  position lerped and its orientation slerped the shorter way in step with it.
- Its steps are the more of its length over 0.01 m and its turn over 0.035 rad, each rounded
  down, plus one: `MaxEEFStep(0.01)`, the step of RViz's MotionPlanning panel
  (`motion_planning_frame_planning.cpp:141`).
- Each step is the KDL search from the state before, the state's check inside the search
  (`setFromIK` at its default timeout, `cartesian_interpolator.cpp:260`,
  `robot_state.cpp:2017-2019`). Each interval keeps the frame at the joints' midpoint within
  1 mm and 0.01 rad of the line's midpoint, or is halved and its midpoint solved from the
  joints' midpoint, down to 1e-5 of the line (`validateAndImproveInterval`): the walk is
  `chain_kinematics::walk_line`, which the OpenArm backbone's `move_arm` walks too with
  `srs_model`'s closed form.
- The first step that fails ends the path. `fraction` sums as MoveIt sums it: one share of
  the poses for each whole line, and the reached share of the line the path ends on.

Past MoveIt's walk, `cartesian_path` checks the path the move runs, its repeats dropped and
its corners rounded as `execute_joint_path` runs it:

- `Collisions::Checked` checks it as `check_path` checks a caller's path, and a fault ends
  the path at the waypoint before it. MoveIt checks each state for collisions and no motion
  between two. `Collisions::Unchecked` checks no collision and no start contact.
- At probes of the rounded path, the frame stands within 0.9 mm and 0.009 rad of the lines'
  pose that pairs with the probe. The probes stand so close that the frame and that pose each
  move at most 0.05 mm and turn at most 0.5 mrad between two, so the frame stays within 1 mm
  and 0.01 rad at every state. A probe outside halves the intervals beside its waypoint, and
  an interval narrower than 1e-5 of a line, whose midway has no state, or whose midway state the
  move drops, ends the path before it. MoveIt checks the midpoint of each interval.
- A start nearer than the clearance to the map or an object no arm holds is refused, as a
  plan's is; any other start short of the clearance leaves in one straight joint run to the
  first state that keeps the clearance, as the start rule walks a departure (see "The
  start"). A path that
  reaches no such state, or whose run leaves the precision, ends at its start with a fraction
  of 0. A corner at the run's end halves the interval after it alone. MoveIt's validity
  callback judges each state alone, so a first state still nearer than its padding ends its
  path there.

## The planning scene

`PlanningScene` holds what MoveIt's planning scene holds besides the map, for a robot
whose links and grasp links `RobotLinks` names, and decides every rule of
motion_planning's scene members; a node parses the wire into its requests and maps its
refusals (`SceneError`), each `rejected` but a link whose pose is not known yet
(`Unplaced`).

- `apply_collision_objects` applies a batch in order, all or none. "add" places an object
  with its shapes, in place of one of the same id, whose allowed pairs stay; "move" reads
  only the id and the pose; "remove" reads only the id, and drops every pair that names
  the object. Refused: an empty id, an id that names a link or `"occupancy_map"`, an
  unknown id for "move" or "remove", an attached object, an object of no shape, and a
  shape that cannot be built, an open mesh among them. A pose's position is finite, and
  its orientation within 0.001 of unit length is normalized (`pose_from_parts`). MoveIt
  removes every object for a "remove" of an empty id; this crate refuses it.
- `apply_attached_collision_object` attaches an object no link carries to an arm's grasp
  link, where it stands, with touch links of the robot; a detach, which reads only the id,
  leaves it where it stands. Attaching an attached object and detaching one no link
  carries are refused, and so is a touch link that names an object: MoveIt takes an
  attached object's id there, and this crate lets two objects touch through
  `set_allowed_collision`. The pairs stay through both.
- `set_allowed_collision` lets an object, attached or not, touch links, objects or
  `"occupancy_map"`, the voxel map, or stops it; a pair is unordered.
- `snapshot` gives what get_planning_scene reports: the objects no link carries and the
  attached objects, each sorted by id, the attached ones in their links' frames; each
  allowed pair once, its first name before its second in sort order, the pairs sorted; and
  the clearance and the voxel size its caller gives it.

Adding, moving or attaching an object clears every voxel of the map it intersects where
it then stands (`VoxelMap::clear_intersecting`); a batch clears where it leaves each
object it adds or moves, and nothing for one it then removes. Each shape of an object
attached to a grasp link rides the child link of the chain's last joint at the grasp
point, the chain's end effector (`PlanningScene::riders`). A plan of one or more arms
meets the scene as `PlanObjects` (`PlanningScene::for_arms`): the objects the arms' grasp
links carry, and the obstacles, which with the parts of the robot the plan does not move
make the scene's `Objects`. A depth frame's intake cuts out the objects no link carries
where they stand and each arm's riders where its joints stood at the frame's stamp
(`PlanningScene::standing_shapes`, `PlanningScene::riders`).

After the same calls, the snapshot holds what MoveIt's planning scene message holds, each
object's shapes in the order the calls gave them, which the message lists primitives first
and then meshes. At configurations of OpenArm v2's left arm, `Surroundings::readings`
checks the pairs FCL measures of the arm's links, fingers and carried objects, the
objects, the voxel map as MoveIt's octomap, and the parts of the robot the arm does not
move. Each distance is no larger than FCL's, or than 0 where FCL finds the pair in
contact, and below 0 where FCL finds it in contact deeper than 1e-6 m; a link whose mesh
is concave reads nearer than FCL where the pieces of the link fill the mesh's hollows, by
up to 16.6 mm of a pair both read apart (openarm_left_link3 and the tote).
Between two shapes of objects, and between an object and the map, the distance equals
FCL's within 1e-6 m. A mesh object is measured by its triangles, so a link in its open
corner reads apart, and as a solid, so a body wholly inside it reads in contact where FCL,
which measures the triangles alone, finds the two apart
(`tests/ros_reference_planning_scene.rs`, on the [`ros_reference`](../ros_reference)
harness's `planning_scene_openarm_v2` scenario).

The departures from MoveIt, measured on that scenario:

- MoveIt's octomap keeps the voxels an object fills where it is added, moved or attached;
  this crate clears them. The scenario's calls clear 5 voxels, adds 4 of them, a move 1
  and an attach 1; over its configurations, 56 readings of a body of the arm and the map
  lie farther on the map the calls leave than on MoveIt's octomap, and 7 of them FCL finds
  in contact where this crate reads them apart: an attached object among the voxels it
  filled.
- MoveIt's path check (`PlanningScene::isPathValid`) checks a path's waypoints alone;
  `check_path` checks the runs and the corners between them too, at its clearance. On
  the scenario's 20 paths, this crate's check fails no later than MoveIt's first invalid
  waypoint: 8 pass both, 6 fail at the same waypoint in both, 4 fail in this crate's check
  before MoveIt's first invalid waypoint, and 2 fail this crate's check between two
  waypoints that MoveIt's check passes, the run through the post among them. Each of the 7
  paths that MoveIt's dense probe, a state every 1e-3 rad, finds in contact nowhere and 2
  cm or more from the objects and the octomap passes this crate's check: the 16.6 mm a
  link reads nearer than FCL, and the 1 mm the check holds, rounded up.
- MoveIt's shape mask grows a box and a cylinder by sharp planes, and a mesh as its convex
  hull with each vertex moved the offset out from the hull's center, so a face moves out
  by less than the offset; this crate rounds a primitive's edges, keeps a mesh concave and
  grows it by the offset all round. At a scale of 1, a probe this crate's cut-out holds
  and MoveIt's does not lies within the offset of a mesh. Of the scenario's 132 probes,
  MoveIt's mask holds 3 that this crate's cut-out does not and this crate's none that
  MoveIt's does not at no padding, none and 4 at an offset of 4 cm, and 1 and 2 at a scale
  of 1.2 and an offset of 2 cm.
