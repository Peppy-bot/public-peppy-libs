# motion_planning_core

Plans the paths of one serial chain of joints around a voxel map of what the robot's
cameras see. It knows no robot: a robot's node gives it the chain's collision geometry,
the parts its links carry, the robot's self model where the robot has one, and a goal
sampler.

- `Arm` binds the chain's collision pieces (`collision_geometry`) and those of the parts
  its links carry rigidly through a plan (`Rider`, such as a gripper finger at the
  gripper's opening, `Arm::riding`; `ChainGeometry::carried` reads them from the URDF),
  how far a joint moves any point of them, and the `SelfModel`. A rider on no joint of
  the arm is refused (`RiderOffTheArm`).
- `Surroundings` says whether the arm at a configuration keeps its clearance from the
  map (`occupancy_map`) and between the pairs of its self model.
- `Goal` holds the constraints a path's last waypoint meets; `IkGoalSampler` finds
  configurations meeting them for any chain.
- `plan_arm_path` plans a `PlanRequest` on a `Scene` with OMPL (`ompl_bridge`) and
  checks the rounded path the robot runs
  (`control_core::path_timing::RoundedPath::joint_path`); `check_arm_path` checks a
  caller's path, a `CheckRequest`, with the same checks.
- The map's intake of a depth frame: `DepthCamera` reads its points, `JointHistory`
  gives the robot's joint positions at its stamp, and `insert_cut_out` inserts the
  points with the robot there, a `RobotMask`, cut out.

## Quick start

```rust
use collision_geometry::chain::{CarriedPart, Rider};
use motion_planning_core::{Arm, IkGoalSampler, PlanRequest, Scene, plan_arm_path};

let fingers: Vec<Rider> =
    left_arm_geometry.carried().iter().map(CarriedPart::rider).collect();
let arm = Arm::new(&left_arm_geometry, Some(&self_model)).riding(&fingers)?;
let sampler = IkGoalSampler::new(left_arm_geometry.chain().clone());
let field = DistanceField::of(&map);
let waypoints = plan_arm_path(
    &arm,
    &Scene { map: &map, field: &field, stamp: last_frame_ns },
    &PlanRequest {
        start: measured,
        start_tolerance: [0.01; 7],
        goal: &goal,
        sampler: &sampler,
        goal_samples: NonZeroUsize::new(64).unwrap(),
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
  joint positions (`PosedChain`), the parts its links carry, such as a gripper's fingers
  at the gripper's opening, and the body fixed around the chains, each piece grown as
  `collision_geometry` grows it for MoveIt's distance rule. `insert_cut_out` inserts the
  scan into the voxel map with the mask cut out (`occupancy_map::RobotCutout`), the
  URDF root at the origin of the scan's frame: a point inside it marks nothing, and its
  ray clears the space before it.

## A valid state

A state keeps the plan's clearance: `min_clearance_m` plus 1 mm from every occupied voxel,
and plus 2 mm between each pair of the self model; the next section says why. Every piece
of a link and of a part a link carries keeps it. The messages of `Unreachable` and
`NoPath` name the plan's clearance in meters. Each piece's bounding sphere screens first,
against the map's distance field: the field reads the distance between voxel centers,
within half a voxel's diagonal of the sphere center's own distance to the nearest occupied
center, which lies within another half diagonal of that voxel's box. A piece whose sphere
does not clear by the screen is measured exactly, its hull against the box of each
occupied voxel near it, by GJK. The screen never changes the hulls' verdict. A voxel no
ray has reached holds nothing to collide with. A distance that is not a number, from GJK
or from the self model, counts as short of every clearance, and a self model that answers
none is in contact.

## A valid motion

A motion is walked by conservative advancement. A joint turning by an angle moves a point
at most that angle times the point's distance from the joint, so how far a step moves any
point of a link or of a rider follows from each joint's reach over every configuration
(`Arm::reach`); a prismatic joint lengthens the arm past it by up to its farthest limit.
At each checked state the walk measures the arm's distance from the map and the least
distance between the pairs of the self model. It measures the pieces nearest screen first,
and sets aside a piece whose screen, or a voxel whose bounding sphere, lies past the least
distance it has found: what it sets aside lies no nearer. A link's distance from an
occupied voxel, which stands still, falls by at most how far the link moves, and the
distance of a pair, each of whose links moves, by at most twice that. So the walk steps no
farther than the least spare: the map's distance above `min_clearance_m`, and half the
pair's, and at most `MAX_STEP_M`, 1.5 cm. Every state between two checked states keeps
`min_clearance_m`. A state from which the step would fall under 1 mm falls short of the
plan's clearance and ends the motion: near a surface at `min_clearance_m` the spare, and
the step with it, shrinks toward 0, and the floor bounds a motion's checks by its length
over 1 mm. The cap keeps a checked state within 1.5 cm of any point of a run, where a
corner's repair adds its waypoints, and the distance from the map is measured exactly up
to `min_clearance_m` plus 1.5 cm, past which a piece's bounding sphere bounds it. Each arc
of the rounded path is walked the same way along the path: a step of its length moves a
point at most that times the arc's lever, the sum over the joints of the largest share of
the path's unit tangent the joint has on the arc times its reach, and at most the joints'
reach in their L2 norm (`Arm::lever_along`). The voxel's size needs no inflation, for a
state is measured against each occupied voxel's whole box, which holds every point the
voxel stands for.

## The start

A joint that starts outside its limits by no more than its start tolerance moves onto
the limit; one farther out refuses the plan, naming the joint by its URDF name
(`StartFault::OutsideLimits`), and a caller asks `start_inside_limits` before it takes
a request. A start in contact with an obstacle or with its own body refuses it too, and
the refusal names the pair (`StartFault::InCollision`, `Contact`).

The robot runs a path from its measured joints, the start. A start closer than the plan's
clearance of a term, the map's or the robot's own body's, such as one a millimeter from
an obstacle, departs along the straight line from the start by the start rule, each term
judged on its own, until it keeps that clearance. The rule holds at the states the walk
checks: each lies no nearer than the farthest checked state before it, or than the
plan's clearance where that is nearer, to the rounding of the arithmetic, 1e-9 m. Between
two checked states the term comes nearer by at most half its room. The start sets each
term's floor: 0 when the start is nearer than `min_clearance_m`, and when it is at least
that far, `min_clearance_m` or the start's distance less twice its moving bodies times
the shortest step, 0.1 mm, whichever is lower. The term's room is its farthest checked
distance above the floor, and a step moves no point farther than the room over twice its
moving bodies, so between two checked states it stays above the middle of its floor and
its farthest distance: from a start below `min_clearance_m` above half that distance, so
nothing touches; from one at least 0.2 mm (map) or 0.4 mm (own body) above it, above
`min_clearance_m`; and from one nearer above it, at most 0.1 mm (map) or 0.2 mm (own
body) below it. A motion ends where its step falls under 0.1 mm. A term the start keeps
the clearance of keeps it as on any run. A run whose path from the start lies on the
straight line from the start to the run's end, past waypoints on it such as one a
corner's repair adds to the first run, is walked by the rule from the start, and so is a
planner's motion whose first state lies on the straight line from the start to its end;
any other run is walked by its spare from its first state, so one that leaves a state
short of the plan's clearance fails there.

Run from the plan's start, the path keeps `min_clearance_m`. Run from up to a tolerance
from the start in each joint, as a robot's controller allows, it comes nearer anything
near its start by at most the sum over the joints of the tolerance times the joint's
reach (`Arm::reach`), as MoveIt's execution starts within its allowed start tolerance and
checks nothing more.

## The goal

A goal holds joints within tolerances above and below their targets, or holds the grasp
point, a link of the chain, or a part a link carries rigidly through a plan (`Frame`; a
part's `Frame::from` its `Rider` stands where the clearance checks place the part):
its origin within a distance of a position of the robot frame, and its orientation
within an angle of a target, the angle of the rotation between them, or, with a free
axis, the angle between that axis and its target direction. Each tolerance is a
`control_core` `PositiveFinite`. A goal that mixes joints with poses, names a joint or a
frame twice in one list, or holds the position of one frame and the orientation of
another is refused
(`GoalError`): a goal holds one frame, whose pose the sampler reaches for. MoveIt keeps
one sampler for a goal on several links, the link whose constraints leave the least
volume, and accepts a sample that meets the others by chance.
`IkGoalSampler` draws up to `goal_samples` configurations, until the plan's deadline
passes, inside the joint limits, the targeted joints inside their targets, and descends
from each toward a pose drawn inside the goal: by `chain_kinematics`' inverse kinematics
for the grasp point, and for a link by damped least squares on the link's Jacobian, each
column taken as the change of the link's pose over a small turn of its joint. It keeps
the distinct ones that meet the goal. A plan whose sampler finds none fails
`Unreachable`, saying so; one whose every configuration is in collision fails
`Unreachable` and says that instead, naming, of the configuration that falls short least,
the pair of the robot's own links, or the link nearest the map, of each clearance it falls
short of and how far apart they stand.

On 200 drawn reachable poses of OpenArm v2's left arm, the sampler reaches every
shoulder and wrist branch that `srs_model`'s closed form finds on 199 of them, 144 of
which have two (`tests/goal_coverage.rs`). It reaches each of 10 drawn reachable poses
of the forearm, of the wrist link, and of a part the hand carries.

## The plan and its path

The request's deadline bounds the plan as MoveIt's allowed planning time bounds a
request: the goal sampler draws up to `goal_samples` configurations, ending early once the
deadline passes, OMPL makes the request's `attempts` in what remains before it, as MoveIt
makes a request's `num_planning_attempts` (ompl_bridge's README, Attempts): each from its
own seed, under the whole of that time and the whole iteration bound, their paths joined
by OMPL's path hybridization, and OMPL's simplifier takes what remains after the
attempts, at most 1000 iterations, as MoveIt simplifies in the allowed planning time less
the plan's (moveit2 2.15.2 `ModelBasedPlanningContext::solve` calls `simplifySolution`
with that time, whatever is left of it). OMPL's simplifier runs its first pass however
little time it has (`PathSimplifier::simplify`, `atLeastOnce`), so a planner that plans
until its time ends, such as RRT*, answers its path simplified once, past the deadline.
A corner's repair runs only before the deadline, and a corner whose rounding fails once
the deadline has passed fails `NoPath`. The hybrid of several attempts and its checks,
and the check of the rounded path, run to their end past the deadline. OMPL plans with
the request's planner and iteration bound, seeded from the request's seed and the map's
stamp, so the same request on the same map gives the same path when the plan ends before
its deadline, for every planner but PRM and PRM*, with any number of attempts. OMPL's
simplifier shortens the path.

The caller's flag `stop` ends a plan at any part of it, and the plan fails `Stopped`:
the goal sampler draws no further configuration, OMPL's planners and simplifier end at
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

The planner keeps every state of its paths but the start at the plan's clearance: its
check of a motion fails one whose first state, the start excepted, falls short, and one
from the start that ends before each term keeps its clearance. OMPL's simplifier keeps
states it checks only as points of a checked motion, so the plan answers the simplified
path when the path the robot runs through it passes the checks above, and the planned
path when only that one passes. MoveIt checks the simplified path alone, and answers
INVALID_MOTION_PLAN when it fails.

## Checking a path

`check_arm_path` checks the path execute_joint_path runs from a caller's start through
its waypoints with the checks of a plan's answer: the start inside its limits by its
tolerance and out of contact, each waypoint inside the joint limits and, after the
start, at the plan's clearances, each run as the plan walks the runs of its answer, the
run whose path from the start lies on the straight line from the start by the start
rule, and each arc of the rounded path as the plan walks its arcs. The walks follow the
run into the first waypoint outside a limit up to where it leaves the limits, and go no
farther. It answers the first fault along the path
(`PathFault`): the start's, a waypoint that holds a joint, by its URDF name, outside its
limits, or a state short of the plan's clearance, where it lies (`Place`: at a waypoint,
on a run, or on the arc at a corner) and how near it comes to the map and between the
robot's own links. It answers too the most waypoints, from the first, whose path, run on
its own, passes: the waypoints up to the one whose run or corner fails, the start
counted as waypoint 0, or before the waypoint that fails. A corner's arc runs before the
rest of the run from its waypoint, and that run before the waypoint it ends at. Each walk
of a run or of a corner starts only before the check's deadline, and a check that
reaches its deadline fails `TimeEnded`.
