# motion_planning_core

Plans the paths of one serial chain of joints around a voxel map of what the robot's
cameras see and the objects of its planning scene. It knows no robot: a robot's node
gives it the chain's collision geometry, the parts its links carry, the robot's self
model where the robot has one, its links, and a goal sampler.

- `Arm` binds the chain's collision pieces (`collision_geometry`) and the bodies its
  links carry rigidly through a plan (`Rider`, `Arm::riding`): parts of the robot, such
  as a gripper finger at the gripper's opening (`ChainGeometry::carried` reads them from
  the URDF), and the shapes of the objects attached to its grasp link
  (`ArmObjects::riders`). It places each as a `Body`, a piece or a shape, and binds how
  far a joint moves any point of them, and the `SelfModel`. A rider on no joint of the
  arm is refused (`RiderOffTheArm`).
- `PlanningScene` holds the collision objects, the attached objects and the pairs
  allowed to touch, as MoveIt's planning scene holds them, and decides every rule of
  motion_planning's scene members (`apply_collision_objects`,
  `apply_attached_collision_object`, `set_allowed_collision`, `get_planning_scene`).
- `Surroundings` says whether the arm at a configuration keeps its clearance from the
  map (`occupancy_map`) and the objects that stand still (`Objects`), and within its own
  body, and reads the distance of each pair it checks (`Surroundings::readings`).
- `Goal` holds the constraints a path's last waypoint meets; `IkGoalSampler` finds
  configurations meeting them for any chain.
- `plan_arm_path` plans a `PlanRequest` on a `Scene` with OMPL (`ompl_bridge`) and
  checks the rounded path the robot runs
  (`control_core::path_timing::RoundedPath::joint_path`); `check_arm_path` checks a
  caller's path, a `CheckRequest`, with the same checks.
- The map's intake of a depth frame: `DepthCamera` reads its points, `JointHistory`
  gives the robot's joint positions at its stamp, and `insert_cut_out` inserts the
  points with the robot there and the objects, a `RobotMask`, cut out.

## Quick start

```rust
use collision_geometry::chain::{CarriedPart, Rider};
use motion_planning_core::{Arm, IkGoalSampler, PlanRequest, Scene, plan_arm_path};

// The left arm's view of the planning scene, the right grasp link where it stands.
let objects = planning_scene.for_arm("openarm_left_tcp", |grasp_link| grasp_pose(grasp_link));
let riders: Vec<Rider> = left_arm_geometry
    .carried()
    .iter()
    .map(CarriedPart::rider)
    .chain(objects.riders(&left_arm_geometry))
    .collect();
let arm = Arm::new(&left_arm_geometry, Some(&self_model)).riding(&riders)?;
let sampler = IkGoalSampler::new(left_arm_geometry.chain().clone());
let field = DistanceField::of(&map);
let waypoints = plan_arm_path(
    &arm,
    &Scene {
        map: &map,
        field: &field,
        stamp: last_frame_ns,
        // The right arm and the body where they stand.
        objects: objects.objects(&standing_parts),
    },
    &PlanRequest {
        start: measured,
        start_tolerance: [0.01; 7],
        goal: &goal,
        sampler: &sampler,
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
  1 mm. Each body of the arm, a piece of a link, a piece of a part a link carries or a
  shape of an object it carries, keeps it from every occupied voxel and from each
  obstacle, an object of the scene where it stands or where another arm holds it; and
  each object the arm carries keeps it from each part of the robot the plan does not
  move, such as the other arm and the torso (`RobotPart`).
- The robot's own body, where both bodies of a pair move: `min_clearance_m` plus 2 mm.
  Each pair of the self model keeps it.
- An attached object against the arm that carries it, where both bodies of a pair move: 0
  plus 2 mm. Each object the arm carries keeps it from each piece of the arm's links and
  of the parts they carry, outside its touch links: between an attached object and the
  links of the arm that carries it, outside its touch_links, the clearance is 0: they keep
  out of contact. Objects one arm carries are never checked against each other. From the
  rest of the robot, the other arm and the torso, which stand still through the plan, an
  attached object keeps the clearance as from an object, and so it does from the objects
  another arm holds.

MoveIt checks an attached object against the robot unpadded and never checks two objects
attached to one link against each other (moveit2 2.15.2 `collision_common.cpp`, the
always-allowed collision of bodies attached to one link). This crate departs from it
there: an attached object keeps 2 mm, the spare of its two moving bodies, from the arm
that carries it at each checked state, and the clearance it keeps from an object from the
parts of the robot the plan does not move, which MoveIt checks it against unpadded. An
object another arm holds is an obstacle here: the arm a plan moves and the object it
carries keep `min_clearance_m` from it, where MoveIt holds it as an attached body of the
other arm and checks those pairs in its self check, unpadded (`CollisionRequest`'s
`pad_self_collisions`, false by default,
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
an object the arm carries from its grasp link and its touch links, or an object another
arm holds from that arm's grasp link and its touch links. The messages of `Unreachable`
and `NoPath` name the plan's clearances in meters.

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

A motion is walked by conservative advancement. A joint turning by an angle moves a point
at most that angle times the point's distance from the joint, so how far a step moves any
point of a link or of a rider, a part or an object it carries, follows from each joint's
reach over every configuration (`Arm::reach`), each body bounded by its bounding sphere; a
prismatic joint lengthens the arm past it by up to its farthest limit. At each checked
state the walk measures the arm's distance from the map and the objects, the least
distance within its own body, and the least between an attached object and the arm that
carries it. It measures the bodies against the map nearest screen first, and sets aside a
body whose screen, a voxel whose bounding sphere, or a pair whose spheres lie apart past
the least distance it has found: what it sets aside lies no nearer. A body's distance from
an occupied voxel or an object, which stand still, falls by at most how far the body
moves, and the distance of a pair of its own body, each of whose bodies moves, by at most
twice that. So the walk steps no farther than the least spare: the distance from the map
and the objects above `min_clearance_m`, half the own body's above it, and half the
distance between an attached object and the arm that carries it, and at most `MAX_STEP_M`,
1.5 cm. Every state between two checked states keeps the clearance asked of each term. A
state from which the step would fall under 1 mm falls short of the plan's clearance and
ends the motion: near a surface at the clearance asked of its term the spare, and the step
with it, shrinks toward 0, and the floor bounds a motion's checks by its length over 1 mm.
The cap keeps a checked state within 1.5 cm of any point of a run, where a corner's repair
adds its waypoints, and the distance from the map is measured exactly up to
`min_clearance_m` plus 1.5 cm, past which a body's bounding sphere bounds it, and so is
the distance from an object or within the robot's own body. Each arc of the rounded path
is walked the same way along the path: a step of its length moves a point at most that
times the arc's lever, the sum over the joints of the largest share of the path's unit
tangent the joint has on the arc times its reach, and at most the joints' reach in their
L2 norm (`Arm::lever_along`). The voxel's size needs no inflation, for a state is measured
against each occupied voxel's whole box, which holds every point the voxel stands for.

## The start

A joint that starts outside its limits by no more than its start tolerance moves onto
the limit; one farther out refuses the plan, naming the joint by its URDF name
(`StartFault::OutsideLimits`), and a caller asks `start_inside_limits` before it takes
a request. A start in contact with an obstacle or with its own body refuses it too, and
the refusal names the pair (`StartFault::InCollision`, `Contact`).

The robot runs a path from its measured joints, the start. A start closer than the plan's
clearance of a term, such as one a millimeter from an obstacle, departs along the straight
line from the start by the start rule, each term judged on its own, until it keeps that
clearance. The rule holds at the states the walk checks: each lies no nearer than the
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
joint's reach (`Arm::reach`), as MoveIt's execution starts within its allowed start
tolerance and checks nothing more.

## The goal

A goal holds joints within tolerances above and below their targets, or holds the grasp
point, a link of the chain or one fixed below it, or a part a link carries through a plan
(`Frame`; `Frame::Part`, from a part's `Rider`, stands where the clearance checks place
the part): its origin within a distance of a position of the robot frame, and its
orientation within an angle of a target, the angle of the rotation between them, or, with
a free axis, the angle between that axis and its target direction. Each tolerance is a
`control_core` `PositiveFinite`. A goal that mixes joints with poses, names a joint or a
frame twice in one list, or holds the position of one frame and the orientation of another
is refused (`GoalError`): a goal holds one frame, whose pose the sampler reaches for.
MoveIt keeps one sampler for a goal on several links, the link whose constraints leave the
least volume, and accepts a sample that meets the others by chance.

The plan samples its goals as MoveIt's `ConstrainedGoalSampler` samples a goal's states
(moveit2 2.15.2 constrained_goal_sampler.cpp; `sample_goals`). A working state starts at
the plan's start. Each goal sample sets out from it with MoveIt's 4 attempts
(`max_state_sampling_attempts`), the plan's clearance check deciding inside the sample, as
MoveIt's validity callback does. The goal a sample finds becomes the working state, so the
goals of a short pose move gather about the first one, which lies on the start's shoulder
and wrist branches where the first attempt from the start reaches the pose; a goal joins
the goals unless it equals one of them. Sampling ends at `goal_samples` goals, as MoveIt's
`max_goal_samples` (10 by default) ends it; once the samples since the last goal found
reach 1000 less the goals found so far, as MoveIt's `max_goal_sampling_attempts` (1000)
bounds the samples its sampling function makes before it returns a goal, counting from the
goals OMPL has taken; or before a sample once the deadline passes or the stop flag is set.
MoveIt samples on a thread beside its planner and ends once the planner has a path; this
crate samples the goals, then plans to them.

`IkGoalSampler` makes a sample. For a goal of joints it draws one configuration inside the
joint limits, the targeted joints inside their targets, as MoveIt's
`JointConstraintSampler` does. For a pose, as MoveIt's `IKConstraintSampler`, each attempt
draws a pose of the goal's frame inside the goal and searches for it by MoveIt's KDL
inverse kinematics, ported (kdl_kinematics_plugin.cpp at its defaults): the first attempt
from the working state, each later one from a configuration drawn inside the limits. A
descent converges where the frame's position and orientation both lie within 1e-5 m and
rad of the pose; a step that does not lower its error is cut, a joint clipped at its limit
weighs 0.01 in the next step, and a stuck descent steps off by a drawn turn, for at most
500 steps. A search re-seeds from a drawn configuration after each descent. KDL's search
ends on a clock, the 0.05 s move_group gives each inverse kinematics call; this one starts
no descent once its descents have made 9274 passes of KDL's loop, 0.05 s over the mean
time of a pass in the reference image, so a seed gives the same goals on every machine.
From the seeds of seven descents of MoveIt's KDL plugin recorded in the reference image,
converged, stuck and at the 500 step cap, the port's descents end within 1e-6 rad of
KDL's, after as many passes (`tests/kdl_descents.json`). The grasp point, and a link that
the chain's every joint moves and that is fixed below the last joint's link, are the tip
of MoveIt's solver or a link fixed to it, and the search solves for them as KDL does. A
link the chain's later joints do not move, and a part a link carries such as a finger,
which hangs on a joint of its own, depart from MoveIt, which has no inverse kinematics for
them: MoveIt builds no IK sampler for a link that is neither the tip nor fixed to it
(`IKConstraintSampler::loadIKSolver`, default_constraint_samplers.cpp:386-439), so its
constraint sampler manager selects none (constraint_sampler_manager.cpp:395) and its goal
sampler draws states uniformly and keeps those that meet the goal by chance
(constrained_goal_sampler.cpp:71-72, :158-166). This sampler descends toward what the goal
names, with KDL's position-only and approximate-solution options applied
(kdl_kinematics_plugin.cpp:344-350, :389), and the goal's constraints decide. A plan whose
sampling finds no goal fails `Unreachable`, saying so; one whose every configuration that
meets the goal is in collision fails `Unreachable` and says that instead, how many such
configurations the sampling rejected, naming, of the configuration that falls short least,
the nearest pair of the robot's own body, of an attached object and the arm that carries
it, or of a body of the arm and the map, an object or a part of the robot, of each
clearance it falls short of and how far apart they stand.

On 200 drawn reachable poses of OpenArm v2's left arm, with the configurations of every
other branch invalid, one goal sample from all joints at zero reaches each shoulder and
wrist branch that `srs_model`'s closed form finds on each of them, 144 of which have two
(`tests/goal_coverage.rs`). The goal sampling reaches each of 10 drawn reachable poses of
the forearm, of the wrist link, and of a part the hand carries.

## The plan and its path

The request's deadline bounds the plan as MoveIt's allowed planning time bounds a
request: the goal sampling samples up to `goal_samples` goals, ending before a sample once
the deadline passes, OMPL makes the request's `attempts` in what remains before it, as
MoveIt makes a request's `num_planning_attempts` (ompl_bridge's README, Attempts): each
from its own seed, under the whole of that time and the whole iteration bound, their
paths joined by OMPL's path hybridization, and OMPL's simplifier takes what remains after
the attempts, at most 1000 iterations, as MoveIt simplifies in the allowed planning time
less the plan's (moveit2 2.15.2 `ModelBasedPlanningContext::solve` calls
`simplifySolution` with that time, whatever is left of it). OMPL's simplifier runs its
first pass however little time it has (`PathSimplifier::simplify`, `atLeastOnce`), so a
planner that plans until its time ends, such as RRT*, has its path simplified once, past
the deadline. A corner's repair runs only before the deadline: when a corner of the
simplified path rounds short of the clearances once the deadline has passed, the plan
answers its planned path if that passes its own check, and fails `NoPath` otherwise. The
hybrid of several attempts and its checks, and the check of the rounded path, run to
their end past the deadline. OMPL plans with the request's planner and iteration bound,
seeded from the request's seed and the map's stamp, so the same request on the same map
gives the same path when the plan ends before its deadline, for every planner but PRM and
PRM*, with any number of attempts. OMPL's simplifier shortens the path.

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

The planner keeps every state of its paths but the start at the plan's clearance: its
check of a motion fails one whose first state, the start excepted, falls short, and one
from the start that ends before each term keeps its clearance. OMPL's simplifier keeps
states it checks only as points of a checked motion, so the plan answers the simplified
path when the path the robot runs through it passes the checks above, and the planned
path when only that one passes. MoveIt checks the simplified path alone, and answers
INVALID_MOTION_PLAN when it fails.

## Checking a path

`check_arm_path` checks the path execute_joint_path runs from a caller's start through its
waypoints with the checks of a plan's answer: the start inside its limits by its tolerance
and out of contact, each waypoint inside the joint limits and, after the start, at the
plan's clearances, each run as the plan walks the runs of its answer, the run whose path
from the start lies on the straight line from the start by the start rule, and each arc of
the rounded path as the plan walks its arcs. The walks follow the run into the first
waypoint outside a limit up to where it leaves the limits, and go no farther. It answers
the first fault along the path (`PathFault`): the start's, a waypoint that holds a joint,
by its URDF name, outside its limits, or a state short of the plan's clearance, where it
lies (`Place`: at a waypoint, on a run, or on the arc at a corner) and how near it comes
to the map and the objects, within the robot's own body, and between an attached object
and the arm that carries it. It answers too the most waypoints, from the first, whose
path, run on its own, passes: the waypoints up to the one whose run or corner fails, the
start counted as waypoint 0, or before the waypoint that fails. A corner's arc runs before
the rest of the run from its waypoint, and that run before the waypoint it ends at. Each
walk of a run or of a corner starts only before the check's deadline, and a check that
reaches its deadline fails `TimeEnded`.

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
object it adds or moves, and nothing for one it then removes. A plan of one arm meets the
scene as `ArmObjects` (`PlanningScene::for_arm`): the objects its grasp link carries,
each shape a rider of the child link of the chain's last joint at the grasp point, the
chain's end effector (`ArmObjects::riders`), and the obstacles, which with the parts of
the robot the plan does not move make the scene's `Objects`. A depth frame's intake cuts
out the objects no link carries where they stand and each arm's riders where its joints
stood at the frame's stamp (`PlanningScene::standing_shapes`, `PlanningScene::riders`).

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
  `check_arm_path` checks the runs and the corners between them too, at its clearance. On
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
