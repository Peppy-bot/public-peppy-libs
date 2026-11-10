# chain_kinematics

Forward kinematics, the geometric Jacobian, damped resolved-rate steps and
MoveIt's KDL inverse kinematics for **any serial chain** read from a URDF. Generic
over the number of joints, with no assumption about topology: a seven-axis arm, a
five-axis arm, a leg, a pan-tilt head. Pure Rust; no hardware, messaging, or async
dependencies.

## Quick start

```rust
use chain_kinematics::{Chain, ChainSpec, JointSelection};

let robot = urdf_rs::read_from_string(urdf)?;
let chain = Chain::<5>::from_urdf(&robot, &ChainSpec {
    base_link: None,                       // None = the URDF root
    tip_link: "gripper_frame_link",        // named, not discovered
    joints: JointSelection::Named(&[
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll",
    ]),
})?;

let ee = chain.at(&[0.0; 5]).ee_pose();
let j = chain.at(&[0.0; 5]).jacobian();
```

## What the caller supplies

A parsed URDF and a `ChainSpec`. Two of its three fields are deliberately
explicit rather than inferred:

- **`tip_link` is named.** Discovering a tip by walking out and counting joints
  only ever works for the arm the rule was written for.
- **`joints` may be named, and then the order is the order of `q`.** Inferring
  joint order from the URDF's own ordering is luck rather than a contract: `q`'s
  order is usually the robot's wire order, which no URDF knows about. A movable
  joint on the path that is not named is frozen at zero and folded in as a fixed
  offset, so a gripper branching off the last link does not have to be actuated.

`N` is checked against the URDF once, at load. After that every type is
fixed-size and a control tick allocates nothing.

## What it refuses at load, and why

Each of these would otherwise produce a plausible wrong answer rather than an
error, so it is a refusal at construction.

| Refused | What you would get otherwise |
|---|---|
| A joint named twice in `JointSelection::Named` | The later slot wins, so the earlier entry of `q` moves nothing: the caller commands two joints and the arm honours one |
| An actuated joint with no usable range (absent `<limit>`, or an infinite or inverted one) | A joint that turns freely confined to an invented range, so the chain refuses motion the mechanism has |
| A `<mimic>` coupling with either end among the actuated joints | `q` poses only the joints it names, so the coupled joint stays behind and the reported tip is a pose the mechanism cannot hold. A coupling clear of the chain, as a gripper's two fingers are, is left alone |
| A tip that is not below the base, or a joint that is not on the path | A frame composed from an unrelated branch |
| A tool link reached through a joint that moves | A "fixed" tool offset that tracks the gripper opening |
| A URDF that is not a single tree, or `N` disagreeing with the chain's joint count | Every fixed-size type below built on a count that is not the robot's |

## What it does

| | |
|---|---|
| `Chain::at(q)` | pose the chain; pure, so `Chain` is `Send + Sync` |
| `Posed::{ee_pose, tip_pose, link_pose_world}` | frames, in the base frame the spec named |
| `Posed::jacobian` | the 6xN geometric Jacobian, revolute and prismatic columns |
| `Posed::point_world_jacobian` | the same, for an arbitrary witness point - what a collision-distance gradient needs |
| `Posed::{mass, com_world, inertia_world}` | per-segment rigid-body data |
| `Posed::{gravity_torques, coriolis_torques}` | feedforward dynamics, revolute and prismatic joints alike, with everything each segment carries folded in |
| `damped_pseudo_inverse` | `Jᵀ(J Jᵀ + λ²I)⁻¹`, defined everywhere including at singularities |
| `IkSolver::search` | the inverse kinematics of a `ChainFrame`, as MoveIt's KDL plugin searches: descents from a seed, then from drawn configurations, until the caller accepts an end or the budget is spent |
| `rate_step` | one damped resolved-rate step under a velocity budget and joint limits, holding a joint it would carry past a limit and solving the task again on the rest |
| `rate_step_toward` | the step toward a pose, capped at an end-effector speed budget (`EeCaps`, `StepLimits`), within the joint limits |
| `interpolate` | the straight line between two poses |
| `walk_line` | a walk along a line by inverse kinematics from a seed: each step solved from the state before by a `LineSolver`, kept or refused by the caller, and with a `LinePrecision` held to the line between states |
| `clamp_into` / `all_within` / `pose_miss` | joint-limit and pose-distance helpers |
| `take_targets` / `Limit::take_target` / `TARGET_SLACK` | a joint target as a robot takes it: a joint at most 0.001 rad outside its limit runs at the limit, one farther out is refused by its joint, position and limits (`OutsideLimits`) |

## Inverse kinematics

`IkSolver` is MoveIt's KDL inverse kinematics (`kdl_kinematics_plugin.cpp` of moveit2
2.15.2 at its defaults, and orocos_kdl 1.5.2) for one frame of a chain. The frame is a
`ChainFrame`: the end effector, or an offset on a segment's link.

```rust
let solver = IkSolver::new(&chain, ChainFrame::EndEffector, Aims::Both, Ends::Converged);
let found = solver.search(seed, &target, &mut draws, &|q| valid(q), SEARCH_ITERATIONS);
```

A search descends from the seed toward the target, a world-frame `Isometry3`, then from
a configuration drawn inside the limits after each descent, and answers the first end its
caller's callback accepts. A descent converges where the frame's position and orientation
both lie within 1e-5 m and rad of the target; a step that does not lower its error is cut,
a joint clipped at its limit weighs 0.01 in the next step, and a stuck descent steps off by
a drawn turn, for at most 500 steps.

KDL's search ends on a clock, the 0.05 s move_group gives each inverse kinematics call. This
one starts no descent once its descents have made the caller's budget of passes of KDL's
loop, and `SEARCH_ITERATIONS`, 9274, is 0.05 s over the mean time of a pass in the
reference image, so a seed gives the same answer on every machine. The numbers come from the
caller's `RandomSource`, `Draws` for a SplitMix64 stream that a seed fixes on every machine,
each joint of a restart in turn and each joint of a step off a singularity in turn.

Two of KDL's options are the caller's. `Aims` names what a descent brings to the target:
the position, the orientation or both, `position_only_ik` generalized, and `PositionFirst`,
both with the orientation's rows weighed at 0.01, `orientation_vs_position`.
`Ends::Approximate` offers the callback each descent's end, converged or not, as
`return_approximate_solution` does.

## Walking a line

`walk_line` walks the line a caller traces, a pose for each blend such as
`interpolate` gives between two poses, through the blends it is given, from a
state of the caller's. A `LineSolver` solves each step from the state before
it; `srs_model` walks its lines with one descent of `IkSolver::search` per
step. The caller gets each
step with the state before it and keeps it or refuses it. The walk ends at the
first step that has no solution, cannot keep the precision, or is refused, and
the caller has kept every step before it.

A `LinePrecision` is MoveIt's `CartesianPrecision`: the frame at each step's
joint midpoint stands within its translational and rotational bounds of the
line's midpoint, or the step is halved, its midpoint solved from the joint
midpoint and each half refined in turn, down to intervals of the precision's
resolution (MoveIt's `validateAndImproveInterval`). A solution that jumps
branch inside a step keeps the pose at both ends while its joint midpoint
leaves the line at every halving, so the walk ends before the jump. Without a
precision a step is its own state alone and the walk allocates nothing.

## The damped step

A discrete IK walk cannot cross the singular surface between two solution
branches. The damped law can: the damping bounds the joint rates while the task
error carries the chain across, deviating from the straight line only where the
geometry forces it.

`rate_step_toward` caps the task error at an end-effector speed budget, so a
target meters away produces the same bounded step as one millimeters away.

A joint the step would carry past its limit is held on it and the task solved
again on the joints left free, so the other joints take up the motion.

The step always returns a configuration. An unreachable target moves toward the
closest pose the step can find and saturates there: a teleoperator pushing past
the edge feels a wall.

## Layout

```text
src/
  lib.rs        Chain, ChainSpec, JointSelection, Posed, Limit, and the joint-limit and pose-distance helpers
  tree.rs       the URDF arena: links, joints, adjacency, and the walks over it
  jacobian.rs   the Jacobian's inverses, generic over N
  chain.rs      the serial chain and its posed view: forward kinematics, the Jacobian, limits
  rate.rs       one damped resolved-rate step, holding the joints it would carry past their limits, and the distance between poses
  line.rs       the straight line between two poses, and the walk along a line by IK from a seed
  ik.rs         MoveIt's KDL inverse kinematics of a frame of a chain: the search and its descents
  dynamics.rs   gravity and Coriolis feedforward over the posed chain
  error.rs      what can go wrong building a chain
  payload.rs    what each segment carries beyond its own link, lumped in
```

## Testing

```sh
cargo test                                     # every suite, each property at 100 cases
HEGEL_TEST_CASES=100000 cargo test --release   # every property at 100000 cases
```

The forward kinematics, the Jacobian, its damped inverse,
the dynamics and the inverse kinematics' search are checked as [Hegel](https://hegel.dev)
properties on drawn robots: serial chains of one to seven revolute or prismatic
joints, each placed anywhere on its parent, about any axis, with any range, and
each link carrying any mass and inertia. The generators of robots live in
`tests/common/properties.rs`, and those `srs_model` shares, configurations,
directions and rotations, in [`kinematics_properties`](../kinematics_properties).
The URDF models of a five-joint SO-101 and a seven-joint OpenArm carry the
claims that belong to a robot: the damped step reaching a pose on both, a wrong joint
count refused at load, and the speed caps held at the tip. No hardware is
involved; qualifying the law on a physical arm is a separate exercise.

`tests/refusals.rs` covers the table above: every rule is checked both ways, so a
refusal that would also reject a valid chain fails the suite.
`tests/line_walk.rs` checks the walk on one-joint solvers whose every answer
is known: each step solved from the state before, the end at the first step it
cannot take, and the precision's halving, which a solution jumping a whole turn
cannot pass.
`tests/ik_search.rs` checks the search: an answer stands at its target inside the
limits, a descent aims at what its solver names, and the callback and the budget
end the search. The evidence for the descents against MoveIt is beside them in
`src/ik.rs`: from the seeds of seven descents of MoveIt's KDL plugin recorded in
the reference image on OpenArm v2's left arm, converged, stuck and at the 500 step
cap, each descent ends within 1e-6 rad of KDL's, after as many passes
(`tests/fixtures/kdl_descents.json`).
`tests/ros_reference_forward_kinematics.rs` is the evidence for the forward
kinematics against MoveIt: at every configuration of the forward kinematics
scenarios of the [ROS reference harness](../ros_reference), the OpenArm v2 arms
and SO-101, each link pose agrees with the global link transform of MoveIt's
robot state within 16 units of `f64::EPSILON * max(1, |x|)`. It reads the
harness's committed reference outputs, so it runs with no ROS on the machine.
