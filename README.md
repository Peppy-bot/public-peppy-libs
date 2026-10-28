# public-peppy-libs

Shared infrastructure libraries for Peppy nodes. Every library here is an independent package, pulled as a git dependency by the nodes that need it.

Peppy nodes live in separate repositories under the nodes hub. Shared code that is needed across multiple independent node repos cannot be a path dependency inside one node's repo: it needs a central place. This repository is that central place.

## One package, one workspace root

Every Rust library here carries an empty `[workspace]` table in its manifest, which makes the package its own workspace root. Cargo's search for a root ends at the manifest whatever sits above the checkout, so the package builds the same however a consumer lays the repository out. Each one commits its `Cargo.lock`, and CI builds and tests it `--locked`, so the lock is the dependency graph a green run attests to.

A library depends on the registry and on its siblings here, by relative path. Nothing here reaches outside the repository, which is what lets a consumer take one library without building the rest of the fleet.

## License

Everything here is licensed under the [Apache License, Version 2.0](./LICENSE).

## Libraries

| Library | Language | Purpose |
|---|---|---|
| [openarm_can](./openarm_can) | Rust | Safe wrapper around the `openarm_can` C++ library for driving the physical OpenArm hardware over CAN (`ArmCan` / `GripperCan`, Damiao motor types, OpenArm v10 constants) |
| [chain_kinematics](./chain_kinematics) | Rust | Forward kinematics, the geometric Jacobian, and damped resolved-rate inverse kinematics for any serial chain read from a URDF - generic over the number of joints, no topology assumed. The Rust sibling of `chain_kinematics_py` |
| [kinematics_properties](./kinematics_properties) | Rust | The [Hegel](https://hegel.dev) generators the property tests of `chain_kinematics`, `srs_model` and `collision_geometry` share: robots of any serial shape, angles, unit vectors, rotations, configurations inside a chain's limits, and the tolerance the properties compare at. A dev-dependency of those crates |
| [ros_reference](./ros_reference) | Rust | The ROS reference harness: the scenarios that a parity test and a ROS reference both read (forward kinematics, with MoveIt's robot state as the reference), the reference outputs that the reference image, ROS 2 Lyrical in Docker, wrote for them, with their digests, and the reading of both for the parity tests of the other crates. A dev-dependency of those crates; ROS runs only in the reference image |
| [srs_model](./srs_model) | Rust | Kinematics and dynamics for a 7-DOF SRS arm: FK, closed-form arm-angle (Shimizu) IK, gravity/Coriolis feedforward, and Jacobians. Robot-agnostic: all geometry derives from the supplied URDF. Builds on `chain_kinematics`; pure Rust, no hardware or messaging deps |
| [collision_geometry](./collision_geometry) | Rust | Collision geometry for any robot: convex pieces fitted to the URDF collision elements (clip regions for a concave body), GJK/EPA distance between convex shapes, and the pieces of any serial chain placed at a joint configuration, each with a bounding sphere, with MoveIt's padded robot-filter test of whether a point lies inside a link or the chain. Builds on `chain_kinematics`, and on Qhull (C, built from source; its build needs libclang) for the exact convex hulls; no hardware or messaging deps |
| [self_collision_model](./self_collision_model) | Rust | Runtime self-collision detection for a robot with two 7-DOF SRS arms: the minimum distance between its bodies over the checked pairs and the analytic distance gradient, for scaling commanded motion near contact. Builds on `collision_geometry` and `srs_model`; no hardware or messaging deps |
| [workspace_core](./workspace_core) | Rust | Where a robot can work, judged alike on the robot and in a simulation: the measuring grid and limits, the grasp directions, the perception camera rule, the field-of-view test, the reach and view verdicts, the largest workable rectangle and the one-line messages, and the `workspace:v1` answer of a robot from its design around the reach its own solver gives (request parsing, reach memo, answer composition). Pure Rust on plain arrays, no math library, no hardware or messaging deps |
| [openarm_description](./openarm_description) | Rust | The OpenArm v1.0 and v2.0 robot descriptions as a single embedded source of truth: the URDFs, the collision meshes (behind the `meshes` feature), the regions that decompose the torso mesh into convex pieces, the elbow singularity control margin, and `Side` and `ArmPair` naming each arm. Pure data: no kinematics or solver deps, so any consumer (`srs_model`, a viz tool, a sim bridge) builds from `urdf()` itself |
| [sim_bridge_core](./sim_bridge_core) | Rust | raw-to-peppygen pipelines for Isaac Sim and MuJoCo bridge nodes; the node supplies the peppylib transport |
| [control_core](./control_core) | Rust | Shared control-loop primitives for the robot nodes and the kinematics crates: fixed-rate pacing, the rate-limited chase, signal filters, rates and tick counts, minimum-jerk blends, path and motion timing, motor health, positive finite values, servo tolerances, throttling, the setpoint stream a node publishes on a fixed interval, the loops that serve a service and receive a subscription, one report or log line per run of repeated failures, and waiting on a future or taking a lock from synchronous code |
| [camera_core](./camera_core) | Rust | What the camera nodes share: the failure of a control write by its kind, the errno mapping and its error reply in a generated service, the value a control answers when it has none, the auto/manual mode, the color encodings and the depth one, the V4L2 control ids, the checks a camera geometry passes, the `z` depth model and the refusal a color camera gives for its depth members, and the pieces of a simulated camera's relay that name no generated type. Pure Rust, no hardware or messaging deps |
| [instance_lock](./instance_lock) | Rust | One running instance per device: an exclusive advisory lock on a file under `/tmp/peppy_instance_locks`, taken in one syscall and released by the kernel when the holding process exits, so a crashed instance leaves its device free for the next start. Names the holder in the refusal the loser returns |
| [documents_core](./documents_core) | Rust | What the documents of every side of peppy share: the root key that names a document's side and kind, the manifest that gives a document its name and revision, and the digest of its canonical JSON (RFC 8785). Parses JSON5 text nested at most 128 levels deep into a value tree that refuses a key twice in one object, NaN, an infinity and a number beyond 2^53, each by its path, and reads a typed document out of that tree after the root key check. Pure Rust, no hardware or messaging deps |
| [simulation_documents](./simulation_documents) | Rust | The documents of the simulation side: `world/v1`, which carries the root key, the manifest and the digest of `documents_core`, and the JSON5 texts without a root key that a simulation exchanges: the setup record it keeps, from which the world of a live simulation is made with the robots of the stack, and the judges that check ground truth in a live world, with their verdicts; and the holder of the setup claim of an applier of worlds. Pure Rust, no hardware or messaging deps |
| [workflow_documents](./workflow_documents) | Rust | The documents of the workflow side: `workflow/v1`, the program of the robots, `run/v1`, one run of it with its roles bound to robots, and `workflow_bundle/v1`, a workflow carried out of the chat with the digest of each file and the lock of the versions it ran with, each with the root key, the manifest and the digest of `documents_core`. Parses the expressions of a workflow, a subset of CEL, types and evaluates them, and holds the structural rules of a workflow that need no contract, the marks the workflow director reads, the JSON texts of the events and the results of a run, and the language reference a model that writes workflows reads. Pure Rust, no hardware or messaging deps |
| [control_core_py](./control_core_py) | Python | `control_core`'s Python sibling: asyncio stream plumbing (paced ticks, gated wire-input consumption, service serving, capture-time wire stamps), parameter validators, and the hardware device-thread skeleton. Pure Python, no hardware or messaging deps |
| [so101_description](./so101_description) | Python | The SO-101's identity for its peppy nodes: joint and motor names, wire-unit conversions, limb-name vocabulary, named postures, setpoint parsing shaped by the STS3215's constraints, the embedded URDF with its limits, kinematics, and pose transforms, and the approach solver of its workspace answer (`ApproachSolver`): the least distance the grasp point comes to a position, and joints that bring it there with the gripper's approach axis along a direction. `openarm_description`'s sibling for the lerobot SO-ARM family |
| [sim_ext_core](./sim_ext_core) | Python | Plugin lifecycle, config loading, and sensor bridge plugins for the in-sim extensions; the node supplies the IO transport |
| [sim_robot_core](./sim_robot_core) | Python | What a simulation engine node knows about the robots it stands, whatever its physics: the registry of standing robots, the four pairing slots and what each pair names, one entry per robot model with its checks, and the camera configuration. The MuJoCo and Isaac Sim nodes install it in their base image |
| [brain_core](./brain_core) | Python | The code every brain node shares: the `item_perception` and `item_manipulation` rules (item ids, the gripper table, one sequence per lane, the result rules), the `brain_model` rules (one model per role, the switch), the `limb_motion` robot proxy, the camera frames and geometry, and the loop of each exposed member. The node passes in its generated peppygen modules and its `ModelSource`, whose models implement `Detector` and `Manipulator`. Pure Python on numpy and Pillow, no peppygen |
| [workspace_core_py](./workspace_core_py) | Python (Rust extension) | `workspace_core`'s Python bindings for a robot's backbone in Python: the limits, the grasp directions, the parsing of a `workspace:v1` request, the grid targets of a surface, the reach memo, and the answers and messages of a robot without a perception camera. A PyO3 module for the stable ABI of Python 3.12: an install compiles it, so the host needs a Rust toolchain |

## Using these libraries

Consumers name the repository; cargo finds a package by name anywhere inside it, so a Rust dependency carries no path. Rust nodes pin `sim_bridge_core` in `Cargo.toml`:

```toml
[dependencies]
sim_bridge_core = { git = "https://github.com/Peppy-bot/public-peppy-libs", rev = "<commit>" }
```

Python consumers install a library from a pinned commit, naming its directory:

```
sim_ext_core @ git+https://github.com/Peppy-bot/public-peppy-libs.git@<commit>#subdirectory=sim_ext_core
```

Pin a commit rather than a branch: node builds happen inside containers and should be reproducible. When a library changes, bump the pin in the consuming node and rebuild it.

See each library's README for the API.
