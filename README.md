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
| [srs_model](./srs_model) | Rust | Kinematics and dynamics for a 7-DOF SRS arm: FK, closed-form arm-angle (Shimizu) IK, gravity/Coriolis feedforward, and Jacobians. Robot-agnostic: all geometry derives from the supplied URDF. Builds on `chain_kinematics`; pure Rust, no hardware or messaging deps |
| [bimanual_collision_model](./bimanual_collision_model) | Rust | Runtime self-collision detection for a bimanual robot: URDF-fitted convex hulls, GJK/EPA minimum distance, the analytic distance gradient, and a proximity band for scaling commanded motion near contact. Builds on `srs_model`; pure Rust, no hardware or messaging deps |
| [openarm_description](./openarm_description) | Rust | The OpenArm v1.0 robot description as a single embedded source of truth: the URDF, the collision meshes (behind the `meshes` feature), and the elbow singularity control-margin constants. Pure data: no kinematics or solver deps, so any consumer (`srs_model`, a viz tool, a sim bridge) builds from `urdf()` itself |
| [sim_bridge_core](./sim_bridge_core) | Rust | raw-to-peppygen pipelines for Isaac Sim and MuJoCo bridge nodes; the node supplies the peppylib transport |
| [control_core](./control_core) | Rust | Shared control-loop primitives for the openarm control nodes: a fixed-rate `Pacer` with overrun accounting |
| [control_core_py](./control_core_py) | Python | `control_core`'s Python sibling: asyncio stream plumbing (paced ticks, gated wire-input consumption, service serving, capture-time wire stamps), parameter validators, and the hardware device-thread skeleton. Pure Python, no hardware or messaging deps |
| [so101_description](./so101_description) | Python | The SO-101's identity for its peppy nodes: joint and motor names, wire-unit conversions, limb-name vocabulary, named postures, setpoint parsing shaped by the STS3215's constraints, and the embedded URDF with its limits, kinematics, and pose transforms. `openarm_description`'s sibling for the lerobot SO-ARM family |
| [sim_ext_core](./sim_ext_core) | Python | Plugin lifecycle, config loading, and sensor bridge plugins for the in-sim extensions; the node supplies the IO transport |
| [sim_robot_core](./sim_robot_core) | Python | What a simulation engine node knows about the robots it stands, whatever its physics: the registry of standing robots, the four pairing slots and what each pair names, one entry per robot model with its checks, and the camera configuration. The MuJoCo and Isaac Sim nodes install it in their base image |

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
