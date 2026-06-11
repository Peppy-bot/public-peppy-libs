# nodes_shared_code

Shared infrastructure libraries for PeppyOS nodes. Each library in this repo is an independent package pulled as a git dependency by nodes that require it.

PeppyOS nodes live in separate repositories under the nodes hub. Shared code that is needed across multiple independent node repos cannot be a path dependency inside one node's repo — it needs a central place. This repo is that central place.

## Libraries

| Library | Language | Purpose |
|---|---|---|
| [openarm_can](./openarm_can) | Rust | Safe wrapper around the `openarm_can` C++ library for driving the physical OpenArm hardware over CAN (`ArmCan` / `GripperCan`, Damiao motor types, OpenArm v10 constants) |
| [sim_bridge_core](./sim_bridge_core) | Rust | raw-to-peppygen pipelines for Isaac Sim and MuJoCo bridge nodes; the node supplies the peppylib transport |
| [sim_ext_core](./sim_ext_core) | Python | Plugin lifecycle, config loading, and sensor bridge plugins for the in-sim extensions; the node supplies the IO transport |

## Using these libraries

Rust nodes pin `sim_bridge_core` in `Cargo.toml`:

```toml
[dependencies]
sim_bridge_core = { git = "https://github.com/Peppy-bot/nodes_shared_code", rev = "<commit>" }
```

Python consumers install `sim_ext_core` from a pinned commit:

```
sim_ext_core @ git+https://github.com/Peppy-bot/nodes_shared_code.git@<commit>#subdirectory=sim_ext_core
```

Pin a commit rather than a branch: node builds happen inside containers and should be reproducible. When this repo changes, bump the pin in the consuming node and rebuild it.

See each library's README for the API.
