# nodes_shared_code

Shared infrastructure libraries for PeppyOS sim nodes. Each library in this repo is an independent package that nodes pull in as a git dependency.

PeppyOS nodes live in separate repositories, so shared code can't be a path dependency inside any one of them, so it needs a central home. This repo is that home.

> **Sim-only.** Real-hardware nodes talk to their devices directly and do not depend on these libraries.

## Libraries

| Library | Language | Purpose |
|---|---|---|
| [sim_bridge_core](./sim_bridge_core) | Rust | peppylib to peppygen translation layer for Isaac Sim and MuJoCo bridge nodes |
| [sim_ext_core](./sim_ext_core) | Python | Plugin lifecycle, peppylib transport, config loading, and sensor bridge plugins for the in-sim extensions |

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
