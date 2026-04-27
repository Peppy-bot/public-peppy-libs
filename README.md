# nodes_shared_code

Shared infrastructure libraries for PeppyOS sim node variants. Each library in this repo is an independent package pulled as a git dependency by nodes that require it.

PeppyOS nodes live in separate repositories under the nodes hub. Shared code that is needed across multiple independent node repos cannot be a path dependency inside one node's repo — it needs a central place. This repo is that central place.

> **Sim-only.** Real node variants talk directly to hardware via peppygen and do not depend on these libraries.

## Libraries

| Library | Language | Purpose |
|---|---|---|
| [sim_bridge_core](./sim_bridge_core) | Rust | peppylib ↔ peppygen translation layer for Isaac Sim and MuJoCo sim bridge nodes |
| [sim_ext_core](./sim_ext_core) | Python | Plugin lifecycle, peppylib transport, config loading, and sensor bridge plugins for Isaac Sim and MuJoCo extensions |

## Quick dependency setup

**Rust** (`Cargo.toml`):
```toml
[dependencies]
sim_bridge_core = { git = "https://github.com/Peppy-bot/nodes_shared_code", package = "sim_bridge_core" }
```

**Python** (`pyproject.toml`):
```toml
[project]
dependencies = ["sim_ext_core"]

[tool.uv.sources]
sim_ext_core = { git = "https://github.com/Peppy-bot/nodes_shared_code", subdirectory = "sim_ext_core" }
```

See each library's README for full API reference and configuration.
