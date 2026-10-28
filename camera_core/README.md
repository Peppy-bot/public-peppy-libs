# camera_core

What the camera nodes share, free of peppy types. Each node maps these onto
its own generated contract members.

| Module | What it holds |
|---|---|
| `controls` | `ControlFailure` and `ControlFailureKind`: why a control write gave no value, by the failure kind of peppy, with the kind of a write the driver refused by its errno. `VALUE_UNAVAILABLE`: the value a control answers when it has none to report. `AutoManualMode`: the mode of a control with a setpoint, by its exact name. `control_failure!`: the error reply of a generated service for a `ControlFailure`. |
| `encoding` | `ColorEncoding`: the color encodings by the names a topic spells (`rgb8`, `bgr8`, `mjpeg`, `yuyv`). `DEPTH_ENCODING`: `z16`. |
| `v4l2` | The V4L2 control ids and exposure values the UVC cameras drive, from `<linux/v4l2-controls.h>`. |
| `geometry` | The checks a pinhole model, a depth range, a depth unit and a pose pass before a consumer computes with them, the `z` depth model, and the refusal a color camera gives for its depth members. |
| `relay` | The pieces of a simulated camera's relay: the camera a relay is in its copy, the timestamp check of a relayed frame, why a forwarded call has no answer, and the bounds and messages of the relay. |

Every name check is exact. A node that takes another spelling (a different
case, say) folds it before the check and words its own refusal.

```toml
[dependencies]
camera_core = { git = "https://github.com/Peppy-bot/public-peppy-libs", package = "camera_core" }
```
