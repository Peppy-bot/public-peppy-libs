# camera_core

What the camera nodes and the crates that read them share, free of peppy types:
`occupancy_map` reads their depth frames, `workspace_core` sees a perception
camera through their pinhole model, and `navigation_core` and `mapping_core` read the
scans of a 2-D lidar. Each node maps these onto its own generated contract members.

| Module | What it holds |
|---|---|
| `controls` | `ControlFailure` and `ControlFailureKind`: why a control write gave no value, by the failure kind of peppy, with the kind of a write the driver refused by its errno. `VALUE_UNAVAILABLE`: the value a control answers when it has none to report. `AutoManualMode`: the mode of a control with a setpoint, by its exact name. `control_failure!`: the error reply of a generated service for a `ControlFailure`. |
| `encoding` | `ColorEncoding`: the color encodings by the names a topic spells (`rgb8`, `bgr8`, `mjpeg`, `yuyv`). `DEPTH_ENCODING`: `z16`. `DepthFrame`: a z16 frame parsed against its width and height, read pixel by pixel. |
| `v4l2` | The V4L2 control ids and exposure values the UVC cameras drive, from `<linux/v4l2-controls.h>`. |
| `geometry` | The checks a pinhole model, a depth range and a depth unit pass before a consumer computes with them, the `z` depth model, and the refusal a color camera gives for its depth members. |
| `pinhole` | `Pinhole`: a stream's pinhole model, parsed once from its camera_geometry:v1 `Intrinsics` with its lens (none, `plumb_bob` or `inverse_plumb_bob`), or a rendered camera's from its vertical field of view, giving back the intrinsics a camera node answers: the ray of a pixel, the pixel that shows a point, none past where the lens turns back, and the point of a depth reading. `DepthModel`: what a depth sample measures, `z` or `range`, parsed from its camera_geometry:v1 name, and the reading of a point. |
| `laser_scan` | `LaserScan`: a 2-D lidar's scan as lidar_2d:v1 carries it, its fields checked and its ranges read once, each a distance or one of the contract's marks (0 for a ray not measured, a range at the scan's maximum or above it, 10000 among them, for a ray that met nothing, as laser_geometry reads the maximum), and each ray's hit, at the ray's angle and time (before the timestamp for a sensor that sweeps clockwise), placed by the sensor's pose at the ray's time, interpolated between its poses at the first and the last ray, given as a `Sweep` of named fields, as laser_geometry interpolates them, each hit kept in the sensor's frame at its ray's time too (`placed_hits`, a `PlacedHit` a hit), and every ray, the marks among them, with the sensor's pose at its time by the same interpolation (`placed_rays`, a `PlacedRay` a ray). |
| `relay` | The pieces of a simulated camera's relay: the camera a relay is in its copy, why a forwarded call has no answer, and the bounds and messages of the relay. |

Every name check is exact. A node that takes another spelling (a different
case, say) folds it before the check and words its own refusal.

```toml
[dependencies]
camera_core = { git = "https://github.com/Peppy-bot/public-peppy-libs", package = "camera_core" }
```
