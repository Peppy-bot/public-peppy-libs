# mapping_core

Room maps from 2-D scans, as Nav2 makes them, and the pose of a base in a map. No hardware
or messaging dependencies; the library takes 2-D scans and names no sensor.

- `LaserProjection` projects depth points to a `ProjectedScan`, a scan as `lidar_2d`'s
  `scan_stream` carries it, as Nav2's pointcloud_to_laserscan 2.1.1 does: the points
  inside a height band and range limits, binned by bearing, the nearest point of each bin,
  and the no-return mark 10000 in a bin with no point, where pointcloud_to_laserscan writes
  +inf. `ProjectedScan::laser_scan` reads it as a `camera_core::laser_scan::LaserScan`.
- `PlacedScan` is a scan with each ray placed where the sensor stood at the ray's time,
  interpolated between the sensor's poses at the first and the last ray as laser_geometry
  interpolates them (`camera_core::laser_scan::LaserScan::placed_rays`), in the base's frame
  at the scan's timestamp, with the base's odometry there.
- `Mapping` maps a room from scans and odometry as slam_toolbox 2.10.0 does in its
  synchronous mode, its map frame the odometry frame where mapping started: each scan that
  passes slam_toolbox's gate is matched against a running buffer of recent scans by karto's
  correlative scan matcher, linked to the scan before it, to the buffer and to the near
  chains of scans it matches, and the loops it closes against chains of scans nearby close,
  with the pose graph optimized by Ceres 2.2.0's Levenberg-Marquardt schedule as
  slam_toolbox configures it, each step's normal equations solved by faer's sparse Cholesky
  factorization. Each scan it adds to the graph is a scan of the map, and answers the
  base's `MapPose`. `Mapping::grid` is the grid of the map as it stands.
- `RoomMap` is a map: its pose graph with its scans, the `OccupancyGrid` that karto makes of
  them, and its `MapSummary`. `RoomMap::save` writes it as a directory: `pose_graph.json`,
  `meta.json`, and `map.pgm` with `map.yaml` as Nav2's map saver writes a trinary map.
  As karto's `ComputeDimensions` sizes it, the grid holds Round(size x (1 / resolution))
  cells along each axis from the lowest corner of the box that holds the scans, and a hit
  whose distance from that corner rounds to that count keys no cell: a map leaves out the
  hits on the box's far x and far y edges, and with them a wall that lies along either
  edge. `RoomMap::load` reads it back, and refuses a directory whose files disagree, or
  whose resolution makes no grid karto can count, naming the file and the field;
  `RoomMap::load_summary` reads its summary from `meta.json` alone.
- `MapStore` keeps maps by `MapName` in one directory, as room_map:v1 keeps them: a name is
  a lowercase letter, then lowercase letters, digits and `_`, at most 64 characters. It
  lists the maps whose description reads, sorted by name, each with its summary; it saves a
  map under a name no saved map has, claiming the name's directory, writing the map into a
  temporary directory, flushing it and renaming it over the claim; it loads a map; and it
  removes one by renaming its directory before deleting it. A listing gives a whole map of a
  name or none; a save that stops before its rename leaves a claimed directory, which lists
  no map, refuses a load and holds the name until a removal frees it.
- `Localization` keeps the base's pose in a map as slam_toolbox does in its localization
  mode: each scan added to the graph, linked and its loops closed, and the oldest added
  scan beyond the buffer dropped. It starts from an `InitialPose` with a search radius and a
  search angle, as `room_map`'s `set_initial_pose` gives them, the scan matched within the
  search as karto matches a loop; from an estimate, as slam_toolbox starts
  from an initial pose; or from a mapping's estimate when `Mapping::finish` ends mapping.
- A `MapPose` carries the standard deviations of its match, the square roots of the larger
  eigenvalue of the match's 2 x 2 position covariance and of its yaw variance, and the base
  is localized while they lie within the localization's matching limits.
- `GlobalLocalization` finds the base's pose in a map from no known pose, as Nav2 1.5.2's
  AMCL localizes globally once `reinitialize_global_localization` starts it:
  `max_particles` particles spread over the free cells of the map's grid, weighed by AMCL's
  likelihood field model, moved by its differential or omnidirectional motion model
  (`robot_model_type`) and resampled by KLD sampling between `min_particles` and
  `max_particles`, every random draw on glibc's `drand48` stream of a seed. Each scan that
  AMCL publishes a pose after answers the mean of the heaviest cluster of particles, its
  `MapPose` carrying the deviations of the whole filter's covariance as `amcl_pose` carries
  it; once they lie within the search limits, the search ends in the `Localization` that
  keeps the pose within its matching limits. The search runs AMCL with recovery rates of 0,
  as Nav2's parameters set them, so no random particle joins the cloud: when the particles
  drawn at the start favor a pose that the map scores close to the true one, the cloud can
  settle there (in drawn L-shaped rooms, the pose turned by pi scored 0.77 to 0.84 of the
  true pose's likelihood).
  `tests/global_localization.rs` counts the rooms of a fixed drawn set in which the search
  localizes the base near its true pose.

## Quick start

```rust
use camera_core::laser_scan::Sweep;
use mapping_core::{
    GlobalLocalization, GlobalLocalizationParameters, InitialPose, LaserProjection,
    LaserProjectionParameters, Localization, LocalizationParameters, Mapping, PlacedScan,
    RoomMap, SearchStep,
};

let projection = LaserProjection::new(LaserProjectionParameters {
    min_height: 0.15,
    max_height: 1.5,
    angle_min: -0.55,
    angle_max: 0.54625,
    angle_increment: 0.005,
    range_min: 0.45,
    range_max: 8.0,
})?;

// slam_toolbox's parameters, each by its name in mapper_params_online_sync.yaml; the map
// frame is the odometry frame.
let mut mapping = Mapping::start(parameters)?;
for frame in frames {
    // The points of a depth frame in the base frame, and the base's odometric pose at the
    // frame's timestamp. The projected scan lies in the base frame, so its sensor stands at
    // the base, every ray at that time.
    let scan = projection.project(&frame.points).laser_scan();
    let still = Sweep { first_ray: frame.odometry, last_ray: frame.odometry };
    mapping.add(&PlacedScan::new(frame.stamp_ns, &scan, &still, frame.odometry)?)?;
}
let localization_parameters = LocalizationParameters { slam_toolbox, matching_limits };
let (map, mut localization) = mapping
    .finish(mapping_start_unix_ns, localization_parameters)?
    .expect("a map of the scans");
map.save(directory)?;

let map = RoomMap::load(directory)?;
let initial = InitialPose::parse(pose, 0.5, 0.5)?;
// The first scan after the call starts the localization at its match within the search;
// each later scan keeps the pose.
let (mut localization, matched) = Localization::from_initial_pose(
    &map,
    localization_parameters,
    &initial,
    odometry_at_call,
    &first_scan,
)?;
if let Some(map_pose) = localization.add(&next_scan)? {
    // map_pose.pose, map_pose.deviations, map_pose.localized
}

// From no known pose: AMCL's parameters, each by its name in Nav2's amcl parameters, the
// search limits, the parameters of the localization that keeps the pose once the search
// ends, and the seed of the random draws.
let parameters = GlobalLocalizationParameters {
    amcl,
    search_limits,
    localization: localization_parameters,
};
let mut search = GlobalLocalization::start(&map, parameters, seed)?;
let localization = loop {
    match search.add(&next_scan) {
        // AMCL's pose after the scan, when it publishes one, outside the search limits.
        SearchStep::Searching(going_on, _map_pose) => search = going_on,
        // The search ended at map_pose: the localization keeps the pose from the next scan.
        SearchStep::Found(localization, _map_pose) => break localization,
    }
};
```

## Parity

`tests/ros_reference_laser_projection.rs` projects the clouds of the ROS reference
harness and compares every scan with the one pointcloud_to_laserscan wrote, bit for bit on
amd64. `tests/ros_reference_map_export.rs` exports the grids of the harness and compares
each image and description with those Nav2's map saver wrote, byte for byte.
`tests/ros_reference_amcl_global_localization.rs` searches the maps of the drawn rooms of
the harness, whose grids are karto's of the same scans cell for cell, and compares each
answer with the `amcl_pose` that Nav2's AMCL published after the same scan, within the
bound the test states.
`tests/ros_reference_room_dataset.rs` maps and localizes on the harness's lidar drive, on
the scans and the seeded odometries that the room dataset references prepared. With every
ray at its scan's stamp, as the references take it, it compares over ten seeds the
trajectory error of mapping and the error of localization with slam_toolbox's, and on the
drive spliced from two places of its map the error and the position deviation after the
splice; and from the first scan of each localization scenario, the search's convergence,
time to converge and final error with AMCL's global localization. With each ray at the time
the drive cast it, it checks that the first seed's map closes its loop, and that searches
started at those first scans and at the splice end within the final errors of AMCL's ten
runs. It reads the results of the local-only references, so `ros_reference/run.sh
--datasets` runs it.

## License

mapping_core is licensed under the GNU General Public License, version 3 or any later version,
in `COPYING`. It translates into Rust the code of the upstream files in the table below.
slam_toolbox 2.10.0 holds its own files under the GNU Lesser General Public License, version
2.1, and Nav2 1.5.2 holds nav2_amcl, as glibc 2.43 holds its `drand48`, under version 2.1 or any
later version of that license; section 3 of the GNU Lesser General Public License, version 2.1,
lets a copy of such code take the GNU General Public License, version 3 or later, in its place.
slam_toolbox 2.10.0 carries karto's files, lib/karto_sdk, under the GNU Lesser General Public
License, version 3 or later, which is the GNU General Public License, version 3, with additional
permissions that a copy may leave out. The BSD licenses of Ceres Solver, map_io.cpp and
pointcloud_to_laserscan let their code go into a work under the GNU General Public License,
version 3.

Each file under `src/` keeps the copyright and license notice of every upstream file it
translates. In the notices of nav2_amcl and glibc, the GNU General Public License, version 3 or
later, takes the place of the GNU Lesser General Public License, version 2.1 or later, as
section 3 of that license asks. `src/cholesky.rs`, which translates Eigen 3.4.0's Cholesky
factorization, is under the Mozilla Public License, version 2.0, the license of Eigen's code.

| Module | Upstream files | License |
|---|---|---|
| `geometry` | slam_toolbox 2.10.0: lib/karto_sdk/include/karto_sdk/Math.h, lib/karto_sdk/include/karto_sdk/Karto.h | LGPL-3.0-or-later (karto_sdk) |
| `grid` | slam_toolbox 2.10.0: lib/karto_sdk/include/karto_sdk/Karto.h | LGPL-3.0-or-later (karto_sdk) |
| `pose_graph` | slam_toolbox 2.10.0: lib/karto_sdk/include/karto_sdk/Mapper.h, lib/karto_sdk/src/Mapper.cpp | LGPL-3.0-or-later (karto_sdk) |
| `scan_matcher` | slam_toolbox 2.10.0: lib/karto_sdk/src/Mapper.cpp, lib/karto_sdk/include/karto_sdk/Mapper.h, lib/karto_sdk/include/karto_sdk/Karto.h | LGPL-3.0-or-later (karto_sdk) |
| `mapper` | slam_toolbox 2.10.0: lib/karto_sdk/src/Mapper.cpp, lib/karto_sdk/include/karto_sdk/Mapper.h, src/slam_toolbox_common.cpp | LGPL-3.0-or-later (karto_sdk); LGPL-2.1-only (slam_toolbox's own files) |
| `localized_scan` | slam_toolbox 2.10.0: lib/karto_sdk/include/karto_sdk/Karto.h, lib/karto_sdk/src/Karto.cpp, src/laser_utils.cpp, src/slam_toolbox_common.cpp | LGPL-3.0-or-later (karto_sdk); LGPL-2.1-only (slam_toolbox's own files) |
| `mapping` | slam_toolbox 2.10.0: src/slam_toolbox_sync.cpp, src/slam_toolbox_common.cpp, src/laser_utils.cpp, lib/karto_sdk/src/Mapper.cpp | LGPL-2.1-only (slam_toolbox's own files); LGPL-3.0-or-later (karto_sdk) |
| `localization` | slam_toolbox 2.10.0: src/slam_toolbox_localization.cpp, src/slam_toolbox_common.cpp, lib/karto_sdk/src/Mapper.cpp | LGPL-2.1-only (slam_toolbox's own files); LGPL-3.0-or-later (karto_sdk) |
| `occupancy_grid` | slam_toolbox 2.10.0: src/slam_mapper.cpp, include/slam_toolbox/visualization_utils.hpp, lib/karto_sdk/include/karto_sdk/Karto.h; Nav2 1.5.2: nav2_map_server/src/map_io.cpp | LGPL-2.1-only (slam_toolbox's own files); LGPL-3.0-or-later (karto_sdk); BSD-3-Clause (map_io.cpp) |
| `optimizer` | slam_toolbox 2.10.0: solvers/ceres_solver.cpp, solvers/ceres_utils.h; Ceres Solver 2.2.0: examples/slam/pose_graph_2d/normalize_angle.h, examples/slam/pose_graph_2d/angle_manifold.h, examples/slam/pose_graph_2d/pose_graph_2d_error_term.h, internal/ceres/trust_region_minimizer.cc, internal/ceres/levenberg_marquardt_strategy.cc, internal/ceres/trust_region_step_evaluator.cc, internal/ceres/dynamic_sparse_normal_cholesky_solver.cc, internal/ceres/compressed_row_sparse_matrix.cc, internal/ceres/problem_impl.cc, internal/ceres/program.cc, include/ceres/types.h, include/ceres/solver.h | LGPL-2.1-only (slam_toolbox's own files); BSD-3-Clause (Ceres Solver) |
| `cholesky` | Eigen 3.4.0: Eigen/src/Cholesky/LLT.h | MPL-2.0 (Eigen's LLT.h) |
| `laser_projection` | pointcloud_to_laserscan 2.1.1: src/pointcloud_to_laserscan_node.cpp | BSD-3-Clause (pointcloud_to_laserscan) |
| `global_localization::amcl` | Nav2 1.5.2: nav2_amcl/src/amcl_node.cpp | LGPL-2.1-or-later (nav2_amcl) |
| `global_localization::cspace` | Nav2 1.5.2: nav2_amcl/src/map/map_cspace.cpp | LGPL-2.1-or-later (nav2_amcl) |
| `global_localization::drand48` | Nav2 1.5.2: nav2_amcl/src/pf/pf_pdf.c; glibc 2.43: stdlib/drand48.c, stdlib/drand48-iter.c, stdlib/erand48_r.c, stdlib/srand48.c, stdlib/srand48_r.c | LGPL-2.1-or-later (nav2_amcl, glibc) |
| `global_localization::kdtree` | Nav2 1.5.2: nav2_amcl/src/pf/pf_kdtree.c | LGPL-2.1-or-later (nav2_amcl) |
| `global_localization::likelihood_field` | Nav2 1.5.2: nav2_amcl/src/sensors/laser/likelihood_field_model.cpp, nav2_amcl/src/amcl_node.cpp, nav2_amcl/include/nav2_amcl/sensors/laser/laser.hpp | LGPL-2.1-or-later (nav2_amcl) |
| `global_localization::map` | Nav2 1.5.2: nav2_amcl/include/nav2_amcl/map/map.hpp, nav2_amcl/src/amcl_node.cpp | LGPL-2.1-or-later (nav2_amcl) |
| `global_localization::motion_model` | Nav2 1.5.2: nav2_amcl/src/motion_model/differential_motion_model.cpp, nav2_amcl/src/motion_model/omni_motion_model.cpp | LGPL-2.1-or-later (nav2_amcl) |
| `global_localization::particle_filter` | Nav2 1.5.2: nav2_amcl/src/pf/pf.c, nav2_amcl/src/amcl_node.cpp | LGPL-2.1-or-later (nav2_amcl) |
| `global_localization::pose_arithmetic` | Nav2 1.5.2: nav2_amcl/src/pf/pf_vector.c, nav2_amcl/include/nav2_amcl/angleutils.hpp, nav2_amcl/src/amcl_node.cpp | LGPL-2.1-or-later (nav2_amcl) |
| `lib`, `map_pose`, `placed_scan`, `saved_map`, `slam_toolbox_parameters`, `global_localization`, `global_localization::parameters` | none: mapping_core's own code | GPL-3.0-or-later |

`COPYING` is the GNU General Public License, version 3. `LICENSE-MPL-2.0` is the Mozilla
Public License, version 2.0, of `src/cholesky.rs`. `NOTICE` holds the BSD notices of Ceres
Solver, map_io.cpp and pointcloud_to_laserscan. A binary distributed with this crate in it
ships these three files with it.
