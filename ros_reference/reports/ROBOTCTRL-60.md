# ROBOTCTRL-60: can a depth camera map a room?

Nav2 maps a room with slam_toolbox and finds the base in a map with AMCL, each on the scans
of a 2-D lidar that sees all around the base. This spike measures a base whose only range
sensor is a depth camera that faces forward: it runs slam_toolbox and AMCL on the depth
frames of the TUM RGB-D sequences of a Kinect on a Pioneer that
`ros_reference/datasets.txt` lists, turned into 2-D scans as Nav2 turns a depth camera into
a laser, and measures how well they map and localize against the sequences' motion capture
ground truth. The sections "What the numbers say" and "The numbers" are rendered from the
scenarios and the committed statistics, and
`ros_reference/tests/ros_reference_room_dataset.rs` checks that the report holds them as
rendered; the other sections state numbers only in code spans.

## How the harness ran it

The ROS reference harness fetches the sequences by address and SHA-256
(`ros_reference/datasets.txt`) into a cache on the developer's machine, and the scenarios
`ros_reference/scenarios/room_dataset_*.json` hold every parameter. pioneer_slam and
pioneer_360 map the room; pioneer_slam2 and pioneer_slam3, which share pioneer_slam's
motion capture frame, localize in pioneer_slam's maps, each seed in the map that the same
seed made.

Each scenario keeps a depth frame each scan period, puts every depth pixel through the
pinhole model with the ROS default intrinsics that TUM recommends, places the points in the
base frame by a fixed mount fitted from the ground truth (the camera's median height above
the floor and its median pitch and roll against the floor's normal), and hands the cloud to
pointcloud_to_laserscan's own callback, which keeps the points inside a height band and
writes the nearest of each angle step. No frame's own attitude enters a scan, as on a robot
whose camera sits on a fixed mount. A frame whose ground truth has a gap longer than the
scenario allows is dropped and counted. The odometry of each seed is the ground truth with
seeded noise on each motion between consecutive kept frames, drawn as AMCL's differential
motion model reads a motion: a turn, a move, a turn.

slam_toolbox 2.10.0 maps in its synchronous mode with the parameters of its
`mapper_params_online_sync.yaml`, but for the departures that a section below lists, and
the trajectory error is the error of the optimized pose of each scan it added, after the
rigid motion that best aligns the ground truth with them. slam_toolbox then localizes each
seed of each localization sequence in the saved map of the same seed of pioneer_slam with
the parameters of its `mapper_params_localization.yaml`, starting from the sequence's first
ground truth pose carried into the map by that alignment. Nav2 1.5.2's AMCL, with the
parameters of Nav2's `nav2_params.yaml`, localizes globally in the same map, exported as
nav2_map_server writes it, from particles spread over the map's free cells.

The statistics take each pose that AMCL publishes on `amcl_pose`, with the covariance of
its whole filter that the message carries. AMCL publishes after each scan at which its
filter resampled, after the first scan of a global localization, and after each scan until
its first pose (`nav2_amcl/src/amcl_node.cpp:546-556`, `:590`). The statistics judge each
such estimate by AMCL's convergence test and by the search limits. AMCL's convergence test
(`pf_update_converged`, `nav2_amcl/src/pf/pf.c:205-236`) holds when every particle of the
filter's current set lies within `pf->dist_threshold = 0.5` m (`pf.c:72`) of the set's
unweighted mean position along x and along y; it reads no heading. The search limits hold
when the standard deviations of the whole filter lie within the limits at which room_map's
global localization (ROBOTCTRL-62) takes AMCL's estimate as localized: the deviation of the
position along its least certain direction, the square root of the larger eigenvalue of the
position's covariance, and the deviation of the heading. The search limits are provisional
values of ROBOTCTRL-62: the counts within them hold for these values, and the least
deviations of a seed show how near its filter came to any limits. A seed localizes by a
rule from the first estimate from which every estimate holds the rule and lies within the
scenario's bound of the truth.

Each run has a process of its own. The crate's `statistics` program scores every run with
`ros_reference::metrics`, and writes the statistics of each seed under
`ros_reference/reference/room_dataset_*.json`.

## What the numbers say

The depth camera's scans hold the bearings within 62.7 deg ahead of the base, where a 2-D lidar sees 360 deg.

On pioneer_360, slam_toolbox maps with a trajectory error of 0.957 ± 0.048 m and a heading error of 32.6 ± 1.7 deg RMS, mean ± standard error over its 10 seeds.

On pioneer_slam, slam_toolbox maps with a trajectory error of 0.663 ± 0.087 m and a heading error of 16.2 ± 0.8 deg RMS, mean ± standard error over its 10 seeds.

On pioneer_slam2, slam_toolbox localizes, each seed in the map of room_dataset_pioneer_slam that the same seed made, from the ground truth carried into that map, with an error of 4.429 ± 0.631 m and 99.4 ± 18.6 deg RMS of heading, mean ± standard error over its 10 seeds.

On pioneer_slam3, slam_toolbox localizes, each seed in the map of room_dataset_pioneer_slam that the same seed made, from the ground truth carried into that map, with an error of 2.248 ± 0.516 m and 29.2 ± 7.6 deg RMS of heading, mean ± standard error over its 10 seeds.

On pioneer_slam2, AMCL localizes, within 0.5 m and 20.1 deg of the truth from some amcl_pose to the end, by its convergence test in 0 of 10 seeds, and by the search limits in 0 of 10 seeds. Its convergence test holds at 0 of 1195 amcl_pose messages, and the deviations of its whole filter lie within the search limits, 0.3 m and 17.2 deg, at 2 of 1195. The least position deviation of a seed is 0.238 to 0.796 m, and its least heading deviation 14.3 to 22.9 deg. The last amcl_pose of a seed lies at most 1.039 m and 29.7 deg from the truth.

On pioneer_slam3, AMCL localizes, within 0.5 m and 20.1 deg of the truth from some amcl_pose to the end, by its convergence test in 0 of 10 seeds, and by the search limits in 0 of 10 seeds. Its convergence test holds at 0 of 1069 amcl_pose messages, and the deviations of its whole filter lie within the search limits, 0.3 m and 17.2 deg, at 2 of 1069. The least position deviation of a seed is 0.279 to 0.602 m, and its least heading deviation 12.7 to 23.8 deg. The last amcl_pose of a seed lies at most 1.452 m and 16.9 deg from the truth.

The error bound for ROBOTCTRL-62 and ROBOTCTRL-65, the largest final error of the seeds that AMCL localized on the localization sequences: by its convergence test, none: it localized 0 of 20 seeds; by the search limits, none: it localized 0 of 20 seeds.

## The depth camera against a 2-D lidar

A 2-D lidar that sees all around the base covers each bearing. The depth camera covers the
bearings ahead of the base within its horizontal field of view, which the section "What the
numbers say" gives, so its scans whose headings differ by more than that field share no
bearing. These runs measure the depth camera alone: no 2-D lidar ran on these sequences, and
the statistics hold no measurement of how much of the errors above the field of view, the
scan matcher or AMCL's sensor model accounts for.

## Where the references depart from the packaged nodes

The references run the packaged slam_toolbox 2.10.0, Nav2 1.5.2's AMCL and map server, and
pointcloud_to_laserscan 2.1.1, and depart from them, and from the parameter files named
above, in these places:

- AMCL's `map_update_cspace` (`nav2_amcl/src/map/map_cspace.cpp`) pushes each occupied cell
  into its priority queue as a `CellData` whose `occ_dist` it never sets, and the queue
  orders cells by `occ_dist`, so the order in which occupied cells seed the likelihood field
  reads uninitialized stack memory: undefined behavior. The program carries a changed copy
  of that file (`ros_reference/reference_programs/amcl_map_cspace.cpp`), under the
  GPL-3.0-or-later, as AMCL's LGPL-2.1-or-later lets code copied into a program be, that
  sets that `occ_dist` to `0`, and the dynamic linker binds AMCL's calls to it.
- slam_toolbox's Ceres solver plugin sets `options_.num_threads = 50` in its `Configure`
  (`solvers/ceres_solver.cpp:166`). The program sets `num_threads` to `1`, as the README's
  rules run every reference: it writes the solver's `num_threads` through a copy of the
  plugin's member layout up to its options (`CeresSolverLayout`,
  `ros_reference/reference_programs/room_dataset_runs.cpp`), since slam_toolbox installs no
  header of the plugin, after checking that the solver is the plugin and that its options
  hold every value that `Configure` sets.
- `enable_interactive_mode` is `false`, where `mapper_params_online_sync.yaml` sets it
  `true`; set `true`, slam_toolbox keeps a copy of each processed scan for RViz's
  interactive markers (`src/slam_toolbox_common.cpp:942-944`). Localization mode sets it
  `false` itself (`src/slam_toolbox_localization.cpp:68`).
- The laser's range in slam_toolbox, `min_laser_range` and `max_laser_range`, is the
  scan's, which the section "The numbers" gives, where the parameter files of mapping and
  localization set `min_laser_range: 0.0` and `max_laser_range: 20.0`.
- The program sets slam_toolbox's `mode`, its frames, `use_map_saver` false and
  `use_lifecycle_manager` false, and AMCL's frames and `tf_broadcast` false. It leaves unset
  the parameters of slam_toolbox's threads that it does not run (`map_update_interval`,
  `transform_publish_period`, `stack_size_to_use`), and saves the map through slam_toolbox's
  `serializePoseGraphCallback` and nav2_map_server's `saveMapToFile`.
- No node is activated and no executor spins a node: the program calls each node's callbacks
  on its own thread (pointcloud_to_laserscan's private `cloudCallback`; slam_toolbox's
  `laserCallback`, adding each scan that synchronous mode queues as its run loop does; and
  AMCL's `laserReceived`), sets each node's clock to the scan's stamp, and puts the odometry
  in each tf buffer at that stamp. It seeds drand48 with the seed right before it calls
  AMCL's `globalLocalizationCallback`.
- The container sets `OMP_NUM_THREADS=1`, and OpenBLAS runs the kernels of a baseline CPU
  (`OPENBLAS_CORETYPE`).

## Reproducing it

```sh
ros_reference/run.sh --datasets          # fetches the sequences, writes the statistics
ros_reference/run.sh --check --datasets  # fails when a statistic differs
```

On an amd64 host, `run.sh --datasets` wrote these statistics, and `run.sh --check
--datasets` found every committed output again. With every reference pinned to the same CPU
of that host, and the `statistics` program under the container's mask, each output came out
the same. From that host's results, the `statistics` program gives the same statistics
unmasked, under `glibc.cpu.hwcaps=-AVX2,-FMA` and under the container's mask, with that
host's glibc.

On an arm64 host, `run.sh --datasets` gave the same counts and every value of these
statistics within a relative difference of `1.23e-13`, and every parity test passed on that
host's outputs, the test of this report among them.

karto's `ScanMatcher::serialize` writes the loop scan matcher's `m_searchAngleResolution`
before and after its `m_nAngles` (`lib/karto_sdk/include/karto_sdk/Mapper.h:1519-1521`),
which its constructor leaves unset (`Mapper.h:1481-1489`) and only `CorrelateScan` sets
(`lib/karto_sdk/src/Mapper.cpp:785-786`), so the serialized pose graphs `map.posegraph` of
the mapping runs can differ between runs in those bytes; the localizations load
pioneer_slam's.

## For the team's decision

These measurements are of the depth camera alone, on these sequences, and the section "What
the numbers say" holds them: slam_toolbox's maps and its localization in a saved map, AMCL's
global localization by its convergence test and by the search limits, and the error bound
that they give ROBOTCTRL-62 and ROBOTCTRL-65. These runs measured no 2-D lidar, and no
camera but the Kinect on a Pioneer that the sequences show in a hall.

## The numbers

Every sequence runs 10 seeds. It keeps the first depth frame at or after each 0.2 s, drops a frame whose ground truth has a gap over 0.1 s, and projects each depth pixel with fx 525, fy 525, cx 319.5, cy 239.5 and 5000 counts a meter, a horizontal field of view of 62.7 deg, where a 2-D lidar sees 360 deg. The scan holds the points 0.15 m to 1.5 m above the floor, 0.45 m to 8 m away, in 0.005 rad steps from -0.55 rad. The odometry noise of each motion between two kept frames is the noise of AMCL's differential motion model with alpha1 0.0025, alpha2 0.0004, alpha3 0.0025 and alpha4 0.0001.

### pioneer_360

1209 depth frames, 361 scans kept, 0 dropped in gaps of the ground truth. Fitted mount: camera 0.559 m above the floor, pitched 5.7 deg down, rolled 0.5 deg.

slam_toolbox mapping:

| Seed | Scans in the map | Trajectory RMSE (m) | Largest error (m) | Heading RMSE (deg) |
|---|---|---|---|---|
| 0 | 21 | 0.976 | 2.022 | 32.0 |
| 1 | 22 | 0.601 | 1.332 | 24.9 |
| 2 | 21 | 0.878 | 2.282 | 32.6 |
| 3 | 21 | 0.982 | 2.390 | 29.7 |
| 4 | 21 | 0.978 | 2.205 | 38.4 |
| 5 | 21 | 1.129 | 2.440 | 30.6 |
| 6 | 21 | 0.941 | 2.369 | 32.0 |
| 7 | 21 | 1.154 | 2.609 | 44.2 |
| 8 | 21 | 0.922 | 2.159 | 30.8 |
| 9 | 21 | 1.011 | 2.496 | 30.6 |
| Mean ± standard error | 21.1 ± 0.1 | 0.957 ± 0.048 | 2.230 ± 0.114 | 32.6 ± 1.7 |

### pioneer_slam

2890 depth frames, 748 scans kept, 17 dropped in gaps of the ground truth. Fitted mount: camera 0.560 m above the floor, pitched 5.4 deg down, rolled 0.5 deg.

slam_toolbox mapping:

| Seed | Scans in the map | Trajectory RMSE (m) | Largest error (m) | Heading RMSE (deg) |
|---|---|---|---|---|
| 0 | 57 | 0.456 | 0.997 | 14.0 |
| 1 | 59 | 0.465 | 1.354 | 12.7 |
| 2 | 57 | 0.674 | 1.577 | 19.0 |
| 3 | 55 | 0.456 | 0.812 | 13.6 |
| 4 | 57 | 0.455 | 1.122 | 14.1 |
| 5 | 58 | 0.770 | 2.162 | 18.7 |
| 6 | 56 | 1.337 | 2.055 | 18.4 |
| 7 | 57 | 0.859 | 1.566 | 15.4 |
| 8 | 57 | 0.600 | 1.250 | 20.3 |
| 9 | 56 | 0.556 | 1.335 | 16.3 |
| Mean ± standard error | 56.9 ± 0.3 | 0.663 ± 0.087 | 1.423 ± 0.137 | 16.2 ± 0.8 |

### pioneer_slam2

2075 depth frames, 537 scans kept, 7 dropped in gaps of the ground truth. Fitted mount: camera 0.559 m above the floor, pitched 5.4 deg down, rolled 0.5 deg.

slam_toolbox localization, each seed in the map of room_dataset_pioneer_slam that the same seed made:

| Seed | Scans localized | RMSE (m) | Largest error (m) | Heading RMSE (deg) |
|---|---|---|---|---|
| 0 | 29 | 5.846 | 9.141 | 118.1 |
| 1 | 28 | 4.420 | 7.229 | 142.8 |
| 2 | 27 | 3.567 | 5.395 | 29.4 |
| 3 | 28 | 7.833 | 11.542 | 162.4 |
| 4 | 27 | 3.555 | 5.588 | 55.6 |
| 5 | 28 | 6.039 | 8.719 | 159.1 |
| 6 | 29 | 1.356 | 2.273 | 22.0 |
| 7 | 28 | 1.669 | 2.877 | 30.1 |
| 8 | 29 | 4.541 | 7.085 | 156.7 |
| 9 | 28 | 5.460 | 8.611 | 118.0 |
| Mean ± standard error | 28.1 ± 0.2 | 4.429 ± 0.631 | 6.846 ± 0.910 | 99.4 ± 18.6 |

AMCL global localization in that map:

| Seed | amcl_pose messages | Convergence test held | Within the search limits | Least position deviation (m) | Least heading deviation (deg) | Localized by the test after (s) | Localized by the limits after (s) | Final position error (m) | Final heading error (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 119 | 0 | 0 | 0.323 | 14.3 | never | never | 0.806 | 22.0 |
| 1 | 120 | 0 | 0 | 0.298 | 14.7 | never | never | 0.597 | 11.6 |
| 2 | 120 | 0 | 0 | 0.305 | 21.3 | never | never | 0.811 | 5.2 |
| 3 | 118 | 0 | 0 | 0.350 | 16.5 | never | never | 0.645 | 13.3 |
| 4 | 121 | 0 | 0 | 0.304 | 14.7 | never | never | 0.612 | 3.9 |
| 5 | 120 | 0 | 0 | 0.320 | 15.4 | never | never | 0.752 | 16.4 |
| 6 | 117 | 0 | 0 | 0.796 | 22.9 | never | never | 0.928 | 19.3 |
| 7 | 120 | 0 | 0 | 0.630 | 20.1 | never | never | 0.578 | 29.7 |
| 8 | 121 | 0 | 2 | 0.238 | 15.6 | never | never | 1.039 | 25.3 |
| 9 | 119 | 0 | 0 | 0.346 | 17.0 | never | never | 1.026 | 20.1 |

### pioneer_slam3

2531 depth frames, 517 scans kept, 16 dropped in gaps of the ground truth. Fitted mount: camera 0.559 m above the floor, pitched 5.3 deg down, rolled 0.7 deg.

slam_toolbox localization, each seed in the map of room_dataset_pioneer_slam that the same seed made:

| Seed | Scans localized | RMSE (m) | Largest error (m) | Heading RMSE (deg) |
|---|---|---|---|---|
| 0 | 28 | 1.376 | 2.495 | 29.6 |
| 1 | 26 | 3.558 | 5.426 | 36.0 |
| 2 | 27 | 2.560 | 4.009 | 60.4 |
| 3 | 28 | 1.539 | 2.424 | 12.9 |
| 4 | 27 | 1.030 | 2.471 | 8.5 |
| 5 | 26 | 0.497 | 0.907 | 7.1 |
| 6 | 26 | 3.643 | 5.557 | 19.8 |
| 7 | 29 | 1.154 | 2.283 | 15.6 |
| 8 | 27 | 5.766 | 8.380 | 80.4 |
| 9 | 27 | 1.353 | 2.225 | 21.4 |
| Mean ± standard error | 27.1 ± 0.3 | 2.248 ± 0.516 | 3.618 ± 0.707 | 29.2 ± 7.6 |

AMCL global localization in that map:

| Seed | amcl_pose messages | Convergence test held | Within the search limits | Least position deviation (m) | Least heading deviation (deg) | Localized by the test after (s) | Localized by the limits after (s) | Final position error (m) | Final heading error (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 106 | 0 | 0 | 0.394 | 16.1 | never | never | 0.637 | 12.3 |
| 1 | 108 | 0 | 0 | 0.318 | 14.3 | never | never | 0.605 | 2.4 |
| 2 | 107 | 0 | 0 | 0.442 | 16.5 | never | never | 0.230 | 11.8 |
| 3 | 106 | 0 | 0 | 0.370 | 15.0 | never | never | 0.763 | 5.0 |
| 4 | 107 | 0 | 2 | 0.279 | 12.7 | never | never | 0.397 | 6.9 |
| 5 | 108 | 0 | 0 | 0.322 | 13.0 | never | never | 0.830 | 4.3 |
| 6 | 104 | 0 | 0 | 0.602 | 23.8 | never | never | 1.452 | 16.9 |
| 7 | 107 | 0 | 0 | 0.513 | 17.3 | never | never | 1.053 | 1.8 |
| 8 | 108 | 0 | 0 | 0.335 | 17.8 | never | never | 0.791 | 14.0 |
| 9 | 108 | 0 | 0 | 0.354 | 17.3 | never | never | 0.880 | 11.1 |
