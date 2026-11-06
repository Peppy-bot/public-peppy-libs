# ROS reference harness

A parity test of this repository checks a library that follows ROS code: it states what
the library guarantees, and the agreement of the library's outputs with those of the ROS
code, within a stated bound, is the evidence. The ROS code runs with ROS 2, in a reference
image: developers run it through `run.sh`, and the parity tests read the outputs that it
wrote.

A parity test reads a **scenario**, an input that the test and the reference both read,
and the **reference output** that the reference wrote for it. The reference outputs are
committed with the digests of them and of their inputs, so the parity tests run in the
routine `cargo test` of each crate, with no ROS on the machine, and fail on an output, a
scenario, a URDF or a mesh that differs from its digest. A change of the digests is a
change of the reference, which `run.sh --check` proves.

| Kind of scenario | Reference | Parity test |
|---|---|---|
| `forward_kinematics` | MoveIt's robot state: the global transform of each link of a chain | `chain_kinematics/tests/ros_reference_forward_kinematics.rs` |
| `chain_collision` | MoveIt's robot model, robot state and robot filter: the links with collision geometry that a chain carries and the center MoveIt grows each of their bodies about, the global transform of each carried link, and whether each probe point lies inside the links' bodies (a convex mesh for a mesh, the primitive itself for a box, a cylinder or a sphere) grown by a padding | `collision_geometry/tests/ros_reference_chain_collision.rs` |
| `self_distance` | MoveIt's FCL collision environment: the signed distance between the collision meshes of each pair of links | `self_collision_model/tests/ros_reference_self_distance.rs` |
| `path_timing` | MoveIt's time-optimal trajectory generation (TOTG): the blended path through waypoints at a maximum deviation, and its time-optimal duration under velocity and acceleration limits, or its refusal of a path that turns back | `control_core/tests/ros_reference_path_timing.rs` |
| `planning` | OMPL's geometric planners: the status of each plan through worlds of box obstacles, and for every planner but PRM its ending, its path and the path's simplification | `ompl_bridge/tests/ros_reference_planning.rs` |
| `occupancy` | MoveIt's point cloud updater: the log-odds of every voxel of MoveIt's occupancy map after a sequence of point clouds, with a sphere cut out as the robot's own body | `occupancy_map/tests/ros_reference_occupancy.rs` |
| `arm_planning` | MoveIt's planning pipeline with the OMPL planner of the scenario's objective, RRTConnect for `any` and RRTstar for `shortest`, its default request adapters and the response adapters AddTimeOptimalParameterization and ValidateSolution, on scenes of an arm of OpenArm v2, or of both its arms planned as one group, the fingers of each gripper checked with its arm, among boxes of voxels as an octomap, or among collision objects of each shape kind, a hand holding a cube, attached with its gripper as its touch links, in some of them; a plan of both arms checks the pairs of the robot's own links that motion_planning_core's self model checks, which the scenario names: MoveIt's answer to each scene, planned to the goal state itself or to a pose of each grasp point, which MoveIt's KDL inverse kinematics reaches, of each arm for a plan of both, whose goal MoveIt samples from the union of the arms' samplers, and MoveIt's collision check, smallest distances, to the scene and, where the scenario names pairs, between the robot's own bodies, and length along its path and along motion_planning_core's, which the scenario carries (`tools/draw_arm_planning.py` writes it, with the answers and the checked pairs of `motion_planning_core`'s example `plan_arm_scenes`, and with `--episodes` the scenes of episodes recorded in simulation, which the example `scenes_from_episodes` makes, with `--objects` the scenes among collision objects, with `--poses` the scenes whose goal is a pose of the grasp point, and with `--both-arms` the scenes of both arms, to a pose of each grasp point with `--poses`) | `motion_planning_core/tests/ros_reference_arm_planning.rs` |
| `planning_scene` | MoveIt's planning scene, its FCL collision environment, its path check and its octomap updater's shape mask: the objects, the attached objects and the allowed pairs that name an object after calls of motion_planning's scene members, as MoveIt's planning scene message holds them, the scenario's voxels its octomap from the start; at each configuration the signed distance of each pair of a link or an attached object and an object or the octomap, and of an attached object and a link it may not touch, with whether one body of a pair FCL finds apart lies wholly inside the other's closed mesh; the waypoints of each path of the arm that MoveIt's path check (`PlanningScene::isPathValid`) finds invalid, the rest of the robot standing still, and its dense probe of the path, a state every 1e-3 rad, whether a state is in collision and how far the arm stands from the objects and the octomap, and both again with the robot's links padded by the scenario's path padding; and whether each probe of the scenario lies inside an object the calls leave standing, grown by each padding as the shape mask grows it | `motion_planning_core/tests/ros_reference_planning_scene.rs` |

## Layout

| Path | What it holds |
|---|---|
| `scenarios/` | One JSON file per scenario. Its `kind` names the reference that runs on it. |
| `robots/` | The URDFs of the robots that the tools draw for scenarios, and their meshes: `primitive_arm.urdf`, an arm of 7 joints dressed in boxes, cylinders and spheres and in `meshes/wedge.stl`, turned about its origin. |
| `reference/` | The reference output of each scenario, under the scenario's file name; `SHA256SUMS`, the SHA-256 digest of each output; and `INPUTS.SHA256SUMS`, the SHA-256 digest of each file of the repository that the references read (each scenario and the files it names), by its path from the repository root, as the reference program lists them. Every number of an output is a JSON number, the shortest decimal that reads back to the reference's own bits. |
| `packages-amd64.txt`, `packages-arm64.txt` | The version of every package of the reference image, on each architecture. |
| `reference_programs/` | The reference program `ros_reference`: one function per kind of scenario. |
| `src/` | The Rust crate `ros_reference`: one module per kind of scenario, which lists the scenarios of the kind and reads each with its reference output, checked with the scenario and the files it names against their digests, for the parity test. |
| `tools/` | The scripts that write the scenarios, one per kind, with the robots they draw, and `drawing.py`, what they share: the paths, the generator of each scenario's draws, the `--drawn` option, a parsed URDF, the joints of an arm of OpenArm v2, the joint limits of a URDF, the configurations of a chain, the origin of a URDF element, the bounds of a link's collision elements, a rotation drawn uniformly, the constants of a Rust file, and `write`, which writes a scenario. |
| `Dockerfile`, `CMakeLists.txt`, `ros-snapshots.asc` | The reference image and the build of the reference program in it. |
| `run.sh` | Runs the reference of every scenario in the reference image. |

## Running the references

```sh
ros_reference/run.sh          # writes what the reference image gives
ros_reference/run.sh --check  # fails when that differs from the committed files
```

The script builds the reference image for the architecture of the host, and runs the
reference program on each scenario in it. It runs the image by the id that its build
gives, so each checkout runs the image of its own files, and `docker image prune` removes
the images of earlier builds. On a host without the base image, the build fetches about
750 MB on amd64 (the base image, 341 MB; the apt indexes, 54.5 MB; and 354 MB of
packages) and about 707 MB on arm64 (326 MB, 55.0 MB and 326 MB), and the image takes
about 3.7 GB of disk on amd64 and 2.6 GB on arm64.
The reference outputs and their checks run on the developer's machine; the Tests
workflow runs the parity tests on the committed outputs.

The host runs bash and a container engine as the `docker` command: Docker, rootful or
rootless, with buildx (its default builder or one of the docker-container driver) or with
its legacy builder; or Podman, rootful or rootless, which names itself in
`docker --version`, as the podman-docker package installs it. The container writes its
outputs as the user who runs the script. Both mounts carry `:z`, so on a host with SELinux
the host relabels the whole checkout for containers to share, and the checkout must be
one that the user lets containers read. On arm64, the host runs cargo too, and builds
each crate that holds a parity test with the system libraries that its README names.

On an amd64 host, `run.sh` writes `reference/`, with its `SHA256SUMS` and
`INPUTS.SHA256SUMS`, and `packages-amd64.txt`, and `run.sh --check` fails when a file
differs from what the image gives, and prints each differing line. The reference outputs
reproduce bit for bit on every amd64 host. Of the libraries that the reference program
loads, libc and libm alone hold IFUNCs, the functions that glibc resolves by CPU feature,
and the selectors of glibc 2.43's math functions read FMA, FMA4, AVX, AVX2 and SSE4_1
(libm) and SSE4_1, FMA, AVX2, AVX512F and AVX512DQ (libmvec). `run.sh` runs the reference
program with `GLIBC_TUNABLES=glibc.cpu.hwcaps=` set to mask all of them, with SSSE3,
SSE4_2, POPCNT, BMI1, BMI2, LZCNT, MOVBE and AVX512BW, CD, VL, ER and PF, so glibc runs
the SSE2 variants of the math functions, of baseline instructions alone, on every amd64
host. glibc 2.43 also masks CMOV, CX8, SSE2, HTT, IBT, SHSTK, RTM, ERMS, XSAVE, XSAVEC
and OSXSAVE by name; no math selector reads them, and the mask leaves them.

On an arm64 host, `run.sh` writes `packages-arm64.txt`, and `run.sh --check` fails when it
differs from what the image gives. After the write or the check, both run each parity
test, `tests/ros_reference_*.rs` of each crate, on the outputs of the arm64 image, which
the environment variable `ROS_REFERENCE_OUTPUTS` names to the test, and fail when a test
does. The committed reference outputs are those of the amd64 image, and tests compare bits
on amd64 alone, but for the occupancy test, which rule 4 describes. The planning parity
test also reads the committed outputs on arm64, and compares each simplification with
them within its bound.

## Recorded episodes

`tools/draw_arm_planning.py --episodes <directory> --launcher <fragment>` reads episodes
of OpenArm v2 recorded in simulation, one plan request of an arm each: each subdirectory
of `<directory>` that holds an `episode.json`. A stamp is in seconds on the clock of the
robot's messages, and a pose is a `position` in meters and an `orientation`, a unit
quaternion [x, y, z, w], in the robot frame unless the member says otherwise.

| Member of `episode.json` | What it holds |
|---|---|
| `index` | The episode's number; the scene's seed is it plus one. |
| `arm` | The planned arm's limb name, `left_arm` or `right_arm`. |
| `start` | The arm's measured joints at the request, in radians. |
| `goal` | The request's pose goal: `link`, the arm's grasp point; `position`; `orientation`; `tolerance_m`; `tolerance_rad`. |
| `camera` | The depth camera as the motion planner reads it: `color_in_robot`, the pose of its color optical frame; `depth_in_color`, the pose of its depth optical frame in the color one; `intrinsics`, its depth stream's `width`, `height`, `fx`, `fy`, `cx`, `cy`, `distortion_model`, `distortion` and `depth_model`; `depth_unit`, the meters of a sample step; and `align_mode`. |
| `limb_states` | Each limb_states message of the recording: `at`, its stamp; `joints`, each arm's joints by limb name; `openings`, each gripper's opening by gripper name. |
| `frames` | Each depth frame that the planner's map took in between the recording's clear of the map and its plan request: `at`, its stamp; `encoding`, `z16`; `width`; `height`; `align_mode`; and `samples`, the file of its samples, little-endian u16, relative to the episode's directory. |

Its other members are for the reader. `motion_planning_core`'s example
`scenes_from_episodes` turns each episode into a scene in the map of the planner's
arguments that the launcher fragment gives, and names each episode it drops and why.

## Rules

1. A reference runs in the reference program's process. A reference with a plain C++ API
   is called directly. A reference that reads its parameters from a ROS node creates that
   node in the reference program's process, sets the scenario's parameters on it, and
   calls the reference on the program's own thread. It initializes rclcpp with ROS
   logging unconfigured: configuring it creates a log directory in the home directory,
   and the container runs as the host's user, whom the image may give none.
2. A reference gives the same output on every run: the container has no network and one
   OpenMP thread, every random generator of a reference is seeded from its scenario, and a
   search ends on an iteration bound that the scenario gives. MoveIt's samplers draw from
   generators of random_numbers that they construct by its default constructor, which
   random_numbers seeds from the clock; the reference program defines that constructor and
   exports it, so MoveIt's libraries construct their generators through it, and the arm
   planning reference seeds them from each scene, and std::rand, which KDL's step off a
   singularity draws from through Eigen. Two things of a pose goal's scene are MoveIt's
   own: KDL's search for a goal ends on its timeout, a clock, and the planner's thread and
   the goal sampler's each build a generator, whose seeds follow the order they are built
   in. The committed pose scenes repeat their bytes in every run measured, each alone and
   after the others, in six processes and under load.
3. A comparison that is statistical by nature, a sampling planner's against another's,
   runs over a drawn set, locally: draw the set with the `--drawn` option of the kind's
   script, run `run.sh`, and run the crate's parity test with `cargo test -- --ignored`,
   which holds the statistical test; the pull request states its tally at its head. For
   arm planning, a set of recorded episodes (see "Recorded episodes") runs the same way:
   `tools/draw_arm_planning.py --episodes <directory> --launcher <fragment>` writes their
   scenes in place of the drawn ones, each the map the motion planner of the launcher
   fragment builds of the episode's depth frames, its measured start, and a goal
   configuration drawn for its pose goal. The arm planning script's `--objective shortest`
   plans each scene with RRTstar, and its `--planning-time <seconds>` ends MoveIt's plans
   on that time alone, their allowed planning time, and motion_planning_core's `any` plans
   on that time or the iteration bound, whichever comes first, as the robot runs (its
   motion planner's launcher fragment bounds a plan by `max_iterations`); a set planned so
   gives other paths on each run, and runs locally, never committed. Its
   `--min-clearance <meters>` plans at that clearance: motion_planning_core's
   `min_clearance_m`, and the padding of each of MoveIt's links against the octomap and
   the objects, the robot meeting itself unpadded, as a collision request's defaults ask.
   The scenes of one arm plan it without its self model, and MoveIt's links may all meet
   each other, so no part of a gap at a clearance comes from this crate's clearance within
   the robot's own body. The object the hand holds keeps the clearance from the parts of
   the robot that stand still, and the arm and that object keep it from the object the
   other hand holds, which MoveIt holds as an attached body and checks against them
   unpadded. MoveIt pads no held object, which takes the padding of the grasp link, a link
   with no collision geometry, which MoveIt pads by 0 (moveit2 2.15.2
   collision_env.cpp:127-133 and :166-176, collision_env_fcl.cpp:195-196): it keeps an
   object a hand holds unpadded from the objects and the octomap, and the tally counts the
   paths of each that bring it nearer than the clearance and 1 mm. MoveIt pads a link's
   mesh by moving each vertex along its normal (geometric_shapes 2.3.4 shapes.cpp:402-404),
   so a face beside a right-angled edge moves out the padding over the square root of 2,
   and one at a box's corner the padding over the square root of 3. The scenes of both arms
   (`--both-arms`) plan them with this crate's self model, and MoveIt's collision matrix
   checks the pairs of the robot's own links that the self model checks, the scenario's
   `checked_pairs`, and each object a hand holds against the other arm and what that arm
   holds. MoveIt checks the robot against itself unpadded, and this crate keeps the
   clearance between the robot's own bodies too, so a gap between them at a clearance above
   0 comes from this crate's clearance. Each drawn scene of both arms starts where the self
   model holds the robot's own bodies apart, and its goal of joints, or the configuration
   whose grasp poses its goal of poses holds in every other scene, stands at least
   `min_clearance_m` apart, which the example `self_distances` measures: a nearer goal is
   one this crate refuses and MoveIt, checking them unpadded, reaches. The self model's hulls
   enclose the meshes MoveIt checks, so MoveIt holds them apart there too. A path of both
   arms at a clearance above 0 holds it where MoveIt's may come nearer, so the test judges
   its length on the scenes both plan where MoveIt's path keeps the clearances, and prints
   the mean over every path beside it. Each arm planning scenario has its own
   statistical test, of one arm or of both, among boxes, among objects or to poses, which a
   filter runs: `cargo test --test ros_reference_arm_planning -- --ignored among_objects`
   runs the test of one arm among objects, and `both_arms` the three of both arms. The
   statistical test refuses a set too small for its intervals. The committed cases of such
   a kind compare bit for bit, as every other output does: the arm planning scenario plans
   `any` on its iterations.
4. A parity test states what the library guarantees, and the bound within which its
   outputs agree with the reference, or the statistic it compares, and why. A number of an
   output reads back to the bits that the reference computed, so a test compares with the
   reference's own double. A test that compares the exact bits of an output runs under
   `#[cfg(target_arch = "x86_64")]`, since the committed outputs hold amd64 bits. The
   occupancy test compares bits with the committed outputs on every architecture, since
   the map under test gives the amd64 bits on arm64 too. Against the outputs that
   `ROS_REFERENCE_OUTPUTS` names, it counts the voxels that differ, held by one map alone
   or at other log-odds, and fails when more than 1% of the reference's voxels do:
   MoveIt's arm64 build fuses multiply-adds in its point cloud callback (31 of them in
   `cloudMsgCallback`). Its map of `occupancy_room` equals the amd64 one in all 398
   voxels, and that of a drawn set of 60 points of each drawn case (rule 6) differs from
   the amd64 one in 6 of 7187 voxels (0.08%). The arm planning test plans and compares
   this crate's paths, and asks for a scene of each outcome, on amd64 alone, where the
   committed outputs hold the answers of MoveIt that the scenes were picked by. Against
   the outputs that `ROS_REFERENCE_OUTPUTS` names, it holds every probe of those paths
   free in MoveIt's check and each refusal as MoveIt's. MoveIt's arm64 build answers the
   committed scenes among boxes otherwise: it refuses the same four starts and goals, and
   of the four scenes it plans, it plans the straight path to other last bits and another
   path with corners, and its own check passes the path that fails it on amd64.
5. One rule lays out every scenario and every reference output: a value goes on one line
   when it is a scalar, an array of scalars, or a record, an object whose members are
   each a scalar or an array of scalars; every other array or object spreads over lines,
   one member a line, indented by two spaces. So a forward kinematics scenario holds one
   configuration a line, and its output one link pose a line. The reference program
   writes every output by it through `write_json`, and every scenario tool writes every
   scenario by it through `write` of `tools/drawing.py`. A number goes as the shortest
   decimal that reads back to its double: `write_json` writes each float through
   `shortest_decimal`, as `std::to_chars` writes it, and refuses a NaN and an infinity,
   which JSON has no number for; Python's `json` writes each float of a scenario as
   `repr` does. On each run, before its reference, the reference program checks with
   `check_shortest_decimals` that `shortest_decimal` writes the doubles at the edges of
   the format as the reader test of `src/lib.rs` reads them, and writes a seeded draw of
   bit patterns as JSON numbers that read back to their bits.
6. A kind commits only the few cases that exercise its behaviors: for forward kinematics,
   eight configurations a chain, all joints at zero, at their lower limits, at their upper
   limits, and five drawn inside them; for chain collision, four configurations a chain,
   all joints at zero, at their lower limits, at their upper limits, and one drawn inside
   them, with four probe points a link at each, and at the first, for each padding, the
   points 10 µm inside and outside the surface of each grown primitive; for self distance,
   the three configurations of OpenArm v2, of the zero configuration, the lower limits,
   the upper limits and thirty drawn inside them, that put each pair of links in each
   state, apart and in contact, that any of them puts it in; for path timing, one path of
   each case of `tools/draw_path_timing.py`, a cornered path blended at each of the four
   deviations, a short-leg path, a near-reversal path, a straight run and a cruising one,
   a path that turns back, two corners either side of TOTG's turning-back threshold and
   two either side of its straight-on threshold, each sampled at its start, its end, the
   middle of each straight piece and three points along each arc; for planning, every
   planner through a wall with a block before it in two dimensions under seed 1 and in
   seven under seed 2, and to a goal walled in, where it finds no path, under seed 1, with
   RRTstar under seed 2 in two dimensions too, each plan bounded at 2000 iterations, at
   which PRM and PRM* grow their roadmap, expand it and grow it again; for occupancy, a
   sensor that reads a scene five times and then a later cloud, with the points of each
   case of `tools/draw_occupancy.py`, one of each drawn case, scattered within the range,
   inside the robot's sphere and beyond the range, a hit on the ray of another hit, a
   voxel that the later cloud crosses after the scene took it to the upper clamp, a
   cut-out voxel on the ray of another point, and a voxel that holds a cut-out point and a
   hit, and the level sensor's two points beyond the range at a voxel face; for arm
   planning, in each of its scenarios, among boxes of voxels and among collision objects,
   one drawn scene of each outcome of `tools/draw_arm_planning.py`, a straight path and a
   path with corners that MoveIt plans too, the scene whose path has the most corners
   among the first 200 drawn, which the script picks, a path of `motion_planning_core`
   where MoveIt's own path fails its check, a start and a goal in a box or an object that
   both refuse, and a start below a lower limit and one above an upper limit, the scenes
   among objects holding a box, a cylinder, a closed mesh and an object of two shapes
   among them, the hand holding a cube in one and a cylinder it sets down on a table it
   may touch in another, and the other hand holding a cube in one, and to poses, each such
   scene (`COMMITTED_POSES`); of both arms, among boxes, among objects and to a pose of
   each grasp point, each such scene, a straight joint line through the robot's own body
   that both plan around, the scenes among objects holding the left hand's object, the
   right hand's cube, both, and the left hand's object set down on a table, and the scene
   whose start, and the one whose goal, holds the robot's own bodies meeting, to poses
   each but a path of `motion_planning_core` where MoveIt's own fails its check, which no
   scene of the first 200 drawn gives, and the straight joint line, which a goal of poses
   has none of (`COMMITTED_BOTH`, `COMMITTED_BOTH_OBJECTS` and `COMMITTED_BOTH_POSES`); its
   statistics over 200 drawn scenes as rule 3 says; for the planning scene, the calls of
   `tools/draw_planning_scene.py` on OpenArm v2 and eight configurations of its left arm,
   all joints at zero, at their lower limits, at their upper limits, the one an item and a
   lid are attached at, the one the lid is detached at, and three drawn inside the limits
   that each place objects around a link: a sphere that holds a link, beside a box that
   fills a link it may touch; a link wholly inside a closed box mesh; and a link in the
   open corner of an L-shaped mesh; sixteen voxels, those the table, the item and the cap
   fill, one beside a link and twelve drawn, and two the calls mark, where the sphere
   moves to and where the item is attached, before the move and the attach; twenty paths,
   each two configurations in turn, the first through a post between two clear waypoints,
   all eight in one, and twelve from the zero configuration, the lower limits or a
   configuration drawn inside the limits to one within 0.3 rad of it in each joint; and
   twelve probes about each of eleven objects at three paddings. A large drawn set runs
   locally and is never committed: draw it into `scenarios/` with the `--drawn` option of
   the kind's script (for forward kinematics,
   `python3 ros_reference/tools/draw_forward_kinematics.py --drawn 500`), run
   `ros_reference/run.sh` and the parity tests, and restore the committed files with
   `git restore ros_reference/scenarios ros_reference/reference`.
7. Every mode of arm planning that a caller selects, each objective and planner that the
   contracts expose, the arms it plans, one or both as one group (`--both-arms`), the
   planning scene it plans in, among collision objects and holding an attached object
   (`--objects`), and the goal a pose of the grasp point that MoveIt reaches through KDL's
   inverse kinematics (`--poses`), has its own arm planning scenario and tally against
   MoveIt at each one's production termination (`--planning-time`): MoveIt's plans ended
   on their allowed planning time alone, and this crate's on that time or the robot's
   iteration bound, whichever comes first, at MoveIt's default of 5 s (moveit2 2.15.2
   `MoveGroupInterfaceImpl`, `allowed_planning_time_ = 5.0`) and at the time that the
   robot's callers ask, over the drawn set, and over the recorded episodes for a mode whose
   goal they give, a configuration, as rule 3 says, at a clearance of 0 and at the
   clearance of the robot's launch (`--min-clearance`), each plan of this crate sampling the
   launch's goals (its fragment's `goal_samples`). The pull request states which modes its
   tally covers.

## Pins

Every input of the image is pinned, so a rebuild gives the same packages:

- the base image `docker.io/library/ros:lyrical-ros-base`, by the digest of its index of
  amd64 and arm64;
- the ROS packages, by a dated snapshot of the ROS repository (`ROS_SNAPSHOT`), signed by
  the key in `ros-snapshots.asc`, which is valid until 2027-06-01;
- the Ubuntu packages, by a dated snapshot of the primary Ubuntu archive
  (`UBUNTU_SNAPSHOT`), which holds amd64 and arm64. The Dockerfile names the primary
  archive in place of the ports archive that the arm64 base image names.

The Ubuntu snapshot is 2026-10-27T00:00:00Z, and the base image was built at
2026-10-27T00:32:46Z for amd64 and at 2026-10-27T00:32:38Z for arm64. On both
architectures, the two snapshots hold the version of every package of the base image, and
apt keeps those versions.

`packages-amd64.txt` and `packages-arm64.txt` record the version of each package that
results. To move a pin, change the Dockerfile, run `run.sh` on an amd64 host and on an
arm64 host, and review the diff of the package lists and of `reference/` in the same pull
request: a reference output that changes is a change of the reference.

### Renewing the ROS snapshot key

`ros-snapshots.asc` is valid until 2027-06-01. To renew it, fetch the key from the
Ubuntu key server and print it:

```sh
(
  export GNUPGHOME=$(mktemp -d)
  gpg --keyserver hkps://keyserver.ubuntu.com --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
  gpg --armor --export 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA > ros_reference/ros-snapshots.asc
  gpg --show-keys ros_reference/ros-snapshots.asc
  rm -rf "$GNUPGHOME"
)
```

The fingerprint that the last command prints must be
`4B63 CF8F DE49 746E 98FA 01DD AD19 BAB3 CBF1 25EA`, the key that
<https://wiki.ros.org/SnapshotRepository> names. Write the expiry date that it prints in
this file and in the Dockerfile, and run `run.sh --check` on an amd64 host and on an
arm64 host.

## Adding a scenario

1. Write the scenario under `scenarios/`. For forward kinematics, add the robot to
   `ROBOTS` in `tools/draw_forward_kinematics.py` and run
   `python3 ros_reference/tools/draw_forward_kinematics.py`, which writes every scenario
   of the kind, each drawn from a generator seeded by its own name, so the others come
   out unchanged.
2. Run `ros_reference/run.sh` on an amd64 host, which writes its reference output and the
   digests, and `ros_reference/run.sh --check` on an arm64 host, which runs the parity
   tests on the outputs of the arm64 image.
3. The parity test of its kind lists the scenarios of the kind from `scenarios/`, so it
   runs on the new one. The forward kinematics parity test poses chains of 1 to 7 joints,
   the chain collision parity test chains of 7 joints, the self distance parity test
   builds the self model of OpenArm v2 and refuses a scenario that names another URDF, the
   path timing parity test asks each scenario for cornered paths that TOTG blends at
   deviations of 0.01, 0.05, 0.1 and 0.2, a straight run, a path that it refuses, a path
   with a corner that it runs straight on, turns that bracket its turning-back threshold
   within 0.02 degrees, and a path with corners at the deviation of execute_joint_path,
   the planning parity test asks each scenario for a plan of each planner that finds a
   path and one that finds none, the occupancy parity test asks each scenario's map for at
   least 199 voxels, the arm planning parity test asks each scenario for a scene of each
   outcome that it names, and each scenario of both arms for a scene of each outcome of a
   plan of both, and the planning scene parity test builds the self model of
   OpenArm v2, refuses a scenario that names another URDF or poses another arm, and asks
   each scenario for a pair of each case that it names.

A routine test of `ros_reference` fails when a scenario has no reference output, a
reference output has no scenario or no digest, or a scenario, a URDF or a mesh differs
from its digest.

## Adding a kind of scenario

1. In `src/`, add the kind to `Kind` in `lib.rs`, and a module with a `kind` tag type of
   its own, which its scenario and its reference output hold, so each refuses another
   kind. The module lists the scenarios of the kind with `scenario_names`, reads each with
   `read_scenario` and its reference output with `read_reference`, and every other file
   of the repository that the reference reads with `read_input`, each checked against its
   digest, and pairs them for the parity test.
2. In `reference_programs/`, add the function that runs the reference on a scenario of
   the kind, reading every file of the repository through `Inputs::read`, which lists it
   for `INPUTS.SHA256SUMS`; a file that a ROS library reads, as MoveIt loads a mesh, goes
   to it by `Inputs::file_uri`, which reads it through `Inputs::read`. Add the function to
   `references` in `main.cpp` and to `CMakeLists.txt`, and install the ROS packages it
   needs in the Dockerfile.
3. In `tools/`, add a script that writes the scenarios through `write` of `drawing.py`,
   its random draws from the `generator` of each scenario, seeded by the scenario's name.
4. Run `run.sh` on an amd64 host, which writes their reference outputs and
   `packages-amd64.txt`, and on an arm64 host, which writes `packages-arm64.txt`.
5. In the crate under test, add `ros_reference` as a dev-dependency and a parity test,
   `tests/ros_reference_<kind>.rs`, that loads every scenario of the kind with its
   reference output and compares, by the rules above. Add the kind to the table at the top
   of this file.
