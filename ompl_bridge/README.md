# ompl_bridge

OMPL's geometric planners from Rust. A plan runs through a real vector state space with
bounds, by a planner chosen by name, under the caller's state and motion checks, and
optionally on the zero set of the caller's constraint; OMPL's path simplifier then
shortens the path. The crate builds OMPL 2.0.2, the release that the ROS reference
image holds, from the source archive of its release tag (see "Building").

- `plan` runs one `Request` and answers a `Plan`: the path when the planner found an
  exact one, its simplification when the request asks for it, and what ended the
  planner and the simplifier.
- `Planner` names the planners: RRTConnect, RRT, RRTstar, BiTRRT, KPIECE1, BKPIECE1,
  LBKPIECE1, EST, SBL, PRM and PRMstar, spelled as OMPL spells their classes.
- `Checks` and `Constraint` are the caller's functions, called from OMPL.

## Quick start

```rust
use std::num::{NonZeroU32, NonZeroU64};
use std::time::Duration;

use ompl_bridge::{CheckError, Checks, Motion, Request, Space, plan};

/// The open cube of half-width 1 about the origin. It is convex, so a straight motion
/// from one of its states stays in it when its end does.
struct Cube;

impl Checks for Cube {
    fn state(&self, state: &[f64]) -> Result<bool, CheckError> {
        Ok(state.iter().all(|x| x.abs() < 1.0))
    }

    fn motion(&self, _from: &[f64], to: &[f64]) -> Result<Motion, CheckError> {
        Ok(if self.state(to)? {
            Motion::Valid
        } else {
            Motion::Invalid { last_valid: 0.0 }
        })
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let space = Space::new(&[(-1.0, 1.0); 7])?;
    let goals = [vec![0.5; 7]];
    let answer = plan(
        &Request {
            planner: "RRTConnect".parse()?,
            space: &space,
            start: &[0.0; 7],
            goals: &goals,
            time_limit: Duration::from_secs(1),
            iterations: NonZeroU64::new(10_000).ok_or("a bound of 0 iterations")?,
            seed: NonZeroU32::MIN,
            simplification: NonZeroU32::new(100),
            projection: None,
        },
        &Cube,
    )?;
    let path = answer.simplified.ok_or("no path within the bounds")?.path;
    println!("a path of {} states", path.len());
    Ok(())
}
```

## Repeating a plan

OMPL seeds each of its random number generators from one seed generator of the process.
A plan reseeds it with the request's seed before it builds anything, and holds it until
it ends, so its planner, samplers and simplifier take their seeds in the order of its
own construction. One plan runs at a time in a process: a plan waits for another until
its time limit, and `plan` refuses a call from a check or the constraint of a plan.

The planner ends when it returns, when it asks whether to stop once more after it has
been told to go on as many times as the request's iteration bound, or at the time limit,
whichever comes first, and `Plan::ended_by` says which. The questions of the thread that
calls `plan` alone count toward the bound. The simplifier ends the same way under the
simplification bound, and `Simplification::ended_by` says which. A plan that the time
limit ends neither in its planner nor in its simplifier repeats, path for path and
simplification for simplification, in the same process and in another, for every
planner but PRM, with any number of goals.

PRM and PRM* plan as OMPL's `PRM::solve` does, with five changes. Three make their
roadmap depend on the plan's questions whether to stop and on nothing else. They take
every goal into the roadmap after the start, before the thread that searches the roadmap
starts, where OMPL's search thread takes each goal after the first in a search of its
own, whenever that search runs. They grow and expand the roadmap in turn, in rounds of
1000 and 500 questions, where OMPL's rounds last 0.4 s and 0.2 s (`ROADMAP_BUILD_TIME` in
`PRM.cpp`), so a slow host builds the roadmap that a fast one builds. And once the
planner has ended, the plan searches the finished roadmap. The fourth sets the best cost
found to infinite before the search thread starts, where OMPL sets it after starting that
thread, which may have written it by then. The fifth leaves out the messages of
`PRM::solve`, and its refusals of a goal that is not a sampleable region and of one that
cannot be sampled, which the plan's goals, an OMPL `GoalStates` of at least one state,
never meet. The search thread's questions do not count toward the bound. PRM ends at the
first path that its search finds, and that path depends on when the search runs; whether
PRM finds a path repeats, and so does PRM*'s path, the shortest in the finished roadmap.

## The time limit

`Request::time_limit` bounds `plan` from its call: its wait while another plan runs, its
planner and its simplification, which starts only before the limit. The planner and the
simplifier end at their first question whether to stop past the limit. What runs between
two questions runs to its end, a call of the caller's functions among it, and so do the
checks of the start and the goals and the simplifier's first pass: OMPL's simplifier runs
that pass to its end whatever its termination condition answers, and bounds it by counts
of attempts, most of them the path's number of states. A plan of PRM or PRM* runs two
more steps to their end whatever the limit. Before its first question, it takes every
goal into the roadmap, with the motion checks of each. And once its planner has ended, it
searches the finished roadmap, with one A* search for each goal that the roadmap joins to
the start, unless PRM's search found a path that ended the planner. Planning the
seven-dimensional wall of `tests/plans.rs` under seed 2 until the limit, in a container
of two CPUs on an x86-64 host, PRM* answered 13.7 ms past a limit of 1 s with one goal,
34.4 ms past it with 32 goals and 55.4 ms past a limit of 3 s with 32 goals, while RRT*
answered within 1.3 ms of each limit: a measurement on that host.

## The simplified path

OMPL's simplifier shortens the path, smooths it with B-splines, and repairs a smoothed
state that fails the checks. It reports when its result may still touch an invalid
state. `Simplification::path` is its result when OMPL reports it valid and it is at most
as long as the planned path, and the planned path otherwise. The simplifier counts its
questions whether to stop toward the request's simplification bound, and stops at the
bound once its first pass has ended. A simplification that the time limit ends before
it starts gives the planned path, and `Simplification::ended_by` says `TimeLimit`.

## Constrained plans

A constrained plan projects its states onto the zero set of the `Constraint` by Newton
steps, as OMPL's projected state space does, within the projection's tolerance. It walks
each motion along the constraint in steps of half `CONSTRAINED_SPACING`, and judges each
state of the walk with `Checks::state`. The plan walks its path and its simplification
out along the constraint the same way, as OMPL's demos of constrained planning and
MoveIt interpolate a constrained path: consecutive states lie at most
`CONSTRAINED_SPACING` apart, and each state the walk adds between two states of the
planner lies within the projection's tolerance of the constraint.

## The caller's functions

`Checks::state` judges a state and `Checks::motion` the straight motion between two
states, its end included: OMPL takes the motion's start as valid. When a motion is
invalid it answers the fraction of the way along it to its last valid state. A plan calls
both on the thread that calls `plan`, and shares them with the thread on which PRM and
PRM* search their roadmap, so both traits are `Sync`.

Before the planner starts, the start passes the state check and lies on the constraint,
or the plan answers `PlanError::InvalidStart`. The plan keeps the goals that pass, and
answers `PlanError::InvalidGoal` when none does. A start or a goal outside the space's
bounds is refused with `PlanError::Request`.

An error or a panic in any of the caller's functions, in the error's message or in its
drop, ends the plan with `PlanError::Check`, which says what the first one said. It
never reaches C++: the bridge answers OMPL a failed check, and the plan's termination
condition ends the plan. The constraint's function and Jacobian find their output filled
with NaN, so a value that a failed call leaves unwritten is NaN, which OMPL takes as off
the constraint. The process goes on. The bridge catches a panic in a binary that unwinds
on panic, Rust's default; in a binary built with `panic = "abort"`, a panic in the
caller's functions aborts the process.

## Parity with OMPL

A plan of every planner but PRM that its iteration bound ends repeats, path for path,
with any number of goals, and a plan of PRM repeats whether it finds a path (see
"Repeating a plan"). `tests/ros_reference_planning.rs` gives the evidence that the plan
is OMPL's: it plans every `planning` scenario of the [`ros_reference`](../ros_reference)
harness, which a C++ program plans in the reference image with the image's OMPL 2.0.2,
built, seeded and bounded as this crate builds, seeds and bounds a plan, with the same
stop rule and the same rounds of roadmap growth, through checks of the same arithmetic.
Every plan finds an exact path exactly where the C++ program's does, and every plan of a
planner other than PRM gives the C++ program's ending, and its path and the path's
length within 1e-12, against the outputs of each image, and its simplification within
1e-12 of the committed outputs of the amd64 image on every architecture; on x86-64, it
gives the path, the length and the simplification of the committed outputs bit for bit.
The arm64 image's simplifier branches on the last bits of its fused paths, and 6 of its
21 simplifications differ from the amd64 ones beyond 1e-12, while this crate built for
aarch64 gave the committed bits of every simplification on a Jetson. The scenario holds a
plan of each planner that finds a path and one that finds none. PRM's path is the first
that its search finds, so the test compares whether it found one alone.

## Building

The build compiles part of OMPL 2.0.2, the release that OMPL tags `2.0.2` at
<https://github.com/ompl/ompl> (commit `1ae6ac0b`), from the source archive of that tag:
the utilities, the data structures, the base module, the geometric core and the eleven
planners here, the configuration tools, and the control core with RRT and KPIECE1, which
the configuration tools name. `build.rs` lists each source it compiles. The base module
comes without the state spaces that no other source of it uses: the discrete, hybrid,
SE(3), SO(3), Klein bottle, Mobius strip, sphere and torus spaces. The Dubins,
Reeds-Shepp, Owen, Vana, VanaOwen and Trochoid spaces and SE(2) stay, since OMPL's space
information chooses its default motion validator among them.

The build downloads the archive from
<https://github.com/ompl/ompl/archive/refs/tags/2.0.2.tar.gz> and checks it against its
SHA-256 digest, `d867190a46a7f730923475a68af2e09fc9e796017a59be395a4bf3900d0c3ae4`. It
stops on an archive of another digest, and names the expected digest and the actual one.
It extracts the release's `LICENSE` and its `src/ompl` into its output directory with a
marker of the digest, and a later build into that directory reuses the extraction and
downloads nothing. In a peppy container build, which starts from an empty target
directory each time, `PEPPY_DOWNLOAD_CACHE` names peppy's download cache: the build reads
the archive from there, stored under its digest, when the file there has that digest, and
otherwise downloads it and stores it there in place of the file, so the archive downloads
once on a machine. A download that has not connected within 30 s, or has not ended within
10 minutes, stops the build with the hint to name a copy of the archive in
`OMPL_SOURCE_TARBALL`, or to store one in the download cache under its digest. To build
offline, name a copy of the archive by its absolute path in `OMPL_SOURCE_TARBALL`: the
build reads it in place of the cache and the download, and checks its digest each time
the build script runs. The archive and the crate's dependencies, fetched once online,
then build offline:

```sh
curl -fLo ompl-2.0.2.tar.gz https://github.com/ompl/ompl/archive/refs/tags/2.0.2.tar.gz
cargo fetch
# Offline from here.
OMPL_SOURCE_TARBALL="$PWD/ompl-2.0.2.tar.gz" cargo build --offline
```

OMPL is under its BSD license. A binary that links this crate links OMPL, and the
license's second clause asks a binary distribution to reproduce OMPL's copyright notice,
conditions and disclaimer. The build extracts the license to `ompl-2.0.2/LICENSE` in its
output directory, `target/<profile>/build/ompl_bridge-<hash>/out/`, which cargo reports
as the `out_dir` of ompl_bridge's `build-script-executed` message under
`--message-format=json`: ship that file with the binary.

The build compiles OMPL against the system's Boost headers, links Boost.Serialization,
which OMPL's planner data storage uses and OMPL's CMake links, and finds Eigen 3 through
pkg-config. On Debian and Ubuntu:

```sh
apt-get install libboost-serialization-dev libeigen3-dev pkg-config
```

The build stops with that hint when it finds no Boost or no Eigen. It compiles with
`-ffp-contract=off`, so each product and each sum of the bridge and of OMPL's own sources
rounds on its own on every target, as on the x86-64 baseline. The flag does not reach
Eigen's vectorized kernels, which fuse multiply-adds by intrinsics on aarch64 and in an
x86-64 build with FMA enabled. OMPL calls them to project the states of a constrained
plan onto its constraint, and to place states in the grid of KPIECE1, BKPIECE1,
LBKPIECE1 and SBL in more than two dimensions. On a Jetson, an aarch64 build gave every
plan of the parity test the bits of the x86-64 baseline, the seven-dimensional plans of
KPIECE1, BKPIECE1, LBKPIECE1 and SBL among them, whose grid placement fuses: a
measurement of that scenario, which a plan whose fused placement lands on another side
of a cell's edge does not share.
OMPL prints its warnings and errors to standard error.
