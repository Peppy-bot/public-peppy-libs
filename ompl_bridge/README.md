# ompl_bridge

OMPL's geometric planners from Rust. A plan runs through a real vector state space with
bounds, by a planner chosen by name, under the caller's state and motion checks, and
optionally on the zero set of the caller's constraint; OMPL's path simplifier then
shortens the path. The crate builds OMPL 2.0.2, the release that the ROS reference
image holds, from the source archive of its release tag (see "Building").

- `plan` runs one `Request` and answers a `Plan`: the path when the planner found an
  exact one, its simplification when the request asks for it, and what ended the planner
  and the simplifier. A request of several attempts makes them `PLANNING_THREADS` at once
  and joins their paths with OMPL's path hybridization, as MoveIt joins the attempts of a
  request (see "Attempts").
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
            attempts: NonZeroU32::new(4).ok_or("no attempts")?,
            simplification: NonZeroU32::new(100),
            simplification_time: Duration::from_millis(200),
            projection: None,
            stop: None,
        },
        &Cube,
    )?;
    let path = answer.simplified.ok_or("no path within the bounds")?.path;
    println!("a path of {} states", path.len());
    Ok(())
}
```

## Repeating a plan

OMPL seeds each of its random number generators from a seed generator, which the
crate's patch of OMPL gives each thread of its own (`ompl::RNG::setThreadSeed`, see
"Building"). Each attempt of a plan seeds the generator of its thread before it builds
anything, attempt 0 from the request's seed and each other from a seed derived from it,
so its planner, samplers and simplifier take their seeds in the order of its own
construction, whatever runs on other threads. A plan of several attempts joins and
simplifies their paths in a problem of its own on the thread that calls `plan`, seeded
the same way from the seed of index `attempts`, one past its last attempt's. Plans run
at once in a process. `plan` refuses a call from a check or the constraint of a plan: a
plan there reseeds the generator of the thread that the other plan draws from.

Each attempt's planner ends when it returns, when the caller's stop flag is set, when it
asks whether to stop once more after it has been told to go on as many times as the
request's iteration bound, or at the time limit, whichever comes first, and
`Plan::ended_by` says which; of several attempts, it says the stop when the stop ended
any, then the time limit, then the iteration bound, and that they returned when each did.
The questions of the attempt's own thread alone count toward the bound. The simplifier
ends the same way under the simplification bound and the simplification's time, and
`Simplification::ended_by` says which. A plan that neither a time limit nor the caller's
stop ends, in its planners or in its simplifier, repeats, path for path and
simplification for simplification, in the same process and in another, for every planner
but PRM, with any number of goals and of attempts.

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

## Attempts

`Request::attempts` is MoveIt's `num_planning_attempts`. One attempt plans on the thread
that calls `plan`. Several plan as MoveIt's `ModelBasedPlanningContext::solve` plans them
through OMPL's `ParallelPlan`, each on a thread of its own: `PLANNING_THREADS`, 4, at
once, MoveIt's default `max_planning_threads`, and each further batch of as many once the
one before it has ended, while the time limit lasts and the plan is not stopped. Each
attempt poses the request's problem on its thread, checks its start and its goals, and
plans under the whole time limit and the whole iteration bound. The plan joins the exact
paths of two or more attempts with OMPL's `PathHybridization`, as `ParallelPlan` joins
them: it records each in turn, joins it to each recorded before it where their states
align by the motions that pass the motion check, and takes the shortest path through them
all, the hybrid. It keeps the shortest of the attempts' paths and the hybrid, the first
of the equally short in attempt order and the hybrid last, and simplifies it.

The plan departs from MoveIt in seven ways, each a choice its caller sees:

- MoveIt's attempts draw the seeds of their samplers from one seed generator of the
  process, in the order their threads construct them, and its hybrid records each path
  as the attempt that found it returns. Here each attempt draws from its own seed, and
  the hybrid records the paths in attempt order once every attempt has ended, so a plan
  of several attempts repeats.
- OMPL joins two paths by a motion it checks from a state of the later path to one of
  the earlier, and its hybrid may take that motion the other way. MoveIt keeps the
  hybrid as OMPL joins it; here it stands only when its start passes the state check and
  each of its motions the motion check, in its order. That check runs the motion check
  once more for each motion of the hybrid.
- The iteration bound counts the questions of each attempt's own thread. MoveIt's
  iteration termination condition, when its planner configuration names one, counts the
  questions of every attempt on one count.
- MoveIt's simplification takes what is left of the request's allowed planning time once
  the attempts have ended. Here it takes `Request::simplification_time`, counted from the
  end of the time limit, whatever the attempts leave of the time limit.
- A stopped plan makes no hybrid and no simplification. MoveIt's `terminate` ends the
  planners, and its hybrid of the paths found by then runs to its end.
- MoveIt's batches add their paths to one problem, and it takes a batch as solved when
  that problem then holds an exact path (`ModelBasedPlanningContext::solve`, the
  `result.val ... && r` of each batch). So a request of more attempts than one batch
  fails unless its first batch finds an exact path. Here a plan answers a path when any
  of its attempts finds one.
- MoveIt's hybrid records every path its planners answer, the approximate ones among
  them (`ParallelPlan::solveMore` records each solution of the problem once a planner's
  `solve` answers one). Here only the exact paths of the attempts join the hybrid.

## Stopping a plan

`Request::stop` is the caller's flag that ends a plan. Once it is set, each planner and
the simplifier end at their next question whether to stop, and the plan starts no further
batch of attempts, no hybrid and no simplification. `Plan::ended_by` says `Stopped` when
the stop ended a planner or kept a batch from starting; a stop once every planner has
returned leaves it saying what ended them. `Simplification::ended_by` says `Stopped` when
the stop ended the simplifier or kept it from starting, and a path whose simplification
the stop kept from starting comes back unsimplified.

## The time limit

`Request::time_limit` bounds the attempts from the call of `plan`, and
`Request::simplification_time` the simplification after them, counted from the end of
the time limit. The planners end at their first question whether to stop past the time
limit, and no batch of attempts starts past it. The simplification starts past the end
of its own time too, as MoveIt's `simplifySolution` starts with whatever time its plan
left, and the simplifier ends at its first question whether to stop past it. What runs between two questions runs to its end, a call of the caller's
functions among it, and so do the checks of the start and the goals, the hybrid of the
attempts' paths, with the motion checks that join the paths and those of the hybrid, as
OMPL's `ParallelPlan` runs it, and the simplifier's first pass: OMPL's simplifier runs
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
bound once its first pass has ended. A simplification that its time or the caller's stop
ends before it starts gives the planned path, and `Simplification::ended_by` says
`TimeLimit` or `Stopped`.

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
both on the thread that calls `plan`, and, when it makes several attempts, on the thread
of each, and shares them with the threads on which PRM and PRM* search their roadmap, so
both traits are `Sync`.

Before the planner starts, the start passes the state check and lies on the constraint,
or the plan answers `PlanError::InvalidStart`. The plan keeps the goals that pass, and
answers `PlanError::InvalidGoal` when none does. A start or a goal outside the space's
bounds is refused with `PlanError::Request`.

An error or a panic in any of the caller's functions, in the error's message or in its
drop, ends the plan with `PlanError::Check`, which says what the first one said, and
ends every attempt of the plan at its next question whether to stop. It
never reaches C++: the bridge answers OMPL a failed check, and the plan's termination
condition ends the plan. The constraint's function and Jacobian find their output filled
with NaN, so a value that a failed call leaves unwritten is NaN, which OMPL takes as off
the constraint. The process goes on. The bridge catches a panic in a binary that unwinds
on panic, Rust's default; in a binary built with `panic = "abort"`, a panic in the
caller's functions aborts the process.

## Parity with OMPL

A plan of every planner but PRM that its iteration bound ends, and no stop, repeats, path
for path,
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

OMPL's sources are unmodified but for one patch, `patches/thread_seed.patch`, which the
build applies to the extracted sources. It adds `ompl::RNG::setThreadSeed`, which gives
the calling thread a seed generator of its own: every random number generator the
thread constructs afterwards takes its seed from it, so each attempt of a plan seeds its
planner and samplers from its own seed whatever the threads of the other attempts do. A
hunk of the patch applies where its context and its removed lines stand in the file, line
for line; a patch that does not apply stops the build and names the file and the hunk.

The build downloads the archive from
<https://github.com/ompl/ompl/archive/refs/tags/2.0.2.tar.gz> and checks it against its
SHA-256 digest, `d867190a46a7f730923475a68af2e09fc9e796017a59be395a4bf3900d0c3ae4`. It
stops on an archive of another digest, and names the expected digest and the actual one.
It extracts the release's `LICENSE` and its `src/ompl` into its output directory,
applies the patch, and writes a marker of the archive's digest and the patch's; a later
build into that directory reuses the extraction and downloads nothing while the patch
stays the same. In a peppy container build, which starts from an empty target
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

OMPL is under its BSD license, which permits the patch's modification of its sources. A
binary that links this crate links OMPL, and the license's second clause asks a binary
distribution to reproduce OMPL's copyright notice, conditions and disclaimer. The build extracts the license to `ompl-2.0.2/LICENSE` in its
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
