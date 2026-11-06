//! Cartesian paths: a frame of an arm along straight lines through poses, as MoveIt's
//! `CartesianInterpolator::computeCartesianPath` computes them for move_group's
//! compute_cartesian_path service (moveit2 2.15.2, cartesian_interpolator.cpp:206-307, at
//! the `CartesianPrecision` defaults, cartesian_path_service_capability.cpp:186-188).
//!
//! Each line runs from the frame's pose at the state the path reached to the next pose,
//! its position lerped and its orientation slerped the shorter way in step
//! (`chain_kinematics::interpolate`), in steps of at most `CARTESIAN_STEP_M` and
//! `CARTESIAN_STEP_RAD` (:237-245). Each step is the inverse kinematics of the frame
//! from the state before, the state's check inside the search (`setFromIK` at its default
//! timeout, :260, robot_state.cpp:2017-2019). Each interval keeps the frame at the joints'
//! midpoint within `PRECISION_M` and `PRECISION_RAD` of the line's midpoint, or is
//! halved and its midpoint solved from there, down to `MAX_RESOLUTION` of the line
//! (`validateAndImproveInterval`, :63-109): `chain_kinematics::walk_line`. The first step
//! that fails ends the path. The fraction counts each whole line as one share of the
//! poses, and the reached share of the line the path ends on (:274-307).
//!
//! A state keeps the plan's clearance, or, on a path from a start nearer than it, falls
//! short of it by no more than the state it is solved from. MoveIt checks a state for collisions
//! alone, and no motion between two states. Here the path the move runs, its repeats
//! dropped and its corners rounded as execute_joint_path runs it, is then checked: for
//! collisions as `check_path` checks a caller's path, a fault ending the path at the
//! waypoint before it, and for the precision at probes so dense that the frame and its
//! point on the line each move at most `PROBE_MOTION_M` and turn at most `PROBE_TURN_RAD`
//! between two. A probe that strays past `PROBE_SHARE` of the precision halves the
//! intervals beside its waypoint, and an interval narrower than `MAX_RESOLUTION`, whose
//! midway has no state, or whose midway state the move drops, ends the path before it.
//!
//! The start rule walks a departure along one straight joint run from the start, and every
//! waypoint after the start keeps the plan's clearance (`crate::plan`). So a path from a
//! start nearer than the clearance leaves it in one run to the first state that keeps the
//! clearance, which holds the precision like any other, and a path that reaches no such
//! state, or whose run breaks the start rule or strays from its line, ends at its start. A
//! corner at the run's end halves the interval after it alone, as a state on the run falls
//! short of the clearance. MoveIt's validity callback judges each state alone, so a first
//! state still nearer than its padding ends its path there.

use std::cell::RefCell;
use std::sync::atomic::AtomicBool;
use std::time::Instant;

use chain_kinematics::{
    Aims, Chain, Draws, Ends, IkSolver, JointKind, LinePrecision, LineSolver, LineState,
    MAX_RESOLUTION, PRECISION, PRECISION_M, PRECISION_RAD, SEARCH_ITERATIONS, WalkEnd, interpolate,
    line_blends, line_extents, line_precision, walk_line,
};
use control_core::path_timing::{RoundedPath, distinct_joint_waypoint_indices};
use control_core::positive_finite::PositiveFinite;
use nalgebra::{Isometry3, Vector3};

use crate::arm::Moving;
use crate::arms::Arms;
use crate::goal::{Frame, frame_pose_at};
use crate::plan::{
    CheckFailure, CheckRequest, PathChecks, Scene, StartFault, check_path_until,
    distances_in_range, start_clear, start_inside_limits, vectors,
};
use crate::stop;

/// The share of the precision a probe of the rounded path keeps: the rest covers the
/// frame's motion and its point's on the line between two probes.
const PROBE_SHARE: f64 = 0.9;
/// The precision a probe of the rounded path keeps.
const PROBE_PRECISION: LinePrecision =
    line_precision(PROBE_SHARE * PRECISION_M, PROBE_SHARE * PRECISION_RAD);
/// How far the frame, and its point on the line, each move between two probes at most
/// (m): half the share of the precision a probe leaves.
const PROBE_MOTION_M: f64 = (1.0 - PROBE_SHARE) / 2.0 * PRECISION_M;
/// How far the frame, and its point on the line, each turn between two probes at most
/// (rad).
const PROBE_TURN_RAD: f64 = (1.0 - PROBE_SHARE) / 2.0 * PRECISION_RAD;

/// A frame a Cartesian path follows: the grasp point, or a link the arm's every joint
/// moves. An inner link, which fewer joints move, and a part a link carries on a joint of
/// its own have no inverse kinematics in MoveIt (`Frame::solved_by_moveit`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowedFrame(Frame);

impl FollowedFrame {
    /// `frame` of `chain`, where a Cartesian path can follow it.
    pub fn of<const N: usize>(chain: &Chain<N>, frame: Frame) -> Option<Self> {
        frame.solved_by_moveit(chain).then_some(Self(frame))
    }
}

/// What the states of a Cartesian path keep clear of.
pub enum Collisions<'a> {
    /// The map and the objects of `scene`, and the robot's own body, at the plan's
    /// clearance for `min_clearance_m`, as a plan's states keep them.
    Checked {
        scene: &'a Scene<'a>,
        min_clearance_m: f64,
    },
    /// Nothing: a test of reach and precision alone.
    Unchecked,
}

/// A Cartesian path of one arm: its frame along straight lines from where the arm
/// stands through poses.
pub struct CartesianRequest<'a, const N: usize> {
    /// Where the arm stands.
    pub start: [f64; N],
    /// How far, per joint, the start may lie outside a joint limit and still be moved
    /// onto it, as a plan's.
    pub start_tolerance: [f64; N],
    /// The frame that follows the lines.
    pub frame: FollowedFrame,
    /// The poses the frame passes through, in the robot frame, at least one.
    pub waypoints: &'a [Isometry3<f64>],
    pub collisions: Collisions<'a>,
    /// The seed of the draws of the inverse kinematics: its restarts and its steps off a
    /// singularity.
    pub seed: u64,
    /// A flag the caller sets to end the path, at any part of it.
    pub stop: Option<&'a AtomicBool>,
}

/// The longest valid part of a Cartesian path from its start.
#[derive(Debug, PartialEq)]
pub struct CartesianPath<const N: usize> {
    /// The start the path used: the request's, each joint outside a limit by no more than
    /// its tolerance moved onto it.
    pub start: [f64; N],
    /// The path's waypoints after the start, as execute_joint_path runs them, no waypoint
    /// repeating the one before it; empty when no step is valid, and the last state alone
    /// when the path never leaves the start.
    pub waypoints: Vec<[f64; N]>,
    /// The share of the poses' lines covered, 0 to 1: each line counts equally, and a
    /// partial line the share of it reached.
    pub fraction: f64,
}

/// Why a Cartesian path gives no answer.
#[derive(Debug, PartialEq, thiserror::Error)]
pub enum CartesianFailure {
    #[error("{0}")]
    Request(String),
    #[error(transparent)]
    Start(StartFault),
    #[error("the Cartesian path was stopped")]
    Stopped,
}

/// The longest valid part of `request`'s Cartesian path for the one arm of `arms`, as the
/// module says.
pub fn cartesian_path<const N: usize>(
    arms: &Arms<N, N>,
    request: &CartesianRequest<N>,
) -> Result<CartesianPath<N>, CartesianFailure> {
    let chain = arms.arms()[0].geometry().chain();
    let FollowedFrame(frame) = request.frame;
    poses_in_range(request.waypoints).map_err(CartesianFailure::Request)?;
    let start = start_of(arms, request)?;
    let follower = Follower {
        chain,
        frame,
        ik: IkSolver::new(chain, frame.into(), Aims::Both, Ends::Converged),
        draws: RefCell::new(Draws::new(request.seed)),
        judge: match request.collisions {
            Collisions::Checked {
                scene,
                min_clearance_m,
            } => Judge::Checked(Box::new(PathChecks::new(
                arms,
                scene,
                start,
                min_clearance_m,
            ))),
            Collisions::Unchecked => Judge::Unchecked,
        },
        stop: request.stop,
        probe_spacing: probe_spacing(chain, frame),
    };
    let (reached, lines) = follower.walk(start, request.waypoints)?;
    let reached = follower.moved_path(arms, request, reached, &Lines::new(lines))?;
    let kept = distinct_joint_waypoint_indices(&joints(&reached));
    let last = reached.last().expect("a path holds its start");
    Ok(CartesianPath {
        start,
        waypoints: kept[1..].iter().map(|&k| reached[k].q).collect(),
        fraction: fraction(last, request.waypoints.len()),
    })
}

/// Refuses no poses, and a pose that is not finite, saying why.
pub fn poses_in_range(waypoints: &[Isometry3<f64>]) -> Result<(), String> {
    if waypoints.is_empty() {
        return Err("waypoints holds no pose: send one or more".to_string());
    }
    match waypoints.iter().position(|pose| !finite(pose)) {
        Some(k) => Err(format!(
            "waypoint {} holds a number that is not finite",
            k + 1
        )),
        None => Ok(()),
    }
}

/// Whether every number of `pose` is finite.
fn finite(pose: &Isometry3<f64>) -> bool {
    pose.translation.vector.iter().all(|v| v.is_finite())
        && pose.rotation.coords.iter().all(|v| v.is_finite())
}

/// The start `request` takes: inside the joint limits by its tolerance and, where its
/// states keep clear of a scene, out of contact.
fn start_of<const N: usize>(
    arms: &Arms<N, N>,
    request: &CartesianRequest<N>,
) -> Result<[f64; N], CartesianFailure> {
    let min_clearance_m = match request.collisions {
        Collisions::Checked {
            min_clearance_m, ..
        } => min_clearance_m,
        Collisions::Unchecked => 0.0,
    };
    distances_in_range(min_clearance_m, &request.start_tolerance)
        .map_err(CartesianFailure::Request)?;
    match request.collisions {
        Collisions::Checked { scene, .. } => {
            start_clear(arms, scene, &request.start, &request.start_tolerance)
        }
        Collisions::Unchecked => start_inside_limits(
            arms.arms()[0].geometry().chain(),
            &request.start,
            &request.start_tolerance,
        ),
    }
    .map_err(CartesianFailure::Start)
}

/// A state a path reached: the joints, the line it lies on and its share of that line.
#[derive(Debug, Clone, Copy)]
struct Reached<const N: usize> {
    q: [f64; N],
    line: usize,
    blend: f64,
}

/// The joints of each of `reached`.
fn joints<const N: usize>(reached: &[Reached<N>]) -> Vec<[f64; N]> {
    reached.iter().map(|state| state.q).collect()
}

/// The fraction of a path of `poses` lines whose last state is `last`, summed as
/// `computeCartesianPath` sums it (cartesian_interpolator.cpp:294-301): each whole line
/// before `last`'s, and its share of its own.
fn fraction<const N: usize>(last: &Reached<N>, poses: usize) -> f64 {
    let poses = poses as f64;
    if last.blend == 1.0 {
        (last.line + 1) as f64 / poses
    } else {
        last.line as f64 / poses + last.blend / poses
    }
}

/// The straight line from the frame's pose `from` to the pose `to`.
struct Line {
    from: Isometry3<f64>,
    to: Isometry3<f64>,
}

impl Line {
    /// The pose at share `blend` of the line.
    fn at(&self, blend: f64) -> Isometry3<f64> {
        interpolate(&self.from, &self.to, blend)
    }

    /// The share of the line at each step.
    fn blends(&self) -> impl Iterator<Item = f64> + use<> {
        line_blends(&self.from, &self.to)
    }

    /// How much way the line takes, in steps: the more of its extents.
    fn way(&self) -> f64 {
        let (moved, turned) = line_extents(&self.from, &self.to);
        moved.max(turned)
    }
}

/// The lines a path walked, each from the way the lines before it take: a place along them
/// measures the steps' way, so a line that neither moves nor turns takes none.
struct Lines {
    lines: Vec<Line>,
    starts: Vec<f64>,
}

impl Lines {
    fn new(lines: Vec<Line>) -> Self {
        let starts = lines
            .iter()
            .scan(0.0, |way, line| {
                let start = *way;
                *way += line.way();
                Some(start)
            })
            .collect();
        Self { lines, starts }
    }

    /// Where `state` lies along the lines.
    fn along<const N: usize>(&self, state: &Reached<N>) -> f64 {
        self.starts[state.line] + state.blend * self.lines[state.line].way()
    }

    /// The line `along` lies on, and its share of that line: the last line that starts at or
    /// before it, whole where that line takes no way.
    fn place(&self, along: f64) -> (usize, f64) {
        let line = self
            .starts
            .partition_point(|start| *start <= along)
            .saturating_sub(1);
        let way = self.lines[line].way();
        let blend = if way > 0.0 {
            ((along - self.starts[line]) / way).min(1.0)
        } else {
            1.0
        };
        (line, blend)
    }

    /// The pose at `along`.
    fn pose_at(&self, along: f64) -> Isometry3<f64> {
        let (line, blend) = self.place(along);
        self.lines[line].at(blend)
    }
}

/// The check of a state of the path, as the module says.
enum Judge<'a, const N: usize> {
    Checked(Box<PathChecks<'a, N, N>>),
    Unchecked,
}

impl<const N: usize> Judge<'_, N> {
    /// Whether a state solved from `before` may follow it: it falls no further short of
    /// the plan's clearance than `before`, nor short of it once `before` keeps it.
    fn accepting(&self, before: &[f64; N]) -> impl Fn(&[f64; N]) -> bool + '_ {
        let floor = match self {
            Self::Checked(checks) => {
                Some((&**checks, checks.shortfall(before, Moving::All).max(0.0)))
            }
            Self::Unchecked => None,
        };
        move |q: &[f64; N]| {
            floor.is_none_or(|(checks, floor)| checks.shortfall(q, Moving::All) <= floor)
        }
    }

    /// Whether `q` falls short of the plan's clearance.
    fn short(&self, q: &[f64; N]) -> bool {
        match self {
            Self::Checked(checks) => checks.shortfall(q, Moving::All) > 0.0,
            Self::Unchecked => false,
        }
    }
}

/// What follows the lines with the frame.
struct Follower<'a, const N: usize> {
    chain: &'a Chain<N>,
    frame: Frame,
    ik: IkSolver<'a, N>,
    draws: RefCell<Draws>,
    judge: Judge<'a, N>,
    stop: Option<&'a AtomicBool>,
    /// The spacing of the probes of the rounded path, as the module says.
    probe_spacing: PositiveFinite,
}

impl<const N: usize> LineSolver<N> for Follower<'_, N> {
    fn solve(&self, pose: &Isometry3<f64>, from: &LineState<N>, _to: f64) -> Option<[f64; N]> {
        self.solve_from(&from.q, &from.q, pose)
    }

    fn pose(&self, q: &[f64; N]) -> Isometry3<f64> {
        frame_pose_at(&self.chain.at(q), self.frame)
    }
}

impl<const N: usize> Follower<'_, N> {
    /// The joints that put the frame at `pose`, searched from `seed`, that may follow
    /// `before`.
    fn solve_from(
        &self,
        seed: &[f64; N],
        before: &[f64; N],
        pose: &Isometry3<f64>,
    ) -> Option<[f64; N]> {
        let accept = self.judge.accepting(before);
        self.ik.search(
            *seed,
            pose,
            &mut *self.draws.borrow_mut(),
            &accept,
            SEARCH_ITERATIONS,
        )
    }

    /// Walks the line to each pose of `waypoints` in turn from `start`, each from the
    /// frame's pose where the line before it ended, up to the first step that fails: the
    /// states it reached, the start first, and the lines it walked.
    fn walk(
        &self,
        start: [f64; N],
        waypoints: &[Isometry3<f64>],
    ) -> Result<(Vec<Reached<N>>, Vec<Line>), CartesianFailure> {
        let mut reached = vec![Reached {
            q: start,
            line: 0,
            blend: 0.0,
        }];
        let mut lines = Vec::with_capacity(waypoints.len());
        for (index, target) in waypoints.iter().enumerate() {
            let from = reached.last().expect("a path holds its start").q;
            let line = Line {
                from: self.pose(&from),
                to: *target,
            };
            let mut stopped = false;
            let end = walk_line(
                self,
                |blend| line.at(blend),
                LineState {
                    blend: 0.0,
                    q: from,
                },
                line.blends(),
                Some(&PRECISION),
                |_, step| {
                    stopped = stop::is_set(self.stop);
                    if !stopped {
                        reached.extend(step.iter().map(|state| Reached {
                            q: state.q,
                            line: index,
                            blend: state.blend,
                        }));
                    }
                    !stopped
                },
            );
            lines.push(line);
            if stopped {
                return Err(CartesianFailure::Stopped);
            }
            if end != WalkEnd::Whole {
                break;
            }
        }
        Ok((reached, lines))
    }

    /// `reached` cut and refined until the path the move runs through it passes, as the
    /// module says.
    fn moved_path(
        &self,
        arms: &Arms<N, N>,
        request: &CartesianRequest<N>,
        mut reached: Vec<Reached<N>>,
        lines: &Lines,
    ) -> Result<Vec<Reached<N>>, CartesianFailure> {
        if self.judge.short(&reached[0].q) {
            let clear = reached.iter().position(|state| !self.judge.short(&state.q));
            reached.drain(1..clear.unwrap_or(reached.len() - 1).max(1));
        }
        loop {
            let kept = distinct_joint_waypoint_indices(&joints(&reached));
            if kept.len() == 1 {
                return Ok(reached);
            }
            if let Collisions::Checked {
                scene,
                min_clearance_m,
            } = request.collisions
            {
                let path: Vec<[f64; N]> = kept.iter().map(|&k| reached[k].q).collect();
                let check = check_path_until(
                    arms,
                    scene,
                    &CheckRequest {
                        start: path[0],
                        start_tolerance: request.start_tolerance,
                        waypoints: &path[1..],
                        min_clearance_m,
                        deadline: Instant::now(),
                    },
                    &|| stop::is_set(self.stop),
                )
                .map_err(|failure| match failure {
                    CheckFailure::TimeEnded => CartesianFailure::Stopped,
                    CheckFailure::Request(message) => CartesianFailure::Request(message),
                })?;
                if check.fault.is_some() {
                    reached.truncate(kept[check.clear_waypoints + 1]);
                    continue;
                }
            }
            let imprecise = self.first_imprecise(&reached, &kept, lines);
            if stop::is_set(self.stop) {
                return Err(CartesianFailure::Stopped);
            }
            match imprecise {
                None => return Ok(reached),
                Some(at) => reached = self.refined(reached, &kept, at, lines),
            }
        }
    }

    /// The waypoint, by its index among `kept`, beside which the first probe of the
    /// rounded path through the states `kept` names of `reached` strays past
    /// `PROBE_PRECISION` of its lines; `None` when none does, or the stop ends the probes.
    fn first_imprecise(
        &self,
        reached: &[Reached<N>],
        kept: &[usize],
        lines: &Lines,
    ) -> Option<usize> {
        let states: Vec<Reached<N>> = kept.iter().map(|&k| reached[k]).collect();
        let path = RoundedPath::joint_path(&vectors(&joints(&states)));
        path.arcs_every(self.probe_spacing)
            .into_iter()
            .take_while(|_| !stop::is_set(self.stop))
            .find_map(|s| {
                let q: [f64; N] = path.at(s).try_into().expect("a probe holds N joints");
                let at = path.waypoint_at(s);
                (!self.near_its_lines(&q, &states, at, lines)).then_some(at)
            })
    }

    /// Whether the frame at `q`, a probe of the rounded path beside waypoint `at` of
    /// `states`, stands within `PROBE_PRECISION` of the lines' pose it pairs with on one of
    /// the legs beside it: the pose as far along the lines between the leg's states as
    /// `q`'s projection on the leg lies along it.
    fn near_its_lines(
        &self,
        q: &[f64; N],
        states: &[Reached<N>],
        at: usize,
        lines: &Lines,
    ) -> bool {
        let actual = self.pose(q);
        legs_beside(at, states.len()).any(|(a, b)| {
            let (a, b) = (&states[a], &states[b]);
            let (from, to) = (lines.along(a), lines.along(b));
            let along = from + share_along(q, &a.q, &b.q) * (to - from);
            PROBE_PRECISION.holds(&actual, &lines.pose_at(along))
        })
    }

    /// `reached` with each leg beside waypoint `at` of `kept` halved: a state solved at the
    /// lines' pose midway along the leg, from the joints' midpoint. A leg no state halves ends
    /// the path before the leg's end. The run from a start nearer than the clearance stays
    /// whole beside a leg after it.
    fn refined(
        &self,
        mut reached: Vec<Reached<N>>,
        kept: &[usize],
        at: usize,
        lines: &Lines,
    ) -> Vec<Reached<N>> {
        let corner_after_the_run = self.judge.short(&reached[0].q) && at == 1 && kept.len() > 2;
        let halves: Vec<(usize, Option<Reached<N>>)> = legs_beside(at, kept.len())
            .skip(usize::from(corner_after_the_run))
            .map(|(a, b)| {
                let end = kept[b];
                (end, self.midway(&reached[kept[a]], &reached[end], lines))
            })
            .collect();
        // The later leg first, so the index of the earlier one stays where it is.
        for &(end, half) in halves.iter().rev() {
            match half.and_then(|state| joined(&reached, state, lines)) {
                Some(halved) => reached = halved,
                None => reached.truncate(end),
            }
        }
        reached
    }

    /// The state midway along the lines from `a` to `b`, solved from their joints' midpoint;
    /// `None` when the leg is no wider than `MAX_RESOLUTION` of the midpoint's line, or the
    /// midpoint has no state.
    fn midway(&self, a: &Reached<N>, b: &Reached<N>, lines: &Lines) -> Option<Reached<N>> {
        let (from, to) = (lines.along(a), lines.along(b));
        let along = 0.5 * (from + to);
        let (line, blend) = lines.place(along);
        if to - from <= MAX_RESOLUTION * lines.lines[line].way() {
            return None;
        }
        let seed: [f64; N] = std::array::from_fn(|i| 0.5 * (a.q[i] + b.q[i]));
        let q = self.solve_from(&seed, &a.q, &lines.lines[line].at(blend))?;
        Some(Reached { q, line, blend })
    }
}

/// `reached`, in order along `lines`, with `state` in its place; `None` when the move drops
/// `state`, so the path it runs stays as it was.
fn joined<const N: usize>(
    reached: &[Reached<N>],
    state: Reached<N>,
    lines: &Lines,
) -> Option<Vec<Reached<N>>> {
    let place = reached.partition_point(|other| lines.along(other) <= lines.along(&state));
    let halved: Vec<Reached<N>> = reached[..place]
        .iter()
        .copied()
        .chain(std::iter::once(state))
        .chain(reached[place..].iter().copied())
        .collect();
    distinct_joint_waypoint_indices(&joints(&halved))
        .contains(&place)
        .then_some(halved)
}

/// The legs beside waypoint `at` of a path of `count` waypoints, each by the indices of
/// its two waypoints: the one that ends at it, and the one that starts from it.
fn legs_beside(at: usize, count: usize) -> impl Iterator<Item = (usize, usize)> {
    [at.checked_sub(1).map(|a| (a, at)), Some((at, at + 1))]
        .into_iter()
        .flatten()
        .filter(move |&(_, b)| b < count)
}

/// How far along the straight joint motion from `a` to `b` the projection of `q` on it
/// lies, 0 to 1; 0 where `a` and `b` are one.
fn share_along<const N: usize>(q: &[f64; N], a: &[f64; N], b: &[f64; N]) -> f64 {
    let run: f64 = (0..N).map(|i| (b[i] - a[i]).powi(2)).sum();
    if run == 0.0 {
        return 0.0;
    }
    let ahead: f64 = (0..N).map(|i| (q[i] - a[i]) * (b[i] - a[i])).sum();
    (ahead / run).clamp(0.0, 1.0)
}

/// The spacing of the probes of a rounded path of `chain` that follows `frame`, in the
/// joints' L2 distance: so short that the frame moves at most `PROBE_MOTION_M` and turns
/// at most `PROBE_TURN_RAD` between two. The frame moves at most its lever times the
/// distance (`frame_lever`), and turns at most the sum of the joints' turns, at most the
/// square root of N times their L2 distance.
fn probe_spacing<const N: usize>(chain: &Chain<N>, frame: Frame) -> PositiveFinite {
    let spacing =
        (PROBE_MOTION_M / frame_lever(chain, frame)).min(PROBE_TURN_RAD / (N as f64).sqrt());
    PositiveFinite::parse(spacing).expect("a lever and a joint count give a positive spacing")
}

/// A bound on how far `frame` moves per unit of the joints' L2 distance, along any joint
/// motion: the L2 norm of each joint's reach of it. A revolute joint reaches the frame
/// across the links between them, at most the sum of the distances from each joint's
/// origin to the next one's and from the last one's to the frame, each fixed by the link
/// between them, a prismatic joint's grown by its farthest travel; a prismatic joint
/// moves the frame as far as it slides.
fn frame_lever<const N: usize>(chain: &Chain<N>, frame: Frame) -> f64 {
    let posed = chain.at(&[0.0; N]);
    let limits = chain.limits();
    let travel = |j: usize| match posed.kind(j) {
        JointKind::Prismatic { .. } => limits[j].lo.abs().max(limits[j].hi.abs()),
        JointKind::Revolute { .. } | JointKind::Fixed => 0.0,
    };
    let origins: Vec<Vector3<f64>> = (0..N)
        .map(|j| posed.origin_world(j))
        .chain(std::iter::once(
            frame_pose_at(&posed, frame).translation.vector,
        ))
        .collect();
    // The longest each link from a joint's origin to the next point can stand: its length
    // at zero, and the next joint's travel where it slides.
    let link = |j: usize| {
        (origins[j + 1] - origins[j]).norm() + if j + 1 < N { travel(j + 1) } else { 0.0 }
    };
    let reach = |j: usize| match posed.kind(j) {
        JointKind::Revolute { .. } => (j..N).map(link).sum::<f64>(),
        JointKind::Prismatic { .. } => 1.0,
        JointKind::Fixed => 0.0,
    };
    (0..N).map(|j| reach(j).powi(2)).sum::<f64>().sqrt()
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::Ordering;

    use chain_kinematics::{ChainSpec, JointSelection};
    use collision_geometry::chain::ChainGeometry;
    use nalgebra::{Translation3, UnitQuaternion};
    use openarm_description::{HardwareVersion, Side};
    use srs_model::ArmAnglePolicy;

    use super::*;
    use crate::arm::Arm;
    use crate::goal::tests::left_arm;
    use crate::plan::tests::world;
    use crate::scene::Objects;

    /// The voxel of the maps, and the clearance of the checked paths, in meters.
    const VOXEL_M: f64 = 0.02;
    const MIN_CLEARANCE_M: f64 = 0.04;
    /// The drawn lines the comparison of the two solvers walks.
    const LINES: usize = 200;
    /// How far each drawn line moves at most along each axis, in meters, and turns at most
    /// about each axis, in radians.
    const LINE_REACH_M: f64 = 0.15;
    const LINE_TURN_RAD: f64 = 0.4;
    /// The share of the closed form's steps the KDL search completes at least: criterion 7
    /// of ROBOTCTRL-54.
    const LEAST_SHARE: f64 = 0.99;
    /// The fewest steps of the closed form the comparison counts, so the share stands on
    /// many: most of the lines walk some steps.
    const FEWEST_STEPS: usize = 2000;

    /// The shoulder's turn from `UPPER_BRANCH` to the end of the short line of the next tests,
    /// and to the end of their long one.
    const SHORT_TURN_RAD: f64 = 0.01;
    const LONG_TURN_RAD: f64 = 0.6;

    /// The closed form of `srs_model`, from the state before at its arm angle.
    struct ClosedForm<'a>(&'a srs_model::Arm);

    impl LineSolver<7> for ClosedForm<'_> {
        fn solve(&self, pose: &Isometry3<f64>, from: &LineState<7>, _to: f64) -> Option<[f64; 7]> {
            self.0
                .solve_ik(&self.0.base_pose(pose), ArmAnglePolicy::FromSeed, &from.q)
                .map(|solution| solution.q)
        }

        fn pose(&self, q: &[f64; 7]) -> Isometry3<f64> {
            self.0.world_ee_pose(q)
        }
    }

    /// A state of the left arm with its gripper down, inside its limits.
    const GRIPPER_DOWN: [f64; 7] = [0.426, -3.075, 0.86, 2.245, -0.41, 0.534, 0.136];

    /// A follower of the grasp point of `chain`.
    fn follower_of<'a, const N: usize>(
        chain: &'a Chain<N>,
        judge: Judge<'a, N>,
        stop: Option<&'a AtomicBool>,
    ) -> Follower<'a, N> {
        Follower {
            chain,
            frame: Frame::GraspPoint,
            ik: IkSolver::new(chain, Frame::GraspPoint.into(), Aims::Both, Ends::Converged),
            draws: RefCell::new(Draws::new(1)),
            judge,
            stop,
            probe_spacing: probe_spacing(chain, Frame::GraspPoint),
        }
    }

    /// The grasp point's pose at `q`, 0.1 m lower.
    fn lowered(chain: &Chain<7>, q: &[f64; 7]) -> Isometry3<f64> {
        let pose = chain.world_ee_pose(q);
        Isometry3::from_parts(
            Translation3::from(pose.translation.vector - Vector3::z() * 0.1),
            pose.rotation,
        )
    }

    /// The state `solver` reaches at `blend` of `line` from `from`, within the precision.
    fn step_of(
        solver: &dyn LineSolver<7>,
        line: &Line,
        from: LineState<7>,
        blend: f64,
    ) -> Option<LineState<7>> {
        let mut reached = None;
        let end = walk_line(
            solver,
            |s| line.at(s),
            from,
            [blend],
            Some(&PRECISION),
            |_, step| {
                reached = step.last().copied();
                true
            },
        );
        (end == WalkEnd::Whole).then_some(reached).flatten()
    }

    // A set stop ends the walk at its next step, whatever the steps left: the walk answers
    // no states of the line.
    #[test]
    fn a_set_stop_ends_the_walk_at_its_next_step() {
        let chain = left_arm();
        let stop = AtomicBool::new(true);
        let follower = follower_of(&chain, Judge::Unchecked, Some(&stop));
        assert!(matches!(
            follower.walk(GRIPPER_DOWN, &[lowered(&chain, &GRIPPER_DOWN)]),
            Err(CartesianFailure::Stopped)
        ));
    }

    /// The grasp point's line 0.1 m down from `GRIPPER_DOWN`.
    fn line_down(chain: &Chain<7>) -> Lines {
        Lines::new(vec![Line {
            from: chain.world_ee_pose(&GRIPPER_DOWN),
            to: lowered(chain, &GRIPPER_DOWN),
        }])
    }

    /// The state `follower` solves at `blend` of the first of `lines`.
    fn state_at(follower: &Follower<'_, 7>, lines: &Lines, blend: f64) -> Reached<7> {
        Reached {
            q: follower
                .solve_from(&GRIPPER_DOWN, &GRIPPER_DOWN, &lines.lines[0].at(blend))
                .expect("a pose beside the start"),
            line: 0,
            blend,
        }
    }

    /// `GRIPPER_DOWN` as the start of a path.
    const AT_THE_START: Reached<7> = Reached {
        q: GRIPPER_DOWN,
        line: 0,
        blend: 0.0,
    };

    // A state the move drops halves nothing, so `refined` ends the path there: a leg of 0.1 mm
    // has its midway state within `MIN_JOINT_CHANGE_RAD` of both ends and unequal to either, on
    // a line wide enough that `MAX_RESOLUTION` leaves the leg to halve. A leg of half the line
    // keeps its midway state.
    #[test]
    fn a_midway_state_the_move_drops_halves_nothing() {
        let chain = left_arm();
        let follower = follower_of(&chain, Judge::Unchecked, None);
        let lines = line_down(&chain);
        let halved = |blend: f64| {
            let reached = vec![AT_THE_START, state_at(&follower, &lines, blend)];
            follower.refined(reached, &[0, 1], 1, &lines).len()
        };
        assert_eq!(halved(0.001), 1);
        assert_eq!(halved(0.5), 3);
    }

    // A halved state takes its place along the lines, where the kept end of its leg is the
    // last of a run of repeats: the path never turns back along its lines.
    #[test]
    fn a_halved_state_takes_its_place_along_the_lines() {
        let chain = left_arm();
        let follower = follower_of(&chain, Judge::Unchecked, None);
        let end = lowered(&chain, &GRIPPER_DOWN);
        let lines = Lines::new(vec![
            Line {
                from: chain.world_ee_pose(&GRIPPER_DOWN),
                to: end,
            },
            Line { from: end, to: end },
        ]);
        let last_of_first = state_at(&follower, &lines, 1.0);
        let reached = vec![
            AT_THE_START,
            last_of_first,
            Reached {
                line: 1,
                ..last_of_first
            },
        ];
        assert_eq!(
            distinct_joint_waypoint_indices(&joints(&reached)),
            [0, 2],
            "the kept end of the leg is the last of its run"
        );
        let halved = follower.refined(reached, &[0, 2], 1, &lines);
        assert_eq!(halved.len(), 4);
        assert!(
            halved
                .windows(2)
                .all(|pair| lines.along(&pair[0]) <= lines.along(&pair[1])),
            "{halved:?}"
        );
    }

    // A set stop ends the probes of the path the move runs, which one strays from otherwise.
    #[test]
    fn a_set_stop_ends_the_probes() {
        let chain = left_arm();
        let stop = AtomicBool::new(false);
        let follower = follower_of(&chain, Judge::Unchecked, Some(&stop));
        let lines = line_down(&chain);
        let mut far = GRIPPER_DOWN;
        far[1] += 0.5;
        let reached = [
            AT_THE_START,
            Reached {
                q: far,
                line: 0,
                blend: 1.0,
            },
        ];
        assert!(
            follower
                .first_imprecise(&reached, &[0, 1], &lines)
                .is_some()
        );
        stop.store(true, Ordering::Relaxed);
        assert_eq!(follower.first_imprecise(&reached, &[0, 1], &lines), None);
    }

    // Criterion 7 of ROBOTCTRL-54: on drawn OpenArm lines from drawn starts, the KDL search
    // alone completes at least 99% of the steps srs_model's closed form completes within
    // the precision, each from the same state before it, the state the closed form reached.
    #[test]
    fn the_kdl_search_completes_the_steps_the_closed_form_completes() {
        let version = HardwareVersion::V2;
        let closed_form = srs_model::Arm::from_urdf(version.urdf(), version.base_link(Side::Left))
            .expect("an SRS arm")
            .with_tool_link(version.tcp_link(Side::Left))
            .expect("its grasp point");
        let chain = left_arm();
        let limits = chain.limits();
        let kdl = follower_of(&chain, Judge::Unchecked, None);
        let mut draws = Draws::new(20261105);
        let (mut steps, mut completed) = (0_usize, 0_usize);
        for _ in 0..LINES {
            let start = draws.within(&limits);
            let from = chain.world_ee_pose(&start);
            let moved: Vector3<f64> =
                Vector3::from_fn(|_, _| draws.between(-LINE_REACH_M, LINE_REACH_M));
            let turned: Vector3<f64> =
                Vector3::from_fn(|_, _| draws.between(-LINE_TURN_RAD, LINE_TURN_RAD));
            let line = Line {
                from,
                to: Isometry3::from_parts(
                    Translation3::from(from.translation.vector + moved),
                    from.rotation * UnitQuaternion::from_scaled_axis(turned),
                ),
            };
            let mut here = LineState {
                blend: 0.0,
                q: start,
            };
            for blend in line.blends() {
                let Some(next) = step_of(&ClosedForm(&closed_form), &line, here, blend) else {
                    break;
                };
                steps += 1;
                completed += usize::from(step_of(&kdl, &line, here, blend).is_some());
                here = next;
            }
        }
        println!("the KDL search completes {completed} of the closed form's {steps} steps");
        assert!(steps >= FEWEST_STEPS, "{steps} steps");
        assert!(
            completed as f64 / steps as f64 >= LEAST_SHARE,
            "{completed} of {steps}"
        );
    }

    /// How far the shoulder of `elbow_arm` turns toward its first limit: past it, the elbow's
    /// branch of the path's start reaches no pose of the line.
    const SHOULDER_LOW: f64 = -0.5;

    /// An arm in a plane: a shoulder and an elbow joined by links of 0.3 m, and a wrist that
    /// sets the hand's turn. The wrist reaches a point by two branches of the elbow; the
    /// shoulder's limits leave only one of them along the far part of the line below.
    fn elbow_arm() -> ChainGeometry<3> {
        let urdf = format!(
            r#"<robot name="elbow_arm">
                <link name="base"/>
                <link name="link_1">
                  <collision><origin xyz="0.15 0 0"/><geometry><box size="0.3 0.04 0.04"/></geometry></collision>
                </link>
                <link name="link_2">
                  <collision><origin xyz="0.15 0 0"/><geometry><box size="0.3 0.04 0.04"/></geometry></collision>
                </link>
                <link name="link_3">
                  <collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision>
                </link>
                <joint name="joint_1" type="revolute">
                  <parent link="base"/><child link="link_1"/><axis xyz="0 0 1"/>
                  <limit lower="{SHOULDER_LOW}" upper="1.5" effort="1" velocity="1"/>
                </joint>
                <joint name="joint_2" type="revolute">
                  <origin xyz="0.3 0 0"/>
                  <parent link="link_1"/><child link="link_2"/><axis xyz="0 0 1"/>
                  <limit lower="-2.8" upper="2.8" effort="1" velocity="1"/>
                </joint>
                <joint name="joint_3" type="revolute">
                  <origin xyz="0.3 0 0"/>
                  <parent link="link_2"/><child link="link_3"/><axis xyz="0 0 1"/>
                  <limit lower="-3.1" upper="3.1" effort="1" velocity="1"/>
                </joint>
              </robot>"#
        );
        ChainGeometry::from_urdf(
            &urdf,
            "",
            &ChainSpec {
                base_link: None,
                tip_link: "link_3",
                joints: JointSelection::PathOrder,
            },
        )
        .expect("the elbow arm")
    }

    /// A state of `elbow_arm`'s upper elbow branch: its other branch at the same pose is
    /// `OTHER_BRANCH`.
    const UPPER_BRANCH: [f64; 3] = [0.0, 1.0, -1.0];
    const OTHER_BRANCH: [f64; 3] = [1.0, -1.0, 0.0];

    /// The end of the line from `UPPER_BRANCH` on which the shoulder reaches its limit on
    /// the start's elbow branch, while the other branch still reaches the line.
    fn past_the_shoulder_limit(geometry: &ChainGeometry<3>) -> Isometry3<f64> {
        let from = geometry.chain().world_ee_pose(&UPPER_BRANCH);
        Isometry3::from_parts(
            Translation3::new(from.translation.x, -from.translation.y, 0.0),
            UnitQuaternion::identity(),
        )
    }

    /// `UPPER_BRANCH` with the shoulder turned `by`.
    fn turned(by: f64) -> [f64; 3] {
        [UPPER_BRANCH[0] + by, UPPER_BRANCH[1], UPPER_BRANCH[2]]
    }

    /// Three lines from `UPPER_BRANCH`: a short one, one that stands still, and a long one;
    /// the states at the ends of the first two, and at `blend` of the long one, its end's
    /// joints.
    fn past_a_line_that_stands_still(chain: &Chain<3>, blend: f64) -> (Vec<Reached<3>>, Lines) {
        let (near, far) = (turned(SHORT_TURN_RAD), turned(LONG_TURN_RAD));
        let (near_pose, far_pose) = (chain.world_ee_pose(&near), chain.world_ee_pose(&far));
        let lines = Lines::new(vec![
            Line {
                from: chain.world_ee_pose(&UPPER_BRANCH),
                to: near_pose,
            },
            Line {
                from: near_pose,
                to: near_pose,
            },
            Line {
                from: near_pose,
                to: far_pose,
            },
        ]);
        let state = |q: [f64; 3], line: usize, blend: f64| Reached { q, line, blend };
        let reached = vec![
            state(UPPER_BRANCH, 0, 0.0),
            state(near, 0, 1.0),
            state(near, 1, 1.0),
            state(far, 2, blend),
        ];
        (reached, lines)
    }

    /// The request of a path of `elbow_arm`'s grasp point that `collisions` checks.
    fn elbow_request<'a>(
        geometry: &ChainGeometry<3>,
        collisions: Collisions<'a>,
    ) -> CartesianRequest<'a, 3> {
        CartesianRequest {
            start: UPPER_BRANCH,
            start_tolerance: [0.0; 3],
            frame: FollowedFrame::of(geometry.chain(), Frame::GraspPoint).expect("followed"),
            waypoints: &[],
            collisions,
            seed: 1,
            stop: None,
        }
    }

    // A collision on the motion just past a line that stands still leaves that line whole:
    // the path holds the state of its end, so the fraction counts it, as a walk that ends
    // there counts it.
    #[test]
    fn a_fault_past_a_line_that_stands_still_counts_that_line_whole() {
        let geometry = elbow_arm();
        let chain = geometry.chain();
        let arms: Arms<3, 3> = Arms::new(vec![Arm::new("arm", &geometry)], None);
        let on_the_way = chain
            .world_ee_pose(&turned(LONG_TURN_RAD / 2.0))
            .translation;
        let (map, field) = world(
            VOXEL_M,
            [-0.7, -0.7, -0.1],
            [0.7, 0.7, 0.1],
            &[[on_the_way.x, on_the_way.y, 0.0]],
        );
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let collisions = Collisions::Checked {
            scene: &scene,
            min_clearance_m: MIN_CLEARANCE_M,
        };
        let (reached, lines) = past_a_line_that_stands_still(chain, 1.0);
        let path = follower_of(chain, Judge::Unchecked, None)
            .moved_path(
                &arms,
                &elbow_request(&geometry, collisions),
                reached,
                &lines,
            )
            .expect("a path");
        let last = path.last().expect("a path holds its start");
        assert_eq!(fraction(last, 3), 2.0 / 3.0, "{path:?}");
    }

    // The same of a leg no state halves: its end stands at so small a share of a long line
    // that `MAX_RESOLUTION` ends the halving.
    #[test]
    fn a_leg_no_state_halves_past_a_line_that_stands_still_counts_that_line_whole() {
        let geometry = elbow_arm();
        let chain = geometry.chain();
        let arms: Arms<3, 3> = Arms::new(vec![Arm::new("arm", &geometry)], None);
        let (reached, lines) = past_a_line_that_stands_still(chain, MAX_RESOLUTION / 2.0);
        let request = elbow_request(&geometry, Collisions::Unchecked);
        let path = follower_of(chain, Judge::Unchecked, None)
            .moved_path(&arms, &request, reached, &lines)
            .expect("a path");
        let last = path.last().expect("a path holds its start");
        assert_eq!(fraction(last, 3), 2.0 / 3.0, "{path:?}");
    }

    // A set stop ends the check of the walked path, with collisions unchecked too.
    #[test]
    fn a_set_stop_ends_the_moved_path() {
        let geometry = elbow_arm();
        let chain = geometry.chain();
        let arms: Arms<3, 3> = Arms::new(vec![Arm::new("arm", &geometry)], None);
        let stop = AtomicBool::new(true);
        let (reached, lines) = past_a_line_that_stands_still(chain, 1.0);
        let request = elbow_request(&geometry, Collisions::Unchecked);
        assert!(matches!(
            follower_of(chain, Judge::Unchecked, Some(&stop))
                .moved_path(&arms, &request, reached, &lines),
            Err(CartesianFailure::Stopped)
        ));
    }

    // The run from a start nearer than the clearance stays whole beside the leg after it where
    // a probe strays at the corner between them, and halves where a probe strays on the run:
    // each stray changes the path, so `moved_path` goes on to its end.
    #[test]
    fn a_stray_beside_the_run_halves_the_leg_it_can() {
        let geometry = elbow_arm();
        let chain = geometry.chain();
        let arms: Arms<3, 3> = Arms::new(vec![Arm::new("arm", &geometry)], None);
        let (map, field) = world(
            VOXEL_M,
            [-0.7, -0.7, -0.1],
            [0.7, 0.7, 0.1],
            &[[0.3, -0.04, 0.0]],
        );
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let checks = PathChecks::new(&arms, &scene, UPPER_BRANCH, MIN_CLEARANCE_M);
        let follower = follower_of(chain, Judge::Checked(Box::new(checks)), None);
        assert!(follower.judge.short(&UPPER_BRANCH));
        let (reached, lines) = past_a_line_that_stands_still(chain, 1.0);
        let kept = distinct_joint_waypoint_indices(&joints(&reached));
        assert_eq!(kept, [0, 1, 3]);
        let at_the_corner = follower.refined(reached.clone(), &kept, 1, &lines);
        assert_eq!(at_the_corner.len(), reached.len() + 1, "{at_the_corner:?}");
        assert_eq!(joints(&at_the_corner)[..3], joints(&reached)[..3]);
        let on_the_run = follower.refined(reached.clone(), &kept, 0, &lines);
        assert_eq!(on_the_run.len(), reached.len() + 1, "{on_the_run:?}");
        assert_eq!(on_the_run[1].blend, 0.5, "{on_the_run:?}");
    }

    // The search answers a state the check accepts: the seed's elbow branch stands in a box
    // at the pose, so a restart answers the other branch.
    #[test]
    fn the_search_answers_the_branch_that_keeps_the_clearance() {
        let geometry = elbow_arm();
        let arms: Arms<3, 3> = Arms::new(vec![Arm::new("arm", &geometry)], None);
        let (map, field) = world(
            VOXEL_M,
            [-0.7, -0.7, -0.1],
            [0.7, 0.7, 0.1],
            &[[0.3, -0.04, 0.0]],
        );
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: Objects::none(),
        };
        let checks = PathChecks::new(&arms, &scene, OTHER_BRANCH, MIN_CLEARANCE_M);
        let follower = follower_of(geometry.chain(), Judge::Checked(Box::new(checks)), None);
        let pose = geometry.chain().world_ee_pose(&UPPER_BRANCH);
        let q = follower
            .solve_from(&UPPER_BRANCH, &OTHER_BRANCH, &pose)
            .expect("the other branch keeps the clearance");
        assert!(q[1] < 0.0, "the state in the box answered: {q:?}");
    }

    // The walk holds the precision itself: on the line past the shoulder's limit the search
    // finds the other branch by a restart, and the walk ends before that step.
    #[test]
    fn the_walk_ends_at_the_step_it_cannot_bring_within_the_precision() {
        let geometry = elbow_arm();
        let follower = follower_of(geometry.chain(), Judge::Unchecked, None);
        let target = past_the_shoulder_limit(&geometry);
        let (reached, _) = follower
            .walk(UPPER_BRANCH, &[target])
            .expect("nothing stops the walk");
        assert!(
            reached.iter().all(|state| state.q[1] > 0.0),
            "the walk left the start's branch"
        );
    }

    // The frame at the joints' midpoint leaves the line at the other elbow branch at every
    // width, so the path ends at the last state of the start's branch: about a third of the
    // line, the elbow never crossing over, no two states far apart.
    #[test]
    fn a_line_that_needs_a_jump_in_joint_space_ends_before_it() {
        let geometry = elbow_arm();
        let arms: Arms<3, 3> = Arms::new(vec![Arm::new("arm", &geometry)], None);
        let start = UPPER_BRANCH;
        let target = past_the_shoulder_limit(&geometry);
        let path = cartesian_path(
            &arms,
            &CartesianRequest {
                start,
                start_tolerance: [0.0; 3],
                frame: FollowedFrame::of(geometry.chain(), Frame::GraspPoint).expect("followed"),
                waypoints: &[target],
                collisions: Collisions::Unchecked,
                seed: 1,
                stop: None,
            },
        )
        .expect("a path");
        assert!(
            path.fraction > 0.3 && path.fraction < 0.36,
            "{}",
            path.fraction
        );
        let mut before = start;
        for q in &path.waypoints {
            assert!(q[1] > 0.0, "the elbow crossed over at {q:?}");
            assert!(
                (0..3).all(|i| (q[i] - before[i]).abs() < 0.2),
                "a jump from {before:?} to {q:?}"
            );
            before = *q;
        }
    }
}
