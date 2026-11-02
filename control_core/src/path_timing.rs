//! Time-optimal timing of a path through waypoints under per-component
//! velocity and acceleration limits, and the trajectories that play it. The
//! path follows the model of Kunz and Stilman ("Time-Optimal Trajectory
//! Generation for Path Following with Bounded Acceleration and Velocity", RSS
//! 2012), which MoveIt's time-optimal trajectory generation (TOTG) implements:
//! straight between its waypoints, rounding each corner on a circular arc
//! ([`RoundedPath`]). Its speed profile is integrated over a grid of cells
//! along the path: a backward pass from rest at the end finds the fastest each
//! cell boundary can be passed at and still stop by the end, and a forward
//! pass from rest at the start then runs each cell as hard as the limits allow
//! within that. The path acceleration is constant over a cell, and a cell's
//! bounds take each component's share of the tangent and its curvature at
//! their largest over the cell, so the limits hold at every point of it; on an
//! arc the two shares are summed, so the bound there is tighter than the
//! limits, and the timing is the fastest under that bound.
//!
//! A waypoint's components are whatever the limits are stated for: a joint
//! configuration, or one extended by an end effector's progress, so a cap on
//! the end effector times the path alongside the joints' limits.

use std::f64::consts::PI;
use std::time::{Duration, Instant};

use crate::positive_finite::PositiveFinite;
use crate::timing::{MotionClock, Pace, duration_at_least};

/// The grid the speed profile is integrated on: each piece of the path is split
/// evenly into cells of at most this share of the path's length, two at least,
/// so a piece the path stops at both ends of has a boundary that carries speed.
/// The profile switches between accelerating, cruising and braking only at
/// cell boundaries, so each switch costs at most one cell's time over the
/// optimum; a straight run has two and takes its trapezoid time to within three
/// parts in this many.
const CELLS_PER_PATH: usize = 1000;

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}

fn scaled(a: &[f64], k: f64) -> Vec<f64> {
    a.iter().map(|v| v * k).collect()
}

fn sum(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x + y).collect()
}

fn difference(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x - y).collect()
}

/// The largest `|a cos(angle) + b sin(angle)|` over the angles `from` to `to`:
/// the amplitude where a peak lies inside the span, the larger end otherwise.
fn max_abs_sinusoid(a: f64, b: f64, from: f64, to: f64) -> f64 {
    // a cos(x) + b sin(x) = amplitude cos(x + phase)
    let amplitude = a.hypot(b);
    let phase = (-b).atan2(a);
    let at = |angle: f64| (a * angle.cos() + b * angle.sin()).abs();
    // A peak of |cos| sits at every multiple of pi from -phase.
    let first_peak = -phase + PI * ((from + phase) / PI).ceil();
    if first_peak <= to {
        amplitude
    } else {
        at(from).max(at(to))
    }
}

/// One piece of the path: a straight run, or the arc that rounds a corner.
#[derive(Debug, Clone, PartialEq)]
enum Piece {
    /// From `start` along the unit `direction` for `length`.
    Straight {
        start: Vec<f64>,
        direction: Vec<f64>,
        length: f64,
    },
    /// From `start` for `turn` radians at `radius`: `tangent` is the unit
    /// tangent at `start` and `normal` the unit vector from `start` toward
    /// the center.
    Arc {
        start: Vec<f64>,
        tangent: Vec<f64>,
        normal: Vec<f64>,
        radius: f64,
        turn: f64,
    },
}

impl Piece {
    fn length(&self) -> f64 {
        match self {
            Self::Straight { length, .. } => *length,
            Self::Arc { radius, turn, .. } => radius * turn,
        }
    }

    /// The largest share of the unit tangent each component has anywhere
    /// between `from` and `to` along the piece.
    fn max_abs_tangent(&self, from: f64, to: f64) -> Vec<f64> {
        match self {
            Self::Straight { direction, .. } => direction.iter().map(|d| d.abs()).collect(),
            Self::Arc {
                tangent,
                normal,
                radius,
                ..
            } => {
                // tangent_i(angle) = tangent_i cos(angle) + normal_i sin(angle)
                tangent
                    .iter()
                    .zip(normal)
                    .map(|(&t, &n)| max_abs_sinusoid(t, n, from / radius, to / radius))
                    .collect()
            }
        }
    }

    /// The largest curvature each component has anywhere between `from` and
    /// `to` along the piece: the rate of its share of the tangent along the
    /// path, zero on a straight run.
    fn max_abs_curvature(&self, from: f64, to: f64) -> Vec<f64> {
        match self {
            Self::Straight { direction, .. } => vec![0.0; direction.len()],
            Self::Arc {
                tangent,
                normal,
                radius,
                ..
            } => {
                // curvature_i(angle) = (normal_i cos(angle) - tangent_i sin(angle)) / radius
                tangent
                    .iter()
                    .zip(normal)
                    .map(|(&t, &n)| {
                        max_abs_sinusoid(n / radius, -t / radius, from / radius, to / radius)
                    })
                    .collect()
            }
        }
    }

    /// The configuration and unit tangent `distance` along the piece.
    fn at(&self, distance: f64) -> (Vec<f64>, Vec<f64>) {
        match self {
            Self::Straight {
                start, direction, ..
            } => (sum(start, &scaled(direction, distance)), direction.clone()),
            Self::Arc {
                start,
                tangent,
                normal,
                radius,
                ..
            } => {
                let angle = distance / radius;
                let (sin, cos) = angle.sin_cos();
                // radius (1 - cos(angle)), in the form that keeps its precision
                // when the angle is small.
                let rise = 2.0 * radius * (0.5 * angle).sin().powi(2);
                let config = sum(
                    start,
                    &sum(&scaled(tangent, radius * sin), &scaled(normal, rise)),
                );
                let unit_tangent = sum(&scaled(tangent, cos), &scaled(normal, sin));
                (config, unit_tangent)
            }
        }
    }
}

/// How a path rounds its corners, and where it stops.
#[derive(Clone, Copy)]
pub enum Corners {
    /// The arc meets the run in, measured from where the arc before it
    /// leaves it, and the run out at half the shorter of the two, so the arcs
    /// of neighboring corners never overlap and the path stays within the
    /// runs it rounds. A waypoint at no positive distance from the one before
    /// it, a NaN waypoint among them, repeats it, and the two are one point; a
    /// corner where the runs are exactly in line runs straight on, and one
    /// where they turn exactly back stops the path.
    HalfShorterRun,
    /// TOTG's blend at its maximum deviation: the arc meets each leg at half
    /// the shorter leg, or nearer the corner where that keeps the arc's
    /// midpoint within this distance of it. Every point of the arc then lies
    /// within this distance of the two legs. The path runs through finite
    /// waypoints, and stops at a waypoint at no distance from the one before
    /// it and where it turns back on itself (`REVERSAL_COS`), and a corner
    /// within `STRAIGHT_SIN` of straight on runs straight on. TOTG blends a
    /// corner so where it turns by about 0.081 degrees or more (a cosine of
    /// at most 0.999999, `CircularPathSegment`, moveit2 2.15.2) and runs a
    /// milder one straight on through its waypoint; this rounds each corner
    /// whose turn has a sine above `STRAIGHT_SIN`, within this distance.
    Within(PositiveFinite),
}

impl Corners {
    /// How far from a corner turning by `turn` radians the arc meets each run:
    /// for [`Corners::HalfShorterRun`] half the shorter of `length_in`, the run
    /// in from where the arc before it leaves it, and `length_out`; for
    /// [`Corners::Within`] TOTG's blend distance (`CircularPathSegment`,
    /// moveit2 2.15.2) between the legs `leg_in` and `length_out`.
    fn tangent_length(self, turn: f64, length_in: f64, leg_in: f64, length_out: f64) -> f64 {
        match self {
            Self::HalfShorterRun => length_in.min(length_out) / 2.0,
            Self::Within(deviation) => {
                let half = 0.5 * turn;
                (leg_in.min(length_out) / 2.0)
                    .min(deviation.get() * half.sin() / (1.0 - half.cos()))
            }
        }
    }

    /// Whether the path stops at a corner whose turn has the cosine
    /// `cos_turn` and the sine `sin_turn`.
    fn turns_back(self, cos_turn: f64, sin_turn: f64) -> bool {
        match self {
            Self::HalfShorterRun => sin_turn == 0.0 && cos_turn < 0.0,
            Self::Within(_) => cos_turn <= REVERSAL_COS,
        }
    }

    /// Whether a corner whose turn has the sine `sin_turn` runs straight on.
    fn runs_straight_on(self, sin_turn: f64) -> bool {
        match self {
            Self::HalfShorterRun => sin_turn == 0.0,
            Self::Within(_) => sin_turn <= STRAIGHT_SIN,
        }
    }

    /// Whether the path stops at a waypoint that repeats the one before it.
    fn rests_at_repeats(self) -> bool {
        matches!(self, Self::Within(_))
    }

    /// The corners of limb_motion's execute_joint_path: within
    /// [`JOINT_PATH_CORNER_DEVIATION`].
    fn joint_path() -> Self {
        Self::Within(
            PositiveFinite::parse(JOINT_PATH_CORNER_DEVIATION)
                .expect("the deviation is positive and finite"),
        )
    }
}

/// The farthest the midpoint of a corner's arc lies from the corner on a path
/// of limb_motion's execute_joint_path, Euclidean over the joints (rad, or m
/// for a prismatic joint).
pub const JOINT_PATH_CORNER_DEVIATION: f64 = 0.1;

/// The cosine of the angle between one leg and the next at or below which the
/// path turns back on itself at their corner: -0.99999, a turn of about
/// 179.74 degrees. The path stops there and turns back. TOTG refuses a path
/// with such a turn (its `angle_tolerance`, moveit2 2.15.2 `Path::create`).
const REVERSAL_COS: f64 = -1.0 + 1e-5;

/// The sine of a turn at or below which a corner within a deviation runs
/// straight on: the runs meet at its point with no arc. It lies a hundred
/// times above the rounding of the sine between runs in line a ten-thousandth
/// long, so sums that round otherwise judge such a corner alike.
const STRAIGHT_SIN: f64 = 1e-9;

/// A piece of the path, the path position it starts at, from 0, and whether
/// the path rests where the piece ends.
#[derive(Debug, Clone, PartialEq)]
struct Placed {
    start_at: f64,
    piece: Piece,
    rests_after: bool,
}

/// Lay `piece` where the last of `placed` ends, the path resting at its end
/// when `rests_after`. A piece of no length goes, and a rest at its end moves
/// to the piece before it, which ends at the same point.
fn lay(placed: &mut Vec<Placed>, piece: Piece, rests_after: bool) {
    let start_at = placed
        .last()
        .map_or(0.0, |before| before.start_at + before.piece.length());
    if piece.length() > 0.0 {
        placed.push(Placed {
            start_at,
            piece,
            rests_after,
        });
    } else if let Some(before) = placed.last_mut() {
        before.rests_after |= rests_after;
    }
}

/// The pieces of the path through `points`, no two consecutive ones the same
/// point: a straight run between each pair, and at each corner an arc tangent
/// to both runs, meeting each where `corners` puts it. A corner the path turns
/// back at, or one `stops` marks, keeps its point, where the path rests, and a
/// corner that `corners` runs straight on keeps its point, which the path runs
/// through.
fn pieces(points: &[&[f64]], stops: &[bool], corners: Corners) -> Vec<Placed> {
    let mut placed: Vec<Placed> = Vec::new();
    let mut from = points[0].to_vec();
    for k in 1..points.len() {
        let to = points[k];
        let direction_in = difference(to, &from);
        let length_in = norm(&direction_in);
        let direction_in = scaled(&direction_in, 1.0 / length_in);
        let Some(next) = points.get(k + 1) else {
            lay(
                &mut placed,
                Piece::Straight {
                    start: from,
                    direction: direction_in,
                    length: length_in,
                },
                true,
            );
            break;
        };
        let leg_in = norm(&difference(to, points[k - 1]));
        let direction_out = difference(next, to);
        let length_out = norm(&direction_out);
        let direction_out = scaled(&direction_out, 1.0 / length_out);
        let cos_turn = dot(&direction_in, &direction_out);
        // The part of the outgoing direction across the incoming one: its
        // length is the sine of the turn, and it points toward the center.
        let across = difference(&direction_out, &scaled(&direction_in, cos_turn));
        let sin_turn = norm(&across);
        let rests = stops[k] || corners.turns_back(cos_turn, sin_turn);
        if rests || corners.runs_straight_on(sin_turn) {
            lay(
                &mut placed,
                Piece::Straight {
                    start: from,
                    direction: direction_in,
                    length: length_in,
                },
                rests,
            );
            from = to.to_vec();
            continue;
        }
        let turn = sin_turn.atan2(cos_turn);
        let tangent_length = corners.tangent_length(turn, length_in, leg_in, length_out);
        let radius = tangent_length / (turn / 2.0).tan();
        lay(
            &mut placed,
            Piece::Straight {
                start: from,
                direction: direction_in.clone(),
                length: length_in - tangent_length,
            },
            false,
        );
        lay(
            &mut placed,
            Piece::Arc {
                start: sum(to, &scaled(&direction_in, -tangent_length)),
                tangent: direction_in,
                normal: scaled(&across, 1.0 / sin_turn),
                radius,
                turn,
            },
            false,
        );
        from = sum(to, &scaled(&direction_out, tangent_length));
    }
    placed
}

/// Whether `waypoint` lies within `within` of `before` in every component.
fn lies_within(waypoint: &[f64], before: &[f64], within: f64) -> bool {
    waypoint
        .iter()
        .zip(before)
        .all(|(a, b)| (a - b).abs() <= within)
}

/// The geometric path through waypoints: straight runs, each corner rounded
/// on an arc as [`Corners`] says, and at a corner the path turns back at, a
/// stop and a turn back.
#[derive(Debug, Clone, PartialEq)]
pub struct RoundedPath {
    pieces: Vec<Placed>,
    end: Vec<f64>,
}

impl RoundedPath {
    /// The path through `waypoints` (one or more, of one dimension; finite
    /// for [`Corners::Within`]), its corners rounded as `corners` says.
    pub fn through(waypoints: &[Vec<f64>], corners: Corners) -> Self {
        Self::resting_at_repeats(waypoints, corners, 0.0)
    }

    /// The path of limb_motion's execute_joint_path through `waypoints` (one
    /// or more, finite, of one dimension): each corner rounded within
    /// [`JOINT_PATH_CORNER_DEVIATION`], resting where it turns back on itself
    /// and at each waypoint within 0.001 (`MIN_JOINT_CHANGE_RAD`) in every
    /// joint of the waypoint given before it. [`JointTrajectory`] plays this
    /// path.
    pub fn joint_path(waypoints: &[Vec<f64>]) -> Self {
        Self::resting_at_repeats(waypoints, Corners::joint_path(), MIN_JOINT_CHANGE_RAD)
    }

    /// The path through `waypoints` (one or more, of one dimension; finite
    /// for [`Corners::Within`]), its corners rounded as `corners` says. A
    /// waypoint at no positive distance from the one before it is the same
    /// point; where `corners` rests at repeats,
    /// the path rests there, and at each waypoint that lies within
    /// `repeat_within` in every component of the waypoint given before it.
    fn resting_at_repeats(waypoints: &[Vec<f64>], corners: Corners, repeat_within: f64) -> Self {
        assert!(!waypoints.is_empty(), "a path has a waypoint");
        let dimension = waypoints[0].len();
        assert!(
            waypoints.iter().all(|w| w.len() == dimension),
            "the waypoints have one dimension"
        );
        let rests_at_repeats = corners.rests_at_repeats();
        assert!(
            !rests_at_repeats || waypoints.iter().flatten().all(|v| v.is_finite()),
            "the waypoints of a path rounded within a deviation are finite"
        );
        // Each point the path runs through, and whether it rests there.
        let (points, stops) = waypoints.windows(2).fold(
            (vec![waypoints[0].as_slice()], vec![false]),
            |(mut points, mut stops): (Vec<&[f64]>, Vec<bool>), pair| {
                let (before, waypoint) = (&pair[0], &pair[1]);
                let last = points.last().expect("the first waypoint is a point");
                if norm(&difference(waypoint, last)) > 0.0 {
                    points.push(waypoint.as_slice());
                    stops.push(rests_at_repeats && lies_within(waypoint, before, repeat_within));
                } else {
                    *stops.last_mut().expect("the first waypoint is a point") |= rests_at_repeats;
                }
                (points, stops)
            },
        );
        Self {
            pieces: pieces(&points, &stops, corners),
            end: points[points.len() - 1].to_vec(),
        }
    }

    /// The number of components of a waypoint.
    fn dimension(&self) -> usize {
        self.end.len()
    }

    /// The path's last waypoint, where it ends.
    pub fn end(&self) -> &[f64] {
        &self.end
    }

    /// The path's length, the distance along it from its first waypoint to
    /// its last.
    pub fn length(&self) -> f64 {
        self.pieces
            .last()
            .map_or(0.0, |placed| placed.start_at + placed.piece.length())
    }

    /// The point at `s` along the path, held to the path's ends: its first
    /// waypoint at or before 0 and its last at or after its length.
    pub fn at(&self, s: f64) -> Vec<f64> {
        self.on_piece(s)
            .map_or_else(|| self.end.clone(), |(config, _)| config)
    }

    /// The configuration and the unit tangent of the path at `s` on the piece
    /// it lies on, or `None` at or after the path's length.
    fn on_piece(&self, s: f64) -> Option<(Vec<f64>, Vec<f64>)> {
        let Placed {
            start_at, piece, ..
        } = self.piece_at(s)?;
        Some(piece.at((s - start_at).clamp(0.0, piece.length())))
    }

    /// The piece the point at `s` lies on, or `None` at or after the path's
    /// length.
    fn piece_at(&self, s: f64) -> Option<&Placed> {
        if s >= self.length() || self.pieces.is_empty() {
            return None;
        }
        let p = self
            .pieces
            .partition_point(|placed| placed.start_at <= s)
            .clamp(1, self.pieces.len())
            - 1;
        Some(&self.pieces[p])
    }

    /// The unit tangent the path heads along from the point at `s`, held to
    /// the path's ends: the way it leaves its first waypoint at or before 0,
    /// and the way it arrives at its last at or after its length. Where the
    /// path rests and turns back, the way it leaves. A path of no length heads
    /// nowhere: every component 0.
    pub fn heading_at(&self, s: f64) -> Vec<f64> {
        if let Some((_, tangent)) = self.on_piece(s) {
            return tangent;
        }
        self.pieces.last().map_or_else(
            || vec![0.0; self.dimension()],
            |placed| placed.piece.at(placed.piece.length()).1,
        )
    }

    /// Positions along the path from its first waypoint, 0, to its last, its
    /// length, in order: evenly spaced no farther apart than `spacing`, and
    /// each position where the path rests on its way, the start of the piece
    /// it leaves the rest on. Where a caller probes the path, with
    /// [`RoundedPath::at`] and [`RoundedPath::heading_at`].
    pub fn arcs_every(&self, spacing: PositiveFinite) -> Vec<f64> {
        let length = self.length();
        let count = (length / spacing.get()).ceil() as usize;
        let even = (0..=count).map(|k| {
            if k == count {
                length
            } else {
                length * k as f64 / count as f64
            }
        });
        let rests = self
            .pieces
            .windows(2)
            .filter(|pair| pair[0].rests_after)
            .map(|pair| pair[1].start_at);
        let mut arcs: Vec<f64> = even.chain(rests).collect();
        arcs.sort_by(f64::total_cmp);
        arcs.dedup();
        arcs
    }
}

/// One cell of the integration grid: its length along the path and, for each
/// component, the largest share of the tangent and the largest curvature it has
/// anywhere in the cell.
struct Cell {
    length: f64,
    tangent_share: Vec<f64>,
    curvature: Vec<f64>,
}

impl Cell {
    /// The fastest the path may run anywhere in the cell: each component's
    /// velocity limit over its share of the tangent, and, where the path
    /// curves, the speed at which the curvature alone takes a component's
    /// whole acceleration limit.
    fn ceiling(&self, max_velocity: &[f64], max_acceleration: &[f64]) -> f64 {
        self.tangent_share
            .iter()
            .zip(&self.curvature)
            .zip(max_velocity.iter().zip(max_acceleration))
            .map(|((&share, &curvature), (&v_max, &a_max))| {
                let by_velocity = if share > 0.0 {
                    v_max / share
                } else {
                    f64::INFINITY
                };
                let by_curvature = if curvature > 0.0 {
                    (a_max / curvature).sqrt()
                } else {
                    f64::INFINITY
                };
                by_velocity.min(by_curvature)
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// The speed at the far end of the cell entered at `speed`, accelerating
    /// along the path as hard as the limits allow at every point of the cell:
    /// each component's acceleration limit, less the curvature's share at the
    /// speed reached, over its share of the tangent.
    fn reach(&self, speed: f64, max_acceleration: &[f64]) -> f64 {
        let acceleration = self
            .tangent_share
            .iter()
            .zip(&self.curvature)
            .zip(max_acceleration)
            .filter(|((share, curvature), _)| **share > 0.0 || **curvature > 0.0)
            .map(|((&share, &curvature), &a_max)| {
                (a_max - curvature * speed * speed) / (share + 2.0 * curvature * self.length)
            })
            .fold(f64::INFINITY, f64::min);
        (speed * speed + 2.0 * acceleration * self.length).sqrt()
    }
}

/// One cell of the grid as the path plays it: where it starts along the path
/// and in time, its length, and the path speed at each of its ends, 0 at the
/// ends of the path.
#[derive(Debug, Clone, PartialEq)]
struct TimedCell {
    start_at: f64,
    length: f64,
    time: f64,
    speed_in: f64,
    speed_out: f64,
}

/// A path through waypoints, timed from rest to rest under per-component
/// velocity and acceleration limits.
#[derive(Debug, Clone, PartialEq)]
pub struct TimedPath {
    path: RoundedPath,
    cells: Vec<TimedCell>,
    duration: f64,
}

impl TimedPath {
    /// The path through `waypoints` (one or more, of the limits' dimension; a
    /// waypoint at no positive distance from the one before it is the same
    /// point), its corners rounded as [`Corners::HalfShorterRun`] rounds
    /// them, timed under `max_velocity` and `max_acceleration` as
    /// [`TimedPath::along`] times it.
    pub fn new(waypoints: &[Vec<f64>], max_velocity: &[f64], max_acceleration: &[f64]) -> Self {
        Self::along(
            RoundedPath::through(waypoints, Corners::HalfShorterRun),
            max_velocity,
            max_acceleration,
        )
    }

    /// `path` timed under `max_velocity` and `max_acceleration`, every
    /// component of which is positive, infinite for a component the limit
    /// leaves free, and which together bound the path's speed.
    pub fn along(path: RoundedPath, max_velocity: &[f64], max_acceleration: &[f64]) -> Self {
        assert!(
            max_velocity.len() == path.dimension() && max_acceleration.len() == path.dimension(),
            "the waypoints and both limits have one dimension"
        );
        assert!(
            max_velocity
                .iter()
                .chain(max_acceleration)
                .all(|&l| l > 0.0),
            "limits are positive"
        );
        let pieces = &path.pieces;
        let length = path.length();
        let cell_length = length / CELLS_PER_PATH as f64;
        let mut cells: Vec<Cell> = Vec::new();
        // Whether the path rests at each cell boundary: at its ends and at each
        // stop between them.
        let mut rests = vec![true];
        for placed in pieces {
            let piece = &placed.piece;
            let count = ((piece.length() / cell_length).ceil() as usize).max(2);
            let step = piece.length() / count as f64;
            for c in 0..count {
                let (from, to) = (step * c as f64, (step * (c + 1) as f64).min(piece.length()));
                cells.push(Cell {
                    length: to - from,
                    tangent_share: piece.max_abs_tangent(from, to),
                    curvature: piece.max_abs_curvature(from, to),
                });
                rests.push(false);
            }
            *rests.last_mut().expect("the piece has cells") = placed.rests_after;
        }
        let positions: Vec<f64> = std::iter::once(0.0)
            .chain(cells.iter().scan(0.0, |s, cell| {
                *s += cell.length;
                Some(*s)
            }))
            .collect();
        let ceilings: Vec<f64> = cells
            .iter()
            .map(|cell| cell.ceiling(max_velocity, max_acceleration))
            .collect();
        let boundary_ceiling = |k: usize| {
            if rests[k] {
                0.0
            } else {
                ceilings[k - 1].min(ceilings[k])
            }
        };
        // Backward: the fastest each boundary can be passed at and still stop
        // by the end of the path.
        let mut stoppable = vec![0.0; cells.len() + 1];
        for k in (0..cells.len()).rev() {
            stoppable[k] = cells[k]
                .reach(stoppable[k + 1], max_acceleration)
                .min(boundary_ceiling(k));
        }
        // Forward: the fastest each boundary is reached at from rest at the
        // start, within what is stoppable from it.
        let mut speeds = vec![0.0; cells.len() + 1];
        for k in 0..cells.len() {
            speeds[k + 1] = cells[k]
                .reach(speeds[k], max_acceleration)
                .min(stoppable[k + 1]);
        }
        let mut time = 0.0;
        let cells: Vec<TimedCell> = cells
            .iter()
            .enumerate()
            .map(|(k, cell)| {
                let (speed_in, speed_out) = (speeds[k], speeds[k + 1]);
                assert!(speed_in + speed_out > 0.0, "a cell is passed at speed");
                let timed = TimedCell {
                    start_at: positions[k],
                    length: cell.length,
                    time,
                    speed_in,
                    speed_out,
                };
                time += 2.0 * cell.length / (speed_in + speed_out);
                timed
            })
            .collect();
        assert!(
            length == 0.0 || time > 0.0,
            "the limits bound the path's speed"
        );
        Self {
            path,
            cells,
            duration: time,
        }
    }

    /// The geometric path this times.
    fn path(&self) -> &RoundedPath {
        &self.path
    }

    /// The number of components of a waypoint.
    pub fn dimension(&self) -> usize {
        self.path.dimension()
    }

    /// The whole path's time, from rest to rest.
    pub fn duration_s(&self) -> f64 {
        self.duration
    }

    /// The configuration at `fraction` of the duration: the first waypoint at
    /// or before 0, the last at or after 1, and between them the path at the
    /// position the speed profile has reached by then.
    pub fn at_fraction(&self, fraction: f64) -> Vec<f64> {
        if fraction >= 1.0 || self.cells.is_empty() {
            return self.path.end.clone();
        }
        let t = fraction.max(0.0) * self.duration;
        let k = self
            .cells
            .partition_point(|cell| cell.time <= t)
            .clamp(1, self.cells.len())
            - 1;
        let cell = &self.cells[k];
        let acceleration =
            (cell.speed_out * cell.speed_out - cell.speed_in * cell.speed_in) / (2.0 * cell.length);
        let tau = t - cell.time;
        let distance =
            (cell.speed_in * tau + 0.5 * acceleration * tau * tau).clamp(0.0, cell.length);
        self.path.at(cell.start_at + distance)
    }
}

/// A [`TimedPath`] played over a duration no shorter than its own, on a
/// [`MotionClock`]: the path's configuration at the fraction of that duration
/// the clock has played.
#[derive(Debug, Clone, PartialEq)]
pub struct TimedTrajectory {
    path: TimedPath,
    clock: MotionClock,
}

impl TimedTrajectory {
    /// `path` over `duration`, starting at `now`.
    pub fn new(path: TimedPath, duration: Duration, now: Instant) -> Self {
        assert!(
            duration >= Duration::from_secs_f64(path.duration_s()),
            "a path plays no faster than its own time"
        );
        Self {
            path,
            clock: MotionClock::starting_at(now, duration),
        }
    }

    pub fn clock(&self) -> &MotionClock {
        &self.clock
    }

    /// The timed path this plays.
    fn path(&self) -> &TimedPath {
        &self.path
    }

    /// The configuration at `now` if the path plays on at full pace from its
    /// last advance: what its player proposes for `now`.
    pub fn sample(&self, now: Instant) -> Vec<f64> {
        self.path.at_fraction(self.clock.fraction(now))
    }

    /// Play the path on to `now` at `pace`.
    fn advance(&mut self, now: Instant, pace: Pace) {
        self.clock.advance(now, pace);
    }
}

/// The most a waypoint of a joint path differs, in every joint, from the
/// waypoint given before it and still repeats it, so the path rests there.
pub const MIN_JOINT_CHANGE_RAD: f64 = 0.001;

/// A path of limb_motion's execute_joint_path played from rest to rest:
/// straight between its waypoints, each corner rounded within
/// [`JOINT_PATH_CORNER_DEVIATION`] as [`RoundedPath::joint_path`] rounds it,
/// resting where it turns back on itself and at each waypoint within 0.001
/// (`MIN_JOINT_CHANGE_RAD`) in every joint of the waypoint given before it,
/// timed under the joints' velocity and
/// acceleration limits to the grid's part in a thousand, and played over the
/// request when that is longer.
#[derive(Debug, Clone, PartialEq)]
pub struct JointTrajectory<const N: usize> {
    timed: TimedTrajectory,
}

impl<const N: usize> JointTrajectory<N> {
    /// The path through `waypoints` (one or more, where the joints stand
    /// first), starting at `now`, over the longer of `requested` and the
    /// path's own time.
    pub fn through(
        waypoints: &[[f64; N]],
        velocity_limits: &[f64; N],
        acceleration_limits: &[f64; N],
        requested: Duration,
        now: Instant,
    ) -> Self {
        let waypoints: Vec<Vec<f64>> = waypoints.iter().map(|q| q.to_vec()).collect();
        let path = TimedPath::along(
            RoundedPath::joint_path(&waypoints),
            velocity_limits,
            acceleration_limits,
        );
        let duration = duration_at_least(path.duration_s(), requested);
        Self {
            timed: TimedTrajectory::new(path, duration, now),
        }
    }

    pub fn clock(&self) -> &MotionClock {
        self.timed.clock()
    }

    /// The geometric path the trajectory plays.
    pub fn path(&self) -> &RoundedPath {
        self.timed.path().path()
    }

    /// The path's last waypoint, where the move ends.
    pub fn end(&self) -> [f64; N] {
        self.path()
            .end()
            .try_into()
            .expect("a joint path has a configuration's components")
    }

    /// The configuration at `now` if the path plays on at full pace: its first
    /// waypoint before the move, its last exactly once the clock reaches the
    /// end, and the path between them in between.
    pub fn sample(&self, now: Instant) -> [f64; N] {
        self.timed
            .sample(now)
            .try_into()
            .expect("a joint path has a configuration's components")
    }

    /// Play the path on to `now` at `pace`.
    pub fn advance(&mut self, now: Instant, pace: Pace) {
        self.timed.advance(now, pace);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators::{self as gs, Generator};

    const JOINTS: usize = 4;

    /// Steps of up to 0.5 in each component on a grid of thousandths.
    fn steps() -> impl hegel::generators::PrintableGenerator<[i32; JOINTS]> {
        gs::arrays(gs::integers::<i32>().min_value(-500).max_value(500))
    }

    fn moving(step: &[i32; JOINTS]) -> bool {
        step.iter().any(|&v| v != 0)
    }

    fn to_f64(step: [i32; JOINTS]) -> Vec<f64> {
        step.iter().map(|&v| f64::from(v) / 1000.0).collect()
    }

    /// Up to twelve waypoints from the origin, each a moving step from the
    /// last, so no two consecutive are equal; the origin alone when every
    /// drawn step is zero.
    fn waypoints(tc: &TestCase) -> Vec<Vec<f64>> {
        let steps: Vec<[i32; JOINTS]> = tc.draw(gs::vecs(steps()).min_size(1).max_size(11));
        std::iter::once(vec![0.0; JOINTS])
            .chain(
                steps
                    .into_iter()
                    .filter(moving)
                    .scan(vec![0.0; JOINTS], |q, step| {
                        *q = sum(q, &to_f64(step));
                        Some(q.clone())
                    }),
            )
            .collect()
    }

    fn limits(tc: &TestCase) -> ([f64; JOINTS], [f64; JOINTS]) {
        (
            tc.draw(gs::arrays(gs::floats().min_value(0.1).max_value(5.0))),
            tc.draw(gs::arrays(gs::floats().min_value(0.1).max_value(20.0))),
        )
    }

    /// Rounding slack on a limit.
    const SLACK: f64 = 1.0 + 1e-6;

    /// The first sample, of configurations sampled `dt` apart, where a
    /// component moves or accelerates past its limit: none when the motion
    /// keeps its limits.
    fn past_the_limits(
        samples: &[Vec<f64>],
        dt: f64,
        max_velocity: &[f64; JOINTS],
        max_acceleration: &[f64; JOINTS],
    ) -> Option<String> {
        samples.windows(3).enumerate().find_map(|(k, w)| {
            (0..JOINTS).find_map(|i| {
                let speed = (w[1][i] - w[0][i]).abs() / dt;
                let acceleration = (w[2][i] - 2.0 * w[1][i] + w[0][i]).abs() / (dt * dt);
                if speed > max_velocity[i] * SLACK {
                    Some(format!(
                        "component {i} at {speed} rad/s past its {} at sample {k}",
                        max_velocity[i]
                    ))
                } else if acceleration > max_acceleration[i] * SLACK {
                    Some(format!(
                        "component {i} at {acceleration} rad/s^2 past its {} at sample {k}",
                        max_acceleration[i]
                    ))
                } else {
                    None
                }
            })
        })
    }

    // A timed path through any waypoints, under any limits, starts at rest on
    // the first and ends at rest on the last; sampled a thousand times through
    // its duration, every component stays within its velocity limit and
    // accelerates within its acceleration limit, and the path stays within the
    // box of the waypoints.
    #[hegel::test]
    fn a_timed_path_runs_from_rest_to_rest_within_its_limits(tc: TestCase) {
        let waypoints = waypoints(&tc);
        if waypoints.len() < 2 {
            return;
        }
        let (max_velocity, max_acceleration) = limits(&tc);
        let path = TimedPath::new(&waypoints, &max_velocity, &max_acceleration);
        let duration = path.duration_s();
        assert!(duration > 0.0 && duration.is_finite(), "a {duration}s path");
        assert_eq!(path.at_fraction(0.0), waypoints[0]);
        assert_eq!(path.at_fraction(1.0), waypoints[waypoints.len() - 1]);
        const SAMPLES: usize = 1000;
        let dt = duration / SAMPLES as f64;
        let samples: Vec<Vec<f64>> = (0..=SAMPLES)
            .map(|k| path.at_fraction(k as f64 / SAMPLES as f64))
            .collect();
        let (lo, hi) = waypoints.iter().fold(
            (vec![f64::INFINITY; JOINTS], vec![f64::NEG_INFINITY; JOINTS]),
            |(lo, hi), w| {
                (
                    lo.iter().zip(w).map(|(l, x)| l.min(*x)).collect(),
                    hi.iter().zip(w).map(|(h, x)| h.max(*x)).collect(),
                )
            },
        );
        for (k, w) in samples.windows(3).enumerate() {
            for i in 0..JOINTS {
                assert!(
                    w[1][i] >= lo[i] - 1e-9 && w[1][i] <= hi[i] + 1e-9,
                    "component {i} left the waypoints' box at sample {k}"
                );
                let speed = (w[1][i] - w[0][i]).abs() / dt;
                assert!(
                    speed <= max_velocity[i] * SLACK,
                    "component {i} at {speed} rad/s past its {} at sample {k}",
                    max_velocity[i]
                );
                let acceleration = (w[2][i] - 2.0 * w[1][i] + w[0][i]).abs() / (dt * dt);
                assert!(
                    acceleration <= max_acceleration[i] * SLACK,
                    "component {i} at {acceleration} rad/s^2 past its {} at sample {k}",
                    max_acceleration[i]
                );
            }
        }
    }

    // A path through any waypoints, rounded within any deviation and timed
    // under any limits, keeps every component within its limits: sampled a
    // thousand times through its duration, every component stays within its
    // velocity limit and accelerates within its acceleration limit.
    #[hegel::test]
    fn a_path_rounded_within_a_deviation_runs_within_its_limits(tc: TestCase) {
        let waypoints = waypoints(&tc);
        if waypoints.len() < 2 {
            return;
        }
        let (max_velocity, max_acceleration) = limits(&tc);
        let path = TimedPath::along(
            RoundedPath::through(&waypoints, Corners::Within(deviation(&tc))),
            &max_velocity,
            &max_acceleration,
        );
        const SAMPLES: usize = 1000;
        let samples: Vec<Vec<f64>> = (0..=SAMPLES)
            .map(|k| path.at_fraction(k as f64 / SAMPLES as f64))
            .collect();
        let dt = path.duration_s() / SAMPLES as f64;
        if let Some(past) = past_the_limits(&samples, dt, &max_velocity, &max_acceleration) {
            panic!("{past}");
        }
    }

    /// Up to twelve waypoints of a joint path from the origin, each a step
    /// from the last, with whether it is a near step: of at most 0.0009 in
    /// every joint on a grid of ten-thousandths, which repeats the waypoint
    /// given before it, or of at least 0.002 in some joint on a grid of
    /// thousandths, which does not.
    fn joint_waypoints(tc: &TestCase) -> Vec<([f64; JOINTS], bool)> {
        let count: usize = tc.draw(gs::integers::<usize>().min_value(1).max_value(11));
        (0..count).fold(vec![([0.0; JOINTS], false)], |mut path, _| {
            let near: bool = tc.draw(gs::booleans());
            let step: [f64; JOINTS] = if near {
                tc.draw(gs::arrays(gs::integers::<i32>().min_value(-9).max_value(9)))
                    .map(|v| f64::from(v) / 10_000.0)
            } else {
                tc.draw(steps().filter(|step| step.iter().any(|v| v.abs() >= 2)))
                    .map(|v| f64::from(v) / 1000.0)
            };
            let (last, _) = path[path.len() - 1];
            path.push((std::array::from_fn(|i| last[i] + step[i]), near));
            path
        })
    }

    // A joint trajectory through any waypoints, near repeats among them, under
    // any limits and request, keeps every joint within its limits through each
    // corner its deviation rounds: sampled on its clock a thousand times
    // through its duration, every joint stays within its velocity limit and
    // accelerates within its acceleration limit.
    #[hegel::test]
    fn a_joint_trajectory_keeps_every_joint_within_its_limits(tc: TestCase) {
        let waypoints: Vec<[f64; JOINTS]> =
            joint_waypoints(&tc).into_iter().map(|(q, _)| q).collect();
        let (max_velocity, max_acceleration) = limits(&tc);
        let requested: f64 = tc.draw(gs::floats().min_value(0.0).max_value(10.0));
        let start = Instant::now();
        let traj = JointTrajectory::through(
            &waypoints,
            &max_velocity,
            &max_acceleration,
            Duration::from_secs_f64(requested),
            start,
        );
        const SAMPLES: u32 = 1000;
        let step = traj.clock().duration() / SAMPLES;
        if step.is_zero() {
            return;
        }
        let samples: Vec<Vec<f64>> = (0..=SAMPLES)
            .map(|k| traj.sample(start + step * k).to_vec())
            .collect();
        if let Some(past) = past_the_limits(
            &samples,
            step.as_secs_f64(),
            &max_velocity,
            &max_acceleration,
        ) {
            panic!("{past}");
        }
    }

    // A straight run between two waypoints takes the time of the trapezoid
    // under the path's speed and acceleration caps, no less, and no more than
    // the grid's three parts in CELLS_PER_PATH.
    #[hegel::test]
    fn a_straight_run_takes_its_trapezoid_time(tc: TestCase) {
        let step: [i32; JOINTS] = tc.draw(steps());
        if !moving(&step) {
            return;
        }
        let end = to_f64(step);
        let (max_velocity, max_acceleration) = limits(&tc);
        let length = norm(&end);
        let direction = scaled(&end, 1.0 / length);
        let cap = |limits: &[f64; JOINTS]| {
            (0..JOINTS)
                .filter(|&i| direction[i] != 0.0)
                .map(|i| limits[i] / direction[i].abs())
                .fold(f64::INFINITY, f64::min)
        };
        let (v, a) = (cap(&max_velocity), cap(&max_acceleration));
        let trapezoid = if length >= v * v / a {
            length / v + v / a
        } else {
            2.0 * (length / a).sqrt()
        };
        let path = TimedPath::new(&[vec![0.0; JOINTS], end], &max_velocity, &max_acceleration);
        let duration = path.duration_s();
        assert!(
            duration >= trapezoid / SLACK,
            "{duration} s beats the {trapezoid} s trapezoid"
        );
        let grid = 1.0 + 3.0 / CELLS_PER_PATH as f64;
        assert!(
            duration <= trapezoid * grid,
            "{duration} s is past the grid's slack on the {trapezoid} s trapezoid"
        );
    }

    // A path that reverses at a corner stops there: out and back along a line
    // takes twice the single run and sits on the corner at half time.
    #[test]
    fn a_reversal_stops_at_its_corner() {
        let (v, a) = ([1.0; JOINTS], [1.0; JOINTS]);
        let corner = vec![0.5, 0.0, 0.0, 0.0];
        let single = TimedPath::new(&[vec![0.0; JOINTS], corner.clone()], &v, &a);
        let out_and_back = TimedPath::new(
            &[vec![0.0; JOINTS], corner.clone(), vec![0.0; JOINTS]],
            &v,
            &a,
        );
        assert!((out_and_back.duration_s() - 2.0 * single.duration_s()).abs() < 1e-9);
        let midway = out_and_back.at_fraction(0.5);
        assert!((midway[0] - corner[0]).abs() < 1e-9, "midway at {midway:?}");
    }

    // A waypoint at no distance from the one before it is the same point: a
    // path through repeats is the path through the distinct waypoints, and a
    // path through one point takes no time and sits on it.
    #[test]
    fn repeated_waypoints_are_one_point() {
        let (v, a) = ([1.0; JOINTS], [1.0; JOINTS]);
        let (origin, corner, end) = (
            vec![0.0; JOINTS],
            vec![0.3, 0.0, 0.0, 0.0],
            vec![0.3, 0.2, 0.0, 0.0],
        );
        let repeated = TimedPath::new(
            &[
                origin.clone(),
                origin.clone(),
                corner.clone(),
                corner.clone(),
                end.clone(),
                end.clone(),
            ],
            &v,
            &a,
        );
        let distinct = TimedPath::new(&[origin.clone(), corner, end.clone()], &v, &a);
        assert_eq!(repeated, distinct);
        let still = TimedPath::new(&[end.clone(), end.clone()], &v, &a);
        assert_eq!(still.duration_s(), 0.0);
        assert_eq!(still.at_fraction(0.3), end);
    }

    // A path rounded within a deviation stops at a waypoint at no distance from
    // the one before it: through a repeated corner, a path takes its two runs each
    // from rest to rest and sits on the corner between them. Repeats at the
    // ends change nothing, and a path through one point takes no time and sits
    // on it.
    #[test]
    fn a_repeated_waypoint_is_a_stop() {
        let (v, a) = ([1.0; JOINTS], [1.0; JOINTS]);
        let (origin, corner, end) = (
            vec![0.0; JOINTS],
            vec![0.5, 0.0, 0.0, 0.0],
            vec![0.5, 0.5, 0.0, 0.0],
        );
        let timed = |waypoints: &[Vec<f64>]| {
            TimedPath::along(
                RoundedPath::through(waypoints, Corners::joint_path()),
                &v,
                &a,
            )
        };
        let single = timed(&[origin.clone(), corner.clone()]);
        let stopping = timed(&[origin.clone(), corner.clone(), corner.clone(), end.clone()]);
        assert!((stopping.duration_s() - 2.0 * single.duration_s()).abs() < 1e-9);
        let midway = stopping.at_fraction(0.5);
        assert!(
            norm(&difference(&midway, &corner)) < 1e-9,
            "midway at {midway:?}, not stopped on the corner"
        );
        let rounded = timed(&[origin.clone(), corner.clone(), end.clone()]);
        assert!(rounded.duration_s() < stopping.duration_s());
        let repeated_ends = timed(&[
            origin.clone(),
            origin.clone(),
            corner,
            end.clone(),
            end.clone(),
        ]);
        assert_eq!(repeated_ends.duration_s(), rounded.duration_s());
        for k in 0..=20 {
            let fraction = f64::from(k) / 20.0;
            assert_eq!(
                repeated_ends.at_fraction(fraction),
                rounded.at_fraction(fraction)
            );
        }
        let still = timed(&[end.clone(), end.clone()]);
        assert_eq!(still.duration_s(), 0.0);
        assert_eq!(still.at_fraction(0.3), end);
    }

    /// The instants a motion of `duration` is sampled at: its start, its
    /// midpoint, its end, past its end, and the nanosecond before its end.
    fn checkpoints(start: Instant, duration: Duration) -> [Instant; 5] {
        let half = Duration::from_secs_f64(duration.as_secs_f64() / 2.0);
        [
            start,
            start + half,
            start + duration,
            start + duration + Duration::from_secs(5),
            start + duration.saturating_sub(Duration::from_nanos(1)),
        ]
    }

    // A joint trajectory between any two configurations, under any limits and
    // request, lasts the longer of the request and its path's own time; it
    // starts at `start`, is halfway at half its duration, holds exactly at
    // `end` from its duration on, and is complete exactly from then.
    #[hegel::test]
    fn a_joint_trajectory_runs_its_sized_duration_from_start_to_end(tc: TestCase) {
        // Configurations on a microradian grid within pi of zero: a step under
        // the grid is no motion.
        let span = || {
            gs::arrays(
                gs::integers::<i32>()
                    .min_value(-3_141_592)
                    .max_value(3_141_592),
            )
        };
        let on_grid = |q: [i32; JOINTS]| q.map(|v| f64::from(v) / 1e6);
        let start = on_grid(tc.draw(span()));
        let end = on_grid(tc.draw(span()));
        let (max_velocity, max_acceleration) = limits(&tc);
        let requested: f64 = tc.draw(gs::floats().min_value(0.0).max_value(10.0));
        let traj = JointTrajectory::through(
            &[start, end],
            &max_velocity,
            &max_acceleration,
            Duration::from_secs_f64(requested),
            Instant::now(),
        );
        let path = TimedPath::new(
            &[start.to_vec(), end.to_vec()],
            &max_velocity,
            &max_acceleration,
        );
        let clock = traj.clock();
        // The clock played at full pace to `now`.
        let played_to = |now: Instant| {
            let mut played = *clock;
            played.advance(now, Pace::FULL);
            played
        };
        let duration = clock.duration().as_secs_f64();
        // The clock keeps whole nanoseconds.
        let want = requested.max(path.duration_s());
        assert!(
            (duration - want).abs() <= 1e-9,
            "lasts {duration}s, not the larger of the {requested}s request and the path's {}s",
            path.duration_s()
        );
        let [at_start, at_half, at_end, past_end, before_end] =
            checkpoints(clock.motion_start(), clock.duration());
        let off = |q: [f64; JOINTS], want: [f64; JOINTS]| {
            q.iter()
                .zip(&want)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max)
        };
        let midway = std::array::from_fn(|i| start[i] + 0.5 * (end[i] - start[i]));
        // The profile from rest to rest is symmetric, so half the travel is
        // done at half the time; the half-time instant is a whole nanosecond,
        // over which the path moves at most at its peak speed, under twice
        // its mean.
        let travel = off(end, start);
        let rounding = if travel > 0.0 {
            2.0 * travel / path.duration_s() * 1e-9
        } else {
            0.0
        };
        if duration > 0.0 {
            assert_eq!(traj.sample(at_start), start);
            assert!(off(traj.sample(at_half), midway) <= 1e-12 + rounding);
            assert!(!played_to(before_end).is_complete());
        }
        assert_eq!(traj.sample(at_end), end);
        assert_eq!(traj.sample(past_end), end);
        assert!(played_to(at_end).is_complete() && played_to(past_end).is_complete());
    }

    // A joint trajectory between two configurations, under any limits and
    // request, is the move TimedPath::new times between them, over the longer
    // of the request and its own time on a clock played at full pace: the same
    // duration, the same sample to the bit at any instant, and complete from
    // its duration on.
    #[hegel::test]
    fn a_joint_trajectory_between_two_configurations_plays_the_timed_path_between_them(
        tc: TestCase,
    ) {
        let span = || {
            gs::arrays(
                gs::integers::<i32>()
                    .min_value(-3_141_592)
                    .max_value(3_141_592),
            )
        };
        let on_grid = |q: [i32; JOINTS]| q.map(|v| f64::from(v) / 1e6);
        let (start, end) = (on_grid(tc.draw(span())), on_grid(tc.draw(span())));
        let (max_velocity, max_acceleration) = limits(&tc);
        let requested =
            Duration::from_secs_f64(tc.draw(gs::floats().min_value(0.0).max_value(10.0)));
        let fractions: Vec<f64> = tc.draw(
            gs::vecs(gs::floats().min_value(0.0).max_value(1.5))
                .min_size(1)
                .max_size(20),
        );
        let now = Instant::now();
        let traj = JointTrajectory::through(
            &[start, end],
            &max_velocity,
            &max_acceleration,
            requested,
            now,
        );
        let path = TimedPath::new(
            &[start.to_vec(), end.to_vec()],
            &max_velocity,
            &max_acceleration,
        );
        let duration = duration_at_least(path.duration_s(), requested);
        let timed = TimedTrajectory::new(path, duration, now);
        assert_eq!(traj.clock().duration(), duration);
        let bits = |q: &[f64]| q.iter().map(|v| v.to_bits()).collect::<Vec<u64>>();
        for fraction in fractions {
            let at = now + duration.mul_f64(fraction);
            assert_eq!(
                bits(&traj.sample(at)),
                bits(&timed.sample(at)),
                "at {fraction} of the duration"
            );
            let mut played = *traj.clock();
            played.advance(at, Pace::FULL);
            assert_eq!(played.is_complete(), at >= now + duration);
        }
    }

    /// The distance from `point` to the segment from `a` to `b`.
    fn distance_to_segment(point: &[f64], a: &[f64], b: &[f64]) -> f64 {
        let run = difference(b, a);
        let length_sq = dot(&run, &run);
        let t = if length_sq > 0.0 {
            (dot(&difference(point, a), &run) / length_sq).clamp(0.0, 1.0)
        } else {
            0.0
        };
        norm(&difference(point, &sum(a, &scaled(&run, t))))
    }

    /// The distance from `point` to the straight runs through `waypoints`.
    fn distance_to_runs(point: &[f64], waypoints: &[Vec<f64>]) -> f64 {
        waypoints
            .windows(2)
            .map(|w| distance_to_segment(point, &w[0], &w[1]))
            .fold(f64::INFINITY, f64::min)
    }

    /// The distance from `point` to the path: exact to its straight pieces,
    /// and to its arcs sampled finely.
    fn distance_to_path(point: &[f64], path: &RoundedPath) -> f64 {
        path.pieces
            .iter()
            .map(|placed| match &placed.piece {
                Piece::Straight {
                    start,
                    direction,
                    length,
                } => distance_to_segment(point, start, &sum(start, &scaled(direction, *length))),
                arc => (0..=1000)
                    .map(|k| {
                        let (q, _) = arc.at(arc.length() * k as f64 / 1000.0);
                        norm(&difference(point, &q))
                    })
                    .fold(f64::INFINITY, f64::min),
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// A deviation from a thousandth to a quarter.
    fn deviation(tc: &TestCase) -> PositiveFinite {
        PositiveFinite::parse(tc.draw(gs::floats().min_value(0.001).max_value(0.25)))
            .expect("drawn positive")
    }

    // A path rounded within a deviation, through any waypoints, never lies
    // farther than the deviation from the straight runs, and passes through
    // every run: through the midpoint of each, which lies within half the
    // run of both its corners.
    #[hegel::test]
    fn a_rounded_path_stays_within_its_deviation_and_passes_through_every_run(tc: TestCase) {
        let waypoints = waypoints(&tc);
        if waypoints.len() < 2 {
            return;
        }
        let deviation = deviation(&tc);
        let path = RoundedPath::through(&waypoints, Corners::Within(deviation));
        const SAMPLES: usize = 2000;
        for k in 0..=SAMPLES {
            let q = path.at(path.length() * k as f64 / SAMPLES as f64);
            let off = distance_to_runs(&q, &waypoints);
            assert!(
                off <= deviation.get() * (1.0 + 1e-9) + 1e-12,
                "{off} from the runs at sample {k}, past the {deviation} deviation"
            );
        }
        for w in waypoints.windows(2) {
            let midpoint = scaled(&sum(&w[0], &w[1]), 0.5);
            let off = distance_to_path(&midpoint, &path);
            assert!(off <= 1e-9, "the path passes {off} from a run's midpoint");
        }
        assert_eq!(path.at(0.0), waypoints[0]);
        assert_eq!(path.at(path.length()), waypoints[waypoints.len() - 1]);
    }

    // A right-angle corner rounded within a deviation under its half runs has
    // its arc's midpoint exactly the deviation from the corner, as TOTG blends
    // it at its maximum deviation.
    #[test]
    fn a_binding_deviation_puts_the_arcs_midpoint_that_far_from_the_corner() {
        let corner = vec![1.0, 0.0];
        let waypoints = [vec![0.0, 0.0], corner.clone(), vec![1.0, 1.0]];
        let deviation = 0.01;
        let path = RoundedPath::through(
            &waypoints,
            Corners::Within(PositiveFinite::parse(deviation).expect("positive")),
        );
        let arc = &path.pieces[1];
        let (midpoint, _) = arc.piece.at(arc.piece.length() / 2.0);
        let off = norm(&difference(&midpoint, &corner));
        assert!(
            (off - deviation).abs() <= 1e-15,
            "the midpoint is {off} off"
        );
    }

    /// A path from the origin to a corner half a unit along the first axis,
    /// and on from it by half of (`cos`, `sin`) in the first two axes: a
    /// turn of that cosine and sine when the two make a unit vector.
    fn turning(cos: f64, sin: f64) -> Vec<Vec<f64>> {
        vec![
            vec![0.0; JOINTS],
            vec![0.5, 0.0, 0.0, 0.0],
            vec![0.5 + 0.5 * cos, 0.5 * sin, 0.0, 0.0],
        ]
    }

    /// Whether `path` runs straight to its one corner, rests there when
    /// `rests`, and runs straight on from it, with no arc.
    fn keeps_its_corner(path: &RoundedPath, rests: bool) -> bool {
        matches!(
            path.pieces.as_slice(),
            [
                Placed { piece: Piece::Straight { .. }, rests_after, .. },
                Placed { piece: Piece::Straight { .. }, .. },
            ] if *rests_after == rests
        )
    }

    /// Whether `path` rounds its one corner on an arc between two straight
    /// runs, with no rest before its end.
    fn rounds_its_corner(path: &RoundedPath) -> bool {
        matches!(
            path.pieces.as_slice(),
            [
                Placed {
                    piece: Piece::Straight { .. },
                    rests_after: false,
                    ..
                },
                Placed {
                    piece: Piece::Arc { .. },
                    rests_after: false,
                    ..
                },
                Placed {
                    piece: Piece::Straight { .. },
                    ..
                },
            ]
        )
    }

    // STRAIGHT_SIN from both sides: a corner whose turn has a sine of 1e-10
    // runs straight on through its point, and one of 1e-8 is rounded.
    #[test]
    fn a_corner_within_the_straight_sine_runs_straight_on_and_one_past_it_is_rounded() {
        let path = |sin: f64| RoundedPath::through(&turning(1.0, sin), Corners::joint_path());
        assert!(keeps_its_corner(&path(1e-10), false));
        assert!(rounds_its_corner(&path(1e-8)));
    }

    #[test]
    #[should_panic(expected = "the waypoints of a path rounded within a deviation are finite")]
    fn a_path_within_a_deviation_through_a_nan_waypoint_is_refused() {
        RoundedPath::joint_path(&[vec![0.0; 2], vec![f64::NAN, 0.0]]);
    }

    #[test]
    #[should_panic(expected = "the waypoints of a path rounded within a deviation are finite")]
    fn a_path_within_a_deviation_through_an_infinite_waypoint_is_refused() {
        RoundedPath::through(
            &[vec![0.0; 2], vec![0.0, f64::INFINITY]],
            Corners::joint_path(),
        );
    }

    // TimedPath::new takes a NaN waypoint, at no positive distance from the
    // waypoint before it, as a repeat of that one: the path through the
    // waypoints either side of it.
    #[test]
    fn a_timed_path_takes_a_nan_waypoint_as_a_repeat() {
        let (v, a) = ([1.0; JOINTS], [1.0; JOINTS]);
        let (origin, corner, end) = (
            vec![0.0; JOINTS],
            vec![0.3, 0.0, 0.0, 0.0],
            vec![0.3, 0.2, 0.0, 0.0],
        );
        let nan = vec![f64::NAN, 0.0, 0.0, 0.0];
        assert_eq!(
            TimedPath::new(&[origin.clone(), corner.clone(), nan, end.clone()], &v, &a),
            TimedPath::new(&[origin, corner, end], &v, &a)
        );
    }

    // The arcs every spacing along a path run in order from 0 to its length
    // and hold each rest on its way, where the path then heads along the
    // piece it leaves on, and the points there start on its first waypoint,
    // end on its last, and lie no farther apart than the spacing.
    #[hegel::test]
    fn arcs_every_spacing_cover_the_path(tc: TestCase) {
        let waypoints = waypoints(&tc);
        let spacing = PositiveFinite::parse(tc.draw(gs::floats().min_value(0.001).max_value(1.0)))
            .expect("positive");
        let path = RoundedPath::through(&waypoints, Corners::joint_path());
        let arcs = path.arcs_every(spacing);
        assert_eq!((arcs[0], arcs[arcs.len() - 1]), (0.0, path.length()));
        assert!(arcs.windows(2).all(|w| w[0] < w[1]), "{arcs:?}");
        for pair in path.pieces.windows(2).filter(|pair| pair[0].rests_after) {
            assert!(arcs.contains(&pair[1].start_at), "a rest is probed");
            assert_eq!(
                path.heading_at(pair[1].start_at),
                pair[1].piece.at(0.0).1,
                "the path leaves a rest along the piece after it"
            );
        }
        let points: Vec<Vec<f64>> = arcs.iter().map(|&s| path.at(s)).collect();
        assert_eq!(points[0], waypoints[0]);
        assert_eq!(points[points.len() - 1], waypoints[waypoints.len() - 1]);
        for w in points.windows(2) {
            let apart = norm(&difference(&w[1], &w[0]));
            assert!(apart <= spacing.get() * (1.0 + 1e-9), "{apart} apart");
        }
    }

    // A path heads along its tangent: within each piece, the point a short
    // step farther along lies that step along the heading, off it by at most
    // the arc's sag over the step, and the heading has unit length. At its
    // ends it heads the way its first piece leaves and its last arrives, and
    // a path of no length heads nowhere.
    #[hegel::test]
    fn a_path_heads_along_its_tangent(tc: TestCase) {
        let waypoints = waypoints(&tc);
        let path = RoundedPath::through(&waypoints, Corners::Within(deviation(&tc)));
        let (Some(first), Some(last)) = (path.pieces.first(), path.pieces.last()) else {
            assert!(path.heading_at(0.0).iter().all(|&h| h == 0.0));
            return;
        };
        assert_eq!(path.heading_at(-1.0), first.piece.at(0.0).1);
        assert_eq!(
            path.heading_at(path.length() + 1.0),
            last.piece.at(last.piece.length()).1
        );
        const STEP: f64 = 1e-7;
        for placed in &path.pieces {
            let length = placed.piece.length();
            let curvature = match &placed.piece {
                Piece::Straight { .. } => 0.0,
                Piece::Arc { radius, .. } => 1.0 / radius,
            };
            for k in 1..10 {
                let s = placed.start_at + length * f64::from(k) / 10.0;
                if s + STEP >= placed.start_at + length {
                    continue;
                }
                let heading = path.heading_at(s);
                assert!((norm(&heading) - 1.0).abs() <= 1e-12, "{heading:?}");
                let ahead = difference(&path.at(s + STEP), &path.at(s));
                let off = norm(&difference(&ahead, &scaled(&heading, STEP)));
                assert!(
                    off <= STEP * STEP * curvature + 1e-12,
                    "{off} off the heading at {s} on a piece of curvature {curvature}"
                );
            }
        }
    }

    /// Where `path` rests: its first waypoint, and the end of each piece the
    /// path rests after, its last waypoint among them.
    fn rests(path: &RoundedPath) -> Vec<Vec<f64>> {
        std::iter::once(path.at(0.0))
            .chain(
                path.pieces
                    .iter()
                    .filter(|placed| placed.rests_after)
                    .map(|placed| placed.piece.at(placed.piece.length()).0),
            )
            .collect()
    }

    /// Where the joint path through `waypoints` rests.
    fn joint_path_rests<const N: usize>(waypoints: &[[f64; N]]) -> Vec<Vec<f64>> {
        let traj = JointTrajectory::through(
            waypoints,
            &[1.0; N],
            &[1.0; N],
            Duration::ZERO,
            Instant::now(),
        );
        rests(traj.path())
    }

    /// Whether `point` lies on one of `points`, to the rounding of a path's
    /// pieces.
    fn among(point: &[f64], points: &[Vec<f64>]) -> bool {
        points.iter().any(|p| norm(&difference(point, p)) <= 1e-12)
    }

    // A joint path rests at each waypoint within MIN_JOINT_CHANGE_RAD in every
    // joint of the waypoint given before it, and at its last, each where it
    // is given; and every point it rests at is a waypoint it is given.
    #[hegel::test]
    fn a_joint_path_rests_at_each_waypoint_that_repeats_the_one_given_before_it(tc: TestCase) {
        let drawn = joint_waypoints(&tc);
        let waypoints: Vec<[f64; JOINTS]> = drawn.iter().map(|(q, _)| *q).collect();
        let rests = joint_path_rests(&waypoints);
        let repeats = drawn
            .iter()
            .skip(1)
            .filter(|(_, near)| *near)
            .map(|(q, _)| q);
        for q in repeats.chain(waypoints.last()) {
            assert!(among(q, &rests), "no rest at {q:?}, only at {rests:?}");
        }
        let given: Vec<Vec<f64>> = waypoints.iter().map(|q| q.to_vec()).collect();
        for rest in &rests {
            assert!(among(rest, &given), "a rest at {rest:?}, no waypoint");
        }
    }

    // MIN_JOINT_CHANGE_RAD is inclusive: a waypoint exactly that far from the
    // one given before it repeats it, and the path rests there; one a hair
    // farther does not, and the path rounds it.
    #[test]
    fn a_waypoint_exactly_the_least_change_from_the_one_before_it_repeats_it() {
        let through = |x: f64| joint_path_rests(&[[0.0, 1.0], [0.0, 0.0], [x, 0.0], [x, 1.0]]);
        let exact = MIN_JOINT_CHANGE_RAD;
        assert!(among(&[exact, 0.0], &through(exact)));
        let past = MIN_JOINT_CHANGE_RAD.next_up();
        assert!(!among(&[past, 0.0], &through(past)));
    }

    // Each waypoint is judged against the waypoint given before it: through
    // near steps that add up past MIN_JOINT_CHANGE_RAD, the path rests at
    // each of them.
    #[test]
    fn a_joint_path_rests_at_each_of_a_run_of_near_steps() {
        let rests = joint_path_rests(&[[0.0, 0.0], [0.0006, 0.0], [0.0012, 0.0], [0.0012, 1.0]]);
        assert_eq!(rests.len(), 4, "rests at {rests:?}");
        assert!(among(&[0.0006, 0.0], &rests) && among(&[0.0012, 0.0], &rests));
    }

    /// Each arc of `path`, through waypoints no two consecutive of which are
    /// the same point, with the index of the waypoint whose corner it rounds:
    /// the inner corners in order, each one an arc or the end of a straight
    /// run that no arc follows.
    fn arcs_with_corners(path: &RoundedPath) -> Vec<(&Piece, usize)> {
        let ends_at_a_corner = |i: usize| {
            !matches!(
                path.pieces.get(i + 1),
                Some(Placed {
                    piece: Piece::Arc { .. },
                    ..
                })
            )
        };
        let (_, arcs) = path.pieces.iter().enumerate().fold(
            (1, Vec::new()),
            |(corner, mut arcs), (i, placed)| match placed.piece {
                Piece::Arc { .. } => {
                    arcs.push((&placed.piece, corner));
                    (corner + 1, arcs)
                }
                Piece::Straight { .. } if ends_at_a_corner(i) => (corner + 1, arcs),
                Piece::Straight { .. } => (corner, arcs),
            },
        );
        arcs
    }

    /// Assert that each arc of `path`, rounded within
    /// JOINT_PATH_CORNER_DEVIATION through `waypoints` (no two consecutive
    /// the same point), is the largest one within both bounds: its midpoint
    /// lies at most the deviation from its corner, its tangent points lie
    /// within half of each leg, and one of the two bounds holds with equality.
    fn assert_each_arc_is_the_largest_within_both_bounds(
        path: &RoundedPath,
        waypoints: &[Vec<f64>],
    ) {
        let tolerance = 1e-9;
        for (arc, k) in arcs_with_corners(path) {
            let (before, corner, after) = (&waypoints[k - 1], &waypoints[k], &waypoints[k + 1]);
            // The tangents at the arc's ends meet at its corner.
            let (start, direction_in) = arc.at(0.0);
            let (end, direction_out) = arc.at(arc.length());
            let tangent =
                norm(&difference(&end, &start)) / norm(&sum(&direction_in, &direction_out));
            let meets = sum(&start, &scaled(&direction_in, tangent));
            assert!(
                norm(&difference(corner, &meets)) <= tolerance * (1.0 + norm(corner)),
                "the arc's tangents meet at {meets:?}, off its corner {corner:?}"
            );
            let (midpoint, _) = arc.at(arc.length() / 2.0);
            let deviation = norm(&difference(&midpoint, corner));
            let half_leg =
                norm(&difference(corner, before)).min(norm(&difference(after, corner))) / 2.0;
            assert!(deviation <= JOINT_PATH_CORNER_DEVIATION * (1.0 + tolerance));
            assert!(tangent <= half_leg * (1.0 + tolerance));
            assert!(
                (deviation - JOINT_PATH_CORNER_DEVIATION).abs() <= tolerance
                    || (tangent - half_leg).abs() <= tolerance * (1.0 + half_leg),
                "an arc {deviation} from its corner, meeting its legs {tangent} from it, \
                 under both bounds"
            );
        }
    }

    /// The joint path through `waypoints`, and the path through them rounded
    /// within its deviation that rests at equal waypoints alone.
    fn rounded_within_the_joint_deviation(waypoints: &[Vec<f64>]) -> [RoundedPath; 2] {
        [
            RoundedPath::joint_path(waypoints),
            RoundedPath::through(waypoints, Corners::joint_path()),
        ]
    }

    // On any path rounded within JOINT_PATH_CORNER_DEVIATION, the joint path
    // or the one resting at equal waypoints alone, each arc is the largest one
    // within both bounds.
    #[hegel::test]
    fn each_corner_arc_of_a_joint_path_is_the_largest_within_both_bounds(tc: TestCase) {
        let waypoints = waypoints(&tc);
        for path in rounded_within_the_joint_deviation(&waypoints) {
            assert_each_arc_is_the_largest_within_both_bounds(&path, &waypoints);
        }
    }

    // A path that comes back through its own waypoints, around a square of
    // thousandths and on along its first side, pairs each arc with the corner
    // it rounds at the later visit of the point.
    #[test]
    fn a_path_back_through_its_own_waypoints_rounds_each_corner_within_both_bounds() {
        let steps = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [1, 0, 0, 0],
        ];
        let waypoints: Vec<Vec<f64>> = std::iter::once(vec![0.0; JOINTS])
            .chain(steps.into_iter().scan(vec![0.0; JOINTS], |q, step| {
                *q = sum(q, &to_f64(step));
                Some(q.clone())
            }))
            .collect();
        for path in rounded_within_the_joint_deviation(&waypoints) {
            assert_each_arc_is_the_largest_within_both_bounds(&path, &waypoints);
        }
    }

    // RoundedPath::joint_path rests at a waypoint within MIN_JOINT_CHANGE_RAD
    // of the one before it, which the path through the same waypoints at the
    // joint path's deviation rounds; JointTrajectory plays the joint path.
    #[test]
    fn the_joint_path_rests_at_a_near_repeat_that_the_rounding_alone_rounds() {
        let waypoints = [
            vec![0.0, 1.0],
            vec![0.0, 0.0],
            vec![0.0005, 0.0],
            vec![0.0005, 1.0],
        ];
        let near = [0.0005, 0.0];
        let [joint_path, through] = rounded_within_the_joint_deviation(&waypoints);
        assert!(among(&near, &rests(&joint_path)));
        assert!(!among(&near, &rests(&through)));
        let traj = JointTrajectory::through(
            &[[0.0, 1.0], [0.0, 0.0], near, [0.0005, 1.0]],
            &[1.0; 2],
            &[1.0; 2],
            Duration::ZERO,
            Instant::now(),
        );
        assert_eq!(traj.path(), &joint_path);
    }

    // These joint paths take these durations (s), held as the timing's
    // reference: a straight run, corners turning by 90 and by 170 degrees, and
    // a path through a near repeat and a reversal.
    #[test]
    fn a_joint_path_takes_its_reference_durations() {
        let turning_by = |degrees: f64| {
            let (sin, cos) = degrees.to_radians().sin_cos();
            vec![
                vec![0.0; JOINTS],
                vec![1.0, 0.0, 0.0, 0.0],
                vec![1.0 + cos, sin, 0.0, 0.0],
            ]
        };
        let cases = [
            (
                vec![vec![0.0; JOINTS], vec![0.5, -0.3, 0.2, 0.1]],
                [1.0, 2.0, 0.5, 1.0],
                [2.0, 1.0, 3.0, 1.0],
                1.095445115010333,
            ),
            (
                turning_by(90.0),
                [1.0; JOINTS],
                [1.0; JOINTS],
                3.520895894988214,
            ),
            (
                turning_by(170.0),
                [1.0; JOINTS],
                [1.0; JOINTS],
                3.8627311178851618,
            ),
            (
                turning_by(170.0),
                [1.0; JOINTS],
                [5.0; JOINTS],
                2.2186623550155398,
            ),
            (
                vec![
                    vec![0.0; JOINTS],
                    vec![0.4, 0.1, 0.0, 0.0],
                    vec![0.4, 0.5, 0.2, 0.0],
                    vec![0.4005, 0.5003, 0.2, 0.0],
                    vec![0.1, 0.2, 0.3, -0.2],
                    vec![0.3, 0.1, 0.4, -0.3],
                    vec![0.1, 0.2, 0.3, -0.2],
                ],
                [1.0, 1.5, 0.8, 2.0],
                [3.0, 2.0, 4.0, 1.5],
                2.8865787714059836,
            ),
        ];
        for (waypoints, max_velocity, max_acceleration, recorded) in cases {
            let duration = TimedPath::along(
                RoundedPath::joint_path(&waypoints),
                &max_velocity,
                &max_acceleration,
            )
            .duration_s();
            assert!(
                (duration - recorded).abs() <= 1e-12 * recorded,
                "{duration} s, recorded {recorded} s"
            );
        }
    }

    // REVERSAL_COS against TOTG's turning-back tolerance, from both sides: a
    // corner turning by 179.735 degrees is rounded, as TOTG blends it, and one
    // turning by 179.75 keeps its point and the path stops there, where TOTG
    // refuses the path. A corner whose cosine is exactly REVERSAL_COS stops
    // too, and one whose cosine lies the least float above it is rounded.
    #[test]
    fn the_reversal_cosine_is_totgs_turning_back_tolerance() {
        let path = |waypoints: Vec<Vec<f64>>| RoundedPath::joint_path(&waypoints);
        let turning_by = |degrees: f64| {
            let (sin, cos) = degrees.to_radians().sin_cos();
            path(turning(cos, sin))
        };
        assert!(rounds_its_corner(&turning_by(179.735)));
        assert!(keeps_its_corner(&turning_by(179.75), true));
        // From the origin back along (cos, sin) after a unit run along the first
        // axis, where cos^2 + sin^2 rounds to 1, so the turn's cosine is cos.
        let back = |cos: f64, sin: f64| {
            assert_eq!(cos * cos + sin * sin, 1.0);
            path(vec![
                vec![-1.0, 0.0, 0.0, 0.0],
                vec![0.0; JOINTS],
                vec![cos, sin, 0.0, 0.0],
            ])
        };
        assert!(keeps_its_corner(
            &back(REVERSAL_COS, 0.004472124774634615),
            true
        ));
        assert!(rounds_its_corner(&back(
            REVERSAL_COS.next_up(),
            0.00447212477465944
        )));
    }
}
