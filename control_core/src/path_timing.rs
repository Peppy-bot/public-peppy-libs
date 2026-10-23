//! Time-optimal timing of a path through waypoints under per-component
//! velocity and acceleration limits, and the trajectories that play it. The
//! path follows the model of Kunz and Stilman ("Time-Optimal Trajectory
//! Generation for Path Following with Bounded Acceleration and Velocity", RSS
//! 2012): straight between its waypoints, rounding each corner on a circular
//! arc. Its speed profile is integrated over a grid of cells along the path: a
//! backward pass from rest at the end finds the fastest each cell boundary can
//! be passed at and still stop by the end, and a forward pass from rest at the
//! start then runs each cell as hard as the limits allow within that. The path
//! acceleration is constant over a cell, and a cell's bounds take each
//! component's share of the tangent and its curvature at their largest over the
//! cell, so the limits hold at every point of it; on an arc the two shares are
//! summed, so the bound there is tighter than the limits, and the timing is the
//! fastest under that bound.
//!
//! A waypoint's components are whatever the limits are stated for: a joint
//! configuration, or one extended by an end effector's progress, so a cap on
//! the end effector times the path alongside the joints' limits.

use std::f64::consts::PI;
use std::time::{Duration, Instant};

use crate::timing::{MotionClock, duration_at_least};

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

/// The pieces of the path through `waypoints`, no two consecutive ones equal:
/// a straight run between each pair, and at each corner an arc tangent to both
/// runs, meeting each at half the shorter of the two, so the arcs of
/// neighboring corners never overlap and the path stays within the runs it
/// rounds. A corner the path reverses at keeps its point, where the path stops.
fn pieces(waypoints: &[&[f64]]) -> Vec<Piece> {
    let mut pieces = Vec::new();
    let mut from = waypoints[0].to_vec();
    for k in 1..waypoints.len() {
        let to = waypoints[k];
        let direction_in = difference(to, &from);
        let length_in = norm(&direction_in);
        let direction_in = scaled(&direction_in, 1.0 / length_in);
        let Some(next) = waypoints.get(k + 1) else {
            pieces.push(Piece::Straight {
                start: from,
                direction: direction_in,
                length: length_in,
            });
            break;
        };
        let direction_out = difference(next, to);
        let length_out = norm(&direction_out);
        let direction_out = scaled(&direction_out, 1.0 / length_out);
        let cos_turn = dot(&direction_in, &direction_out);
        // The part of the outgoing direction across the incoming one: its
        // length is the sine of the turn, and it points toward the center.
        let across = difference(&direction_out, &scaled(&direction_in, cos_turn));
        let sin_turn = norm(&across);
        // Collinear runs meet at a point: straight on, the path runs through
        // it; reversed, it stops there.
        if sin_turn == 0.0 {
            pieces.push(Piece::Straight {
                start: from,
                direction: direction_in,
                length: length_in,
            });
            from = to.to_vec();
            continue;
        }
        let turn = sin_turn.atan2(cos_turn);
        let tangent_length = length_in.min(length_out) / 2.0;
        let radius = tangent_length / (turn / 2.0).tan();
        pieces.push(Piece::Straight {
            start: from,
            direction: direction_in.clone(),
            length: length_in - tangent_length,
        });
        pieces.push(Piece::Arc {
            start: sum(to, &scaled(&direction_in, -tangent_length)),
            tangent: direction_in,
            normal: scaled(&across, 1.0 / sin_turn),
            radius,
            turn,
        });
        from = sum(to, &scaled(&direction_out, tangent_length));
    }
    pieces.retain(|piece| piece.length() > 0.0);
    pieces
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

/// A piece of the path and the path position it starts at, from 0.
#[derive(Debug, Clone, PartialEq)]
struct Placed {
    from: f64,
    piece: Piece,
}

/// One cell of the grid as the path plays it: where it starts along the path
/// and in time, its length, and the path speed at each of its ends, 0 at the
/// ends of the path.
#[derive(Debug, Clone, PartialEq)]
struct TimedCell {
    from: f64,
    length: f64,
    time: f64,
    speed_in: f64,
    speed_out: f64,
}

/// A path through waypoints, timed from rest to rest under per-component
/// velocity and acceleration limits.
#[derive(Debug, Clone, PartialEq)]
pub struct TimedPath {
    pieces: Vec<Placed>,
    cells: Vec<TimedCell>,
    duration: f64,
    end: Vec<f64>,
}

impl TimedPath {
    /// The path through `waypoints` (one or more, of the limits' dimension; a
    /// waypoint at no distance from the one before it is the same point),
    /// timed under `max_velocity` and `max_acceleration`, every component of
    /// which is positive, infinite for a component the limit leaves free, and
    /// which together bound the path's speed.
    pub fn new(waypoints: &[Vec<f64>], max_velocity: &[f64], max_acceleration: &[f64]) -> Self {
        assert!(!waypoints.is_empty(), "a path has a waypoint");
        let dimension = max_velocity.len();
        assert!(
            max_acceleration.len() == dimension && waypoints.iter().all(|w| w.len() == dimension),
            "the waypoints and both limits have one dimension"
        );
        assert!(
            max_velocity
                .iter()
                .chain(max_acceleration)
                .all(|&l| l > 0.0),
            "limits are positive"
        );
        let distinct: Vec<&[f64]> = waypoints.iter().map(Vec::as_slice).fold(
            Vec::new(),
            |mut distinct: Vec<&[f64]>, waypoint| {
                if distinct
                    .last()
                    .is_none_or(|last| norm(&difference(waypoint, last)) > 0.0)
                {
                    distinct.push(waypoint);
                }
                distinct
            },
        );
        let pieces: Vec<Placed> = pieces(&distinct)
            .into_iter()
            .scan(0.0, |from, piece| {
                let placed = Placed { from: *from, piece };
                *from += placed.piece.length();
                Some(placed)
            })
            .collect();
        let length = pieces
            .last()
            .map_or(0.0, |placed| placed.from + placed.piece.length());
        let cell_length = length / CELLS_PER_PATH as f64;
        let mut cells: Vec<Cell> = Vec::new();
        // Whether the path rests at each cell boundary: at its ends and at a
        // corner it reverses at.
        let mut rests = vec![true];
        for (p, placed) in pieces.iter().enumerate() {
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
            let reverses = match pieces.get(p + 1) {
                Some(next) => dot(&piece.at(piece.length()).1, &next.piece.at(0.0).1) < 0.0,
                None => true,
            };
            *rests.last_mut().expect("the piece has cells") = reverses;
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
                    from: positions[k],
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
            pieces,
            cells,
            duration: time,
            end: distinct[distinct.len() - 1].to_vec(),
        }
    }

    /// The number of components of a waypoint.
    pub fn dimension(&self) -> usize {
        self.end.len()
    }

    /// The whole path's time, from rest to rest.
    pub fn duration_s(&self) -> f64 {
        self.duration
    }

    /// The configuration at `fraction` of the duration: the first waypoint at
    /// or before 0, the last at or after 1, and between them the path at the
    /// position the speed profile has reached by then.
    pub fn at_fraction(&self, fraction: f64) -> Vec<f64> {
        if fraction >= 1.0 || self.pieces.is_empty() {
            return self.end.clone();
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
        self.at_position(cell.from + distance)
    }

    /// The path's length in its own measure, the norm of its components.
    fn length(&self) -> f64 {
        self.pieces
            .last()
            .map_or(0.0, |placed| placed.from + placed.piece.length())
    }

    /// The time (s) the path reaches position `s`: the cell holding `s`,
    /// entered at its time and speed, run at its constant acceleration for
    /// the distance to `s`.
    fn time_at_position(&self, s: f64) -> f64 {
        let k = self
            .cells
            .partition_point(|cell| cell.from <= s)
            .clamp(1, self.cells.len())
            - 1;
        let cell = &self.cells[k];
        let distance = (s - cell.from).clamp(0.0, cell.length);
        if distance == 0.0 {
            return cell.time;
        }
        if cell.speed_in == cell.speed_out {
            return cell.time + distance / cell.speed_in;
        }
        let acceleration =
            (cell.speed_out * cell.speed_out - cell.speed_in * cell.speed_in) / (2.0 * cell.length);
        let speed_at = (cell.speed_in * cell.speed_in + 2.0 * acceleration * distance)
            .max(0.0)
            .sqrt();
        cell.time + (speed_at - cell.speed_in) / acceleration
    }

    /// The path's configurations every `step` of its length from its start,
    /// and at its end, each with the fraction of the path's duration it is
    /// reached at: the probes a check that walks the path before it plays
    /// reads. No component changes by more than `step` between neighbours,
    /// since a component's change is at most the path's.
    pub fn probes_every(&self, step: f64) -> Vec<(f64, Vec<f64>)> {
        assert!(step > 0.0, "a probe step is a positive length");
        if self.pieces.is_empty() {
            return vec![(0.0, self.end.clone())];
        }
        let length = self.length();
        (0..)
            .map(|k| k as f64 * step)
            .take_while(|&s| s < length)
            .map(|s| {
                (
                    self.time_at_position(s) / self.duration,
                    self.at_position(s),
                )
            })
            .chain(std::iter::once((1.0, self.end.clone())))
            .collect()
    }

    /// The configuration at path position `s`.
    fn at_position(&self, s: f64) -> Vec<f64> {
        let p = self
            .pieces
            .partition_point(|placed| placed.from <= s)
            .clamp(1, self.pieces.len())
            - 1;
        let Placed { from, piece } = &self.pieces[p];
        piece.at((s - from).clamp(0.0, piece.length())).0
    }
}

/// A [`TimedPath`] played over a duration no shorter than its own: at time
/// `now`, the path's configuration at the elapsed fraction of that duration.
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

    pub fn sample(&self, now: Instant) -> Vec<f64> {
        self.path.at_fraction(self.clock.fraction(now))
    }

    /// The path's probes every `step` of its length (see
    /// [`TimedPath::probes_every`]), each with the instant this trajectory
    /// plays it.
    pub fn probes_every(&self, step: f64) -> Vec<(Instant, Vec<f64>)> {
        self.path
            .probes_every(step)
            .into_iter()
            .map(|(fraction, q)| {
                (
                    self.clock.motion_start() + self.clock.duration().mul_f64(fraction),
                    q,
                )
            })
            .collect()
    }
}

/// A straight joint-space move from `start` to `end`: the trapezoid under the
/// joints' velocity and acceleration limits, to the grid's part in a thousand,
/// played over the request when that is longer.
#[derive(Debug, Clone, PartialEq)]
pub struct JointTrajectory<const N: usize> {
    timed: TimedTrajectory,
}

impl<const N: usize> JointTrajectory<N> {
    /// The move from `start` to `end`, starting at `now`, over the longer of
    /// `requested` and the path's own time.
    pub fn new(
        start: [f64; N],
        end: [f64; N],
        velocity_limits: &[f64; N],
        acceleration_limits: &[f64; N],
        requested: Duration,
        now: Instant,
    ) -> Self {
        let path = TimedPath::new(
            &[start.to_vec(), end.to_vec()],
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

    /// The configuration at `now`: `start` before the move, `end` exactly once
    /// it is complete, and the path between them in between.
    pub fn sample(&self, now: Instant) -> [f64; N] {
        joints(self.timed.sample(now))
    }

    /// The move's probes every `step` of its joint-space length, each with
    /// the instant the move plays it (see [`TimedPath::probes_every`]).
    pub fn probes_every(&self, step: f64) -> Vec<(Instant, [f64; N])> {
        self.timed
            .probes_every(step)
            .into_iter()
            .map(|(at, q)| (at, joints(q)))
            .collect()
    }
}

/// A joint path's sample as the configuration it is.
fn joints<const N: usize>(sample: Vec<f64>) -> [f64; N] {
    sample
        .try_into()
        .expect("a joint path has a configuration's components")
}

#[cfg(test)]
mod tests {
    use super::*;
    use hegel::TestCase;
    use hegel::generators as gs;

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

    // Probed every `step` of its length, a path gives its start first, at
    // the start of its time, and its end last, at the end of its time, with
    // the probes in time order, no component changing by more than `step`
    // between neighbours, and each probe where the path stands at the time
    // it names.
    #[hegel::test]
    fn probes_walk_the_path_at_the_step_from_its_start_to_its_end(tc: TestCase) {
        let waypoints = waypoints(&tc);
        let (max_velocity, max_acceleration) = limits(&tc);
        let step: f64 = tc.draw(gs::floats().min_value(0.01).max_value(1.0));
        let path = TimedPath::new(&waypoints, &max_velocity, &max_acceleration);
        let probes = path.probes_every(step);
        assert_eq!(probes[0], (0.0, waypoints[0].clone()));
        let (last_fraction, last) = &probes[probes.len() - 1];
        assert_eq!(last, &waypoints[waypoints.len() - 1]);
        if waypoints.len() == 1 {
            assert_eq!(probes.len(), 1, "a path of no length is one probe");
            return;
        }
        assert!((last_fraction - 1.0).abs() <= 1e-9, "{last_fraction}");
        for pair in probes.windows(2) {
            let ((f0, q0), (f1, q1)) = (&pair[0], &pair[1]);
            assert!(f1 >= f0, "{f0} then {f1}");
            for (a, b) in q0.iter().zip(q1) {
                assert!(
                    (b - a).abs() <= step * SLACK,
                    "{a} to {b} over a step of {step}"
                );
            }
        }
        for (fraction, q) in &probes {
            let at = path.at_fraction(*fraction);
            for (a, b) in at.iter().zip(q) {
                assert!(
                    (a - b).abs() <= 1e-6,
                    "{q:?} probed where the path stands at {at:?}"
                );
            }
        }
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
        let traj = JointTrajectory::new(
            start,
            end,
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
            assert!(!clock.is_complete(before_end));
        }
        assert_eq!(traj.sample(at_end), end);
        assert_eq!(traj.sample(past_end), end);
        assert!(clock.is_complete(at_end) && clock.is_complete(past_end));
    }
}
