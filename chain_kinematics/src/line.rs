//! The straight line between two poses, the interpolation a Cartesian move
//! walks, and the walk along a line by inverse kinematics from a seed.

use std::slice;

use nalgebra::Isometry3;

/// Below this sine of the half-angle between two orientations, a turn of
/// 2e-6 rad, they are one orientation, and the arc between them is whatever
/// the last few bits of the quaternion happened to be. Slerp divides by that
/// sine, so the goal's orientation is taken instead: four orders below a line's
/// orientation precision.
pub const COINCIDENT_ARC_EPS: f64 = 1e-6;

/// Pose on the straight line between two poses: position lerped, orientation
/// slerped along the shorter arc, the goal's where the two orientations are
/// one (see [`COINCIDENT_ARC_EPS`]), so a held orientation is held exactly
/// along the whole line.
pub fn interpolate(a: &Isometry3<f64>, b: &Isometry3<f64>, s: f64) -> Isometry3<f64> {
    let s = s.clamp(0.0, 1.0);
    Isometry3::from_parts(
        (a.translation.vector + s * (b.translation.vector - a.translation.vector)).into(),
        a.rotation
            .try_slerp(&b.rotation, s, COINCIDENT_ARC_EPS)
            .unwrap_or(b.rotation),
    )
}

/// Where a walk along a line stands: the share of the line it reached, and the
/// joints there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineState<const N: usize> {
    pub blend: f64,
    pub q: [f64; N],
}

/// What walks a line: inverse kinematics that continues from the state before,
/// and the pose of the frame it walks.
pub trait LineSolver<const N: usize> {
    /// Joints that put the walked frame at `pose`, the line's pose at blend
    /// `to`, continuing from `from`; `None` when no solution continues it.
    fn solve(&self, pose: &Isometry3<f64>, from: &LineState<N>, to: f64) -> Option<[f64; N]>;

    /// The walked frame's pose at `q`, in the frame of the line's poses.
    fn pose(&self, q: &[f64; N]) -> Isometry3<f64>;
}

/// The longest step of a line (m): the step of RViz's MotionPlanning panel
/// (motion_planning_frame_planning.cpp:141), which a caller of compute_cartesian_path
/// names.
pub const CARTESIAN_STEP_M: f64 = 0.01;
/// The largest turn of a step of a line (rad): `MaxEEFStep`'s 3.5 rad for each meter of
/// its step (cartesian_interpolator.hpp:97).
pub const CARTESIAN_STEP_RAD: f64 = 3.5 * CARTESIAN_STEP_M;
/// How far the frame stands from its line at most (m): `CartesianPrecision`'s
/// `translational` (cartesian_interpolator.hpp:50).
pub const PRECISION_M: f64 = 1e-3;
/// How far the frame stands turned from its line at most (rad): `CartesianPrecision`'s
/// `rotational` (cartesian_interpolator.hpp:51).
pub const PRECISION_RAD: f64 = 0.01;
/// The narrowest interval of a line a walk halves, as a share of the line:
/// `CartesianPrecision`'s `max_resolution` (cartesian_interpolator.hpp:52).
pub const MAX_RESOLUTION: f64 = 1e-5;

/// How far the line from `from` to `to` moves over [`CARTESIAN_STEP_M`], and how far it
/// turns over [`CARTESIAN_STEP_RAD`], each.
pub fn line_extents(from: &Isometry3<f64>, to: &Isometry3<f64>) -> (f64, f64) {
    let length = (to.translation.vector - from.translation.vector).norm();
    let turn = from.rotation.angle_to(&to.rotation);
    (length / CARTESIAN_STEP_M, turn / CARTESIAN_STEP_RAD)
}

/// The share of the line from `from` to `to` at each step: as many steps as
/// `computeCartesianPath` takes, the more of its extents, each rounded down, plus one
/// (cartesian_interpolator.cpp:237-245).
pub fn line_blends(
    from: &Isometry3<f64>,
    to: &Isometry3<f64>,
) -> impl Iterator<Item = f64> + use<> {
    let (moved, turned) = line_extents(from, to);
    let steps = (moved.floor().max(turned.floor()) as usize).saturating_add(1);
    (1..=steps).map(move |k| k as f64 / steps as f64)
}

/// How closely a walk keeps its frame on its line between two states, MoveIt's
/// CartesianPrecision: the frame at the joints' midpoint stands within
/// `translational_m` of the line's midpoint and turned within
/// `rotational_rad` of it, or the interval is halved and its midpoint solved,
/// down to intervals `max_resolution` of the line wide.
pub struct LinePrecision {
    translational_m: f64,
    rotational_rad: f64,
    max_resolution: f64,
}

/// A value that is not a [`LinePrecision`]'s: zero, negative or not finite.
#[derive(Debug, thiserror::Error)]
pub enum PrecisionError {
    #[error("translational precision {0} m is not a finite positive distance")]
    Translational(f64),
    #[error("rotational precision {0} rad is not a finite positive angle")]
    Rotational(f64),
    #[error("precision resolution {0} is not a finite positive share of a line")]
    Resolution(f64),
}

/// True if `value` is finite and above zero.
const fn finite_positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

impl LinePrecision {
    /// The precision, or a refusal of the first value that is not finite and positive.
    pub const fn new(
        translational_m: f64,
        rotational_rad: f64,
        max_resolution: f64,
    ) -> Result<Self, PrecisionError> {
        if !finite_positive(translational_m) {
            return Err(PrecisionError::Translational(translational_m));
        }
        if !finite_positive(rotational_rad) {
            return Err(PrecisionError::Rotational(rotational_rad));
        }
        if !finite_positive(max_resolution) {
            return Err(PrecisionError::Resolution(max_resolution));
        }
        Ok(Self {
            translational_m,
            rotational_rad,
            max_resolution,
        })
    }

    /// True when `actual` stands within the precision of `target`.
    pub fn holds(&self, actual: &Isometry3<f64>, target: &Isometry3<f64>) -> bool {
        (actual.translation.vector - target.translation.vector).norm() <= self.translational_m
            && actual.rotation.angle_to(&target.rotation) <= self.rotational_rad
    }
}

/// A precision of these positive values, halving down to [`MAX_RESOLUTION`].
pub const fn line_precision(translational_m: f64, rotational_rad: f64) -> LinePrecision {
    match LinePrecision::new(translational_m, rotational_rad, MAX_RESOLUTION) {
        Ok(precision) => precision,
        Err(_) => panic!("a precision's values are finite and positive"),
    }
}

/// The precision each interval of a line keeps at its joints' midpoint.
pub const PRECISION: LinePrecision = line_precision(PRECISION_M, PRECISION_RAD);

/// Why a walk along a line ended, and at which blend's step.
#[derive(Debug, PartialEq)]
pub enum WalkEnd {
    /// Every step was taken.
    Whole,
    /// The pose at `blend` has no solution continuing from the state before.
    NoSolution { blend: f64 },
    /// The step to `blend` cannot keep the precision.
    Imprecise { blend: f64 },
    /// The caller refused the step to `blend`.
    Refused { blend: f64 },
}

/// Walks the line `line` traces from `from` through each blend of `blends`
/// in turn, solving each by `solver` from the state before; a blend equal to
/// the one before solves the same pose again.
///
/// With a `precision`, each step holds it at its joint midpoint: the frame
/// there stands within the precision of the line's midpoint, or the step is
/// halved, its midpoint solved from the joint midpoint and each half refined
/// in turn, down to the precision's resolution (MoveIt's
/// validateAndImproveInterval). Without one, a step is its own state alone and
/// the walk allocates nothing.
///
/// `take` gets the state before each step and the step's states in order, its
/// own blend last, and keeps them or refuses the step. The walk ends at the
/// first step whose pose has no solution, whose interval cannot be made
/// precise, or that `take` refuses; `take` has kept every step before it.
pub fn walk_line<const N: usize, S: LineSolver<N> + ?Sized>(
    solver: &S,
    line: impl Fn(f64) -> Isometry3<f64>,
    from: LineState<N>,
    blends: impl IntoIterator<Item = f64>,
    precision: Option<&LinePrecision>,
    mut take: impl FnMut(&LineState<N>, &[LineState<N>]) -> bool,
) -> WalkEnd {
    let mut step = Vec::new();
    let walked = blends.into_iter().try_fold(from, |here, blend| {
        let q = solver
            .solve(&line(blend), &here, blend)
            .ok_or(WalkEnd::NoSolution { blend })?;
        let there = LineState { blend, q };
        let kept = match precision {
            None => take(&here, slice::from_ref(&there)),
            Some(precision) => {
                step.clear();
                if !refine(solver, &line, precision, &here, &there, &mut step) {
                    return Err(WalkEnd::Imprecise { blend });
                }
                take(&here, &step)
            }
        };
        if !kept {
            return Err(WalkEnd::Refused { blend });
        }
        Ok(there)
    });
    match walked {
        Ok(_) => WalkEnd::Whole,
        Err(end) => end,
    }
}

/// Pushes onto `states` the states that keep the frame within `precision` of
/// the line from `a` to `b`, `b` last; false when an interval narrower than
/// the resolution misses the line at its joint midpoint, or a midpoint has no
/// solution.
fn refine<const N: usize, S: LineSolver<N> + ?Sized>(
    solver: &S,
    line: &impl Fn(f64) -> Isometry3<f64>,
    precision: &LinePrecision,
    a: &LineState<N>,
    b: &LineState<N>,
    states: &mut Vec<LineState<N>>,
) -> bool {
    let midway = LineState {
        blend: 0.5 * (a.blend + b.blend),
        q: std::array::from_fn(|i| a.q[i] + (b.q[i] - a.q[i]) * 0.5),
    };
    let pose = line(midway.blend);
    if precision.holds(&solver.pose(&midway.q), &pose) {
        states.push(*b);
        return true;
    }
    if (b.blend - a.blend).abs() < precision.max_resolution {
        return false;
    }
    let Some(q) = solver.solve(&pose, &midway, midway.blend) else {
        return false;
    };
    let mid = LineState {
        blend: midway.blend,
        q,
    };
    refine(solver, line, precision, a, &mid, states)
        && refine(solver, line, precision, &mid, b, states)
}
