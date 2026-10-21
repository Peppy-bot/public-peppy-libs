//! Closed-form arm-angle inverse kinematics for a clean SRS (spherical-revolute-
//! spherical) 7-DOF arm.
//!
//! The geometry drives everything. The shoulder (joints 1-2-3) is concurrent at
//! `S`, the wrist (joints 5-6-7) at `W`, joint 4 is the elbow between them, and
//! the EE origin coincides with `W` (enforced in [`crate::model::ArmModel`]). A
//! 6-DOF target leaves one redundant DOF: with `S`, `W`, and the elbow flex
//! fixed, the elbow is still free to swing on a circle about the `S`-`W` line.
//! The angle around that circle is the **arm angle** `psi`.
//!
//! So a solve is just geometry:
//!   1. the wrist center is the target position (EE == `W`);
//!   2. the elbow flex `theta4` follows from the `S`-`W` distance (law of cosines);
//!   3. `psi` places the elbow on the circle, which fixes the upper-arm pose and so
//!      the shoulder rotation `R_s`, leaving a residual wrist rotation `R_w`;
//!   4. `R_s` decomposes into `theta1..3` and `R_w` into `theta5..7`.
//!
//! Step 4, splitting a rotation into three fixed-axis rotations, is the only
//! non-obvious part. It uses the **Paden-Kahan subproblems**
//! ([`subproblem1`]/[`subproblem2`]): standard, closed-form geometric primitives
//! for "what angle about this axis carries p onto q". They take the joint axes
//! read straight from the FK-validated [`crate::model::ArmModel`], so signs and
//! offsets are correct by construction (no DH transcription), the two discrete
//! branches fall out of the algebra, and the same code handles the mirror arm and
//! any other SRS URDF unchanged.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use nalgebra::{Isometry3, Matrix3, Rotation3, Unit, Vector3, Vector6};

use chain_kinematics::manipulability;

use crate::model::ArmModel;
use crate::{ARM_DOF, ELBOW, Jacobian, JointVec, Limit, PARALLEL_SIN_EPS, SHOULDER, WRIST};

/// How to resolve the redundant arm angle when the caller doesn't pin it.
#[derive(Debug, Clone, Copy)]
pub enum ArmAnglePolicy {
    /// Use the arm angle of the seed configuration (continuity; servoing).
    FromSeed,
    /// Use this exact arm angle (radians). Infeasible if it violates a limit.
    Fixed(f64),
    /// Start from the seed's arm angle (like `FromSeed`) and step at most
    /// `max_step_rad` toward higher manipulability, so a solve chain steers the
    /// elbow away from singular postures instead of freezing it. The bounded step
    /// keeps consecutive solutions continuous; a chain of solves follows the local
    /// manipulability ridge as the target moves. `max_step_rad = 0` reduces to
    /// `FromSeed`. Solvable exactly when `FromSeed` is.
    MaxManipulability { max_step_rad: f64 },
}

/// One inverse-kinematics solution.
#[derive(Debug, Clone)]
pub struct Solution {
    pub q: JointVec,
    /// The arm angle the solution was built at.
    pub arm_angle: f64,
}

// ---------------------------------------------------------------------------
// Rotation / screw primitives, plus an independent forward map built by
// composing the per-joint screws. It must agree with the k-chain FK (see tests),
// which is what validates the axes/points the IK trusts.
// ---------------------------------------------------------------------------

fn exp_so3(axis: Vector3<f64>, angle: f64) -> Rotation3<f64> {
    Rotation3::from_axis_angle(&Unit::new_normalize(axis), angle)
}

/// Pure rotation by `angle` about the line through `point` with direction
/// `axis`, as a homogeneous point map `p -> R(p - point) + point`.
fn screw_point(
    axis: Vector3<f64>,
    point: Vector3<f64>,
    angle: f64,
    p: Vector3<f64>,
) -> Vector3<f64> {
    let r = exp_so3(axis, angle);
    r * (p - point) + point
}

/// Forward kinematics from the PoE screw data: EE position in the base frame.
/// Independent path from the `k`-chain FK; the two must agree (see tests),
/// which validates the screw axes/points the IK relies on.
fn fk_poe_position(model: &ArmModel, q: &JointVec) -> Vector3<f64> {
    let mut p = model.home_tip.translation.vector;
    for i in (0..ARM_DOF).rev() {
        p = screw_point(model.axes[i], model.points[i], q[i], p);
    }
    p
}

// ---------------------------------------------------------------------------
// Paden-Kahan subproblems (axes through a common point taken as the origin;
// callers pass vectors already relative to that point).
// ---------------------------------------------------------------------------

/// Subproblem 1: angle `theta` with `exp(axis, theta) p = q` (rotation about a
/// line through the origin). `p`, `q` need not be perpendicular to `axis`; only
/// their components in the rotation plane are used.
fn subproblem1(axis: Vector3<f64>, p: Vector3<f64>, q: Vector3<f64>) -> f64 {
    let u = p - axis * axis.dot(&p);
    let v = q - axis * axis.dot(&q);
    f64::atan2(axis.dot(&u.cross(&v)), u.dot(&v))
}

/// Numerically-zero slack for the O(1) algebraic quantities in the closed-form
/// solvers: the subproblem-2 discriminant `gamma`(²) (below it the two branches
/// coincide) and, in [`sincos_roots`], the `a sin + b cos` amplitude and the
/// half-width below which its two roots are one. All are O(1) for this arm's
/// orthogonal axes, so this is a roundoff floor, not a degeneracy angle.
const ALGEBRAIC_EPS: f64 = 1e-9;

/// Minimum `sin(angle)` between the probe axis and `w3` in [`decompose_three_axes`].
/// Deliberately coarser than [`PARALLEL_SIN_EPS`]: we want a *well-conditioned*
/// probe to normalize, not merely a non-degenerate one, and we fall back to a
/// second axis when the first is too aligned.
const PROBE_AXIS_SIN_MIN: f64 = 1e-3;

/// Subproblem 2: solve `exp(w1, t1) exp(w2, t2) p = q` for axes `w1`, `w2`
/// intersecting at the origin. Returns up to two `(t1, t2)` branches.
fn subproblem2(
    w1: Vector3<f64>,
    w2: Vector3<f64>,
    p: Vector3<f64>,
    q: Vector3<f64>,
) -> Vec<(f64, f64)> {
    let w1w2 = w1.dot(&w2);
    let denom = w1w2 * w1w2 - 1.0; // -sin^2(angle between the axes)
    if denom.abs() < PARALLEL_SIN_EPS * PARALLEL_SIN_EPS {
        // Degenerate-input guard: w1 parallel to w2. The shoulder/wrist are always
        // decomposed about their fixed orthogonal home axes, so this never fires
        // for this arm; it protects against a non-SRS chain being fed in.
        return Vec::new();
    }
    let alpha = (w1w2 * w2.dot(&p) - w1.dot(&q)) / denom;
    let beta = (w1w2 * w1.dot(&q) - w2.dot(&p)) / denom;
    let cross = w1.cross(&w2);
    let cross_n2 = cross.norm_squared();
    let gamma_sq =
        (p.norm_squared() - alpha * alpha - beta * beta - 2.0 * alpha * beta * w1w2) / cross_n2;
    if gamma_sq < -ALGEBRAIC_EPS {
        return Vec::new();
    }
    let gamma = gamma_sq.max(0.0).sqrt();
    let branch = |g: f64| {
        let z = alpha * w1 + beta * w2 + g * cross;
        let t2 = subproblem1(w2, p, z);
        let t1 = subproblem1(w1, z, q);
        (t1, t2)
    };
    if gamma < ALGEBRAIC_EPS {
        vec![branch(0.0)]
    } else {
        vec![branch(gamma), branch(-gamma)]
    }
}

/// Decompose a rotation into three fixed-axis rotations:
/// `R = exp(w1, t1) exp(w2, t2) exp(w3, t3)`. Returns up to two branches.
/// Works because `exp(w3, t3)` fixes `w3`, so `R w3 = exp(w1,t1) exp(w2,t2) w3`
/// is a subproblem 2 for `(t1, t2)`; `t3` then follows from subproblem 1.
fn decompose_three_axes(
    r: &Rotation3<f64>,
    w1: Vector3<f64>,
    w2: Vector3<f64>,
    w3: Vector3<f64>,
) -> Vec<(f64, f64, f64)> {
    let d1 = r * w3;
    // pick a probe vector not parallel to w3
    let seed = if w3.cross(&Vector3::x()).norm() > PROBE_AXIS_SIN_MIN {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let v = (seed - w3 * w3.dot(&seed)).normalize();
    subproblem2(w1, w2, w3, d1)
        .into_iter()
        .map(|(t1, t2)| {
            let rc = exp_so3(w2, t2).inverse() * exp_so3(w1, t1).inverse() * r;
            let t3 = subproblem1(w3, v, rc * v);
            (t1, t2, t3)
        })
        .collect()
}

/// Rounding slack on a joint limit (radians): a joint the closed form puts at
/// most this far past a limit is that limit, rounded.
const LIMIT_SLACK: f64 = 1e-9;

/// `branches` of a spherical limb's decomposition, plus, when the limb is at its
/// gimbal lock, each branch's outer joints re-split nearest the seed's.
///
/// At the lock the middle joint turns the last axis onto `±first`, so the two
/// outer joints turn about one line and the limb's rotation `r` fixes only
/// `t1 ± t3`: every split of that sum reaches `r`, and the decomposition
/// returns an arbitrary one. The split nearest `seed` within `limits` keeps a
/// solve continuous with the configuration it starts from.
fn with_lock_splits(
    branches: Vec<(f64, f64, f64)>,
    r: &Rotation3<f64>,
    (first, last): (Vector3<f64>, Vector3<f64>),
    seed: (f64, f64),
    limits: (Limit, Limit),
) -> Vec<(f64, f64, f64)> {
    /// Largest sine between the first axis and the last axis carried through `r`
    /// at which the limb counts as locked. Re-splitting the outer joints there
    /// moves the limb by at most this times a turn (6e-8 rad); past it, rounding
    /// leaves the decomposition's outer joints within 1e-8 rad.
    const LOCK_SIN: f64 = 1e-8;
    let carried = r * last;
    if carried.cross(&first).norm() >= LOCK_SIN {
        return branches;
    }
    let sign = carried.dot(&first).signum();
    let splits: Vec<(f64, f64, f64)> = branches
        .iter()
        .filter_map(|&(t1, t2, t3)| {
            split_nearest(t1 + sign * t3, sign, seed, limits).map(|(u1, u3)| (u1, t2, u3))
        })
        .collect();
    branches.into_iter().chain(splits).collect()
}

/// The split `(t1, t3)` of `t1 + sign * t3 = sum` (mod 2π) nearest the seed's
/// outer joints with both inside `limits`, or `None` if no split fits them. A
/// seed with both joints on limits has the corner as its only split, which
/// rounding can leave [`LIMIT_SLACK`] outside them.
fn split_nearest(
    sum: f64,
    sign: f64,
    (seed_first, seed_last): (f64, f64),
    (first, last): (Limit, Limit),
) -> Option<(f64, f64)> {
    let seed_sum = seed_first + sign * seed_last;
    let nearest_sum = seed_sum + wrap_pi(sum - seed_sum);
    [nearest_sum, nearest_sum - TAU, nearest_sum + TAU]
        .into_iter()
        .filter_map(|sum| {
            // t3 = sign * (sum - t1) inside `last` bounds t1 to an interval.
            let (from_last_lo, from_last_hi) = if sign > 0.0 {
                (sum - last.hi, sum - last.lo)
            } else {
                (sum + last.lo, sum + last.hi)
            };
            let (lo, hi) = (first.lo.max(from_last_lo), first.hi.min(from_last_hi));
            (lo <= hi + LIMIT_SLACK).then(|| {
                let hi = hi.max(lo);
                // Nearest point of the line t1 + sign * t3 = sum to the seed.
                let t1 = (0.5 * (seed_first + sum - sign * seed_last)).clamp(lo, hi);
                (t1, sign * (sum - t1))
            })
        })
        .min_by(|x, y| {
            let cost =
                |&(t1, t3): &(f64, f64)| (t1 - seed_first).powi(2) + (t3 - seed_last).powi(2);
            cost(x).total_cmp(&cost(y))
        })
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/// Float slack on the reach interval so a target sitting essentially *at* the
/// max-reach (straight arm) or min-reach boundary is rejected rather than driven
/// into the ill-conditioned region just inside it. The exact straight-arm pose is
/// also caught downstream by the arm-plane normal check ([`PARALLEL_SIN_EPS`])
/// and, for seeds, by [`SINGULAR_RADIUS`].
const REACH_EPS: f64 = 1e-9;

/// Numerical floor (m) on the elbow-circle radius below which the arm angle is
/// taken as undefined. The arm angle is the *direction* of the elbow off the
/// shoulder-wrist line, so it stays well-defined and numerically stable for any
/// physically non-straight configuration even when that offset is sub-millimetre
/// (a near-straight seed still carries a usable arm angle, e.g. the home pose at
/// q4 ≈ 0.05 has a ~5 mm radius and a clean angle). Only at the exact straight-arm
/// pose (radius → 0, elbow on the line) is the angle genuinely undefined; this
/// floor guards just that degenerate case, where the offset direction is lost to
/// rounding.
const SINGULAR_RADIUS: f64 = 1e-6;

/// Arm angle of configuration `q`: the angle of its elbow on the redundancy
/// circle about the S-W line, measured from the reference direction. `None` only
/// at the exact straight-arm singularity (elbow-circle radius below
/// [`SINGULAR_RADIUS`]), where the elbow sits on the S-W line and the angle is
/// geometrically undefined (returning a fabricated 0.0 there would mislead
/// callers). A merely near-straight `q` still has a well-defined angle.
pub(crate) fn arm_angle_of(model: &ArmModel, q: &JointVec) -> Option<f64> {
    let circle = circle_frame(model, fk_poe_position(model, q));
    circle.azimuth(elbow_position(model, q))
}

/// Elbow center of configuration `q`: joints 1-3 carry the home elbow about S;
/// joint 4+ leave it fixed (it lies on joint 4's axis).
fn elbow_position(model: &ArmModel, q: &JointVec) -> Vector3<f64> {
    let mut e = model.elbow_home;
    for i in (0..3).rev() {
        e = screw_point(model.axes[i], model.points[i], q[i], e);
    }
    e
}

/// The elbow redundancy circle for a wrist center: the elbow rides this circle
/// about the S-W line as the arm angle varies. Depends only on the wrist center,
/// so it is computed once per solve and reused across all arm angles.
struct Circle {
    radius: f64,
    center: Vector3<f64>,
    /// `psi = 0` reference direction (in the circle plane).
    a_hat: Vector3<f64>,
    /// `psi = π/2` direction; `(a_hat, b_hat, n_hat)` is right-handed.
    b_hat: Vector3<f64>,
}

impl Circle {
    fn elbow(&self, psi: f64) -> Vector3<f64> {
        self.center + self.radius * (psi.cos() * self.a_hat + psi.sin() * self.b_hat)
    }

    /// Azimuth of a point about this circle's axis, in the `(a_hat, b_hat)`
    /// basis: the inverse of [`elbow`](Self::elbow) for a point on the circle,
    /// and the projected direction for one off it (a seed's elbow measured on
    /// another configuration's circle). `None` when the point projects onto the
    /// axis itself, where no direction is defined; for a point on the circle
    /// that is exactly the [`SINGULAR_RADIUS`] straight-arm test.
    fn azimuth(&self, p: Vector3<f64>) -> Option<f64> {
        let e = p - self.center;
        let (a, b) = (e.dot(&self.a_hat), e.dot(&self.b_hat));
        (a.hypot(b) >= SINGULAR_RADIUS).then(|| f64::atan2(b, a))
    }
}

fn circle_frame(model: &ArmModel, p_w: Vector3<f64>) -> Circle {
    let s = model.shoulder;
    let n = p_w - s;
    // d is bounded below by the arm's minimum reach |l_su - l_uw| > 0 for any
    // joint configuration, so n_hat is well defined (callers pass a reachable
    // wrist center: either FK of a real q, or a target past solve()'s reach check).
    let d = n.norm();
    let n_hat = n / d;
    let h = (model.l_su * model.l_su - model.l_uw * model.l_uw + d * d) / (2.0 * d);
    let center = s + h * n_hat;
    let radius = (model.l_su * model.l_su - h * h).max(0.0).sqrt();
    // psi=0 reference: project the coordinate axis least aligned with the S-W
    // line onto the circle plane. The least-aligned axis is at most 54.7° off the
    // plane, so a_hat is always well-conditioned (no thin near-vertical band).
    let (ax, ay, az) = (
        n_hat.dot(&Vector3::x()).abs(),
        n_hat.dot(&Vector3::y()).abs(),
        n_hat.dot(&Vector3::z()).abs(),
    );
    let r0 = if ax <= ay && ax <= az {
        Vector3::x()
    } else if ay <= az {
        Vector3::y()
    } else {
        Vector3::z()
    };
    let a_hat = (r0 - n_hat * n_hat.dot(&r0)).normalize();
    let b_hat = n_hat.cross(&a_hat);
    Circle {
        radius,
        center,
        a_hat,
        b_hat,
    }
}

/// Solve inverse kinematics for a `target` EE pose in the arm base frame,
/// resolving redundancy per `arm_angle`. `seed` selects the discrete branch (and
/// the arm angle when `FromSeed`). The target is the same [`Isometry3`] type the
/// forward kinematics emit, so an FK pose feeds straight back in; convert a
/// world-frame target first with [`ArmModel::base_pose`].
///
/// In `FromSeed` mode the seed's arm angle is used when it is feasible
/// (continuity); otherwise the closest feasible arm angle is taken from the
/// exact feasible set, so a solution is still returned when the seed's angle is
/// infeasible for this target, or when the seed is at the exact straight-arm
/// singularity and so carries no arm angle (a merely near-straight seed, e.g. the
/// home pose at q4 ≈ 0.05, still does and is tracked continuously). In `Fixed`
/// mode the given arm angle is used verbatim and infeasibility yields `None`.
pub(crate) fn solve(
    model: &ArmModel,
    limits: &[Limit; ARM_DOF],
    target: &Isometry3<f64>,
    arm_angle: ArmAnglePolicy,
    seed: &JointVec,
) -> Option<Solution> {
    let ctx = PsiSolve::new(model, limits, target)?;
    match arm_angle {
        ArmAnglePolicy::Fixed(psi) => ctx
            .solve_at_psi(psi, seed)
            .map(|q| Solution { q, arm_angle: psi }),
        ArmAnglePolicy::FromSeed => ctx
            .seeded_solution(seed)
            .map(|(psi, q)| Solution { q, arm_angle: psi }),
        ArmAnglePolicy::MaxManipulability { max_step_rad } => {
            // Anchor at the FromSeed angle (continuity, guaranteed feasible), then
            // step within the bounded window toward higher manipulability.
            let (psi0, _) = ctx.seeded_solution(seed)?;
            let (psi, q) = ctx.max_manipulability_step(psi0, max_step_rad, seed)?;
            Some(Solution {
                q,
                arm_angle: wrap_pi(psi),
            })
        }
    }
}

/// Everything about one solve that is fixed once the target passes the reach
/// check: the model, the decomposed target (rotation and wrist center), the elbow
/// flex, and the redundancy circle. The per-psi machinery hangs off it as
/// methods, so each varies only its psi/seed rather than rethreading the target.
struct PsiSolve<'a> {
    model: &'a ArmModel,
    /// The joint limits every returned branch must satisfy, including any control
    /// floor layered over the URDF.
    limits: &'a [Limit; ARM_DOF],
    r_d: Rotation3<f64>,
    p_w: Vector3<f64>,
    theta4: f64,
    circle: Circle,
}

impl<'a> PsiSolve<'a> {
    /// The solve of `target` within `limits`, or `None` when the target is not
    /// finite or its wrist center lies outside the reach, the straight-arm band
    /// included.
    fn new(
        model: &'a ArmModel,
        limits: &'a [Limit; ARM_DOF],
        target: &Isometry3<f64>,
    ) -> Option<Self> {
        let r_d = target.rotation.to_rotation_matrix();
        let p_d = target.translation.vector;
        if !(p_d.x.is_finite() && p_d.y.is_finite() && p_d.z.is_finite()) {
            return None; // reject NaN/Inf position up front
        }
        if !r_d.matrix().iter().all(|x| x.is_finite()) {
            return None; // reject NaN/Inf orientation up front
        }
        let p_w = p_d; // EE origin coincides with the wrist center
        let d = (p_w - model.shoulder).norm();
        if d > model.l_su + model.l_uw - REACH_EPS
            || d < (model.l_su - model.l_uw).abs() + REACH_EPS
        {
            return None; // unreachable / straight-arm singular boundary
        }

        // theta4: elbow flex from reach alone.
        let cos4 = ((d * d - model.l_su * model.l_su - model.l_uw * model.l_uw)
            / (2.0 * model.l_su * model.l_uw))
            .clamp(-1.0, 1.0);

        // The elbow circle depends only on the wrist center, so compute it once
        // and reuse it across all arm angles.
        Some(PsiSolve {
            model,
            limits,
            r_d,
            p_w,
            theta4: cos4.acos(),
            circle: circle_frame(model, p_w),
        })
    }
}

impl PsiSolve<'_> {
    /// The arm angle within `max_step_rad` of `psi0` with the highest
    /// manipulability, and its joint solution. Evaluates the window's endpoints
    /// and center, refines once with a parabolic fit through them, and keeps the
    /// best solvable candidate; `psi0` itself is always a candidate, so this
    /// returns `None` only if `psi0` has no in-limit branch (which the caller's
    /// feasibility selection rules out).
    fn max_manipulability_step(
        &self,
        psi0: f64,
        max_step_rad: f64,
        seed: &JointVec,
    ) -> Option<(f64, JointVec)> {
        // A non-finite or negative step degenerates to FromSeed rather than
        // poisoning the candidate angles.
        let h = if max_step_rad.is_finite() {
            max_step_rad.max(0.0)
        } else {
            0.0
        };
        let eval = |psi: f64| {
            self.solve_at_psi(psi, seed)
                .map(|q| (psi, manipulability(&jacobian_poe(self.model, &q)), q))
        };
        let center = eval(psi0)?;
        let below = eval(psi0 - h);
        let above = eval(psi0 + h);
        // With all three samples, a concave parabola through them estimates the
        // peak; clamped into the window so the step bound holds regardless of fit
        // quality.
        let vertex = match (&below, &above) {
            (Some((_, w_lo, _)), Some((_, w_hi, _))) => {
                let curvature = w_lo - 2.0 * center.1 + w_hi;
                if curvature < 0.0 {
                    let offset = 0.5 * h * (w_lo - w_hi) / curvature;
                    eval(psi0 + offset.clamp(-h, h))
                } else {
                    None
                }
            }
            _ => None,
        };
        // Strict improvement over the center keeps the chain anchored on plateaus
        // (no drift when manipulability is locally flat).
        let best = [below, above, vertex]
            .into_iter()
            .flatten()
            .fold(center, |acc, c| if c.1 > acc.1 { c } else { acc });
        Some((best.0, best.2))
    }
}

/// Geometric Jacobian of configuration `q` from the PoE screw data: column `i` is
/// `[z x (p_ee - p); z]` with joint `i`'s axis `z` and axis point `p` carried
/// through joints `1..i-1`, matching [`Posed::jacobian`]'s layout (rows 0..3
/// linear, 3..6 angular, arm base frame, EE origin reference; the two agree, see
/// tests). Lets the solver score manipulability without the FK chain's `&mut`.
///
/// [`Posed::jacobian`]: crate::fk::Posed::jacobian
fn jacobian_poe(model: &ArmModel, q: &JointVec) -> Jacobian {
    let p_ee = fk_poe_position(model, q);
    // Accumulate the affine map of joints 1..i-1 (p -> r_acc p + t_acc); joint i's
    // current axis and axis point are its home screw carried through it.
    let mut r_acc = Rotation3::identity();
    let mut t_acc = Vector3::zeros();
    let mut cols = [Vector6::zeros(); ARM_DOF];
    for i in 0..ARM_DOF {
        let z = r_acc * model.axes[i];
        let p = r_acc * model.points[i] + t_acc;
        let linear = z.cross(&(p_ee - p));
        cols[i] = Vector6::new(linear.x, linear.y, linear.z, z.x, z.y, z.z);
        let r_i = exp_so3(model.axes[i], q[i]);
        t_acc += r_acc * (model.points[i] - r_i * model.points[i]);
        r_acc *= r_i;
    }
    Jacobian::from_columns(&cols)
}

impl PsiSolve<'_> {
    /// The shoulder rotation `R_s(psi)` (home upper-arm frame to the target one)
    /// and the residual wrist rotation `R_w(psi)` left to invert, for one arm
    /// angle. Both are smooth functions of `psi`:
    /// [`solve_at_psi`](Self::solve_at_psi) decomposes them into joint angles, and
    /// [`feasible_psi_intervals`](Self::feasible_psi_intervals) samples them to
    /// recover the per-joint `psi` dependence in closed form. `None` at the
    /// straight-arm pose, where the arm plane (and so `R_s`) is undefined.
    fn shoulder_wrist_rotations(&self, psi: f64) -> Option<(Rotation3<f64>, Rotation3<f64>)> {
        let PsiSolve {
            model,
            r_d,
            p_w,
            theta4,
            circle,
            ..
        } = self;
        let elbow = circle.elbow(psi);

        // Upper-arm / forearm directions and the arm-plane normal.
        let e_t = (elbow - model.shoulder) / model.l_su;
        let f_t = (*p_w - elbow) / model.l_uw;
        let n_plane = e_t.cross(&f_t); // |.| = sin(angle between upper arm and forearm)
        if n_plane.norm() < PARALLEL_SIN_EPS {
            return None; // straight arm: arm plane undefined
        }
        let n_plane = n_plane.normalize();

        // Shoulder rotation R_s maps the home upper-arm frame to the target one.
        let u0 = (model.elbow_home - model.shoulder) / model.l_su;
        let a4 = model.axes[ELBOW];
        let r_s = frame_map(u0, a4, e_t, n_plane);

        // Rotation through joint 4, then the residual wrist rotation to invert.
        let r_upto4 = r_s * exp_so3(a4, *theta4);
        let r_home = model.home_tip.rotation.to_rotation_matrix();
        let r_w = r_upto4.inverse() * r_d * r_home.inverse();
        Some((r_s, r_w))
    }

    /// The seed's elbow azimuth measured on **this target's** redundancy circle,
    /// so it is directly comparable with
    /// [`feasible_psi_intervals`](Self::feasible_psi_intervals) and with the psi
    /// that [`solve_at_psi`](Self::solve_at_psi) builds from. Measuring it on the
    /// seed's own circle instead compares two angles taken against different
    /// references, and the two circles' frames diverge as the seed's wrist center
    /// moves away from the target's.
    ///
    /// `None` when the seed's elbow projects onto this circle's axis, where there
    /// is no direction to preserve.
    fn preferred_psi(&self, seed: &JointVec) -> Option<f64> {
        self.circle.azimuth(elbow_position(self.model, seed))
    }

    /// The `FromSeed` arm angle and its solution: the seed's own arm angle when a
    /// solution exists there (continuity), else the feasible arm angle nearest
    /// it. The redundancy circle is resolved analytically: every joint angle is
    /// a closed-form function of psi, so each joint limit maps to an exact
    /// feasible-psi interval. At a gimbal lock the limit events of the two outer
    /// joints are read off near-zero matrix entries, so the interval search can
    /// miss a feasible set that is the seed's own arm angle alone.
    fn seeded_solution(&self, seed: &JointVec) -> Option<(f64, JointVec)> {
        let preferred = self.preferred_psi(seed);
        if let Some(found) =
            preferred.and_then(|psi| self.solve_at_psi(psi, seed).map(|q| (psi, q)))
        {
            return Some(found);
        }
        let intervals = self.feasible_psi_intervals(seed);
        let psi = nearest_feasible_psi(&intervals, preferred.unwrap_or(0.0))?;
        self.solve_at_psi(psi, seed).map(|q| (psi, q))
    }

    /// Best in-limit joint solution for a fixed arm angle `psi`, or `None` if no
    /// branch is in limits. Reach and `theta4` are already established by
    /// [`solve`].
    fn solve_at_psi(&self, psi: f64, seed: &JointVec) -> Option<JointVec> {
        let model = self.model;
        let (r_s, r_w) = self.shoulder_wrist_rotations(psi)?;

        let limb = |r: &Rotation3<f64>, [first, middle, last]: [usize; 3]| {
            with_lock_splits(
                decompose_three_axes(r, model.axes[first], model.axes[middle], model.axes[last]),
                r,
                (model.axes[first], model.axes[last]),
                (seed[first], seed[last]),
                (self.limits[first], self.limits[last]),
            )
        };
        let shoulders = limb(&r_s, SHOULDER);
        let wrists = limb(&r_w, WRIST);

        // Keep the in-limits branch nearest the seed. Each joint is normalized
        // into its limit window so the returned value itself respects the
        // declared limits (not merely a 2π-equivalent of it).
        let mut best: Option<(f64, JointVec)> = None;
        for &(t1, t2, t3) in &shoulders {
            for &(t5, t6, t7) in &wrists {
                let Some(q) =
                    normalize_into_limits(self.limits, &[t1, t2, t3, self.theta4, t5, t6, t7])
                else {
                    continue;
                };
                let cost = seed_distance(&q, seed);
                if best.as_ref().is_none_or(|(c, _)| cost < *c) {
                    best = Some((cost, q));
                }
            }
        }
        best.map(|(_, q)| q)
    }
}

// ---------------------------------------------------------------------------
// Analytic feasible arm-angle (psi) intervals.
//
// With the wrist center, theta4 and the discrete branch fixed, the shoulder and
// the wrist are each a spherical joint (three orthonormal intersecting axes)
// whose home rotation is R_s(psi) / R_w(psi). Conjugating that rotation into its
// own axis frame B = [w1 w2 w3] gives a standard XYZ-Euler matrix
//   M(psi) = Bᵀ R(psi) B = Rx(σθ1) Ry(σθ2) Rz(σθ3),   σ = det(B) = ±1,
// so the joint angles read off entries of M: the two "pivot" joints (θ1, θ3)
// from atan2 of two entries, the middle "hinge" joint (θ2) from one. Crucially
// every entry of M(psi) is a pure sinusoid `a sin psi + b cos psi + c` (rotating
// by psi about the fixed S-W axis acts linearly in (sin psi, cos psi)), so a
// joint reaching a limit L is the root set of `a sin psi + b cos psi = c`, in
// closed form. The union of those roots over all six joints and both limits is a
// superset of every feasibility boundary; classifying the arcs between them (one
// feasibility test per arc) yields the exact feasible set. This is the Shimizu /
// Elias-Wen feasible-arm-angle computation cited in the README, specialized to
// the model's PoE axes. See `tests/psi_completeness.rs` for the end-to-end
// guarantee this buys, and
// `ik::tests::the_feasible_intervals_hold_the_arm_angles_that_solve` for the
// check that the analytic set is the set of arm angles a fixed policy solves.

/// Step taken just inside a feasible arc when the nearest feasible arm angle
/// lands on an arc boundary (radians), so the exact-limit endpoint clears the
/// inclusive joint-limit check. Capped at the arc half-width in
/// [`nearest_feasible_psi`], so it always lands strictly inside a feasible arc
/// rather than overshooting a thin one.
const PSI_BOUNDARY_NUDGE: f64 = 1e-6;

/// All `psi in (-pi, pi]` solving `a sin psi + b cos psi = c`, a tangency that
/// misses `c` by at most [`LIMIT_SLACK`] included: a joint whose travel peaks
/// within that slack of a limit is on the limit for the limit check, so the
/// peak is a feasibility boundary however small the travel's amplitude.
fn sincos_roots(a: f64, b: f64, c: f64) -> Vec<f64> {
    // a sin psi + b cos psi = r cos(psi - phi), with r = hypot(a, b).
    let r = a.hypot(b);
    if r < ALGEBRAIC_EPS {
        return Vec::new(); // no psi dependence: this limit yields no boundary
    }
    if c.abs() > r + LIMIT_SLACK {
        return Vec::new(); // limit never reached on this circle
    }
    let delta = (c / r).clamp(-1.0, 1.0).acos();
    let phi = a.atan2(b);
    if delta < ALGEBRAIC_EPS {
        vec![wrap_pi(phi)]
    } else {
        vec![wrap_pi(phi + delta), wrap_pi(phi - delta)]
    }
}

/// Per-entry sinusoid coefficients `(a_sin, b_cos, c_const)` of
/// `M(psi) = Bᵀ R(psi) B`, recovered exactly from three samples (`M` is
/// sinusoidal in `psi`). `basis` carries the three joint axes as columns; the
/// samples are `R` at `psi = 0`, `+pi/2`, `-pi/2`.
fn euler_matrix_coeffs(
    basis: &Matrix3<f64>,
    r0: &Rotation3<f64>,
    r_plus: &Rotation3<f64>,
    r_minus: &Rotation3<f64>,
) -> (Matrix3<f64>, Matrix3<f64>, Matrix3<f64>) {
    let bt = basis.transpose();
    let m = |r: &Rotation3<f64>| bt * r.matrix() * basis;
    let (m0, mp, mn) = (m(r0), m(r_plus), m(r_minus));
    let a_sin = (mp - mn) * 0.5;
    let c_const = (mp + mn) * 0.5;
    let b_cos = m0 - c_const;
    (a_sin, b_cos, c_const)
}

/// Boundary `psi` values where any of a spherical limb's three joints reaches a
/// limit, appended to `events`. A superset of the true feasibility boundaries
/// (spurious extras are harmless: each arc between boundaries is classified by an
/// explicit feasibility test). `sigma = det(basis)` is the limb's handedness.
fn limb_event_angles(
    a_sin: &Matrix3<f64>,
    b_cos: &Matrix3<f64>,
    c_const: &Matrix3<f64>,
    sigma: f64,
    limits: [Limit; 3],
    events: &mut Vec<f64>,
) {
    // A "pivot" joint (θ1 from atan2(-M12, M22); θ3 from atan2(-M01, M00))
    // reaches L where  M[i] sin(σL) + M[j] cos(σL) = 0.
    {
        let mut pivot = |i: (usize, usize), j: (usize, usize), lim: Limit| {
            for l in [lim.lo, lim.hi] {
                let (s, co) = (sigma * l).sin_cos();
                let a = a_sin[i] * s + a_sin[j] * co;
                let b = b_cos[i] * s + b_cos[j] * co;
                let c = -(c_const[i] * s + c_const[j] * co);
                events.extend(sincos_roots(a, b, c));
            }
        };
        pivot((2, 2), (1, 2), limits[0]); // θ1: entries M22, M12
        pivot((0, 0), (0, 1), limits[2]); // θ3: entries M00, M01
    }
    // The "hinge" joint θ2 (sin θ2 = M02) reaches L where M02 = sin(σL).
    for l in [limits[1].lo, limits[1].hi] {
        let target = (sigma * l).sin();
        events.extend(sincos_roots(
            a_sin[(0, 2)],
            b_cos[(0, 2)],
            target - c_const[(0, 2)],
        ));
    }
}

/// The exact set of feasible arm angles for this target, as `(start, end)` arcs
/// in radians (`end` may exceed `pi` for the wrap-around arc, and equals `start`
/// for a single feasible arm angle). Empty if no arm
/// angle is feasible (including the straight-arm pose, where `R_s` is undefined
/// for every `psi`).
impl PsiSolve<'_> {
    fn feasible_psi_intervals(&self, seed: &JointVec) -> Vec<(f64, f64)> {
        let model = self.model;
        // Sample R_s / R_w at psi = 0, ±pi/2 to recover their exact sinusoidal form.
        let sample = |psi: f64| self.shoulder_wrist_rotations(psi);
        let (Some((rs0, rw0)), Some((rsp, rwp)), Some((rsn, rwn))) =
            (sample(0.0), sample(FRAC_PI_2), sample(-FRAC_PI_2))
        else {
            return Vec::new(); // straight arm: feasibility is "none" for every psi
        };

        let bs = Matrix3::from_columns(&SHOULDER.map(|joint| model.axes[joint]));
        let bw = Matrix3::from_columns(&WRIST.map(|joint| model.axes[joint]));
        let (sa_s, cb_s, cc_s) = euler_matrix_coeffs(&bs, &rs0, &rsp, &rsn);
        let (sa_w, cb_w, cc_w) = euler_matrix_coeffs(&bw, &rw0, &rwp, &rwn);

        let mut events = Vec::new();
        let shoulder_limits = SHOULDER.map(|joint| self.limits[joint]);
        let wrist_limits = WRIST.map(|joint| self.limits[joint]);
        limb_event_angles(
            &sa_s,
            &cb_s,
            &cc_s,
            bs.determinant(),
            shoulder_limits,
            &mut events,
        );
        limb_event_angles(
            &sa_w,
            &cb_w,
            &cc_w,
            bw.determinant(),
            wrist_limits,
            &mut events,
        );

        intervals_from_events(events, |psi| self.solve_at_psi(psi, seed).is_some())
    }
}

/// Partition the circle at `events` and keep the arcs whose interior is feasible,
/// plus, as the single-angle arc `(event, event)`, each feasible event between
/// two infeasible arcs: there a joint touches a limit without crossing it, and
/// that arm angle is the only one reaching the target. Events need no
/// de-duplication: coincident ones only yield zero-width arcs, which are
/// harmless (their would-be interior collapses to a single, already
/// feasibility-classified point).
fn intervals_from_events(mut events: Vec<f64>, feasible: impl Fn(f64) -> bool) -> Vec<(f64, f64)> {
    events.sort_by(|a, b| a.partial_cmp(b).expect("psi roots are finite"));
    if events.is_empty() {
        // Feasibility is constant over the whole circle.
        return if feasible(0.0) {
            vec![(-PI, PI)]
        } else {
            Vec::new()
        };
    }
    let n = events.len();
    let arcs: Vec<(f64, f64)> = (0..n)
        .map(|k| {
            let end = if k + 1 < n {
                events[k + 1]
            } else {
                events[0] + TAU
            };
            (events[k], end)
        })
        .collect();
    let arc_feasible: Vec<bool> = arcs
        .iter()
        .map(|&(start, end)| feasible(wrap_pi(0.5 * (start + end))))
        .collect();
    // Event k starts arc k and ends arc k - 1.
    let touches = (0..n)
        .filter(|&k| !arc_feasible[k] && !arc_feasible[(k + n - 1) % n] && feasible(events[k]))
        .map(|k| (events[k], events[k]));
    arcs.iter()
        .zip(&arc_feasible)
        .filter(|&(_, &is_feasible)| is_feasible)
        .map(|(&arc, _)| arc)
        .chain(touches)
        .collect()
}

/// The feasible arm angle closest (on the circle) to `preferred`, or `None` if
/// no arm angle is feasible. When the nearest point is an arc boundary it is
/// nudged just inside the arc so the returned angle is strictly feasible; a
/// single-angle arc yields that angle.
fn nearest_feasible_psi(intervals: &[(f64, f64)], preferred: f64) -> Option<f64> {
    let mut best: Option<(f64, f64)> = None; // (circular distance, psi)
    for &(start, end) in intervals {
        // Bring `preferred` into [start, start + 2pi) to test arc membership.
        let p = start + (preferred - start).rem_euclid(TAU);
        let psi = if p <= end {
            p // preferred itself is feasible
        } else {
            // Nearest endpoint, nudged just inside the (possibly thin) arc.
            let nudge = (0.5 * (end - start)).min(PSI_BOUNDARY_NUDGE);
            if wrap_pi(end - preferred).abs() <= wrap_pi(start - preferred).abs() {
                end - nudge
            } else {
                start + nudge
            }
        };
        let dist = wrap_pi(psi - preferred).abs();
        if best.is_none_or(|(d, _)| dist < d) {
            best = Some((dist, psi));
        }
    }
    best.map(|(_, psi)| wrap_pi(psi))
}

/// Rotation taking the orthonormal frame `[x0, y0, x0×y0]` to
/// `[x1, y1, x1×y1]`. `x0⊥y0` and `x1⊥y1` are required (asserted by callers'
/// geometry).
fn frame_map(
    x0: Vector3<f64>,
    y0: Vector3<f64>,
    x1: Vector3<f64>,
    y1: Vector3<f64>,
) -> Rotation3<f64> {
    let h = Matrix3::from_columns(&[x0, y0, x0.cross(&y0)]);
    let t = Matrix3::from_columns(&[x1, y1, x1.cross(&y1)]);
    Rotation3::from_matrix_unchecked(t * h.transpose())
}

/// Normalize each joint to its 2π-equivalent inside the limit window and return
/// the wrapped configuration, or `None` if any joint has no in-limit
/// representative. A representative within rounding of a limit is clamped onto
/// it, so a target that holds a joint exactly at a limit is solved, and every
/// returned joint lies inside its declared limits.
fn normalize_into_limits(limits: &[Limit; ARM_DOF], q: &JointVec) -> Option<JointVec> {
    let mut out = [0.0; ARM_DOF];
    for (slot, (&v, l)) in out.iter_mut().zip(q.iter().zip(limits)) {
        let w = wrap_into(v, l.lo, l.hi);
        if !(l.lo - LIMIT_SLACK <= w && w <= l.hi + LIMIT_SLACK) {
            return None;
        }
        *slot = l.clamp(w);
    }
    Some(out)
}

/// Distance to the seed, comparing each joint at the representative closest to
/// the seed (so a `+2π` alias doesn't look far).
fn seed_distance(q: &JointVec, seed: &JointVec) -> f64 {
    q.iter()
        .zip(seed)
        .map(|(&v, &s)| {
            let d = wrap_pi(v - s);
            d * d
        })
        .sum()
}

/// Wrap `x` toward `[lo, hi]` by multiples of `2π` (joint angles are modular);
/// returns the representative nearest the interval midpoint. For any window
/// narrower than `2π` (all OpenArm joints) this is the unique in-limit alias
/// when one exists.
fn wrap_into(x: f64, lo: f64, hi: f64) -> f64 {
    let mid = 0.5 * (lo + hi);
    mid + wrap_pi(x - mid)
}

/// Wrap an angle to the principal range `[-π, π]`: rounding takes the float
/// just below `-π` to `π`.
fn wrap_pi(x: f64) -> f64 {
    (x + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fk::ForwardKinematics;
    use crate::properties::{STRAIGHT_ELBOW_BAND, sides};
    use crate::test_support::{v1_fk, v1_limits, v1_model};
    use hegel::generators as gs;
    use kinematics_properties::{TOLERANCE, angles, configs, rotations, unit_vectors};
    use nalgebra::{Isometry3, UnitQuaternion};

    /// Full PoE forward rotation: `exp(w1 q1)...exp(w7 q7) * R_home`, the
    /// rotation half of the PoE map the IK is built on.
    fn fk_poe_rotation(model: &ArmModel, q: &JointVec) -> Rotation3<f64> {
        let mut r = model.home_tip.rotation.to_rotation_matrix();
        for i in (0..ARM_DOF).rev() {
            r = exp_so3(model.axes[i], q[i]) * r;
        }
        r
    }

    /// EE pose at `q` (thin wrapper; `ee_pose` already poses the chain).
    fn pose(fk: &ForwardKinematics, q: &JointVec) -> Isometry3<f64> {
        fk.at(q).ee_pose()
    }

    /// Assemble an EE target pose from a rotation + position (for the tests that
    /// build a target directly rather than from FK).
    fn iso(r: Rotation3<f64>, p: Vector3<f64>) -> Isometry3<f64> {
        Isometry3::from_parts(p.into(), UnitQuaternion::from_rotation_matrix(&r))
    }

    /// A whole number of turns, positive or negative.
    fn turns() -> gs::IntegerGenerator<i32> {
        gs::integers().min_value(-3).max_value(3)
    }

    /// A point in the cube of half-width 1 m about the origin.
    #[hegel::composite]
    fn points(tc: &hegel::TestCase) -> Vector3<f64> {
        Vector3::from(tc.draw(gs::arrays(gs::floats().min_value(-1.0).max_value(1.0))))
    }

    /// A configuration's own pose, solved with that configuration as the seed,
    /// returns the configuration: `FromSeed` keeps the arm angle the seed holds,
    /// a fixed policy at that arm angle reaches the same joints, and either keeps
    /// the shoulder and wrist branch the seed is on. The fixture's limits leave
    /// one branch per arm angle, so the limits are the fixture's or a full turn
    /// either way on every joint but the elbow, whose flex the reach fixes; that
    /// window holds every branch. The explicit case is the hanging home posture,
    /// its elbow near straight.
    #[hegel::test]
    #[hegel::explicit_test_case(
        side = "left",
        wide = false,
        fixed = false,
        q = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
    )]
    fn a_configuration_seeded_with_itself_solves_back_to_itself(tc: hegel::TestCase) {
        let side = tc.draw(sides());
        let fk = v1_fk(side);
        let m = ArmModel::from_fk(fk).unwrap();
        let fixture = fk.limits();
        let wide = tc.draw(gs::booleans());
        let lim = if wide {
            std::array::from_fn(|joint| match joint {
                ELBOW => fixture[ELBOW],
                _ => Limit { lo: -TAU, hi: TAU },
            })
        } else {
            fixture
        };
        let fixed = tc.draw(gs::booleans());
        let q = tc.draw(configs(lim));
        if q[ELBOW] < STRAIGHT_ELBOW_BAND {
            return;
        }
        let policy = if fixed {
            ArmAnglePolicy::Fixed(arm_angle_of(&m, &q).expect("a bent elbow has an arm angle"))
        } else {
            ArmAnglePolicy::FromSeed
        };
        let target = pose(fk, &q);
        let solution = solve(&m, &lim, &target, policy, &q)
            .unwrap_or_else(|| panic!("no solution for the pose of {q:?} seeded with itself"));
        for (joint, (&got, &want)) in solution.q.iter().zip(&q).enumerate() {
            assert!(
                wrap_pi(got - want).abs() < TOLERANCE,
                "joint {joint} solved to {got} for the pose of {q:?} seeded with itself"
            );
        }
    }

    #[test]
    fn cartesian_trajectory_servos_continuously() {
        // Trace a smooth closed Cartesian path (a circle in position plus a gentle
        // orientation wobble) and servo along it in FromSeed mode, seeding each
        // solve with the *previous* solution, as a real servo consumer would.
        // This is the one test that exercises the README's continuity guarantee:
        // along a smooth path the joints move smoothly, with no branch flip or psi
        // jump, while every pose is still hit exactly.
        let fk = v1_fk("left");
        let lim = fk.limits();
        let m = ArmModel::from_fk(fk).unwrap();

        // A comfortable mid-range posture, well off the straight-arm singularity.
        let q0 = [0.2, -0.3, 0.3, 1.0, -0.4, 0.5, 0.3];
        let home = pose(fk, &q0);
        let p0 = home.translation.vector;
        let r0 = home.rotation;
        // Orthonormal basis for the position circle (base-frame X and Z).
        let (u, w) = (Vector3::x(), Vector3::z());
        let radius = 0.03; // 3 cm: stays inside reach and off the singular band

        let steps = 240;
        let mut seed = q0;
        let mut max_step = 0.0_f64;
        for k in 0..=steps {
            let t = k as f64 * std::f64::consts::TAU / steps as f64;
            let p_d = p0 + radius * (t.cos() * u + t.sin() * w);
            // Gentle wrist wobble (vanishes at t=0 and t=2π so the loop closes).
            let r_d = r0 * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.15 * t.sin());
            let target = Isometry3::from_parts(p_d.into(), r_d);

            let sol = solve(&m, &lim, &target, ArmAnglePolicy::FromSeed, &seed)
                .expect("trajectory point should solve");

            let got = pose(fk, &sol.q);
            assert!(
                (got.translation.vector - p_d).norm() < 1e-6,
                "pos drift at step {k}"
            );
            assert!(got.rotation.angle_to(&r_d) < 1e-6, "rot drift at step {k}");

            // Continuity (the actual guarantee): the largest single-joint change
            // between steps stays small. A branch flip or psi jump would show up
            // here as a step of order 1 rad or more.
            if k > 0 {
                let step = sol
                    .q
                    .iter()
                    .zip(&seed)
                    .map(|(a, b)| wrap_pi(a - b).abs())
                    .fold(0.0, f64::max);
                assert!(
                    step < 0.05,
                    "joint jump {step:.4} at step {k}: branch flip / psi jump"
                );
                max_step = max_step.max(step);
            }
            seed = sol.q;
        }
        // A closed *task-space* loop need NOT close in joint space: the arm angle is
        // measured against a reference that rotates with the S-W direction, so the
        // wrist-center direction tracing a cone of nonzero solid angle leaves a net
        // psi offset (a geometric phase / holonomy). So this is a loose sanity bound,
        // not a tight invariant; the per-step bound above is the real guarantee.
        let closure = seed
            .iter()
            .zip(&q0)
            .map(|(a, b)| wrap_pi(a - b).abs())
            .fold(0.0, f64::max);
        println!(
            "trajectory max per-step joint change {max_step:.4} rad, loop closure {closure:.4} rad"
        );
        assert!(
            closure < 0.3,
            "loop drift {closure:.4} far larger than holonomy expects"
        );
    }

    /// The feasible intervals hold the arm angles a fixed policy solves: an arm
    /// angle inside an interval solves, a solution at an arm angle outside every
    /// interval holds a joint within [`TOLERANCE`] of a limit, and the pose of a
    /// configuration with its elbow bent past the band has a feasible arm angle.
    /// The explicit case holds joint 1 on its lower limit at the peak of its
    /// travel over the arm angles, which a nearly straight elbow makes 2e-3 rad
    /// high: its limit event is a tangency that misses by 2e-12.
    #[hegel::test]
    #[hegel::explicit_test_case(
        side = "left",
        q = [-3.4906589999999995, -1.0, 0.0, 0.001953125, 0.0, 0.0, 0.0],
        seed = [0.0; 7],
        psi = 0.0
    )]
    fn the_feasible_intervals_hold_the_arm_angles_that_solve(tc: hegel::TestCase) {
        let side = tc.draw(sides());
        let fk = v1_fk(side);
        let lim = fk.limits();
        let m = ArmModel::from_fk(fk).unwrap();
        let q = tc.draw(configs(lim));
        let seed = tc.draw(configs(lim));
        let psi = tc.draw(angles());
        let target = pose(fk, &q);
        let Some(ctx) = PsiSolve::new(&m, &lim, &target) else {
            assert!(
                q[ELBOW] < STRAIGHT_ELBOW_BAND,
                "the pose of {q:?} is outside the reach"
            );
            return;
        };
        let intervals = ctx.feasible_psi_intervals(&seed);
        assert!(
            q[ELBOW] < STRAIGHT_ELBOW_BAND || !intervals.is_empty(),
            "no feasible arm angle for the pose of {q:?}"
        );
        let inside = intervals
            .iter()
            .any(|&(start, end)| start + (psi - start).rem_euclid(TAU) <= end);
        match solve(&m, &lim, &target, ArmAnglePolicy::Fixed(psi), &seed) {
            None => assert!(
                !inside,
                "the arm angle {psi} lies in {intervals:?} and does not solve"
            ),
            Some(solution) => {
                let on_a_limit = solution.q.iter().zip(&lim).any(|(&value, limit)| {
                    value - limit.lo <= TOLERANCE || limit.hi - value <= TOLERANCE
                });
                assert!(
                    inside || on_a_limit,
                    "the arm angle {psi} solves to {:?} outside {intervals:?}",
                    solution.q
                );
            }
        }
    }

    #[hegel::test]
    fn poe_fk_matches_an_independent_chain(tc: hegel::TestCase) {
        // Three implementations must agree on where the end effector is: the
        // PoE screw map the IK is built on, this crate's own URDF chain, and the
        // `k` crate reading the same URDF. The third is the point - the first two
        // are ours, so on their own they could share a mistake. `k` is carried as
        // a dev-dependency for exactly this and nothing else. It loads the
        // fixture file with its own URDF parser, so the only thing it shares
        // with this crate's chain is the file.
        //
        // Compared component-wise rather than by subtracting `Isometry3`s, so the
        // check survives `k` and this crate resolving different `nalgebra`
        // versions.
        use k::nalgebra as kna;

        let side = tc.draw(sides());
        let fk = v1_fk(side);
        let m = ArmModel::from_fk(fk).unwrap();
        let q = tc.draw(configs(fk.limits()));

        let ours = pose(fk, &q);
        let p_poe = fk_poe_position(&m, &q);
        assert!(
            (ours.translation.vector - p_poe).norm() < TOLERANCE,
            "PoE and chain FK disagree on position"
        );
        let r_poe = fk_poe_rotation(&m, &q);
        assert!(
            (r_poe.matrix() - ours.rotation.to_rotation_matrix().matrix()).norm() < TOLERANCE,
            "PoE and chain FK disagree on orientation"
        );

        // One tree holds both arms; each case sets all seven joints of its side.
        static CHAIN: std::sync::LazyLock<k::Chain<f64>> = std::sync::LazyLock::new(|| {
            k::Chain::<f64>::from_urdf_file(crate::test_support::FIXTURE_URDF_PATH)
                .expect("k loads the fixture")
        });
        let chain = &*CHAIN;
        let tip = chain
            .find_link(&format!("openarm_{side}_link7"))
            .expect("fixture has link7");
        let serial = k::SerialChain::from_end(tip);
        serial.set_joint_positions_clamped(&q);
        serial.update_transforms();
        let base = chain
            .find_link(&format!("openarm_{side}_link0"))
            .expect("fixture has link0")
            .world_transform()
            .expect("base world transform");
        let theirs = base.inverse() * tip.world_transform().expect("tip world transform");

        let dp = [
            theirs.translation.vector.x - ours.translation.vector.x,
            theirs.translation.vector.y - ours.translation.vector.y,
            theirs.translation.vector.z - ours.translation.vector.z,
        ];
        assert!(
            dp.iter().all(|d| d.abs() < TOLERANCE),
            "independent chain disagrees on position by {dp:?}"
        );
        let theirs_r: kna::Matrix3<f64> = *theirs.rotation.to_rotation_matrix().matrix();
        let ours_r = ours.rotation.to_rotation_matrix();
        for r in 0..3 {
            for c in 0..3 {
                let d = theirs_r[(r, c)] - ours_r.matrix()[(r, c)];
                assert!(
                    d.abs() < TOLERANCE,
                    "independent chain disagrees on orientation at ({r},{c}) by {d}"
                );
            }
        }
    }

    // --- Direct unit tests for the pure helpers --------------------------

    #[hegel::test]
    fn wrap_pi_returns_an_alias_in_the_principal_range(tc: hegel::TestCase) {
        let angle = tc.draw(angles());
        let turns = tc.draw(turns());
        let wrapped = wrap_pi(angle + f64::from(turns) * TAU);
        let apart = (wrapped - angle) / TAU;
        assert!(
            (-PI..=PI).contains(&wrapped) && (apart - apart.round()).abs() < TOLERANCE,
            "wrap_pi({angle} + {turns} turns) = {wrapped}"
        );
    }

    #[hegel::test]
    fn wrap_into_returns_the_alias_inside_a_joint_window(tc: hegel::TestCase) {
        let limits = v1_limits(tc.draw(sides()));
        let Limit { lo, hi } = limits[tc.draw(gs::integers::<usize>().max_value(ARM_DOF - 1))];
        let inside = tc.draw(gs::floats().min_value(lo).max_value(hi));
        let x = inside + f64::from(tc.draw(turns())) * TAU;
        let wrapped = wrap_into(x, lo, hi);
        assert!(
            (wrapped - inside).abs() < TOLERANCE,
            "wrap_into({x}, {lo}, {hi}) = {wrapped}; the alias inside is {inside}"
        );
    }

    #[hegel::test]
    fn normalize_into_limits_recovers_every_in_limit_configuration(tc: hegel::TestCase) {
        let limits = v1_limits(tc.draw(sides()));
        let q = tc.draw(configs(limits));
        let turns: [i32; ARM_DOF] = tc.draw(gs::arrays(turns()));
        let raw: JointVec = std::array::from_fn(|joint| q[joint] + f64::from(turns[joint]) * TAU);
        let normalized = normalize_into_limits(&limits, &raw)
            .unwrap_or_else(|| panic!("no in-limit alias found for {raw:?}, though {q:?} is one"));
        for (joint, ((&got, &want), limit)) in normalized.iter().zip(&q).zip(&limits).enumerate() {
            assert!(
                limit.contains(got) && (got - want).abs() < TOLERANCE,
                "joint {joint}: normalized to {got}; the in-limit alias is {want} in [{}, {}]",
                limit.lo,
                limit.hi
            );
        }
    }

    /// At a gimbal lock (shoulder: joint 2 turns joint 3's axis onto joint 1's;
    /// wrist: joint 6 turns joint 7's onto joint 5's) the outer two joints turn
    /// about one line, so every split of their sum reaches the pose; FromSeed
    /// solving a seed's own pose returns the seed's split. The fixture's joint 6
    /// stops short of the wrist lock, so the wrist case widens it to a full turn.
    /// The explicit cases are a seed whose split no single outer joint can hold,
    /// one whose outer joints both rest on limits, so its split is the corner,
    /// and one whose corner, with joint 5 on a limit and the elbow nearly
    /// straight, leaves the seed's own arm angle as the only feasible one.
    #[hegel::test]
    #[hegel::explicit_test_case(
        side = "left",
        middle = 1usize,
        lock = -FRAC_PI_2,
        q = [1.0, -FRAC_PI_2, 1.4, 1.0, 0.1, 0.2, 0.1]
    )]
    #[hegel::explicit_test_case(
        side = "right",
        middle = 1usize,
        lock = FRAC_PI_2,
        q = [v1_limits("right")[0].hi, FRAC_PI_2, v1_limits("right")[2].hi, 1.0, 0.0, 0.0, 0.0]
    )]
    #[hegel::explicit_test_case(
        side = "left",
        middle = 1usize,
        lock = -FRAC_PI_2,
        q = [
            v1_limits("left")[0].hi,
            -FRAC_PI_2,
            v1_limits("left")[2].hi,
            0.004107937311346177,
            v1_limits("left")[4].hi,
            0.0,
            0.0
        ]
    )]
    fn from_seed_returns_the_seed_at_a_gimbal_lock(tc: hegel::TestCase) {
        let side = tc.draw(sides());
        let fk = v1_fk(side);
        let m = ArmModel::from_fk(fk).unwrap();
        let middle = tc.draw(gs::sampled_from([1usize, 5].as_slice()));
        let widened: [Limit; ARM_DOF] = std::array::from_fn(|joint| match (joint, middle) {
            (5, 5) => Limit { lo: -PI, hi: PI },
            _ => fk.limits()[joint],
        });
        let locks: Vec<f64> = [-FRAC_PI_2, FRAC_PI_2]
            .into_iter()
            .filter(|&lock| widened[middle].contains(lock))
            .collect();
        let lock = tc.draw(gs::sampled_from(locks));
        let pinned: [Limit; ARM_DOF] = std::array::from_fn(|joint| match joint {
            _ if joint == middle => Limit { lo: lock, hi: lock },
            ELBOW => Limit {
                lo: STRAIGHT_ELBOW_BAND,
                ..widened[joint]
            },
            _ => widened[joint],
        });
        let q = tc.draw(configs(pinned));
        let target = pose(fk, &q);
        let solution = solve(&m, &widened, &target, ArmAnglePolicy::FromSeed, &q)
            .unwrap_or_else(|| panic!("the {side} arm's own pose at {q:?} is unsolvable"));
        for (joint, (&got, &want)) in solution.q.iter().zip(&q).enumerate() {
            assert!(
                wrap_pi(got - want).abs() < TOLERANCE,
                "joint {joint}: got {got}, want {want}; the seed's split was not kept"
            );
        }
    }

    /// The whole redundancy circle maps to the same EE pose: on an arm whose
    /// joints turn a full circle, every arm angle solves the pose of every bent
    /// configuration, and each solution reaches it. This is the non-circular
    /// check that psi is a true redundancy parameter.
    #[hegel::test]
    fn every_arm_angle_solves_the_pose_of_an_unlimited_arm(tc: hegel::TestCase) {
        let side = tc.draw(sides());
        let fk = v1_fk(side);
        let m = ArmModel::from_fk(fk).unwrap();
        let unlimited = [Limit { lo: -PI, hi: PI }; ARM_DOF];
        let bent: [Limit; ARM_DOF] = std::array::from_fn(|joint| match joint {
            ELBOW => Limit {
                lo: STRAIGHT_ELBOW_BAND,
                ..fk.limits()[joint]
            },
            _ => unlimited[joint],
        });
        let q = tc.draw(configs(bent));
        let psi = tc.draw(angles());
        let target = pose(fk, &q);
        let solution = solve(&m, &unlimited, &target, ArmAnglePolicy::Fixed(psi), &q)
            .unwrap_or_else(|| panic!("arm angle {psi} does not solve the pose of {q:?}"));
        let reached = pose(fk, &solution.q);
        let position_miss = (reached.translation.vector - target.translation.vector).norm();
        let orientation_miss = reached.rotation.angle_to(&target.rotation);
        assert!(
            position_miss < TOLERANCE && orientation_miss < TOLERANCE,
            "arm angle {psi} missed the pose of {q:?} by {position_miss:.2e} m and \
             {orientation_miss:.2e} rad"
        );
    }

    #[hegel::test]
    fn normalize_into_limits_refuses_a_joint_with_no_alias_in_its_window(tc: hegel::TestCase) {
        let limits = v1_limits(tc.draw(sides()));
        let q = tc.draw(configs(limits));
        let joint = tc.draw(gs::integers::<usize>().max_value(ARM_DOF - 1));
        let Limit { lo, hi } = limits[joint];
        let outside = tc.draw(
            gs::floats()
                .min_value(hi + TOLERANCE)
                .max_value(lo + TAU - TOLERANCE),
        );
        let turns = tc.draw(turns());
        let raw: JointVec = std::array::from_fn(|j| {
            if j == joint {
                outside + f64::from(turns) * TAU
            } else {
                q[j]
            }
        });
        assert!(
            normalize_into_limits(&limits, &raw).is_none(),
            "joint {joint} = {} has no alias in [{lo}, {hi}], yet {raw:?} normalized",
            raw[joint]
        );
    }

    #[hegel::test]
    fn subproblem1_recovers_the_rotation_angle(tc: hegel::TestCase) {
        // Only p's component off the axis rotates, and its scale cancels, so a
        // point one unit off the axis at any height stands for every point.
        let frame = tc.draw(rotations());
        let height: f64 = tc.draw(gs::floats().min_value(-1.0).max_value(1.0));
        let (axis, p) = (frame * Vector3::z(), frame * Vector3::new(1.0, 0.0, height));
        let theta = tc.draw(angles());
        let recovered = subproblem1(axis, p, exp_so3(axis, theta) * p);
        assert!(
            wrap_pi(recovered - theta).abs() < TOLERANCE,
            "recovered {recovered} rad from a {theta} rad rotation"
        );
    }

    #[hegel::test]
    fn subproblem2_finds_a_branch_reaching_every_reachable_target(tc: hegel::TestCase) {
        let frame = tc.draw(rotations());
        let (w1, w2) = (frame * Vector3::x(), frame * Vector3::y());
        let p = tc.draw(points());
        let (t1, t2) = (tc.draw(angles()), tc.draw(angles()));
        let q = exp_so3(w1, t1) * exp_so3(w2, t2) * p;
        let branches = subproblem2(w1, w2, p, q);
        assert!(
            branches
                .iter()
                .any(|&(b1, b2)| (exp_so3(w1, b1) * exp_so3(w2, b2) * p - q).norm() < TOLERANCE),
            "no branch of {branches:?} reaches {q:?}"
        );
    }

    #[hegel::test]
    fn subproblem2_refuses_parallel_axes(tc: hegel::TestCase) {
        let w1 = tc.draw(unit_vectors());
        let w2 = if tc.draw(gs::booleans()) { w1 } else { -w1 };
        let (p, q) = (tc.draw(points()), tc.draw(points()));
        assert!(subproblem2(w1, w2, p, q).is_empty());
    }

    #[test]
    fn subproblem2_refuses_a_target_off_the_reachable_circle() {
        // Rotation about x keeps p's x-component (0.4), and rotation about z then
        // keeps the resulting z-component, so a same-norm pure-+z target is
        // unreachable: gamma_sq < 0.
        let p = Vector3::new(0.4, 0.5, 0.6);
        let unreachable = Vector3::z() * p.norm();
        assert!(subproblem2(Vector3::z(), Vector3::x(), p, unreachable).is_empty());
    }

    #[hegel::test]
    fn decompose_three_axes_reconstructs_the_rotation(tc: hegel::TestCase) {
        // Either handedness of axis frame, as the two mirrored arms have.
        let frame = tc.draw(rotations());
        let (w1, w2) = (frame * Vector3::x(), frame * Vector3::y());
        let w3 = if tc.draw(gs::booleans()) {
            w1.cross(&w2)
        } else {
            w2.cross(&w1)
        };
        let (t1, t2, t3) = (tc.draw(angles()), tc.draw(angles()), tc.draw(angles()));
        let r = exp_so3(w1, t1) * exp_so3(w2, t2) * exp_so3(w3, t3);
        let branches = decompose_three_axes(&r, w1, w2, w3);
        assert!(
            branches.iter().any(|&(b1, b2, b3)| {
                let rebuilt = exp_so3(w1, b1) * exp_so3(w2, b2) * exp_so3(w3, b3);
                (rebuilt.matrix() - r.matrix()).norm() < TOLERANCE
            }),
            "no branch of {branches:?} rebuilds {r:?}"
        );
    }

    #[hegel::test]
    fn frame_map_is_the_rotation_taking_one_frame_to_the_other(tc: hegel::TestCase) {
        let (from, to) = (tc.draw(rotations()), tc.draw(rotations()));
        let (x0, y0) = (from * Vector3::x(), from * Vector3::y());
        let (x1, y1) = (to * Vector3::x(), to * Vector3::y());
        let r = frame_map(x0, y0, x1, y1);
        assert!((r * x0 - x1).norm() < TOLERANCE && (r * y0 - y1).norm() < TOLERANCE);
        let m = r.matrix();
        assert!(
            (m.transpose() * m - Matrix3::identity()).norm() < TOLERANCE
                && (m.determinant() - 1.0).abs() < TOLERANCE,
            "frame_map returned {m}, which is not a rotation"
        );
    }

    #[hegel::test]
    fn a_target_beyond_the_reach_is_refused(tc: hegel::TestCase) {
        let m = v1_model("left");
        let lim = v1_limits("left");
        // The tip rides the wrist center at a fixed distance, so no configuration
        // takes it farther from the shoulder than the two links plus that hand.
        let hand = (m.home_tip.translation.vector - m.wrist_home).norm();
        let reach = m.l_su + m.l_uw + hand;
        let direction = tc.draw(unit_vectors());
        let distance = tc.draw(
            gs::floats()
                .min_value(reach + TOLERANCE)
                .max_value(2.0 * reach),
        );
        let seed = tc.draw(configs(lim));
        let target = iso(tc.draw(rotations()), m.shoulder + direction * distance);
        assert!(solve(&m, &lim, &target, ArmAnglePolicy::FromSeed, &seed).is_none());
    }

    #[test]
    fn cartesian_departure_from_near_straight_start_is_continuous() {
        // Walking the EE from a near-straight hanging start to a bent reach pose,
        // seeding each solve from the previous solution, must keep every joint step
        // small: the solution tracks the seed continuously, with no instantaneous
        // reconfiguration as the arm leaves the near-singular region.
        let fk = v1_fk("left");
        let lim = fk.limits();
        let m = ArmModel::from_fk(fk).unwrap();
        let start_q = [0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0]; // hanging
        let reach_q = [0.0, -0.3, 0.0, 1.2, 0.0, 0.4, 0.0]; // bent reach
        let start = pose(fk, &start_q);
        let end = pose(fk, &reach_q);

        let n = 500; // ~5 s move at 100 Hz
        let mut seed = start_q;
        let mut prev = start_q;
        let mut worst = 0.0_f64;
        for k in 0..=n {
            let s = k as f64 / n as f64;
            let p = start.translation.vector.lerp(&end.translation.vector, s);
            let r = start
                .rotation
                .try_slerp(&end.rotation, s, 1e-6)
                .unwrap_or(end.rotation);
            let target = Isometry3::from_parts(p.into(), r);
            let sol = solve(&m, &lim, &target, ArmAnglePolicy::FromSeed, &seed)
                .expect("departure path stays reachable");
            let step = sol
                .q
                .iter()
                .zip(&prev)
                .map(|(&a, &b)| wrap_pi(a - b).abs())
                .fold(0.0, f64::max);
            worst = worst.max(step);
            prev = sol.q;
            seed = sol.q;
        }
        println!("worst joint step over departure: {worst:.4} rad");
        assert!(
            worst < 0.1,
            "largest joint step {worst} rad — discontinuous departure"
        );
    }

    #[hegel::test]
    fn a_target_with_a_non_finite_coordinate_is_refused(tc: hegel::TestCase) {
        let m = v1_model("left");
        let lim = v1_limits("left");
        let finite = tc.draw(points());
        let axis = tc.draw(gs::integers::<usize>().max_value(2));
        let non_finite = tc.draw(gs::sampled_from(
            [f64::NAN, f64::INFINITY, f64::NEG_INFINITY].as_slice(),
        ));
        let position =
            Vector3::from_fn(|row, _| if row == axis { non_finite } else { finite[row] });
        let policy = if tc.draw(gs::booleans()) {
            ArmAnglePolicy::FromSeed
        } else {
            ArmAnglePolicy::Fixed(tc.draw(angles()))
        };
        let seed = tc.draw(configs(lim));
        let target = iso(Rotation3::identity(), position);
        assert!(solve(&m, &lim, &target, policy, &seed).is_none());
    }

    /// The PoE Jacobian that scores manipulability equals the chain's geometric
    /// Jacobian entrywise, on either arm.
    #[hegel::test]
    fn poe_jacobian_matches_chain_jacobian(tc: hegel::TestCase) {
        let side = tc.draw(sides());
        let fk = v1_fk(side);
        let m = ArmModel::from_fk(fk).unwrap();
        let q = tc.draw(configs(fk.limits()));
        let err = (jacobian_poe(&m, &q) - fk.at(&q).jacobian()).abs().max();
        assert!(err < TOLERANCE, "the Jacobians at {q:?} disagree by {err}");
    }

    /// A manipulability step solves exactly when `FromSeed` does, from the
    /// `FromSeed` anchor: it hits the pose, moves the arm angle at most the step,
    /// never scores below the anchor, and a step of zero returns the anchor.
    #[hegel::test]
    fn max_manipulability_tracks_the_target_and_never_scores_below_from_seed(tc: hegel::TestCase) {
        let side = tc.draw(sides());
        let fk = v1_fk(side);
        let lim = fk.limits();
        let m = ArmModel::from_fk(fk).unwrap();
        let q = tc.draw(configs(lim));
        let seed = tc.draw(configs(lim));
        let step: f64 = tc.draw(gs::floats().min_value(0.0).max_value(PI));
        let target = pose(fk, &q);
        let policy = ArmAnglePolicy::MaxManipulability { max_step_rad: step };
        let Some(anchor) = solve(&m, &lim, &target, ArmAnglePolicy::FromSeed, &seed) else {
            assert!(
                solve(&m, &lim, &target, policy, &seed).is_none(),
                "a step solved the pose of {q:?}, which FromSeed refuses"
            );
            return;
        };
        let sol =
            solve(&m, &lim, &target, policy, &seed).expect("solvable exactly when FromSeed is");
        let got = pose(fk, &sol.q);
        assert!((got.translation.vector - target.translation.vector).norm() < TOLERANCE);
        assert!(got.rotation.angle_to(&target.rotation) < TOLERANCE);
        assert!(
            wrap_pi(sol.arm_angle - anchor.arm_angle).abs() <= step + TOLERANCE,
            "the arm angle stepped {} past the {step} bound",
            wrap_pi(sol.arm_angle - anchor.arm_angle).abs()
        );
        let w_anchor = manipulability(&jacobian_poe(&m, &anchor.q));
        let w_opt = manipulability(&jacobian_poe(&m, &sol.q));
        assert!(
            w_opt >= w_anchor - TOLERANCE,
            "manipulability regressed: {w_opt} < {w_anchor}"
        );
        if step == 0.0 {
            assert!(
                (0..ARM_DOF).all(|joint| (sol.q[joint] - anchor.q[joint]).abs() < TOLERANCE),
                "a zero step moved {:?} to {:?}",
                anchor.q,
                sol.q
            );
        }
    }

    #[test]
    fn max_manipulability_chain_converges_and_climbs() {
        // Chaining solves at a fixed target (seeding each with the previous
        // solution, as the planner and the servo loop do) must climb toward a
        // manipulability ridge and settle there, not oscillate.
        let fk = v1_fk("left");
        let lim = fk.limits();
        let m = ArmModel::from_fk(fk).unwrap();
        let q0 = [0.2, -0.3, 0.3, 1.0, -0.4, 0.5, 0.3];
        let target = pose(fk, &q0);
        let mut seed = q0;
        let mut w_prev = manipulability(&jacobian_poe(&m, &q0));
        let mut psi_prev = arm_angle_of(&m, &q0).unwrap();
        let mut last_delta = f64::INFINITY;
        for _ in 0..60 {
            let sol = solve(
                &m,
                &lim,
                &target,
                ArmAnglePolicy::MaxManipulability { max_step_rad: 0.05 },
                &seed,
            )
            .expect("fixed reachable target stays solvable");
            let w = manipulability(&jacobian_poe(&m, &sol.q));
            assert!(w >= w_prev - 1e-12, "climb must be monotone");
            last_delta = wrap_pi(sol.arm_angle - psi_prev).abs();
            psi_prev = sol.arm_angle;
            w_prev = w;
            seed = sol.q;
        }
        assert!(
            last_delta < 1e-3,
            "psi still moving {last_delta} rad after 60 chained solves"
        );
    }

    /// `FromSeed` keeps the elbow's azimuth about the shoulder-wrist axis where
    /// the seed had it, measured on the target's redundancy circle: the frame the
    /// feasible intervals and `solve_at_psi` live in. The seed's own circle is a
    /// different reference, up to pi apart at arm's length. So whenever the
    /// seed's azimuth on the target's circle is feasible, `FromSeed` returns
    /// exactly that arm angle, however far the seed sits from the target.
    #[hegel::test]
    fn from_seed_keeps_the_elbow_where_the_seed_had_it(tc: hegel::TestCase) {
        let side = tc.draw(sides());
        let fk = v1_fk(side);
        let lim = fk.limits();
        let m = ArmModel::from_fk(fk).unwrap();
        let q = tc.draw(configs(lim));
        let seed = tc.draw(configs(lim));
        let target = pose(fk, &q);
        let Some(ctx) = PsiSolve::new(&m, &lim, &target) else {
            return;
        };
        // The azimuth comes from the target's circle here, apart from
        // `preferred_psi`, so a defect there cannot pass on its own answer.
        let e = elbow_position(&m, &seed) - ctx.circle.center;
        let (ax, bx) = (e.dot(&ctx.circle.a_hat), e.dot(&ctx.circle.b_hat));
        if ax.hypot(bx) < SINGULAR_RADIUS {
            return;
        }
        let preferred = f64::atan2(bx, ax);
        if ctx.solve_at_psi(preferred, &seed).is_none() {
            return;
        }
        let sol = solve(&m, &lim, &target, ArmAnglePolicy::FromSeed, &seed)
            .expect("a feasible preferred psi yields a solution");
        assert!(
            wrap_pi(sol.arm_angle - preferred).abs() < TOLERANCE,
            "FromSeed drifted off a feasible preference by {} rad (got {}, wanted {})",
            wrap_pi(sol.arm_angle - preferred).abs(),
            sol.arm_angle,
            preferred,
        );
    }
}
