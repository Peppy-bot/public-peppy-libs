// Software License Agreement (BSD License)
//
//  Copyright (c) 2009, Willow Garage, Inc.
//  Copyright (c) 2012, Willow Garage, Inc.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of Willow Garage nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//
// This file translates MoveIt 2.15.2's chomp_motion_planner into Rust: chomp_optimizer.cpp,
// chomp_optimizer.hpp, chomp_cost.cpp, chomp_trajectory.cpp and chomp_utils.hpp
// (Copyright (c) 2009, Willow Garage, Inc.), and the values of chomp_interface.cpp's
// planner plugin (Copyright (c) 2012, Willow Garage, Inc.).

//! MoveIt's CHOMP optimizer (chomp_motion_planner 2.15.2) on the arms of a plan, with the
//! values of its planner plugin.
//!
//! A trajectory is CHOMP's 101 points, 3 s at 0.03 s (`ChompPlanner::solve`), padded with
//! its first and last points, five at each end, for the finite differences; its 99 points
//! between the start and the goal are free. It starts evenly spaced along the path the robot
//! runs through the planner's path, its corners rounded as execute_joint_path rounds them;
//! MoveIt's `fillInFromTrajectory` fills its points by waypoint index through the trajectory
//! it is given (chomp_trajectory.cpp:188-209). Its cost is its smoothness cost, the squared
//! acceleration of each joint weighed by 0.1, plus its collision cost: at each free point,
//! each collision sphere's potential times its speed. A sphere's potential is MoveIt's of
//! its center's distance in the field less its radius, 0 from a clearance of 0.2 m; the
//! field's distances and gradient are capped at MoveIt's propagation distance, 0.25 m,
//! rounded up to whole voxels as MoveIt's field rounds it, and a sphere whose center lies
//! 0.25 m or more from an obstacle, or outside the field, has none. A step of descent takes
//! the smoothness increments of every free point and the collision increments of one free
//! point drawn at random, through the inverse of the smoothness cost, each joint's step held
//! to 0.1 rad, then pulls each joint back inside its limits along a column of that inverse.
//!
//! The spheres are those of each link of each arm and of each part its links carry, such as
//! a gripper finger (`collision_geometry::spheres`), grown by the plan's clearance, as MoveIt
//! grows a link's bodies by its padding. A sphere's Jacobian is the derivative of its
//! center: for a revolute joint, the joint's axis crossed with the center's offset from the
//! joint. MoveIt's `computeJointProperties`
//! (chomp_optimizer.cpp:749) takes `axis = joint_transform * axis` on an
//! `Eigen::Isometry3d`, which adds the joint's position to its axis. A sphere of no speed
//! adds no collision increment; MoveIt divides by its speed.

use std::ops::RangeInclusive;

use chain_kinematics::{Draws, Limit, Posed};
use collision_geometry::chain::RiderBody;
use collision_geometry::spheres::SphereSource;
use control_core::path_timing::RoundedPath;
use control_core::positive_finite::PositiveFinite;
use nalgebra::{DMatrix, DVector, Isometry3, Matrix3, Point3, Vector3};
use occupancy_map::DistanceField;

use crate::arms::Arms;
use crate::plan::Scene;

/// The points of a trajectory, the start and the goal among them, `ChompPlanner::solve`'s
/// trajectory of 3 s at [`DISCRETIZATION_S`].
const POINTS: usize = 101;
/// The time between two points, in seconds.
const DISCRETIZATION_S: f64 = 0.03;
/// MoveIt's `smoothness_cost_weight`.
const SMOOTHNESS_COST_WEIGHT: f64 = 0.1;
/// MoveIt's `obstacle_cost_weight`.
const OBSTACLE_COST_WEIGHT: f64 = 1.0;
/// MoveIt's `learning_rate`.
const LEARNING_RATE: f64 = 0.01;
/// The weights of the trajectory's velocity, acceleration and jerk in its smoothness
/// cost: MoveIt's `smoothness_cost_velocity`, `_acceleration` and `_jerk`.
const DERIVATIVE_COSTS: [f64; 3] = [0.0, 1.0, 0.0];
/// MoveIt's `ridge_factor`.
const RIDGE_FACTOR: f64 = 0.0;
/// The largest step of a joint in one iteration, MoveIt's `joint_update_limit`.
const JOINT_UPDATE_LIMIT_RAD: f64 = 0.1;
/// The clearance within which a sphere has a potential, MoveIt's `collision_clearance`.
const POTENTIAL_CLEARANCE_M: f64 = 0.2;
/// The distance at which MoveIt's field stops, `max_propogation_distance`
/// (collision_env_distance_field.hpp:55).
const PROPAGATION_DISTANCE_M: f64 = 0.25;
/// MoveIt's `max_iterations`.
const MAX_ITERATIONS: u32 = 200;
/// The iterations between two walks of the trajectory of least cost
/// (chomp_optimizer.cpp:368).
const WALK_EVERY: u32 = 10;
/// The potential below which a sphere adds no collision increment.
const LEAST_POTENTIAL: f64 = 1e-4;
/// The violation of a joint limit that CHOMP leaves.
const LIMIT_SLACK_RAD: f64 = 1e-6;
/// The pulls of a joint back inside its limits in one iteration at most: MoveIt's loop
/// ends after its eleventh.
const LIMIT_PULLS: usize = 11;

/// The finite-difference rules of the velocity, the acceleration and the jerk, from three
/// points before to three after (`chomp_utils.hpp`).
const DIFF_RULE_LENGTH: usize = 7;
const DIFF_RULES: [[f64; DIFF_RULE_LENGTH]; 3] = [
    [0.0, 0.0, -2.0 / 6.0, -3.0 / 6.0, 1.0, -1.0 / 6.0, 0.0],
    [
        0.0,
        -1.0 / 12.0,
        16.0 / 12.0,
        -30.0 / 12.0,
        16.0 / 12.0,
        -1.0 / 12.0,
        0.0,
    ],
    [
        0.0,
        1.0 / 12.0,
        -17.0 / 12.0,
        46.0 / 12.0,
        -46.0 / 12.0,
        17.0 / 12.0,
        -1.0 / 12.0,
    ],
];

/// The padding at each end of the trajectory, its first or last point repeated, so that
/// `DIFF_RULE_LENGTH - 1` points lie before the first free point and after the last.
const PAD: usize = DIFF_RULE_LENGTH - 2;
/// The points of the padded trajectory.
const PADDED: usize = POINTS + 2 * PAD;
/// The free points, between the start and the goal.
const FREE_START: usize = DIFF_RULE_LENGTH - 1;
const FREE_END: usize = PADDED - DIFF_RULE_LENGTH;
const FREE: usize = FREE_END - FREE_START + 1;

/// CHOMP's refinement of `path` of `arms`, from its start to its end, in `field`, each
/// link's spheres grown by `padding`: its points evenly spaced along the path the robot runs
/// through `path` ([`along_the_path_run`]), and [`MAX_ITERATIONS`] steps of descent, each
/// drawing from the stream of `seed`. It weighs the trajectories MoveIt's optimizer walks,
/// each of a cost below that of the trajectory it starts from: the trajectory of least cost
/// so far at every [`WALK_EVERY`]-th iteration from the first, when it changed since the one
/// before, and the one after the last iteration. It runs every iteration; MoveIt's optimizer
/// stops at its first clear walk, or five iterations after its collision cost falls below
/// 0.07 (chomp_optimizer.cpp:368-418, 477-492), and walks nothing after its last iteration.
/// It ends early when `ended` says so, or when a value of the trajectory is no number. It
/// then walks those trajectories with `walk`, from the least cost up, and answers what
/// `walk` answers for the first that walks clear, or none.
///
/// # Panics
///
/// When `path` holds fewer than two waypoints.
pub(crate) fn refine<const N: usize, const D: usize, T>(
    arms: &Arms<N, D>,
    field: &DistanceField,
    path: &[[f64; D]],
    padding: f64,
    seed: u64,
    ended: impl Fn() -> bool,
    mut walk: impl FnMut(&[[f64; D]]) -> Option<T>,
) -> Option<T> {
    let mut chomp = Chomp::of_path(arms, field, path, padding);
    let mut draws = Draws::new(seed);
    let mut best = (chomp.waypoints(), chomp.cost());
    let mut weighed: Vec<(Vec<[f64; D]>, f64)> = Vec::new();
    let mut changed = false;
    for iteration in 0..MAX_ITERATIONS {
        if ended() {
            break;
        }
        let cost = chomp.cost();
        if cost < best.1 {
            best = (chomp.waypoints(), cost);
            changed = true;
        }
        chomp.step(&mut draws);
        if iteration.is_multiple_of(WALK_EVERY) && changed {
            weighed.push(best.clone());
            changed = false;
        }
        if !chomp.is_finite() {
            break;
        }
    }
    if changed {
        weighed.push(best);
    }
    weighed.sort_by(|a, b| a.1.total_cmp(&b.1));
    weighed
        .into_iter()
        .find_map(|(trajectory, _)| walk(&trajectory))
}

/// CHOMP's cost of the trajectory it starts from along `path`, as [`refine`] weighs the
/// path it starts from.
#[cfg(test)]
pub(crate) fn path_cost<const N: usize, const D: usize>(
    arms: &Arms<N, D>,
    field: &DistanceField,
    path: &[[f64; D]],
    padding: f64,
) -> f64 {
    Chomp::of_path(arms, field, path, padding).cost()
}

/// CHOMP's points along the path the robot runs through `path`, one row per point: the
/// path rounded as execute_joint_path rounds it ([`RoundedPath::joint_path`]), its points
/// evenly spaced along it, the first `path`'s start and the last its end.
fn along_the_path_run<const D: usize>(path: &[[f64; D]]) -> DMatrix<f64> {
    assert!(path.len() >= 2, "a path of two waypoints or more");
    let waypoints: Vec<Vec<f64>> = path.iter().map(|q| q.to_vec()).collect();
    let rounded = RoundedPath::joint_path(&waypoints);
    let last = POINTS - 1;
    let rows: Vec<[f64; D]> = (0..POINTS)
        .map(|i| match i {
            0 => path[0],
            i if i == last => path[path.len() - 1],
            i => {
                let q = rounded.at(rounded.length() * i as f64 / last as f64);
                std::array::from_fn(|j| q[j])
            }
        })
        .collect();
    DMatrix::from_fn(POINTS, D, |i, j| rows[i][j])
}

/// The smoothness cost of a joint, MoveIt's `ChompCost`: the quadratic form of the
/// trajectory's weighed derivatives over all its points, and the inverse of its block over
/// the free points, each scaled by the inverse's largest entry, as `ChompOptimizer`
/// scales each joint's cost by the largest of any joint's.
struct SmoothnessCost {
    full: DMatrix<f64>,
    free_inverse: DMatrix<f64>,
}

impl SmoothnessCost {
    fn new() -> Self {
        let half = DIFF_RULE_LENGTH as i64 / 2;
        let full = DIFF_RULES
            .iter()
            .zip(DERIVATIVE_COSTS)
            .scan(1.0, |multiplier, (rule, weight)| {
                *multiplier *= DISCRETIZATION_S;
                let difference = DMatrix::from_fn(PADDED, PADDED, |i, j| {
                    let offset = j as i64 - i as i64;
                    if offset.abs() <= half {
                        rule[(offset + half) as usize]
                    } else {
                        0.0
                    }
                });
                Some((difference.transpose() * &difference) * (weight * *multiplier))
            })
            .fold(DMatrix::zeros(PADDED, PADDED), |sum, term| sum + term)
            + DMatrix::identity(PADDED, PADDED) * RIDGE_FACTOR;
        let free_inverse = full
            .view((FREE_START, FREE_START), (FREE, FREE))
            .into_owned()
            .try_inverse()
            .expect("the smoothness cost over the free points is invertible");
        let scale = free_inverse.max();
        Self {
            full: full * scale,
            free_inverse: free_inverse * (1.0 / scale),
        }
    }

    /// The derivative of the cost of `joint`, one joint's padded trajectory.
    fn derivative(&self, joint: &DVector<f64>) -> DVector<f64> {
        &self.full * (joint * 2.0)
    }

    fn cost(&self, joint: &DVector<f64>) -> f64 {
        joint.dot(&(&self.full * joint))
    }
}

/// One collision sphere of the arms: its arm, the joint whose child link carries it, and
/// its center in that link's frame.
struct Sphere {
    arm: usize,
    segment: usize,
    center: Point3<f64>,
    radius: f64,
}

/// A sphere at one point of the trajectory.
#[derive(Clone, Copy, Default)]
struct SphereAt {
    position: Vector3<f64>,
    potential: f64,
    /// The gradient of the field's distance at the sphere's center.
    gradient: Vector3<f64>,
    velocity: Vector3<f64>,
    acceleration: Vector3<f64>,
    speed: f64,
}

/// One CHOMP optimization of a trajectory of `arms`, `N` joints each and `D` in all.
struct Chomp<'a, const N: usize, const D: usize> {
    arms: &'a Arms<'a, N, D>,
    field: &'a DistanceField,
    limits: [Limit; D],
    /// The padded trajectory, one row per point and one column per joint.
    trajectory: DMatrix<f64>,
    smoothness: SmoothnessCost,
    spheres: Vec<Sphere>,
    /// Each sphere at each point of the padded trajectory.
    at: Vec<Vec<SphereAt>>,
}

impl<'a, const N: usize, const D: usize> Chomp<'a, N, D> {
    /// CHOMP on the trajectory along the path the robot runs through `path`, as [`refine`]
    /// starts, in `field`, each link's spheres grown by `padding`.
    fn of_path(
        arms: &'a Arms<'a, N, D>,
        field: &'a DistanceField,
        path: &[[f64; D]],
        padding: f64,
    ) -> Self {
        let points = along_the_path_run(path);
        let trajectory = DMatrix::from_fn(PADDED, D, |i, j| {
            points[(i.saturating_sub(PAD).min(POINTS - 1), j)]
        });
        let spheres = spheres_of(arms, padding);
        let at = vec![vec![SphereAt::default(); spheres.len()]; PADDED];
        let mut chomp = Self {
            arms,
            field,
            limits: arms.limits(),
            trajectory,
            smoothness: SmoothnessCost::new(),
            spheres,
            at,
        };
        chomp.place_spheres(0..=PADDED - 1);
        chomp
    }

    fn point(&self, i: usize) -> [f64; D] {
        std::array::from_fn(|j| self.trajectory[(i, j)])
    }

    /// The trajectory's cost, its smoothness cost and its collision cost, weighed, with the
    /// spheres where the last placement put them.
    fn cost(&self) -> f64 {
        self.collision_cost() + self.smoothness_cost()
    }

    /// The waypoints of the trajectory, from the start to the goal, each free point held
    /// inside the joint limits, past which the limit pull leaves up to `LIMIT_SLACK_RAD`, and
    /// what its last pull leaves.
    fn waypoints(&self) -> Vec<[f64; D]> {
        (PAD..PAD + POINTS)
            .map(|i| {
                let q = self.point(i);
                if (FREE_START..=FREE_END).contains(&i) {
                    std::array::from_fn(|j| q[j].clamp(self.limits[j].lo, self.limits[j].hi))
                } else {
                    q
                }
            })
            .collect()
    }

    /// Each arm posed at point `i` of the trajectory, in the group's order.
    fn posed(&self, i: usize) -> Vec<Posed<'_, N>> {
        let q = self.point(i);
        self.arms
            .arms()
            .iter()
            .enumerate()
            .map(|(k, arm)| arm.geometry().chain().at(&Arms::<N, D>::slice(&q, k)))
            .collect()
    }

    /// Whether every value of the trajectory is a number.
    fn is_finite(&self) -> bool {
        self.trajectory.iter().all(|x| x.is_finite())
    }

    /// One iteration's step: the smoothness increments of every free point and the
    /// collision increments of one free point drawn from `draws`, projected through the
    /// inverse of the smoothness cost, each joint's step scaled to move it at most
    /// `JOINT_UPDATE_LIMIT_RAD`; then each joint pulled back inside its limits, and the
    /// spheres placed at the free points.
    fn step(&mut self, draws: &mut Draws) {
        let mut increments = DMatrix::zeros(FREE, D);
        for j in 0..D {
            let derivative = self
                .smoothness
                .derivative(&self.trajectory.column(j).into_owned());
            increments.set_column(
                j,
                &(-derivative.rows(FREE_START, FREE) * SMOOTHNESS_COST_WEIGHT),
            );
        }
        let drawn = (draws.unit() * (FREE_END - FREE_START) as f64 + FREE_START as f64) as usize;
        let i = drawn.clamp(FREE_START, FREE_END);
        let collision = self.collision_increments(i);
        for j in 0..D {
            increments[(i - FREE_START, j)] += collision[j] * OBSTACLE_COST_WEIGHT;
        }
        for j in 0..D {
            let step = &self.smoothness.free_inverse * increments.column(j) * LEARNING_RATE;
            let scale = 1.0_f64
                .min(JOINT_UPDATE_LIMIT_RAD / step.max().abs())
                .min(JOINT_UPDATE_LIMIT_RAD / step.min().abs());
            let mut free = self.trajectory.view_mut((FREE_START, j), (FREE, 1));
            free += step * scale;
        }
        self.hold_to_limits();
        self.place_spheres(FREE_START..=FREE_END);
    }

    /// Places each sphere at the points of `points`, with its potential and the field's
    /// gradient, as `performForwardKinematics` does, and then gives each free point's
    /// spheres their velocities and accelerations by finite differences.
    fn place_spheres(&mut self, points: RangeInclusive<usize>) {
        let cap = PositiveFinite::parse(PROPAGATION_DISTANCE_M).expect("a positive cap");
        for i in points {
            let posed = self.posed(i);
            let placed: Vec<SphereAt> = self
                .spheres
                .iter()
                .zip(&self.at[i])
                .map(|(sphere, at)| {
                    let position =
                        posed[sphere.arm].link_pose_world(sphere.segment) * sphere.center;
                    let near = self
                        .field
                        .gradient(position.coords.into(), cap)
                        .ok()
                        .flatten()
                        .filter(|near| near.distance < cap.get());
                    SphereAt {
                        position: position.coords,
                        potential: near.map_or(0.0, |near| potential(near.distance, sphere.radius)),
                        gradient: near
                            .map_or(Vector3::zeros(), |near| Vector3::from(near.gradient)),
                        ..*at
                    }
                })
                .collect();
            self.at[i] = placed;
        }
        let inverse_time = 1.0 / DISCRETIZATION_S;
        let half = DIFF_RULE_LENGTH / 2;
        for i in FREE_START..=FREE_END {
            for j in 0..self.spheres.len() {
                let (velocity, acceleration) = (0..DIFF_RULE_LENGTH).fold(
                    (Vector3::zeros(), Vector3::zeros()),
                    |(velocity, acceleration), k| {
                        let position = self.at[i + k - half][j].position;
                        (
                            velocity + position * (inverse_time * DIFF_RULES[0][k]),
                            acceleration
                                + position * (inverse_time * inverse_time * DIFF_RULES[1][k]),
                        )
                    },
                );
                let sphere = &mut self.at[i][j];
                sphere.velocity = velocity;
                sphere.acceleration = acceleration;
                sphere.speed = velocity.norm();
            }
        }
    }

    fn smoothness_cost(&self) -> f64 {
        let total: f64 = (0..D)
            .map(|j| {
                self.smoothness
                    .cost(&self.trajectory.column(j).into_owned())
            })
            .sum();
        SMOOTHNESS_COST_WEIGHT * total
    }

    fn collision_cost(&self) -> f64 {
        let total: f64 = (FREE_START..=FREE_END)
            .map(|i| {
                self.at[i]
                    .iter()
                    .map(|sphere| sphere.potential * sphere.speed)
                    .sum::<f64>()
            })
            .sum();
        OBSTACLE_COST_WEIGHT * total
    }

    /// The collision increment of each joint at free point `i`: for each sphere with a
    /// potential and a speed there, CHOMP's Cartesian gradient of its cost, taken back
    /// through the transpose of the sphere's Jacobian, which moves only the joints of its
    /// arm from the base to its link.
    fn collision_increments(&self, i: usize) -> [f64; D] {
        let posed = self.posed(i);
        self.spheres
            .iter()
            .zip(&self.at[i])
            .filter(|(_, at)| at.potential >= LEAST_POTENTIAL && at.speed > 0.0)
            .fold([0.0; D], |increments, (sphere, at)| {
                let normalized = at.velocity / at.speed;
                let projector = Matrix3::identity() - normalized * normalized.transpose();
                let curvature = projector * at.acceleration / (at.speed * at.speed);
                let cartesian = at.speed * (projector * -at.gradient - at.potential * curvature);
                let columns = posed[sphere.arm]
                    .point_world_jacobian(&Point3::from(at.position), sphere.segment);
                std::array::from_fn(|k| {
                    let (arm, joint) = (k / N, k % N);
                    if arm == sphere.arm {
                        increments[k] - columns[joint].dot(&cartesian)
                    } else {
                        increments[k]
                    }
                })
            })
    }

    /// Pulls each joint back inside its limits, as `handleJointLimits` does: the free point
    /// farthest past a limit, by more than `LIMIT_SLACK_RAD`, moves onto it along its
    /// column of the smoothness cost's inverse, which moves the points around it smoothly
    /// with it, `LIMIT_PULLS` times at most.
    fn hold_to_limits(&mut self) {
        for (j, limit) in self.limits.iter().enumerate() {
            for _ in 0..LIMIT_PULLS {
                let worst = (FREE_START..=FREE_END)
                    .map(|i| (i, beyond(self.trajectory[(i, j)], limit)))
                    .filter(|(_, amount)| amount.abs() > LIMIT_SLACK_RAD)
                    .fold(
                        None,
                        |worst: Option<(usize, f64)>, (i, amount)| match worst {
                            Some((_, most)) if most.abs() >= amount.abs() => worst,
                            _ => Some((i, amount)),
                        },
                    );
                let Some((i, amount)) = worst else {
                    break;
                };
                let k = i - FREE_START;
                let column = self.smoothness.free_inverse.column(k)
                    * (amount / self.smoothness.free_inverse[(k, k)]);
                let mut free = self.trajectory.view_mut((FREE_START, j), (FREE, 1));
                free += column;
            }
        }
    }
}

/// CHOMP's field of `scene` when it holds an obstacle: the map's distances, with the
/// voxels each obstacle's shapes touch occupied (`VoxelMap::occupy_intersecting`), every
/// object that stands in the world and every object an arm the plan does not move holds, as
/// MoveIt's CHOMP holds every object of its world in one field and every object a link
/// outside the group holds in another (collision_env_distance_field.cpp:908-930), allowed
/// pairs included. None when the scene holds no obstacle: CHOMP's field is then the map's
/// own. The robot's own links stay out of it, where MoveIt's second field holds every link
/// outside the group.
pub(crate) fn field_with_obstacles(scene: &Scene) -> Option<DistanceField> {
    let shapes: Vec<_> = scene
        .objects
        .obstacles
        .iter()
        .flat_map(|obstacle| obstacle.shapes())
        .collect();
    if shapes.is_empty() {
        return None;
    }
    let mut map = scene.map.clone();
    for shape in shapes {
        map.occupy_intersecting(shape);
    }
    Some(DistanceField::of(&map))
}

/// The collision spheres of `arms` grown by `padding`: those of each piece of each link,
/// and of each piece of each part its links carry, such as a gripper finger at its
/// opening, as MoveIt places spheres on every link the arms' joints move. An object a link
/// carries has none: MoveIt's field gradient reads the links alone
/// (collision_env_distance_field.cpp:1649).
fn spheres_of<const N: usize, const D: usize>(arms: &Arms<N, D>, padding: f64) -> Vec<Sphere> {
    let placed = move |arm: usize, segment: usize, frame: Isometry3<f64>, source: &SphereSource| {
        source
            .spheres(padding)
            .into_iter()
            .map(move |sphere| Sphere {
                arm,
                segment,
                center: frame * sphere.center,
                radius: sphere.radius,
            })
    };
    arms.arms()
        .iter()
        .enumerate()
        .flat_map(|(arm, a)| {
            let links = a.geometry().links().iter().flat_map(move |link| {
                link.pieces().iter().flat_map(move |piece| {
                    placed(arm, link.segment(), *link.offset(), piece.sphere_source())
                })
            });
            let parts = a
                .riders()
                .iter()
                .filter_map(|rider| match rider.body {
                    RiderBody::Part(part) => Some((rider, part)),
                    RiderBody::Shape(_) => None,
                })
                .flat_map(move |(rider, part)| {
                    part.sphere_sources()
                        .iter()
                        .flat_map(move |source| placed(arm, rider.segment, rider.offset, source))
                });
            links.chain(parts)
        })
        .collect()
}

/// How far `x` must move to come back inside `limit`: 0 inside it.
fn beyond(x: f64, limit: &Limit) -> f64 {
    if x > limit.hi {
        limit.hi - x
    } else if x < limit.lo {
        limit.lo - x
    } else {
        0.0
    }
}

/// MoveIt's potential (`ChompOptimizer::getPotential`) of a sphere of `radius` whose
/// center stands `distance` from the nearest obstacle, `d` the distance less the radius: 0
/// at a clearance of `POTENTIAL_CLEARANCE_M` and beyond, `(d - clearance)^2 / (2
/// clearance)` within it, and `-d + clearance / 2` inside.
fn potential(distance: f64, radius: f64) -> f64 {
    let d = distance - radius;
    if d >= POTENTIAL_CLEARANCE_M {
        return 0.0;
    }
    if d < 0.0 {
        return -d + 0.5 * POTENTIAL_CLEARANCE_M;
    }
    let difference = d - POTENTIAL_CLEARANCE_M;
    0.5 * (difference / POTENTIAL_CLEARANCE_M) * difference
}

#[cfg(test)]
mod tests {
    use control_core::path_timing::JOINT_PATH_CORNER_DEVIATION;
    use occupancy_map::VoxelMap;

    use super::*;
    use crate::plan::tests::{gantry, one_arm, two_links, world};
    use crate::scene::{
        AttachRequest, ObjectFields, ObjectRequest, Objects, PlanningScene, RobotLinks, ShapeFields,
    };

    // MoveIt's potential of a sphere: 0 at the clearance and beyond, half the squared
    // shortfall over the clearance within it, and the depth plus half the clearance inside
    // an obstacle, each of the distance less the radius.
    #[test]
    fn the_potential_has_moveits_three_pieces() {
        assert_eq!(potential(0.2, 0.0), 0.0);
        assert_eq!(potential(0.5, 0.25), 0.0);
        assert!((potential(0.1, 0.0) - 0.025).abs() < 1e-15);
        assert!((potential(0.25, 0.125) - 0.0140625).abs() < 1e-15);
        assert!((potential(-0.05, 0.0) - 0.15).abs() < 1e-15);
        assert!((potential(0.0625, 0.125) - 0.1625).abs() < 1e-15);
    }

    // The padded trajectory has MoveIt's indices: 101 points padded by 5 at each end, its
    // free points from 6 to 104.
    #[test]
    fn the_padded_trajectory_has_moveits_indices() {
        assert_eq!((PADDED, FREE_START, FREE_END, FREE), (111, 6, 104, 99));
    }

    // The inverse of the smoothness cost over the free points, scaled, peaks at 1, and
    // inverts the free block of the cost scaled the other way.
    #[test]
    fn the_inverse_of_the_free_smoothness_cost_peaks_at_one() {
        let cost = SmoothnessCost::new();
        assert!((cost.free_inverse.max() - 1.0).abs() < 1e-12);
        let block = cost
            .full
            .view((FREE_START, FREE_START), (FREE, FREE))
            .into_owned();
        let identity = block * &cost.free_inverse;
        assert!((identity - DMatrix::identity(FREE, FREE)).amax() < 1e-6);
    }

    // The points lie evenly along the path the robot runs: of a path that turns a right
    // angle halfway, the first and the last are its ends, the middle one is the midpoint of
    // the rounded corner, the corner deviation from the corner, and each point lies at its
    // share of the rounded path's length.
    #[test]
    fn the_points_lie_evenly_along_the_path_the_robot_runs() {
        let path = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]];
        let points = along_the_path_run(&path);
        assert_eq!(points.shape(), (POINTS, 2));
        let point = |i: usize| [points[(i, 0)], points[(i, 1)]];
        assert_eq!((point(0), point(POINTS - 1)), (path[0], path[2]));
        let [x, y] = point(POINTS / 2);
        let from_corner = ((x - 1.0).powi(2) + y.powi(2)).sqrt();
        assert!(
            (from_corner - JOINT_PATH_CORNER_DEVIATION).abs() < 1e-9,
            "{from_corner}"
        );
        let rounded = RoundedPath::joint_path(&path.map(|q| q.to_vec()));
        for i in [1, 25, 75, POINTS - 2] {
            let q = rounded.at(rounded.length() * i as f64 / (POINTS - 1) as f64);
            assert_eq!(point(i), [q[0], q[1]]);
        }
    }

    /// The gantry's world: 1 cm voxels 1.2 m across, `voxels` occupied.
    fn gantry_world(voxels: &[[f64; 3]]) -> (VoxelMap, DistanceField) {
        world(0.01, [-0.6, -0.6, -0.1], [0.6, 0.6, 0.1], voxels)
    }

    // Steps of descent push a straight path away from a voxel 6 cm beside its middle: its
    // collision cost falls, and its middle moves away from the voxel.
    #[test]
    fn descent_moves_the_path_away_from_an_obstacle() {
        let (_, field) = gantry_world(&[[0.0, 0.06, 0.0]]);
        let geometry = gantry();
        let arms = one_arm(&geometry, None);
        let mut chomp = Chomp::of_path(&arms, &field, &[[-0.3, 0.0], [0.3, 0.0]], 0.02);
        let middle = PAD + POINTS / 2;
        let cost = chomp.collision_cost();
        assert!(
            cost > 0.0,
            "the voxel lies within the potential's clearance"
        );
        let mut draws = Draws::new(7);
        for _ in 0..50 {
            chomp.step(&mut draws);
        }
        assert!(
            chomp.collision_cost() < cost,
            "{} against {cost}",
            chomp.collision_cost()
        );
        assert!(chomp.point(middle)[1] < 0.0, "{:?}", chomp.point(middle));
    }

    // Steps of descent push a path of two turning joints away from a voxel 6 cm beside the
    // second link: the spheres move by the turning joints' Jacobian, and the collision cost
    // falls.
    #[test]
    fn descent_moves_a_turning_arm_away_from_an_obstacle() {
        let (_, field) = gantry_world(&[[0.45, 0.06, 0.0]]);
        let geometry = two_links();
        let arms = one_arm(&geometry, None);
        let mut chomp = Chomp::of_path(&arms, &field, &[[-0.3, 0.0], [0.3, 0.0]], 0.02);
        let cost = chomp.collision_cost();
        assert!(
            cost > 0.0,
            "the voxel lies within the potential's clearance"
        );
        let mut draws = Draws::new(7);
        for _ in 0..50 {
            chomp.step(&mut draws);
        }
        assert!(
            chomp.collision_cost() < cost,
            "{} against {cost}",
            chomp.collision_cost()
        );
    }

    // The middle free point past a joint's upper limit comes back within the slack, the
    // points around it move by less than it moves, and the fixed points stay: the pull moves
    // the free points along a column of the smoothness cost's inverse, which peaks at the
    // middle point's own place for the middle point.
    #[test]
    fn a_joint_past_its_limit_comes_back_within_the_slack() {
        let (_, field) = gantry_world(&[]);
        let geometry = gantry();
        let arms = one_arm(&geometry, None);
        let mut chomp = Chomp::of_path(&arms, &field, &[[0.0, 0.0], [0.9, 0.0]], 0.02);
        let row = FREE_START + FREE / 2;
        chomp.trajectory[(row, 0)] = 1.2;
        let before = chomp.trajectory.clone();
        chomp.hold_to_limits();
        let moved = |i: usize| (chomp.trajectory[(i, 0)] - before[(i, 0)]).abs();
        assert!((chomp.trajectory[(row, 0)] - 1.0).abs() <= LIMIT_SLACK_RAD);
        assert!(moved(row - 1) < moved(row) && moved(row + 1) < moved(row));
        for i in (0..FREE_START).chain(FREE_END + 1..PADDED) {
            assert_eq!(moved(i), 0.0, "fixed point {i}");
        }
    }

    // The waypoints hold each free point inside the joint limits, whatever the trajectory
    // holds.
    #[test]
    fn the_waypoints_hold_each_free_point_inside_the_limits() {
        let (_, field) = gantry_world(&[]);
        let geometry = gantry();
        let arms = one_arm(&geometry, None);
        let mut chomp = Chomp::of_path(&arms, &field, &[[0.0, 0.0], [0.9, 0.0]], 0.02);
        chomp.trajectory[(FREE_START + 10, 0)] = 1.5;
        let waypoints = chomp.waypoints();
        assert_eq!(waypoints[FREE_START + 10 - PAD][0], 1.0);
        assert!(
            waypoints
                .iter()
                .all(|q| q.iter().all(|x| (-1.0..=1.0).contains(x)))
        );
    }

    // CHOMP's field holds every obstacle of the plan: an object standing in the world and an
    // object an arm the plan does not move holds read 0 at their centers, where the map alone
    // reads farther. A scene with no obstacle gives no field of its own.
    #[test]
    fn the_field_holds_standing_and_held_objects() {
        let (mut map, field) = gantry_world(&[]);
        let hands = ["left_hand", "right_hand"].map(String::from);
        let mut planning = PlanningScene::new(RobotLinks::new(hands.clone(), hands));
        let object = |id: &str, position: [f64; 3]| ObjectRequest {
            operation: "add".to_string(),
            object: ObjectFields {
                id: id.to_string(),
                position,
                orientation: [0.0, 0.0, 0.0, 1.0],
                shapes: vec![ShapeFields {
                    kind: "box".to_string(),
                    dimensions: vec![0.05; 3],
                    mesh_vertices: Vec::new(),
                    mesh_triangles: Vec::new(),
                    position: [0.0; 3],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                }],
            },
        };
        let (standing, held) = ([0.3, 0.0, 0.0], [-0.3, 0.0, 0.0]);
        planning
            .apply_collision_objects(&[object("box", standing), object("cup", held)], &mut map)
            .expect("the objects apply");
        let hand = Isometry3::translation(held[0], held[1], held[2]);
        planning
            .apply_attached_collision_object(
                &AttachRequest {
                    object_id: "cup".to_string(),
                    link_name: "right_hand".to_string(),
                    touch_links: Vec::new(),
                    operation: "attach".to_string(),
                },
                |link| (link == "right_hand").then_some(hand),
                &mut map,
            )
            .expect("the cup attaches");
        let objects = planning.for_arms(&["left_hand"], |_| hand);
        let scene = Scene {
            map: &map,
            field: &field,
            stamp: 0,
            objects: objects.objects(&[]),
        };
        let with_obstacles = field_with_obstacles(&scene).expect("the scene holds obstacles");
        for at in [standing, held] {
            assert_eq!(with_obstacles.distance(at), Ok(0.0), "{at:?}");
            assert!(field.distance(at).expect("a point") > 0.1, "{at:?}");
        }
        let empty = Scene {
            objects: Objects::none(),
            ..scene
        };
        assert!(field_with_obstacles(&empty).is_none());
    }
}
