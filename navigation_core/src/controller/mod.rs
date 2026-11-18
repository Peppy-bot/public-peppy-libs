//! A regulated pure pursuit controller, ported from Nav2 1.5.2
//! (nav2_regulated_pure_pursuit_controller
//! src/regulated_pure_pursuit_controller.cpp: `computeVelocityCommands`,
//! `getLookAheadDistance`, `calculateCurvature`,
//! `shouldRotateToGoalHeading`, `shouldRotateToPath`, `rotateToHeading` and
//! `applyConstraints`; the regulations of
//! include/nav2_regulated_pure_pursuit_controller/regulation_functions.hpp;
//! and `CollisionChecker::isCollisionImminent` of src/collision_checker.cpp),
//! with rotation to heading, curvature and cost regulation and collision
//! detection on, and reversing, the fixed curvature lookahead, the dynamic
//! window, the cancel deceleration (`use_cancel_deceleration`) and the
//! obstacle check beyond the goal (`allow_obstacle_checking_beyond_goal`)
//! off: a base comes to rest through its limiter.
//!
//! Each [`PurePursuit::compute`] takes the base's pose, its speed, the goal
//! checker and the plan the path handler pruned, which carries the whole
//! plan's goal, as the controller server hands the goal to its controller
//! (nav2_controller src/controller_server.cpp:699, 755), all in the frame of
//! the cost grid. It finds the carrot on the plan at the lookahead
//! distance and the curvature of the arc to it. Once the goal checker finds
//! the goal's position reached, the base turns in place towards the last
//! heading of the plan; while the carrot lies more than
//! `rotate_to_heading_min_angle_rad` off the base's heading, it turns in
//! place towards the carrot. Otherwise the base drives the arc at
//! `max_linear_vel_m_s`, slowed by the arc's curvature below
//! `regulated_linear_scaling_min_radius_m`, by the cost under the base
//! within `cost_scaling_dist_m` of an obstacle, the distance read from the
//! cost through the grid's inflation scaling factor and inscribed radius, and
//! on the approach to the plan's end within `approach_velocity_scaling_dist_m`.
//! The footprint is checked along the arc of the command ahead of the base,
//! and a contact, or a vertex of the footprint off the grid, fails the command
//! with [`ControlError::NoValidControl`]. The command is a forward speed and a
//! turn rate alone, so a holonomic base drives as a differential one.

mod collision;
mod lookahead;

use camera_core::pose::PlanarPose;
use control_core::non_negative_finite::NonNegativeFinite;
use control_core::positive_finite::PositiveFinite;
use control_core::twist::Twist;

use crate::footprint::PaddedFootprint;
use crate::goal_checker::GoalChecker;
use crate::grid::{CostGrid, FREE, UNKNOWN};
use crate::inflation::Inflation;
use crate::path_handler::PrunedPlan;
use crate::pose::{heading, path_length, point_distance};
use collision::FootprintCheck;
use lookahead::lookahead_point;

/// The natural logarithm of 253 in single precision, widened, as
/// `costConstraint` adds it (`log(253.0f)`, which picks the float overload).
const LOG_253_SINGLE: f64 = 5.533389568328857;

/// The squared distance to the carrot, in square meters, at or below which
/// the curvature is 0 (`calculateCurvature`).
const MIN_CARROT_DISTANCE_SQ: f64 = 0.001;

/// The speeds below which the arc check takes a command for a turn in
/// place: a forward speed below 0.01 m/s with a turn rate above 0.01 rad/s.
const IN_PLACE_SPEED: f64 = 0.01;

/// How far ahead the carrot lies.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Lookahead {
    /// A fixed distance in meters (`lookahead_dist`, with
    /// `use_velocity_scaled_lookahead_dist` false).
    Fixed(PositiveFinite),
    /// The distance the base covers at its speed in a time, within bounds
    /// (`use_velocity_scaled_lookahead_dist` true).
    VelocityScaled(VelocityScaledLookahead),
}

/// A lookahead of the distance the base covers in `time_s` at its forward
/// speed, between `min_m` and `max_m`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityScaledLookahead {
    time_s: PositiveFinite,
    min_m: NonNegativeFinite,
    max_m: NonNegativeFinite,
}

/// Bounds of a lookahead that hold no distance.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
#[error(
    "min_lookahead_dist {min_m} m lies above max_lookahead_dist {max_m} m: give the shorter \
     distance as the minimum"
)]
pub struct InvertedLookahead {
    pub min_m: f64,
    pub max_m: f64,
}

impl VelocityScaledLookahead {
    /// `lookahead_time`, `min_lookahead_dist` and `max_lookahead_dist`.
    pub fn parse(
        time_s: PositiveFinite,
        min_m: NonNegativeFinite,
        max_m: NonNegativeFinite,
    ) -> Result<Self, InvertedLookahead> {
        if min_m.get() > max_m.get() {
            return Err(InvertedLookahead {
                min_m: min_m.get(),
                max_m: max_m.get(),
            });
        }
        Ok(Self {
            time_s,
            min_m,
            max_m,
        })
    }

    /// The longest lookahead, in meters (`max_lookahead_dist`).
    pub fn max_m(&self) -> f64 {
        self.max_m.get()
    }
}

/// The controller's settings, by Nav2's names.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PurePursuitParams {
    /// The forward speed of a straight arc far from obstacles, in m/s
    /// (`max_linear_vel`).
    pub max_linear_vel_m_s: PositiveFinite,
    pub lookahead: Lookahead,
    /// The turn rate of a turn in place, in rad/s
    /// (`rotate_to_heading_angular_vel`).
    pub rotate_to_heading_angular_vel_rad_s: PositiveFinite,
    /// The angle between the base's heading and the carrot, in radians,
    /// past which the base turns in place (`rotate_to_heading_min_angle`).
    pub rotate_to_heading_min_angle_rad: NonNegativeFinite,
    /// The angular acceleration of a turn in place, in rad/s²
    /// (`max_angular_accel`).
    pub max_angular_accel_rad_s2: PositiveFinite,
    /// The slowest forward speed on the approach to the plan's end, in m/s
    /// (`min_approach_linear_velocity`).
    pub min_approach_linear_velocity_m_s: PositiveFinite,
    /// The distance along the plan to its end, in meters, within which the
    /// base slows on its approach (`approach_velocity_scaling_dist`).
    pub approach_velocity_scaling_dist_m: NonNegativeFinite,
    /// The arc radius, in meters, below which the forward speed falls with
    /// the curvature (`regulated_linear_scaling_min_radius`).
    pub regulated_linear_scaling_min_radius_m: NonNegativeFinite,
    /// The slowest forward speed the curvature and cost regulations give,
    /// in m/s (`regulated_linear_scaling_min_speed`).
    pub regulated_linear_scaling_min_speed_m_s: NonNegativeFinite,
    /// The distance to an obstacle, in meters, within which the forward
    /// speed falls with the cost under the base (`cost_scaling_dist`).
    pub cost_scaling_dist_m: PositiveFinite,
    /// The gain of the cost regulation (`cost_scaling_gain`).
    pub cost_scaling_gain: NonNegativeFinite,
    /// How far ahead in time, in seconds, the footprint is checked along
    /// the arc (`max_allowed_time_to_collision_up_to_carrot`).
    pub max_allowed_time_to_collision_up_to_carrot_s: NonNegativeFinite,
    /// A distance, in meters, that lengthens the arc check, none to check up
    /// to the carrot within `max_allowed_time_to_collision_up_to_carrot_s`
    /// alone (`min_distance_to_obstacle`): the check runs for at least the
    /// time the command takes to cover it, at no less than
    /// `min_approach_linear_velocity_m_s`, and reaches no farther than the
    /// carrot with a fixed lookahead, where Nav2 warns that the lookahead
    /// caps it (parameter_handler.cpp:131-137); with a velocity scaled
    /// lookahead it reaches as far as the carrot or as this distance and the
    /// plan's end, whichever is nearer, when that lies farther, within the
    /// longest lookahead (collision_checker.cpp:93-113).
    pub min_distance_to_obstacle_m: Option<PositiveFinite>,
    /// The rate the controller runs at, in Hz (`controller_frequency`).
    pub controller_frequency_hz: PositiveFinite,
}

/// Why the controller gives no command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ControlError {
    /// The footprint meets a lethal cell, or a vertex of it lies off the cost
    /// grid with its center on it, where the base stands or along the arc of
    /// the command ahead of it (`NoValidControl`, on `isCollisionImminent`;
    /// nav2_costmap_2d src/footprint_collision_checker.cpp:55-56, 66-67).
    #[error(
        "the footprint meets a lethal cell, or a vertex of it lies off the cost grid, where the \
         base stands or along the commanded arc ahead of it"
    )]
    NoValidControl,
    /// The base stands off the cost grid (`ControllerException`).
    #[error("the base stands off the cost grid: give a grid that holds the base")]
    BaseOffGrid,
    /// A pose of the plan lies so far from the base that its position in the
    /// base's frame is not finite: the plan carried into the base's frame,
    /// whose failure Nav2's controller names `ControllerTFError`
    /// (regulated_pure_pursuit_controller.cpp:175-180), holds no pose there.
    #[error(
        "a pose of the plan lies so far from the base that its position in the base's frame is \
         not finite: give the base's pose and the plan in one frame"
    )]
    PlanNotInBaseFrame,
}

/// The controller.
#[derive(Debug)]
pub struct PurePursuit {
    params: PurePursuitParams,
    /// The padded footprint of the grid's inflation: the arc check reads it,
    /// and the cost under the base gives its distance to an obstacle through
    /// its inscribed radius.
    footprint: PaddedFootprint,
    /// The grid's inflation scaling factor, per meter, by which the cost
    /// under the base gives its distance to an obstacle
    /// (`inflation_cost_scaling_factor`); none for a factor of 0, which turns
    /// the cost regulation off, as Nav2 turns it off for a factor at or
    /// below 0.
    inflation_cost_scaling_factor: Option<PositiveFinite>,
    control_duration_s: f64,
}

impl PurePursuit {
    /// The controller on grids inflated by `inflation`: the arc check reads
    /// the padded footprint the inflation was made for, and the cost
    /// regulation reads that footprint's inscribed radius and the
    /// inflation's scaling factor. Nav2's controller reads its cost map's
    /// footprint (collision_checker.cpp:168-169) and the layered costmap's
    /// inscribed radius (regulation_functions.hpp:72), and holds its own
    /// `inflation_cost_scaling_factor` (parameter_handler.cpp:92-93), apart
    /// from the inflation layer's; the port reads all three from `inflation`,
    /// one value for each, so the cost under the base reads back as its
    /// distance to an obstacle on a grid that `inflation` inflated.
    pub fn new(params: PurePursuitParams, inflation: &Inflation) -> Self {
        Self {
            control_duration_s: 1.0 / params.controller_frequency_hz.get(),
            params,
            footprint: inflation.footprint().clone(),
            inflation_cost_scaling_factor: PositiveFinite::parse(
                inflation.cost_scaling_factor().get(),
            )
            .ok(),
        }
    }

    /// `computeVelocityCommands`: the command for a base at `pose` moving
    /// at `speed`, with `plan` ahead of it towards its goal, on `grid`.
    pub fn compute(
        &self,
        pose: PlanarPose,
        speed: Twist,
        goal_checker: &mut GoalChecker,
        plan: &PrunedPlan,
        grid: &CostGrid,
    ) -> Result<Twist, ControlError> {
        let check = FootprintCheck {
            grid,
            footprint: &self.footprint,
        };
        let local: Vec<PlanarPose> = plan
            .poses()
            .iter()
            .map(|p| p.seen_from(pose))
            .collect::<Result<_, _>>()
            .map_err(|_| ControlError::PlanNotInBaseFrame)?;
        let carrot = lookahead_point(self.lookahead_distance(speed), &local);
        let curvature = curvature(carrot);
        let (linear, angular) = if goal_checker.is_goal_xy_reached(pose, plan) {
            let angle_to_goal = local.last().expect("a pruned plan holds a pose").yaw();
            (0.0, self.rotate_to_heading(angle_to_goal, speed))
        } else if let Some(angle_to_path) = self.angle_to_rotate_to_path(carrot) {
            (0.0, self.rotate_to_heading(angle_to_path, speed))
        } else {
            let pose_cost = check
                .cost_at_pose(pose.x(), pose.y())
                .ok_or(ControlError::BaseOffGrid)?;
            let linear = self.regulated_linear_velocity(curvature, pose_cost, &local);
            (linear, linear * curvature)
        };
        let carrot_dist = carrot[0].hypot(carrot[1]);
        let arc = CommandArc {
            linear,
            angular,
            carrot_dist,
            dist_to_path_end: path_length(&local),
        };
        if self.collision_imminent(&check, pose, arc) {
            return Err(ControlError::NoValidControl);
        }
        Ok(Twist::new(linear, 0.0, angular).expect("a regulated command is finite"))
    }

    /// `getLookAheadDistance`.
    fn lookahead_distance(&self, speed: Twist) -> f64 {
        match self.params.lookahead {
            Lookahead::Fixed(distance) => distance.get(),
            Lookahead::VelocityScaled(scaled) => (speed.linear_x().abs() * scaled.time_s.get())
                .clamp(scaled.min_m.get(), scaled.max_m.get()),
        }
    }

    /// `shouldRotateToPath`: the angle from the base's heading to the
    /// carrot, when it is wide enough to turn in place.
    fn angle_to_rotate_to_path(&self, carrot: [f64; 2]) -> Option<f64> {
        let angle_to_path = carrot[1].atan2(carrot[0]);
        (angle_to_path.abs() > self.params.rotate_to_heading_min_angle_rad.get())
            .then_some(angle_to_path)
    }

    /// `rotateToHeading`: the turn rate of a turn in place by
    /// `angle_to_path`, within the angular acceleration of one control
    /// period from the current turn rate, and slow enough to stop at the
    /// angle.
    fn rotate_to_heading(&self, angle_to_path: f64, speed: Twist) -> f64 {
        let sign = if angle_to_path > 0.0 { 1.0 } else { -1.0 };
        let accel = self.params.max_angular_accel_rad_s2.get();
        let dt = self.control_duration_s;
        let min_feasible_angular_speed = speed.angular_z() - accel * dt;
        let max_feasible_angular_speed = speed.angular_z() + accel * dt;
        let angular_vel = (sign * self.params.rotate_to_heading_angular_vel_rad_s.get())
            .clamp(min_feasible_angular_speed, max_feasible_angular_speed);
        let max_vel_to_stop = (2.0 * accel * angle_to_path.abs()).sqrt();
        if angular_vel.abs() > max_vel_to_stop {
            sign * max_vel_to_stop
        } else {
            angular_vel
        }
    }

    /// `applyConstraints`: the forward speed of an arc of `curvature`, the
    /// base on a cell of `pose_cost`, with `local` ahead of it in its frame.
    fn regulated_linear_velocity(
        &self,
        curvature: f64,
        pose_cost: f64,
        local: &[PlanarPose],
    ) -> f64 {
        let max_linear_vel = self.params.max_linear_vel_m_s.get();
        let curvature_vel = curvature_constraint(
            max_linear_vel,
            curvature,
            self.params.regulated_linear_scaling_min_radius_m.get(),
        );
        let cost_vel = self.cost_constraint(max_linear_vel, pose_cost);
        let linear_vel = cost_vel
            .min(curvature_vel)
            .max(self.params.regulated_linear_scaling_min_speed_m_s.get());
        let linear_vel = self.approach_velocity_constraint(linear_vel, local);
        linear_vel.abs().clamp(0.0, max_linear_vel)
    }

    /// `costConstraint`: the forward speed, slowed by the distance to an
    /// obstacle the inflated cost under the base gives.
    fn cost_constraint(&self, raw_linear_vel: f64, pose_cost: f64) -> f64 {
        let Some(scaling_factor) = self.inflation_cost_scaling_factor else {
            return raw_linear_vel;
        };
        if pose_cost == f64::from(UNKNOWN) || pose_cost == f64::from(FREE) {
            return raw_linear_vel;
        }
        let scaling_factor = scaling_factor.get();
        let inscribed_radius = self.footprint.inscribed_radius_m();
        let min_distance_to_obstacle =
            (scaling_factor * inscribed_radius - pose_cost.ln() + LOG_253_SINGLE) / scaling_factor;
        let cost_scaling_dist = self.params.cost_scaling_dist_m.get();
        if min_distance_to_obstacle < cost_scaling_dist {
            return raw_linear_vel
                * (self.params.cost_scaling_gain.get() * min_distance_to_obstacle
                    / cost_scaling_dist);
        }
        raw_linear_vel
    }

    /// `approachVelocityConstraint`: the forward speed, slowed in
    /// proportion to the straight distance to the plan's end once the plan
    /// ahead is shorter than the approach distance, to no less than the
    /// minimum approach speed.
    fn approach_velocity_constraint(
        &self,
        constrained_linear_vel: f64,
        local: &[PlanarPose],
    ) -> f64 {
        let scaling_dist = self.params.approach_velocity_scaling_dist_m.get();
        let velocity_scaling = if path_length(local) < scaling_dist {
            let last = local.last().expect("a pruned plan holds a pose");
            last.x().hypot(last.y()) / scaling_dist
        } else {
            1.0
        };
        let min_approach_velocity = self.params.min_approach_linear_velocity_m_s.get();
        let approach_vel = (constrained_linear_vel * velocity_scaling).max(min_approach_velocity);
        constrained_linear_vel.min(approach_vel)
    }

    /// `isCollisionImminent`: whether the footprint touches an obstacle at
    /// the base's pose or at a pose along `arc` ahead of it.
    fn collision_imminent(
        &self,
        check: &FootprintCheck<'_>,
        pose: PlanarPose,
        arc: CommandArc,
    ) -> bool {
        let yaw = heading(pose.yaw());
        if check.in_collision(pose.x(), pose.y(), yaw) {
            return true;
        }
        let resolution = check.grid.geometry().resolution_m();
        let projection_time =
            if arc.linear.abs() < IN_PLACE_SPEED && arc.angular.abs() > IN_PLACE_SPEED {
                let max_radius = self.footprint.circumscribed_radius_m();
                2.0 * ((resolution / 2.0) / max_radius).sin() / arc.angular.abs()
            } else {
                resolution / arc.linear.abs()
            };
        let (max_time, simulation_distance_limit) = self.simulation_limits(arc);
        let (mut x, mut y, mut read_yaw) = (pose.x(), pose.y(), yaw);
        let mut i: i64 = 1;
        while i as f64 * projection_time < max_time {
            i += 1;
            let mut theta = read_yaw;
            x += projection_time * (arc.linear * theta.cos());
            y += projection_time * (arc.linear * theta.sin());
            theta += projection_time * arc.angular;
            read_yaw = heading(theta);
            if point_distance([x, y], [pose.x(), pose.y()]) > simulation_distance_limit {
                break;
            }
            if check.in_collision(x, y, theta) {
                return true;
            }
        }
        false
    }

    /// The time ahead and the distance from the base within which the arc
    /// check runs: up to the carrot, and, with a minimum distance to
    /// obstacles, at least the time to cover it, and with a velocity
    /// scaled lookahead at least that distance or the plan's end, within
    /// the longest lookahead.
    fn simulation_limits(&self, arc: CommandArc) -> (f64, f64) {
        let max_allowed = self
            .params
            .max_allowed_time_to_collision_up_to_carrot_s
            .get();
        let Some(min_distance) = self.params.min_distance_to_obstacle_m else {
            return (max_allowed, arc.carrot_dist);
        };
        let min_distance = min_distance.get();
        let max_time = max_allowed.max(
            min_distance
                / arc
                    .linear
                    .abs()
                    .max(self.params.min_approach_linear_velocity_m_s.get()),
        );
        let Lookahead::VelocityScaled(scaled) = self.params.lookahead else {
            return (max_time, arc.carrot_dist);
        };
        let effective_min_dist = min_distance.min(arc.dist_to_path_end);
        let base_simulation_dist = arc.carrot_dist.max(effective_min_dist);
        (max_time, base_simulation_dist.min(scaled.max_m.get()))
    }
}

/// A command and what bounds its arc check.
#[derive(Debug, Clone, Copy)]
struct CommandArc {
    linear: f64,
    angular: f64,
    carrot_dist: f64,
    dist_to_path_end: f64,
}

/// `calculateCurvature`: the curvature of the arc from the base through the
/// carrot, in the base's frame.
fn curvature([x, y]: [f64; 2]) -> f64 {
    let carrot_dist2 = x * x + y * y;
    if carrot_dist2 > MIN_CARROT_DISTANCE_SQ {
        2.0 * y / carrot_dist2
    } else {
        0.0
    }
}

/// `curvatureConstraint`: the forward speed, slowed in proportion to how
/// far the arc's radius falls below `min_radius`.
fn curvature_constraint(raw_linear_vel: f64, curvature: f64, min_radius: f64) -> f64 {
    let radius = (1.0 / curvature).abs();
    if radius < min_radius {
        return raw_linear_vel * (1.0 - ((radius - min_radius).abs() / min_radius));
    }
    raw_linear_vel
}

#[cfg(test)]
mod tests {
    use super::*;

    // The constant is the single precision logarithm of 253 that the
    // compiler folds `log(253.0f)` to, widened: bits 0x40162230e0000000.
    #[test]
    fn the_logarithm_of_253_is_the_single_precision_one() {
        assert_eq!(LOG_253_SINGLE.to_bits(), 0x4016_2230_e000_0000);
        assert_eq!(LOG_253_SINGLE, f64::from(253.0_f64.ln() as f32));
    }

    // A carrot on the base's axis draws a straight arc, one beside it a
    // circle through both, and one at the base none.
    #[test]
    fn the_curvature_is_that_of_the_circle_through_the_carrot() {
        assert_eq!(curvature([1.0, 0.0]), 0.0);
        assert_eq!(curvature([1.0, 1.0]), 1.0);
        assert_eq!(curvature([0.01, 0.01]), 0.0);
    }

    // Below the minimum radius the speed falls in proportion; at or above
    // it, and on a straight arc, it stands.
    #[test]
    fn the_curvature_constraint_scales_below_the_minimum_radius() {
        assert_eq!(curvature_constraint(1.0, 2.0, 1.0), 0.5);
        assert_eq!(curvature_constraint(1.0, 1.0, 1.0), 1.0);
        assert_eq!(curvature_constraint(1.0, 0.0, 1.0), 1.0);
    }
}
