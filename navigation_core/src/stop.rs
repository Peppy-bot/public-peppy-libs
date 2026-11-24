//! A stop before obstacles, as Nav2 1.5.2's collision monitor stops a base in
//! its approach mode, with the base's padded footprint as its polygon. Where
//! the base stands and at each step of the projection, the stop makes two
//! checks: each source's newest points against the footprint, as the monitor
//! checks its sources' points, and the cost grid under the footprint's
//! outline, as Nav2's controller checks a pose. [`Occupied`] holds what both
//! read, as
//! [`LiveObstacles::occupied_seen_from`](crate::obstacles::LiveObstacles::occupied_seen_from)
//! gives it.
//!
//! The points are those of each source's newest frame, before the clearing
//! under the footprint, in the base's frame: each return whose height lies in
//! the band, as the monitor's point cloud source keeps a point
//! (nav2_collision_monitor src/pointcloud.cpp:174), and the center of each
//! column the frame shows a drop in, which the monitor has no counterpart of.
//! The stop checks them as `CollisionMonitor::process` reads its sources
//! (src/collision_monitor_node.cpp:449-465) and `processApproach` slows the
//! base (:608-638), by `Polygon::getCollisionTime` (src/polygon.cpp:339-388)
//! and `getPointsInside` (:293-319), with `transformPoints` and `projectState`
//! (src/kinematics.cpp:22-68), `Source::sourceValid` (src/source.cpp:179-196)
//! and nav2_util's `isPointInsidePolygon`
//! (include/nav2_util/geometry_utils.hpp:200-233). A return inside the base's
//! unpadded outline counts as the robot's own body and gives no point, where
//! the monitor reads every point: a real obstacle there does not hold the
//! base, where the monitor stops the base for it. A body whose surface lies on
//! the outline scatters returns across its edge, and those outside it hold the
//! base, as they hold the monitor.
//!
//! The grid holds what the sensors have seen, the obstacles and drops out of
//! the newest frames' view among them. The stop reads it at each pose by
//! `FootprintCollisionChecker::footprintCostAtPose` (nav2_costmap_2d
//! src/footprint_collision_checker.cpp:129-144), which Nav2's regulated pure
//! pursuit reads a pose by (nav2_regulated_pure_pursuit_controller
//! src/collision_checker.cpp:168-177): the footprint's vertices placed at the
//! pose, and the cells of each edge read by `lineCost` (:87-107) in
//! `footprintCost` (:48-85), a vertex off the grid lethal (:55-56, :66-67).
//! The footprint meets the grid where that cost is lethal
//! ([`PaddedFootprint::collides_at`](crate::PaddedFootprint::collides_at),
//! with its departure), and so at a pose whose center lies off the grid, which
//! the regulated pure pursuit passes as clear (src/collision_checker.cpp:159-166).
//! The check reads whole cells, so it reads an obstacle's cell as an edge's
//! raster reaches it, wherever in the cell the obstacle's face stands. The
//! obstacle layer clears the cells under the padded footprint at each frame's
//! base pose, so what stood there is the newest points' alone.
//!
//! A source is what feeds the grid, as Nav2's collision monitor reads its
//! sources, sources 0 to one fewer than the count of the live obstacles each
//! call of the stop names, the indexes they name a frame's source by. The
//! stop reads its polygon, the padded footprint, and its count of sources
//! from those live obstacles at each call, and holds neither. It keeps the
//! arrival of each source's newest reading, which [`Stop::reading_arrived`]
//! gives it, and [`Stop::silent_sources`] gives back, in the order of the
//! sources, each source with no reading, or whose newest reading arrived more
//! than `source_timeout_s` before now.
//!
//! The stop's clock is the caller's, and the stop keeps the time of its
//! previous decision. A decision whose now lies before that time comes from a
//! clock that went back: from it on, every source reads silent
//! ([`StopCause::NoReading`]) until its next reading arrives, so no reading
//! given before that decision counts on the new clock. A reading that arrived
//! after a decision's now counts from that decision: its age there, now less
//! its arrival, lies below 0, within the timeout, and the decision gives it
//! that now as its arrival, so the reading holds for at most the source
//! timeout from the first decision that reads it. A caller that stamps each
//! frame as it arrives and decides at the slot of its period gives frames that
//! arrived after the slot's start, and they read live. [`Stop::decide`]
//! applies the rule to what the stop keeps: it forgets every reading at a
//! clock gone back, gives each arrival after its now that now, and keeps its
//! now as the previous decision's time; [`Stop::silent_sources`] answers by
//! the same rule and changes nothing. The limiter finds a clock that went back
//! by the same rule, a tick before its last call, and gives a setpoint whose
//! time lies after a tick's time that tick's time (`control_core::limiter`);
//! where the limiter then holds its setpoint for its timeout from that tick,
//! the stop reads every source silent until its next reading.
//!
//! The stop departs from the monitor's `sourceValid` (src/source.cpp:185-186)
//! twice. The monitor ages a source's newest data from its stamp
//! (`sourceValid(data_->header.stamp, curr_time)`, src/pointcloud.cpp:117,
//! src/scan.cpp:83), where the stop ages a reading from its arrival on the
//! caller's clock, as base_navigation:v1 judges a sensor silent when no frame
//! of it has arrived within the timeout: a source that delivers old frames on
//! time stays live here. And the monitor holds a stamp after now valid until
//! the clock passes the stamp plus `source_timeout`, so a source whose
//! readings stopped before the clock went back reads live while the clock
//! catches up to them; here a reading that arrived after a decision's now
//! holds for at most the source timeout from the first decision that reads
//! it, after a step back the stop finds and after one it does not: one before
//! its first decision, or shorter than the time since its previous decision.
//!
//! [`Stop::decide`] takes the live obstacles, the velocity the controller
//! commands, what [`Occupied`] holds, and the time now, on the caller's clock.
//! A source with no reading, or a silent one, stops the base, as the monitor
//! stops on an invalid source. Otherwise the footprint is projected along the
//! velocity, the velocity turning with the base, a simulation step at a time,
//! while the step's time lies within `time_before_collision_s`
//! (polygon.cpp:368). A pose of the projection, the base where it stands
//! first, is a contact when a point lies inside the footprint there, one point
//! a contact, the monitor's `min_points` at 1, or when the footprint there
//! meets the grid. At the first contact the velocity is scaled by the share of
//! that time the contact leaves, and a contact where the base stands scales it
//! to rest. The first contact of either check times the slowing: the slower of
//! the velocities the two checks allow apart, as the monitor keeps the slowest
//! velocity its approach polygons allow (collision_monitor_node.cpp:498-523,
//! 628). Within `time_before_collision_s`, the scaled velocity covers the
//! projection up to the step before the contact, so at no step of the
//! projection the scaled velocity covers does a point lie inside the padded
//! footprint or the footprint meet the grid. The footprint between two steps
//! is not checked, as the monitor checks it at its steps alone. The
//! footprint's vertices stay in double precision, where the monitor parses a
//! polygon's points into single precision
//! (include/nav2_collision_monitor/polygon_utils.hpp:56). Times are integer
//! nanoseconds, as ROS compares them.
//!
//! [`Stop::parse`] refuses a `simulation_time_step_s` longer than
//! `time_before_collision_s`, whose one step reaches past the time, and one so
//! short that some time of the projection, from 0 up to
//! `time_before_collision_s`, does not advance by it in double precision, with
//! which the projection never ends. Each decision projects a step at each sum
//! of steps from 0 while that sum is at most `time_before_collision_s`: one
//! more than the whole number of steps in that time, or that number where the
//! summed times round past the last.

use std::collections::BTreeMap;

use control_core::planar_pose::PlanarPose;
use control_core::positive_finite::PositiveFinite;
use control_core::time::{elapsed_ns, nanoseconds, seconds};
use control_core::twist::PlanarTwist;

use crate::footprint::{PaddedFootprint, is_point_inside_polygon};
use crate::grid::CostGrid;
use crate::obstacles::LiveObstacles;

/// The stop's settings, by the collision monitor's names.
#[derive(Debug, Clone, PartialEq)]
pub struct StopParams {
    /// How far ahead in time, in seconds, the footprint is projected along
    /// the velocity (`time_before_collision`).
    pub time_before_collision_s: PositiveFinite,
    /// The time of one step of the projection, in seconds
    /// (`simulation_time_step`).
    pub simulation_time_step_s: PositiveFinite,
    /// How long ago, in seconds, the newest reading of each source may have
    /// arrived (`source_timeout`).
    pub source_timeout_s: PositiveFinite,
}

/// Why the stop's settings make no stop.
#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum StopError {
    #[error(
        "simulation_time_step_s {step_s} s is longer than time_before_collision_s {horizon_s} \
         s, so the projection's one step reaches past that time: give a \
         simulation_time_step_s of at most time_before_collision_s"
    )]
    StepPastHorizon { step_s: f64, horizon_s: f64 },
    #[error(
        "simulation_time_step_s {step_s} s does not advance every time from 0 up to \
         time_before_collision_s {horizon_s} s in double precision, so the projection never \
         ends: the least step that advances each is {least_s} s, and each decision projects one \
         step more than the whole number of simulation_time_step_s in time_before_collision_s, \
         as the summed times round, so give a step that keeps that count within the time a \
         decision may take"
    )]
    StepTooShort {
        step_s: f64,
        horizon_s: f64,
        least_s: f64,
    },
}

/// A source the live obstacles do not read: `index` among their `sources`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error(
    "source {index} is not one of the base's {sources} sources: give a source from 0 to {}",
    .sources - 1
)]
pub struct UnknownSource {
    pub index: usize,
    pub sources: usize,
}

/// What the stop reads at a decision, as the module gives it: each source's
/// newest points, and the cost grid with the base's pose on it.
#[derive(Debug)]
pub struct Occupied<'a> {
    /// Each source's newest points, in the base's frame.
    pub points: Vec<[f64; 2]>,
    /// The cost grid whose cells under the footprint's outline the stop reads.
    pub grid: &'a CostGrid,
    /// The base's pose in the grid's frame.
    pub base: PlanarPose,
}

/// What the stop does with the commanded velocity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StopDecision {
    /// The commanded velocity stands: no step of the projection before
    /// `time_before_collision_s` is a contact, or the scaled velocity is no
    /// slower, as for a base at rest.
    Continue { velocity: PlanarTwist },
    /// The first step of the projection that is a contact lies at
    /// `collision_time_s`, as `getCollisionTime` times it,
    /// 0 where the base stands: the velocity scaled by `collision_time_s /
    /// time_before_collision_s`, to rest for a contact where the base stands
    /// (the monitor's approach).
    Slow {
        velocity: PlanarTwist,
        collision_time_s: f64,
    },
    /// The base must come to rest: a source's readings stopped arriving (the
    /// monitor's invalid source).
    Stop(StopCause),
}

/// Why the base must come to rest, naming the source by its place among the
/// stop's sources.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StopCause {
    /// No reading of the source has arrived on the caller's clock as it runs
    /// now: none since the stop was made, or none since a decision found the
    /// clock gone back.
    NoReading { source: usize },
    /// The source's newest reading arrived longer ago than the source
    /// timeout.
    StaleSource { source: usize, age_s: f64 },
}

impl StopDecision {
    /// The velocity the base may execute.
    pub fn velocity(&self) -> PlanarTwist {
        match self {
            StopDecision::Continue { velocity } | StopDecision::Slow { velocity, .. } => *velocity,
            StopDecision::Stop(_) => PlanarTwist::ZERO,
        }
    }
}

/// The stop, the arrival of each source's newest reading, and the time of its
/// previous decision.
#[derive(Debug, PartialEq)]
pub struct Stop {
    params: StopParams,
    source_timeout_ns: i64,
    /// The arrival of each source's newest reading, by the source's index, at
    /// most the now of each decision since [`Stop::reading_arrived`] gave it;
    /// none for a source with no reading on the clock as it runs now.
    newest_readings_ns: BTreeMap<usize, i64>,
    /// The time of the previous decision, none before the first.
    previous_decision_ns: Option<i64>,
}

impl Stop {
    /// The stop of `params`, no reading of a source yet: its simulation step
    /// at most `time_before_collision_s` and long enough to advance every
    /// time from 0 up to it.
    pub fn parse(params: StopParams) -> Result<Self, StopError> {
        let step_s = params.simulation_time_step_s.get();
        let horizon_s = params.time_before_collision_s.get();
        if step_s > horizon_s {
            return Err(StopError::StepPastHorizon { step_s, horizon_s });
        }
        let least_s = least_advancing_step(horizon_s);
        if step_s < least_s {
            return Err(StopError::StepTooShort {
                step_s,
                horizon_s,
                least_s,
            });
        }
        Ok(Self {
            source_timeout_ns: nanoseconds(params.source_timeout_s.get()),
            params,
            newest_readings_ns: BTreeMap::new(),
            previous_decision_ns: None,
        })
    }

    /// A reading of `source`, one of the sources of `live`, arrived at
    /// `arrived_ns` on the caller's clock: the source's newest reading from
    /// now on. A source `live` does not read is refused, and nothing changes.
    pub fn reading_arrived(
        &mut self,
        live: &LiveObstacles,
        source: usize,
        arrived_ns: i64,
    ) -> Result<(), UnknownSource> {
        live.known_source(source)?;
        self.newest_readings_ns.insert(source, arrived_ns);
        Ok(())
    }

    /// What to do with `velocity`, by what `occupied` holds
    /// ([`LiveObstacles::occupied_seen_from`](crate::obstacles::LiveObstacles::occupied_seen_from)),
    /// at `now_ns` on the caller's clock, the polygon and the sources those of
    /// `live`: the cause of the first of [`Stop::silent_sources`] first, then
    /// the projection, each pose checked against the points and the grid. A
    /// decision before the previous one forgets every reading, and each
    /// arrival after `now_ns` takes `now_ns` as its time.
    pub fn decide(
        &mut self,
        live: &LiveObstacles,
        velocity: PlanarTwist,
        occupied: &Occupied<'_>,
        now_ns: i64,
    ) -> StopDecision {
        let silent = self.silent_sources(live, now_ns).into_iter().next();
        if self.clock_went_back(now_ns) {
            self.newest_readings_ns.clear();
        }
        for arrived_ns in self.newest_readings_ns.values_mut() {
            *arrived_ns = (*arrived_ns).min(now_ns);
        }
        self.previous_decision_ns = Some(now_ns);
        if let Some(cause) = silent {
            return StopDecision::Stop(cause);
        }
        let footprint = live.padded_footprint();
        let Some(collision_time_s) = self
            .projection(velocity)
            .find(|&pose| in_contact(footprint, occupied, pose))
            .map(|pose| pose.time)
        else {
            return StopDecision::Continue { velocity };
        };
        let change_ratio = collision_time_s / self.params.time_before_collision_s.get();
        let safe = velocity
            .map_axes(|_, value| value * change_ratio)
            .expect("a velocity times a contact's share of the horizon, 0 to 1, is finite");
        if squared_magnitude(safe) < squared_magnitude(velocity) {
            return StopDecision::Slow {
                velocity: safe,
                collision_time_s,
            };
        }
        StopDecision::Continue { velocity }
    }

    /// `sourceValid` of each source of `live` at `now_ns` on the caller's
    /// clock, by the module's rule: each source with no reading, as every
    /// source has at a now before the previous decision's, or whose newest
    /// reading arrived more than the source timeout before `now_ns`, in the
    /// order of the sources; none while every source's newest reading arrived
    /// within the source timeout before `now_ns`, or after it.
    pub fn silent_sources(&self, live: &LiveObstacles, now_ns: i64) -> Vec<StopCause> {
        let went_back = self.clock_went_back(now_ns);
        (0..live.sources())
            .filter_map(|source| {
                let newest = self.newest_readings_ns.get(&source).filter(|_| !went_back);
                let Some(&arrived_ns) = newest else {
                    return Some(StopCause::NoReading { source });
                };
                let age_ns = elapsed_ns(arrived_ns, now_ns);
                (age_ns > self.source_timeout_ns).then_some(StopCause::StaleSource {
                    source,
                    age_s: seconds(age_ns),
                })
            })
            .collect()
    }

    /// Whether `now_ns` lies before the previous decision's time: a clock
    /// that went back.
    fn clock_went_back(&self, now_ns: i64) -> bool {
        self.previous_decision_ns
            .is_some_and(|previous_ns| now_ns < previous_ns)
    }

    /// The poses at which the footprint is checked for `velocity`.
    fn projection(&self, velocity: PlanarTwist) -> Projection {
        Projection {
            pose: ProjectedPose::default(),
            velocity: velocity.axes(),
            time: 0.0,
            step: self.params.simulation_time_step_s.get(),
            horizon: self.params.time_before_collision_s.get(),
            started: false,
        }
    }
}

/// Whether `pose` of the projection is a contact: a point of `occupied` inside
/// the footprint there ([`holds_a_point`]), or the footprint there meeting
/// `occupied`'s grid ([`meets_the_grid`]).
fn in_contact(footprint: &PaddedFootprint, occupied: &Occupied, pose: ProjectedPose) -> bool {
    holds_a_point(footprint.points(), &occupied.points, pose)
        || meets_the_grid(footprint, occupied, pose)
}

/// `footprintCostAtPose` at `pose` of the projection, placed in the grid's
/// frame by the base's pose: whether the cost under the footprint's outline
/// is lethal ([`PaddedFootprint::collides_at`]). A pose whose place in the
/// grid's frame is not finite puts the footprint's vertices off the grid,
/// which reads lethal.
fn meets_the_grid(footprint: &PaddedFootprint, occupied: &Occupied, pose: ProjectedPose) -> bool {
    let base = occupied.base;
    let [x, y] = base.apply([pose.x, pose.y]);
    let Ok(placed) = PlanarPose::new(x, y, base.yaw() + pose.theta) else {
        return true;
    };
    footprint.collides_at(occupied.grid, placed)
}

/// `getPointsInside` after `transformPoints`: whether a point of `points` lies
/// inside `polygon`, the footprint, at `pose`.
fn holds_a_point(polygon: &[[f64; 2]], points: &[[f64; 2]], pose: ProjectedPose) -> bool {
    let (sin_theta, cos_theta) = (pose.theta.sin(), pose.theta.cos());
    points.iter().any(|point| {
        let mul_x = point[0] - pose.x;
        let mul_y = point[1] - pose.y;
        let x = mul_x * cos_theta + mul_y * sin_theta;
        let y = -mul_x * sin_theta + mul_y * cos_theta;
        is_point_inside_polygon(x, y, polygon)
    })
}

/// A pose of the projection in the frame the base stands in now, and the
/// time `getCollisionTime` gives a contact at it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct ProjectedPose {
    x: f64,
    y: f64,
    theta: f64,
    time: f64,
}

/// The base where it stands, then after each simulation step while the
/// step's time lies within the horizon, as `getCollisionTime` steps it.
struct Projection {
    pose: ProjectedPose,
    velocity: [f64; 3],
    time: f64,
    step: f64,
    horizon: f64,
    started: bool,
}

impl Iterator for Projection {
    type Item = ProjectedPose;

    fn next(&mut self) -> Option<ProjectedPose> {
        if !self.started {
            self.started = true;
            return Some(self.pose);
        }
        if self.time > self.horizon {
            return None;
        }
        self.project_state();
        self.pose.time = self.time;
        self.time += self.step;
        Some(self.pose)
    }
}

impl Projection {
    /// `projectState`: the pose moved by the velocity over one step, and the
    /// velocity turned by the step's turn.
    fn project_state(&mut self) {
        let dt = self.step;
        let [vx, vy, tw] = self.velocity;
        let theta = tw * dt;
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();
        self.pose.x += vx * dt;
        self.pose.y += vy * dt;
        self.pose.theta += theta;
        self.velocity = [
            vx * cos_theta - vy * sin_theta,
            vx * sin_theta + vy * cos_theta,
            tw,
        ];
    }
}

/// The least step that advances every time from 0 up to `horizon_s` in
/// double precision: the double above half the spacing of the doubles at
/// `horizon_s`. A step of that half spacing ties at the power of two at or
/// below the horizon, whose last bit is even, and rounds back to it.
fn least_advancing_step(horizon_s: f64) -> f64 {
    ((horizon_s.next_up() - horizon_s) / 2.0).next_up()
}

/// The squared magnitude by which the collision monitor orders velocities
/// (`Velocity::operator<`), a turn rate counting as a speed.
fn squared_magnitude(velocity: PlanarTwist) -> f64 {
    let [x, y, tw] = velocity.axes();
    x * x + y * y + tw * tw
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{FREE, GridGeometry};
    use control_core::footprint::Footprint;
    use control_core::non_negative_finite::NonNegativeFinite;

    // A pose of the projection whose place in the grid's frame is not finite puts the
    // footprint's vertices off the grid, so it meets the grid; the footprint at the grid's
    // center meets nothing on a free grid.
    #[test]
    fn a_pose_past_double_precision_meets_the_grid() {
        let footprint = PaddedFootprint::new(
            &Footprint::parse(&[[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2]]).unwrap(),
            NonNegativeFinite::parse(0.0).unwrap(),
        )
        .unwrap();
        let geometry =
            GridGeometry::new(20, 20, PositiveFinite::parse(0.1).unwrap(), [-1.0, -1.0]).unwrap();
        let grid = CostGrid::filled(geometry, FREE);
        let occupied = Occupied {
            points: Vec::new(),
            grid: &grid,
            base: PlanarPose::IDENTITY,
        };
        let at = |x: f64| ProjectedPose {
            x,
            ..ProjectedPose::default()
        };
        assert!(!meets_the_grid(&footprint, &occupied, at(0.0)));
        assert!(meets_the_grid(&footprint, &occupied, at(f64::INFINITY)));
    }

    // The projection starts where the base stands, at time 0, then steps
    // while the step's time lies within the horizon, a turning base's
    // velocity turning with it.
    #[test]
    fn the_projection_steps_to_the_horizon() {
        let mut projection = Projection {
            pose: ProjectedPose::default(),
            velocity: [1.0, 0.0, std::f64::consts::FRAC_PI_2],
            time: 0.0,
            step: 1.0,
            horizon: 1.5,
            started: false,
        };
        assert_eq!(projection.next(), Some(ProjectedPose::default()));
        let first = projection.next().unwrap();
        assert_eq!((first.x, first.y, first.time), (1.0, 0.0, 0.0));
        let second = projection.next().unwrap();
        assert!((second.x - 1.0).abs() < 1e-15 && (second.y - 1.0).abs() < 1e-15);
        assert_eq!(second.time, 1.0);
        assert_eq!(projection.next(), None);
    }

    // A step whose time equals the horizon is projected, as the monitor's
    // loop runs while its time is at most `time_before_collision`: the pose
    // where the base stands, then the steps at 0, 0.125, ..., 2 s.
    #[test]
    fn the_projection_takes_the_step_at_the_horizon() {
        let projection = Projection {
            pose: ProjectedPose::default(),
            velocity: [1.0, 0.0, 0.0],
            time: 0.0,
            step: 0.125,
            horizon: 2.0,
            started: false,
        };
        let times: Vec<f64> = projection.map(|pose| pose.time).collect();
        assert_eq!(times.len(), 18, "{times:?}");
        assert_eq!(times.last(), Some(&2.0));
    }

    // The least advancing step advances the power of two at or below the
    // horizon, where the spacing is the horizon's and the last bit even, and
    // the double below it does not, whether the horizon's last bit is even or
    // odd; a projection by it from that power of two, and from four spacings
    // below the horizon, ends within a dozen poses.
    #[test]
    fn the_least_advancing_step_advances_every_time_up_to_the_horizon() {
        for horizon_s in [2.0, 0.1, 0.3, 1.0_f64.next_up(), 1e-3, 7.3] {
            let least_s = least_advancing_step(horizon_s);
            let power_of_two = 2.0_f64.powi(horizon_s.log2().floor() as i32);
            assert_ne!(power_of_two + least_s, power_of_two, "{horizon_s}");
            assert_eq!(
                power_of_two + least_s.next_down(),
                power_of_two,
                "{horizon_s}"
            );
            let spacing = horizon_s.next_up() - horizon_s;
            for (from, to) in [
                (power_of_two, power_of_two + 4.0 * spacing),
                (horizon_s - 4.0 * spacing, horizon_s),
            ] {
                let projection = Projection {
                    pose: ProjectedPose::default(),
                    velocity: [1.0, 0.0, 0.0],
                    time: from,
                    step: least_s,
                    horizon: to,
                    started: false,
                };
                let poses = projection.take(12).count();
                assert!(poses < 12, "{horizon_s} from {from}: {poses} poses");
            }
        }
    }
}
