// The arms of a scenario's group in a scene, as MoveIt holds them: what the references of
// arm planning, of Cartesian paths and of CHOMP refinement share. The robot model of the
// scenario's description, its KDL inverse kinematics, the seeds of MoveIt's own random generators,
// the planning scene of a scene of the scenario, MoveIt's CHOMP refining a joint path in it,
// and MoveIt's probe of a joint path in it.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <nlohmann/json.hpp>

#include "robot.hpp"
#include "scenario.hpp"

namespace ros_reference {

// How many of MoveIt's random generators the reference program's constructor has built.
std::atomic<std::uint64_t>& moveit_generators();

// Seeds the generator of the seeds of MoveIt's own random generators, each built after this
// call drawing from it, and std::rand, from `seed`.
void seed_moveit(std::uint32_t seed);

// The scenario's `group` of `model`'s SRDF and its arms.
ArmGroup group_of(const moveit::core::RobotModel& model, const nlohmann::json& scenario);

// The robot model of the scenario's `description`, the joints of its `group` each bounded in
// acceleration.
moveit::core::RobotModelPtr arm_model(const nlohmann::json& scenario, Inputs& inputs);

// MoveIt's KDL inverse kinematics on `solver_groups`, an ArmGroup's, which a pose goal's
// constraint sampler reads. It loads as move_group loads a moveit_config's kinematics.yaml
// that names only the solver: at the search resolution and the timeout that
// moveit_ros_planning's kinematics_parameters.yaml gives by default (0.1 and 0.05 s), the
// timeout set on each group as RobotModelLoader::loadKinematicsSolvers sets it (moveit2
// 2.15.2 robot_model_loader.cpp:310-318), which the constraint sampler gives each IK call
// (default_constraint_samplers.cpp:268). A group of fewer joints than a pose has freedoms
// solves the position alone, its kinematics.yaml naming position_only_ik too: KDL's search
// otherwise converges on no pose such a group does not hold.
void load_kdl(const moveit::core::RobotModelPtr& model,
              const std::vector<std::string>& solver_groups);

std::vector<double> numbers(const nlohmann::json& values);

// The scene `s`: its boxes, each the voxels from its first key to its last on every axis,
// in an octomap of `voxel_size`; its objects; each object it attaches to a grasp link with
// its touch links, at its pose in the link's frame, the arms at the scene's start, which
// stays the scene's state, an arm outside the group at its zero configuration; and its
// allowed pairs. Every link with collision geometry is padded by `padding` against the
// octomap and the objects, and the robot meets itself unpadded, as a collision request's
// defaults pad_environment_collisions and pad_self_collisions ask. An attached object
// takes the padding of its link (moveit2 2.15.2 collision_env_fcl.cpp:195-196), and a
// grasp link has no collision geometry, which setPadding pads alone
// (collision_env.cpp:127-133), so its padding is 0 (collision_env.cpp:166-176): MoveIt
// keeps a held object unpadded from the objects. Collisions between the robot's own links
// are allowed, but, when `group` holds every arm of the description, those of each pair
// the SRDF does not disable, the pairs motion_planning_core's self model checks; and so are
// those of every link that does not move with the group's joints, the
// body, the arms' bases, the head camera, and an arm outside the group and its gripper,
// but against an object an arm of the group holds.
planning_scene::PlanningScenePtr scene_of(const moveit::core::RobotModelPtr& model,
                                         const nlohmann::json& s, double voxel_size,
                                         const ArmGroup& group, double padding);

// The joints of each of `waypoints`, an array of arrays of numbers.
std::vector<Eigen::VectorXd> points_of(const nlohmann::json& waypoints);

// The joints of `arm` at each waypoint of `trajectory`.
std::vector<Eigen::VectorXd> points_of(const robot_trajectory::RobotTrajectory& trajectory,
                                       const moveit::core::JointModelGroup& arm);

// CHOMP's answer to a path: its error code, MoveIt's of a plan, and the arm's joints at each
// waypoint of its path where its optimizer ran, whether it succeeded or not.
struct ChompRefined {
  int status = 0;
  std::optional<std::vector<Eigen::VectorXd>> path;
};

// MoveIt's CHOMP refining `input`, a path of the group in `scene` from the scene's state, to
// the path's last waypoint: CHOMP's optimizer at the parameters MoveIt's CHOMP plugin loads
// from a node that sets none of them, its initial trajectory `input` (`fillTrajectory`), in a
// diff of `scene` in the hybrid collision environment, as the plugin plans, on a thread of its
// own whose random generator it seeds from `seed`.
ChompRefined chomp_refined(const planning_scene::PlanningScene& scene,
                           const robot_trajectory::RobotTrajectoryPtr& input, std::uint32_t seed);

// An orientation a path holds, as plan_joint_path states it: the angle of the rotation from
// `orientation`, or with a free axis the angle of that axis from its direction there, at most
// `tolerance`.
struct HeldTarget {
  const moveit::core::LinkModel* link;
  Eigen::Quaterniond orientation;
  std::optional<Eigen::Vector3d> free_axis;
  double tolerance;
};

// What the probes of a scenario's paths measure, `spacing` apart in the joints' L2 norm,
// besides the scene: the objects the chains' tip links `tip_links` hold, with `self` the
// robot's own bodies, and the orientations `held` the scene holds; and how a path is rounded
// and timed for its measures: its corners at `corner_tolerance`, each joint at its
// `max_velocity` and `max_acceleration`.
struct Probing {
  std::string group;
  std::vector<std::string> tip_links;
  bool self = false;
  double spacing = 0.0;
  std::vector<HeldTarget> held;
  double corner_tolerance = 0.0;
  Eigen::VectorXd max_velocity;
  Eigen::VectorXd max_acceleration;
};

// The probing of the scenario's paths by the group of `chains` of `model`: of the objects the
// chains' tip links hold, at the scenario's `probe_spacing`, rounded at its
// `corner_tolerance` and timed at the bounds of the group's joints; no self distance and no
// held orientation, which a reference adds.
Probing probing_of(const nlohmann::json& scenario, const ArmGroup& group,
                   const moveit::core::RobotModel& model);

// MoveIt's collision check and distances at the probes of a joint path: to the scene,
// from the objects the grasp links `holding` hold, and, with `self`, between the robot's
// own bodies.
struct Probed {
  bool self = false;
  std::size_t collisions = 0;
  double clearance = std::numeric_limits<double>::infinity();
  // The grasp links that hold an object, and the least unpadded distance from what they
  // hold to the objects and the octomap.
  std::set<const moveit::core::LinkModel*> holding;
  double held = std::numeric_limits<double>::infinity();
  // The least unpadded distance between the robot's own bodies that the scene's matrix
  // leaves checked, when `self` asks for it.
  double self_clearance = std::numeric_limits<double>::infinity();
  // The orientations the path holds, and the largest share of its tolerance any of them
  // reaches.
  std::vector<HeldTarget> held_targets;
  double share_of_tolerance = 0.0;
};

// The probe of a path in `scene` as `probing` asks, before its first state: of each tip
// link that holds an object.
Probed probe_of(const planning_scene::PlanningScene& scene, const Probing& probing);

// Adds to `probed` MoveIt's check and distances in `scene` at the arm's joints `q`, `state`
// the scene's state that holds them.
void probe(const planning_scene::PlanningScene& scene, moveit::core::RobotState& state,
           const moveit::core::JointModelGroup& arm, const Eigen::VectorXd& q, Probed& probed);

// The number of probes of a stretch `length` long, each at most `spacing` from the next.
std::size_t steps_of(double length, double spacing);

// MoveIt's time-optimal path through `points`, its corners rounded at `corner_tolerance`.
trajectory_processing::Path rounded_path(const std::vector<Eigen::VectorXd>& points,
                                         double corner_tolerance);

// `probed` as the references write it: its collisions; its clearances to the scene, from what
// the grasp links hold, null where they hold nothing, and between the robot's own bodies,
// null without `self`; the largest share of a held orientation's tolerance, null where the
// path holds none; the path's `length`; and of `rounded`, the path rounded at the probing's
// corner tolerance, its `turning`, the integral of the norm of its curvature over its length,
// and its `bending`, the integral of the squared norm, each summed at the midpoints of equal
// steps at most the probing's spacing long, and its `duration`, the time MoveIt's time-optimal
// trajectory takes along it at the probing's joint bounds, null where it times none.
nlohmann::json measured(const Probed& probed, double length,
                        const trajectory_processing::Path& rounded, const Probing& probing);

// MoveIt's probe of the path through `points`, straight between them, as `probing` asks,
// written by `measured` with the path's length and the path rounded.
nlohmann::json polyline_measured(const planning_scene::PlanningScene& scene,
                                 moveit::core::RobotState& state,
                                 const moveit::core::JointModelGroup& arm,
                                 const std::vector<Eigen::VectorXd>& points,
                                 const Probing& probing);

// MoveIt's probe of motion_planning_core's path `waypoints`, rounded by MoveIt's
// time-optimal path at the probing's corner tolerance and probed along it as `probing` asks.
nlohmann::json ours_probed(const planning_scene::PlanningScene& scene,
                           const moveit::core::RobotModelPtr& model,
                           const nlohmann::json& waypoints, const Probing& probing);

}  // namespace ros_reference
