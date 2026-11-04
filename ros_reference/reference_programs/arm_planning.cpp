#include "arm_planning.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <octomap/OcTree.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>

#include "planning_scene.hpp"
#include "robot.hpp"

namespace ros_reference {
namespace {

// The generator of the seeds of MoveIt's own random generators, which the reference
// reseeds from each scene's seed before it plans the scene, and the lock under which a
// generator of MoveIt's, on the planner's thread or the goal sampler's, draws its seed.
std::mt19937& moveit_seeds() {
  static std::mt19937 seeds;
  return seeds;
}

std::mutex& moveit_seeds_lock() {
  static std::mutex lock;
  return lock;
}

// How many of MoveIt's random generators the reference program's constructor has built.
std::atomic<std::uint64_t>& moveit_generators() {
  static std::atomic<std::uint64_t> built{0};
  return built;
}

}  // namespace
}  // namespace ros_reference

// MoveIt's state sampler and its constraint samplers each draw from a
// random_numbers::RandomNumberGenerator that they construct by its default constructor,
// which random_numbers seeds from the clock. The reference program defines that
// constructor, and its link exports it (CMakeLists.txt), so MoveIt's libraries construct
// every such generator through it, seeded from `moveit_seeds`, and a plan repeats.
random_numbers::RandomNumberGenerator::RandomNumberGenerator()
    : RandomNumberGenerator([] {
        const std::lock_guard<std::mutex> held(ros_reference::moveit_seeds_lock());
        return static_cast<std::uint32_t>(ros_reference::moveit_seeds()());
      }()) {
  ++ros_reference::moveit_generators();
}

namespace ros_reference {
namespace {

constexpr const char* group = "arm";
constexpr const char* pipeline_namespace = "ompl";
// OctoMap's key of the voxel at the origin of each axis.
constexpr int tree_max_val = 32768;
// The acceleration bound of each joint of the arm, in rad/s^2, for the pipeline's time
// parameterization: it times the path and leaves its shape to the path tolerance.
constexpr double max_acceleration = 5.0;
// MoveIt's error codes of a plan that succeeds, and of one whose planned path fails
// MoveIt's own check: both ran the planner.
constexpr int moveit_success = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
constexpr int moveit_invalid_motion_plan = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;

// The robot model of the scenario, whose group "arm" is the chain from `base_link` to
// `tip_link`, its joints the scenario's `joints` in their order, each bounded in
// acceleration.
moveit::core::RobotModelPtr arm_model(const nlohmann::json& scenario, Inputs& inputs) {
  auto model = robot_model_with_meshes(
      inputs, scenario.at("urdf").get<std::string>(), scenario.at("meshes").get<std::string>(),
      ChainGroup{group, scenario.at("base_link").get<std::string>(),
                 scenario.at("tip_link").get<std::string>()});
  const auto joints = scenario.at("joints").get<std::vector<std::string>>();
  if (model->getJointModelGroup(group)->getActiveJointModelNames() != joints) {
    throw std::runtime_error(
        "the group from base_link to tip_link holds other joints than the scenario's joints, "
        "or holds them in another order");
  }
  // A URDF bounds no acceleration, and the pipeline's time parameterization needs one,
  // as a moveit_config's joint_limits.yaml gives it.
  for (const auto* joint : model->getJointModelGroup(group)->getActiveJointModels()) {
    auto* writable = model->getJointModel(joint->getName());
    for (const auto& variable : writable->getVariableNames()) {
      moveit::core::VariableBounds bounds = writable->getVariableBounds(variable);
      bounds.acceleration_bounded_ = true;
      bounds.min_acceleration_ = -max_acceleration;
      bounds.max_acceleration_ = max_acceleration;
      writable->setVariableBounds(variable, bounds);
    }
  }
  return model;
}

// MoveIt's KDL inverse kinematics on the group "arm", which a pose goal's constraint
// sampler reads. It loads as move_group loads a moveit_config's kinematics.yaml that names
// only the solver: at the search resolution and the timeout that moveit_ros_planning's
// kinematics_parameters.yaml gives by default (0.1 and 0.05 s), the timeout set on each
// group as RobotModelLoader::loadKinematicsSolvers sets it (moveit2 2.15.2
// robot_model_loader.cpp:310-318), which the constraint sampler gives each IK call
// (default_constraint_samplers.cpp:268).
void load_kdl(const moveit::core::RobotModelPtr& model) {
  start_ros();
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides(
      {{std::string("robot_description_kinematics.") + group + ".kinematics_solver",
        "kdl_kinematics_plugin/KDLKinematicsPlugin"}});
  auto node = std::make_shared<rclcpp::Node>("arm_planning_kinematics", options);
  kinematics_plugin_loader::KinematicsPluginLoader loader(node);
  const auto allocator = loader.getLoaderFunction(std::make_shared<srdf::Model>(*model->getSRDF()));
  std::map<std::string, moveit::core::SolverAllocatorFn> allocators;
  for (const auto& name : loader.getKnownGroups()) {
    allocators[name] = allocator;
  }
  model->setKinematicsAllocators(allocators);
  for (const auto& [name, timeout] : loader.getIKTimeout()) {
    if (model->hasJointModelGroup(name)) {
      model->getJointModelGroup(name)->setDefaultIKTimeout(timeout);
    }
  }
  if (!model->getJointModelGroup(group)->getSolverInstance()) {
    throw std::runtime_error("MoveIt's KDL plugin loads no inverse kinematics for the arm");
  }
}

// The planner configuration of the scenario's objective, by its name and its parameters:
// RRTConnect for `any` and RRTstar for `shortest`, each at the parameters of MoveIt's
// default planner configurations (moveit2 2.15.2
// moveit_configs_utils/default_configs/ompl_defaults.yaml, RRTConnect and RRTstar).
struct PlannerConfig {
  std::string name;
  std::vector<rclcpp::Parameter> parameters;
};

PlannerConfig planner_config(const std::string& objective, const std::string& ns) {
  const auto config = [&](const std::string& name, const std::string& type,
                          std::vector<std::pair<std::string, rclcpp::ParameterValue>> values) {
    std::vector<rclcpp::Parameter> parameters{{ns + ".planner_configs." + name + ".type", type}};
    for (auto& [key, value] : values) {
      parameters.emplace_back(ns + ".planner_configs." + name + "." + key, std::move(value));
    }
    return PlannerConfig{name, parameters};
  };
  if (objective == "any") {
    return config("RRTConnectkConfigDefault", "geometric::RRTConnect",
                  {{"range", rclcpp::ParameterValue(0.0)}});
  }
  if (objective == "shortest") {
    return config("RRTstarkConfigDefault", "geometric::RRTstar",
                  {{"range", rclcpp::ParameterValue(0.0)},
                   {"goal_bias", rclcpp::ParameterValue(0.05)},
                   {"delay_collision_checking", rclcpp::ParameterValue(1)}});
  }
  throw std::runtime_error("the scenario's objective is any or shortest, and it is " + objective);
}

// The pipeline: the OMPL planner of the scenario's objective, ended as its termination
// says, by its `iterations` (MoveIt's termination condition Iteration) or by the
// request's allowed planning time alone (MoveIt's default); MoveIt's default request
// adapters, a start outside a joint limit by any amount refused; and the response
// adapters AddTimeOptimalParameterization and ValidateSolution: MoveIt's defaults but
// DisplayMotionPath, which publishes the path for display. MoveIt simplifies each path
// in the time left, as simplify_solutions, true by default, says.
std::shared_ptr<planning_pipeline::PlanningPipeline> pipeline(
    const moveit::core::RobotModelPtr& model, const nlohmann::json& scenario,
    const PlannerConfig& planner) {
  start_ros();
  const std::string ns = pipeline_namespace;
  std::vector<rclcpp::Parameter> parameters{
      {ns + ".planning_plugins", std::vector<std::string>{"ompl_interface/OMPLPlanner"}},
      {ns + ".request_adapters",
       std::vector<std::string>{"default_planning_request_adapters/ResolveConstraintFrames",
                                "default_planning_request_adapters/ValidateWorkspaceBounds",
                                "default_planning_request_adapters/CheckStartStateBounds",
                                "default_planning_request_adapters/CheckStartStateCollision"}},
      {ns + ".response_adapters",
       std::vector<std::string>{"default_planning_response_adapters/AddTimeOptimalParameterization",
                                "default_planning_response_adapters/ValidateSolution"}},
      {ns + ".start_state_max_bounds_error", 0.0},
      {ns + "." + group + ".planner_configs", std::vector<std::string>{planner.name}},
  };
  parameters.insert(parameters.end(), planner.parameters.begin(), planner.parameters.end());
  const auto termination = scenario.at("termination").get<std::string>();
  if (termination == "iterations") {
    parameters.emplace_back(
        ns + ".planner_configs." + planner.name + ".termination_condition",
        "Iteration[" + std::to_string(scenario.at("iterations").get<std::uint64_t>()) + "]");
  } else if (termination != "time") {
    throw std::runtime_error("the scenario's termination is iterations or time, and it is " +
                             termination);
  }
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides(parameters);
  auto node = std::make_shared<rclcpp::Node>("arm_planning_reference", options);
  return std::make_shared<planning_pipeline::PlanningPipeline>(model, node, ns);
}

std::vector<double> numbers(const nlohmann::json& values) {
  return values.get<std::vector<double>>();
}

Eigen::VectorXd vector_of(const std::vector<double>& values) {
  return Eigen::Map<const Eigen::VectorXd>(values.data(),
                                           static_cast<Eigen::Index>(values.size()));
}

// Whether `link` moves with the group's joints: one of them lies on its way up to the
// root, as the hand's fingers hang below the hand.
bool moves_with_group(const moveit::core::LinkModel* link,
                      const moveit::core::JointModelGroup& arm) {
  for (const moveit::core::JointModel* joint = link->getParentJointModel(); joint != nullptr;) {
    if (arm.hasJointModel(joint->getName())) {
      return true;
    }
    const moveit::core::LinkModel* parent = joint->getParentLinkModel();
    joint = parent == nullptr ? nullptr : parent->getParentJointModel();
  }
  return false;
}

// The grasp link of the other arm, which holds a scene's other_arm_holds: the mirror of
// `tip_link`, OpenArm's name of it with left and right swapped, as ros_reference's
// Scenes::other_tip_link names it.
std::string other_tip_link(const std::string& tip_link) {
  const bool left = tip_link.find("left") != std::string::npos;
  const std::string side = left ? "left" : "right";
  const auto at = tip_link.find(side);
  if (at == std::string::npos) {
    throw std::runtime_error("the tip link " + tip_link + " names no side, left or right");
  }
  return std::string(tip_link).replace(at, side.size(), left ? "right" : "left");
}

// The scene `s`: its boxes, each the voxels from its first key to its last on every axis,
// in an octomap of `voxel_size`; its objects; and the object it attaches to `tip_link`
// with its touch links, at its pose in the link's frame, the arm at the scene's start,
// which stays the scene's state; the object it attaches to the other arm's grasp link,
// that arm at its zero configuration; and its allowed pairs. Every link with collision
// geometry is padded by `padding` against the octomap and the objects, and the robot meets
// itself unpadded, as a collision request's defaults pad_environment_collisions and
// pad_self_collisions ask. An attached object takes the padding of its link (moveit2
// 2.15.2 collision_env_fcl.cpp:195-196), and the grasp link has no collision geometry,
// which setPadding pads alone (collision_env.cpp:127-133), so its padding is 0
// (collision_env.cpp:166-176): MoveIt keeps a held object unpadded from the objects.
// Collisions between the robot's own links are allowed, and those of every link that does
// not move with the group's joints, the body, both arms' bases, the head camera, and the
// other arm and its gripper, but against the object the arm holds.
planning_scene::PlanningScenePtr scene_of(const moveit::core::RobotModelPtr& model,
                                         const nlohmann::json& s, double voxel_size,
                                         const std::string& tip_link, double padding) {
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  scene->getCollisionEnvNonConst()->setPadding(padding);
  auto tree = std::make_shared<octomap::OcTree>(voxel_size);
  for (const auto& box : s.at("boxes")) {
    const auto first = box.at("first").get<std::vector<int>>();
    const auto last = box.at("last").get<std::vector<int>>();
    for (int x = first[0]; x <= last[0]; ++x) {
      for (int y = first[1]; y <= last[1]; ++y) {
        for (int z = first[2]; z <= last[2]; ++z) {
          tree->updateNode(octomap::OcTreeKey(static_cast<uint16_t>(x + tree_max_val),
                                              static_cast<uint16_t>(y + tree_max_val),
                                              static_cast<uint16_t>(z + tree_max_val)),
                           true);
        }
      }
    }
  }
  scene->getWorldNonConst()->addToObject(planning_scene::PlanningScene::OCTOMAP_NS,
                                         std::make_shared<const shapes::OcTree>(tree),
                                         Eigen::Isometry3d::Identity());
  const auto frame = model->getModelFrame();
  for (const auto& entry : s.at("objects")) {
    if (!scene->processCollisionObjectMsg(object_added(entry, frame))) {
      throw std::runtime_error("MoveIt refused the object " + entry.dump());
    }
  }
  auto& state = scene->getCurrentStateNonConst();
  state.setToDefaultValues();
  state.setJointGroupPositions(group, numbers(s.at("start")));
  state.update();
  const auto attach = [&](const nlohmann::json& held, const std::string& link) {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = link;
    attached.object = object_added(held.at("object"), link);
    attached.touch_links = held.at("touch_links").get<std::vector<std::string>>();
    if (!scene->processAttachedCollisionObjectMsg(attached)) {
      throw std::runtime_error("MoveIt refused the attached object " + held.dump());
    }
  };
  const auto& held = s.at("attached");
  if (!held.is_null()) {
    attach(held, tip_link);
  }
  const auto& other = s.at("other_arm_holds");
  if (!other.is_null()) {
    attach(other, other_tip_link(tip_link));
  }
  auto& acm = scene->getAllowedCollisionMatrixNonConst();
  const auto& links = model->getLinkModelNamesWithCollisionGeometry();
  acm.setEntry(links, links, true);
  const auto& arm = *model->getJointModelGroup(group);
  std::vector<std::string> standing;
  for (const auto* link : model->getLinkModelsWithCollisionGeometry()) {
    if (!moves_with_group(link, arm)) {
      acm.setDefaultEntry(link->getName(), true);
      standing.push_back(link->getName());
    }
  }
  // The object the arm holds meets the links that stand still, as it meets the parts of
  // the robot motion_planning_core's plan does not move.
  if (!held.is_null()) {
    acm.setEntry(held.at("object").at("id").get<std::string>(), standing, false);
  }
  // The object the other arm holds stands still with that arm: it meets the arm the plan
  // moves and what it holds, as an obstacle another arm holds meets them.
  if (!other.is_null()) {
    const auto id = other.at("object").at("id").get<std::string>();
    acm.setEntry(id, scene->getWorld()->getObjectIds(), true);
    acm.setEntry(id, standing, true);
  }
  for (const auto& pair : s.at("allowed")) {
    acm.setEntry(pair.at(0).get<std::string>(), pair.at(1).get<std::string>(), true);
  }
  return scene;
}

// MoveIt's collision check and distance at the probes of a joint path.
struct Probed {
  std::size_t collisions = 0;
  double clearance = std::numeric_limits<double>::infinity();
  // The grasp link when it holds an object, and the least unpadded distance from what it
  // holds to the objects and the octomap.
  const moveit::core::LinkModel* holding = nullptr;
  double held = std::numeric_limits<double>::infinity();
};

// The probe of a path in `scene`, whose grasp link `tip_link` holds an object or none.
Probed probe_of(const planning_scene::PlanningScene& scene, const std::string& tip_link) {
  Probed probed;
  std::vector<const moveit::core::AttachedBody*> held;
  const auto* tip = scene.getRobotModel()->getLinkModel(tip_link);
  scene.getCurrentState().getAttachedBodies(held, tip);
  probed.holding = held.empty() ? nullptr : tip;
  return probed;
}

void probe(const planning_scene::PlanningScene& scene, moveit::core::RobotState& state,
           const moveit::core::JointModelGroup& arm, const Eigen::VectorXd& q, Probed& probed) {
  state.setJointGroupPositions(&arm, q);
  state.update();
  if (scene.isStateColliding(state, group)) {
    ++probed.collisions;
  }
  // FCL's distance to an octomap a body touches is no measure: the planning scene's dense
  // probe read -2.3e190 of one. MoveIt reads -1 for a body in contact otherwise, so -1
  // stands for every distance below it.
  probed.clearance =
      std::min(probed.clearance, std::max(scene.distanceToCollisionUnpadded(state), -1.0));
  if (probed.holding != nullptr) {
    collision_detection::DistanceRequest request;
    const std::set<const moveit::core::LinkModel*> holding{probed.holding};
    request.active_components_only = &holding;
    request.acm = &scene.getAllowedCollisionMatrix();
    collision_detection::DistanceResult result;
    scene.getCollisionEnvUnpadded()->distanceRobot(request, result, state);
    probed.held = std::min(probed.held, std::max(result.minimum_distance.distance, -1.0));
  }
}

// The number of probes of a stretch `length` long, each at most `spacing` from the next.
std::size_t steps_of(double length, double spacing) {
  return static_cast<std::size_t>(std::max(std::ceil(length / spacing), 1.0));
}

// MoveIt's trajectory, straight between its points, probed at most `spacing` apart.
Probed probe_polyline(const planning_scene::PlanningScene& scene,
                      moveit::core::RobotState& state, const moveit::core::JointModelGroup& arm,
                      const std::vector<Eigen::VectorXd>& points, double spacing,
                      const std::string& tip_link) {
  Probed probed = probe_of(scene, tip_link);
  probe(scene, state, arm, points.front(), probed);
  for (std::size_t i = 1; i < points.size(); ++i) {
    const std::size_t steps = steps_of((points[i] - points[i - 1]).norm(), spacing);
    for (std::size_t k = 1; k <= steps; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(steps);
      probe(scene, state, arm, points[i - 1] + (points[i] - points[i - 1]) * t, probed);
    }
  }
  return probed;
}

double polyline_length(const std::vector<Eigen::VectorXd>& points) {
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += (points[i] - points[i - 1]).norm();
  }
  return length;
}

nlohmann::json measured(const Probed& probed, double length) {
  return {{"collisions", probed.collisions},
          {"clearance", probed.clearance},
          {"held_clearance",
           probed.holding != nullptr ? nlohmann::json(probed.held) : nlohmann::json(nullptr)},
          {"length", length}};
}

// MoveIt's constraints of the goal `goal`. A configuration is the goal state itself,
// each joint within 0 of it: every goal state that MoveIt's goal-sampling thread adds is
// that one state, however many it has added when the planner takes one. A pose of
// `tip_link` holds its position within the radius `position_tolerance` and each component
// of the rotation vector from its target orientation within `orientation_tolerance`
// (constructGoalConstraints, ROTATION_VECTOR, moveit2 2.15.2 utils.cpp:220-226), which
// MoveIt samples by KDL's inverse kinematics, each goal sample's first attempt seeded from
// its working state, the start for the first sample.
moveit_msgs::msg::Constraints goal_constraints(const nlohmann::json& goal,
                                               const moveit::core::RobotState& start,
                                               const moveit::core::JointModelGroup& arm,
                                               const std::string& tip_link) {
  if (goal.is_array()) {
    moveit::core::RobotState state(start);
    state.setJointGroupPositions(&arm, numbers(goal));
    state.update();
    return kinematic_constraints::constructGoalConstraints(state, &arm, 0.0, 0.0);
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = start.getRobotModel()->getModelFrame();
  const auto position = numbers(goal.at("position"));
  const auto orientation = numbers(goal.at("orientation"));
  pose.pose.position.x = position.at(0);
  pose.pose.position.y = position.at(1);
  pose.pose.position.z = position.at(2);
  pose.pose.orientation.x = orientation.at(0);
  pose.pose.orientation.y = orientation.at(1);
  pose.pose.orientation.z = orientation.at(2);
  pose.pose.orientation.w = orientation.at(3);
  return kinematic_constraints::constructGoalConstraints(
      tip_link, pose, goal.at("position_tolerance").get<double>(),
      goal.at("orientation_tolerance").get<double>());
}

// MoveIt's answer to a scene, planned within `time_limit` seconds: its error code, and
// its path's probe when it planned one.
nlohmann::json moveit_answer(planning_pipeline::PlanningPipeline& planner,
                             const std::string& planner_id,
                             const planning_scene::PlanningScenePtr& scene,
                             const moveit::core::RobotModelPtr& model, const nlohmann::json& s,
                             const std::string& tip_link, double time_limit, double spacing) {
  const auto& arm = *model->getJointModelGroup(group);
  // The scene's state: the arm at the scene's start, holding what the scene attaches.
  const moveit::core::RobotState start(scene->getCurrentState());

  planning_interface::MotionPlanRequest request;
  request.group_name = group;
  request.pipeline_id = pipeline_namespace;
  request.planner_id = planner_id;
  request.num_planning_attempts = 1;
  request.allowed_planning_time = time_limit;
  request.max_velocity_scaling_factor = 1.0;
  request.max_acceleration_scaling_factor = 1.0;
  moveit::core::robotStateToRobotStateMsg(start, request.start_state);
  request.goal_constraints.push_back(goal_constraints(s.at("goal"), start, arm, tip_link));

  const auto seed = s.at("seed").get<std::uint32_t>();
  const auto level = ompl::msg::getLogLevel();
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(level);
  {
    const std::lock_guard<std::mutex> held(moveit_seeds_lock());
    moveit_seeds().seed(seed);
  }
  // KDL's step off a singularity draws from Eigen's Random, which reads std::rand.
  std::srand(seed);
  planning_interface::MotionPlanResponse response;
  const std::uint64_t built = moveit_generators();
  const bool planned = planner.generatePlan(scene, request, response);
  const int status = response.error_code.val;
  if ((status == moveit_success || status == moveit_invalid_motion_plan) &&
      moveit_generators() == built) {
    throw std::runtime_error(
        "MoveIt planned a scene with no random generator built by the reference program's "
        "constructor: its samplers draw from the clock");
  }

  nlohmann::json answer = {{"status", status},
                            {"collisions", nullptr},
                            {"clearance", nullptr},
                            {"held_clearance", nullptr},
                            {"length", nullptr}};
  if (!planned || status != moveit_success || !response.trajectory) {
    return answer;
  }
  std::vector<Eigen::VectorXd> points;
  for (std::size_t i = 0; i < response.trajectory->getWayPointCount(); ++i) {
    std::vector<double> q;
    response.trajectory->getWayPoint(i).copyJointGroupPositions(&arm, q);
    points.push_back(vector_of(q));
  }
  moveit::core::RobotState state(start);
  answer.update(measured(probe_polyline(*scene, state, arm, points, spacing, tip_link),
                         polyline_length(points)));
  return answer;
}

// MoveIt's probe of motion_planning_core's path `waypoints`, rounded by MoveIt's
// time-optimal path at `corner_tolerance` and probed at most `spacing` apart along it.
nlohmann::json ours_probed(const planning_scene::PlanningScene& scene,
                           const moveit::core::RobotModelPtr& model,
                           const nlohmann::json& waypoints, double corner_tolerance,
                           double spacing, const std::string& tip_link) {
  const auto& arm = *model->getJointModelGroup(group);
  std::vector<Eigen::VectorXd> points;
  for (const auto& w : waypoints) {
    points.push_back(vector_of(numbers(w)));
  }
  const auto path = trajectory_processing::Path::create(points, corner_tolerance);
  if (!path) {
    throw std::runtime_error("MoveIt's time-optimal path does not round a path of ours");
  }
  moveit::core::RobotState state(scene.getCurrentState());
  Probed probed = probe_of(scene, tip_link);
  const double length = path->getLength();
  const std::size_t steps = steps_of(length, spacing);
  for (std::size_t k = 0; k <= steps; ++k) {
    const double at = length * static_cast<double>(k) / static_cast<double>(steps);
    probe(scene, state, arm, path->getConfig(at), probed);
  }
  return measured(probed, length);
}

}  // namespace

nlohmann::json arm_planning(const nlohmann::json& scenario, Inputs& inputs) {
  const auto model = arm_model(scenario, inputs);
  const PlannerConfig config =
      planner_config(scenario.at("objective").get<std::string>(), pipeline_namespace);
  auto planner = pipeline(model, scenario, config);
  const auto tip_link = scenario.at("tip_link").get<std::string>();
  const double voxel_size = scenario.at("voxel_size").get<double>();
  // MoveIt keeps the plans' clearance as the padding of the robot's links.
  const double padding = scenario.at("min_clearance").get<double>();
  const double spacing = scenario.at("probe_spacing").get<double>();
  const double corner_tolerance = scenario.at("corner_tolerance").get<double>();
  const double time_limit = scenario.at("time_limit").get<double>();

  auto answers = nlohmann::json::array();
  for (const auto& s : scenario.at("scenes")) {
    // A scene of a pose goal loads MoveIt's inverse kinematics, which its constraint
    // sampler reads, as a solver of its own: its random restarts draw from a generator
    // that no scene before it drew from. A scene of a configuration loads none, and its
    // joint constraint reads none.
    if (s.at("goal").is_object()) {
      load_kdl(model);
    }
    const auto scene = scene_of(model, s, voxel_size, tip_link, padding);
    const auto& ours = s.at("ours");
    answers.push_back({
        {"moveit",
         moveit_answer(*planner, config.name, scene, model, s, tip_link, time_limit, spacing)},
        {"ours", ours.contains("waypoints")
                     ? ours_probed(*scene, model, ours.at("waypoints"), corner_tolerance, spacing,
                                   tip_link)
                     : nlohmann::json(nullptr)},
    });
  }
  return {{"kind", "arm_planning"}, {"scenes", answers}};
}

}  // namespace ros_reference
