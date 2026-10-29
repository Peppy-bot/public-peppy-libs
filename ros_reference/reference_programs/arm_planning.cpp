#include "arm_planning.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometric_shapes/shapes.h>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <octomap/OcTree.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>

#include "robot.hpp"

namespace ros_reference {
namespace {

// The generator of the seeds of MoveIt's own random generators, which the reference
// reseeds from each scene's seed before it plans the scene.
std::mt19937& moveit_seeds() {
  static std::mt19937 seeds;
  return seeds;
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
    : RandomNumberGenerator(static_cast<std::uint32_t>(ros_reference::moveit_seeds()())) {
  ++ros_reference::moveit_generators();
}

namespace ros_reference {
namespace {

constexpr const char* group = "arm";
constexpr const char* pipeline_namespace = "ompl";
constexpr const char* planner_config = "RRTConnectkConfigDefault";
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

// The pipeline: OMPL's RRTConnect, ended by `iterations` (MoveIt's termination condition
// Iteration), MoveIt's default request adapters, a start outside a joint limit by any
// amount refused, and the response adapters AddTimeOptimalParameterization and
// ValidateSolution: MoveIt's defaults but DisplayMotionPath, which publishes the path for
// display.
std::shared_ptr<planning_pipeline::PlanningPipeline> pipeline(
    const moveit::core::RobotModelPtr& model, std::uint64_t iterations) {
  start_ros();
  const std::string ns = pipeline_namespace;
  const std::string config = ns + ".planner_configs." + planner_config;
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
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
      {ns + "." + group + ".planner_configs", std::vector<std::string>{planner_config}},
      {config + ".type", std::string("geometric::RRTConnect")},
      {config + ".termination_condition", "Iteration[" + std::to_string(iterations) + "]"},
  });
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

// The scene of `boxes`, each the voxels from its first key to its last on every axis, in
// an octomap of `voxel_size`: collisions between the robot's own links allowed, and those
// of every link that does not move with the group's joints: the body, both arms' bases,
// the head camera, and the other arm and its gripper.
planning_scene::PlanningScenePtr scene_of(const moveit::core::RobotModelPtr& model,
                                         const nlohmann::json& boxes, double voxel_size) {
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  auto tree = std::make_shared<octomap::OcTree>(voxel_size);
  for (const auto& box : boxes) {
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
  auto& acm = scene->getAllowedCollisionMatrixNonConst();
  const auto& links = model->getLinkModelNamesWithCollisionGeometry();
  acm.setEntry(links, links, true);
  const auto& arm = *model->getJointModelGroup(group);
  for (const auto* link : model->getLinkModelsWithCollisionGeometry()) {
    if (!moves_with_group(link, arm)) {
      acm.setDefaultEntry(link->getName(), true);
    }
  }
  return scene;
}

// MoveIt's collision check and distance at the probes of a joint path.
struct Probed {
  std::size_t collisions = 0;
  double clearance = std::numeric_limits<double>::infinity();
};

void probe(const planning_scene::PlanningScene& scene, moveit::core::RobotState& state,
           const moveit::core::JointModelGroup& arm, const Eigen::VectorXd& q, Probed& probed) {
  state.setJointGroupPositions(&arm, q);
  state.update();
  if (scene.isStateColliding(state, group)) {
    ++probed.collisions;
  }
  probed.clearance = std::min(probed.clearance, scene.distanceToCollision(state));
}

// The number of probes of a stretch `length` long, each at most `spacing` from the next.
std::size_t steps_of(double length, double spacing) {
  return static_cast<std::size_t>(std::max(std::ceil(length / spacing), 1.0));
}

// MoveIt's trajectory, straight between its points, probed at most `spacing` apart.
Probed probe_polyline(const planning_scene::PlanningScene& scene,
                      moveit::core::RobotState& state, const moveit::core::JointModelGroup& arm,
                      const std::vector<Eigen::VectorXd>& points, double spacing) {
  Probed probed;
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
  return {{"collisions", probed.collisions}, {"clearance", probed.clearance}, {"length", length}};
}

// MoveIt's answer to a scene, planned within `time_limit` seconds: its error code, and
// its path's probe when it planned one.
nlohmann::json moveit_answer(planning_pipeline::PlanningPipeline& planner,
                             const planning_scene::PlanningScenePtr& scene,
                             const moveit::core::RobotModelPtr& model, const nlohmann::json& s,
                             double time_limit, double spacing) {
  const auto& arm = *model->getJointModelGroup(group);
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  start.setJointGroupPositions(&arm, numbers(s.at("start")));
  start.update();
  moveit::core::RobotState goal(start);
  goal.setJointGroupPositions(&arm, numbers(s.at("goal")));
  goal.update();

  planning_interface::MotionPlanRequest request;
  request.group_name = group;
  request.pipeline_id = pipeline_namespace;
  request.planner_id = planner_config;
  request.num_planning_attempts = 1;
  request.allowed_planning_time = time_limit;
  request.max_velocity_scaling_factor = 1.0;
  request.max_acceleration_scaling_factor = 1.0;
  moveit::core::robotStateToRobotStateMsg(start, request.start_state);
  // The goal is the goal state itself, each joint within 0 of it: every goal state that
  // MoveIt's goal-sampling thread adds is that one state, however many it has added when
  // the planner takes one.
  request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(goal, &arm, 0.0, 0.0));

  const auto seed = s.at("seed").get<std::uint32_t>();
  const auto level = ompl::msg::getLogLevel();
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(level);
  moveit_seeds().seed(seed);
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

  nlohmann::json answer = {
      {"status", status}, {"collisions", nullptr}, {"clearance", nullptr}, {"length", nullptr}};
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
  answer.update(measured(probe_polyline(*scene, state, arm, points, spacing),
                         polyline_length(points)));
  return answer;
}

// MoveIt's probe of motion_planning_core's path `waypoints`, rounded by MoveIt's
// time-optimal path at `corner_tolerance` and probed at most `spacing` apart along it.
nlohmann::json ours_probed(const planning_scene::PlanningScene& scene,
                           const moveit::core::RobotModelPtr& model,
                           const nlohmann::json& waypoints, double corner_tolerance,
                           double spacing) {
  const auto& arm = *model->getJointModelGroup(group);
  std::vector<Eigen::VectorXd> points;
  for (const auto& w : waypoints) {
    points.push_back(vector_of(numbers(w)));
  }
  const auto path = trajectory_processing::Path::create(points, corner_tolerance);
  if (!path) {
    throw std::runtime_error("MoveIt's time-optimal path does not round a path of ours");
  }
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  Probed probed;
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
  auto planner = pipeline(model, scenario.at("iterations").get<std::uint64_t>());
  const double voxel_size = scenario.at("voxel_size").get<double>();
  const double spacing = scenario.at("probe_spacing").get<double>();
  const double corner_tolerance = scenario.at("corner_tolerance").get<double>();
  const double time_limit = scenario.at("time_limit").get<double>();

  auto answers = nlohmann::json::array();
  for (const auto& s : scenario.at("scenes")) {
    const auto scene = scene_of(model, s.at("boxes"), voxel_size);
    const auto& ours = s.at("ours");
    answers.push_back({
        {"moveit", moveit_answer(*planner, scene, model, s, time_limit, spacing)},
        {"ours", ours.contains("waypoints")
                     ? ours_probed(*scene, model, ours.at("waypoints"), corner_tolerance, spacing)
                     : nlohmann::json(nullptr)},
    });
  }
  return {{"kind", "arm_planning"}, {"scenes", answers}};
}

}  // namespace ros_reference
