#include "arm_scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>
#include <utility>

#include <Eigen/Geometry>
#include <chomp_motion_planner/chomp_parameters.hpp>
#include <chomp_motion_planner/chomp_planner.hpp>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.hpp>
#include <moveit/planning_interface/planning_request.hpp>
#include <moveit/planning_interface/planning_response.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <octomap/OcTree.h>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>
#include <rsl/random.hpp>

#include "planning_scene.hpp"

namespace ros_reference {
namespace {

// The generator of the seeds of MoveIt's own random generators, which a reference reseeds
// from each scene's seed before it runs MoveIt on the scene, and the lock under which a
// generator of MoveIt's, on the planner's thread or the goal sampler's, draws its seed.
std::mt19937& moveit_seeds() {
  static std::mt19937 seeds;
  return seeds;
}

std::mutex& moveit_seeds_lock() {
  static std::mutex lock;
  return lock;
}

}  // namespace

std::atomic<std::uint64_t>& moveit_generators() {
  static std::atomic<std::uint64_t> built{0};
  return built;
}

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

// The joints from which a chain holds a frame at any pose it reaches: a pose's freedoms.
constexpr std::size_t pose_freedoms = 6;

// OctoMap's key of the voxel at the origin of each axis.
constexpr int tree_max_val = 32768;
// The acceleration bound of each joint of the arm, in rad/s^2, for the pipeline's time
// parameterization: it times the path and leaves its shape to the path tolerance.
constexpr double max_acceleration = 5.0;

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

// The vector of `values`.
Eigen::VectorXd vector_of(const std::vector<double>& values) {
  return Eigen::Map<const Eigen::VectorXd>(values.data(),
                                           static_cast<Eigen::Index>(values.size()));
}

// The share of its tolerance that `target` reaches at `state`.
double share_of_tolerance(const HeldTarget& target, const moveit::core::RobotState& state) {
  const Eigen::Quaterniond actual(state.getGlobalLinkTransform(target.link).linear());
  if (!target.free_axis) {
    return target.orientation.angularDistance(actual) / target.tolerance;
  }
  const Eigen::Vector3d wanted = target.orientation * *target.free_axis;
  const Eigen::Vector3d held = actual * *target.free_axis;
  return std::atan2(wanted.cross(held).norm(), wanted.dot(held)) / target.tolerance;
}

// The parameters that MoveIt's CHOMP plugin loads from a node that sets none of them
// (moveit2 2.15.2 chomp_interface.cpp:55-88), with the input path as CHOMP's initial
// trajectory, `fillTrajectory`, which the plugin loads from
// chomp.trajectory_initialization_method.
chomp::ChompParameters plugin_parameters() {
  chomp::ChompParameters parameters;
  parameters.planning_time_limit_ = 10.0;
  parameters.max_iterations_ = 200;
  parameters.max_iterations_after_collision_free_ = 5;
  parameters.smoothness_cost_weight_ = 0.1;
  parameters.obstacle_cost_weight_ = 1.0;
  parameters.learning_rate_ = 0.01;
  parameters.smoothness_cost_velocity_ = 0.0;
  parameters.smoothness_cost_acceleration_ = 1.0;
  parameters.smoothness_cost_jerk_ = 0.0;
  parameters.ridge_factor_ = 0.0;
  parameters.use_pseudo_inverse_ = false;
  parameters.pseudo_inverse_ridge_factor_ = 1e-4;
  parameters.joint_update_limit_ = 0.1;
  parameters.min_clearance_ = 0.2;
  parameters.collision_threshold_ = 0.07;
  parameters.use_stochastic_descent_ = true;
  parameters.enable_failure_recovery_ = false;
  parameters.max_recovery_attempts_ = 5;
  if (!parameters.setTrajectoryInitializationMethod("fillTrajectory")) {
    throw std::runtime_error("CHOMP refuses the initial trajectory fillTrajectory");
  }
  return parameters;
}

// CHOMP's answer to `request` in `scene`, refining `input`, on a thread of its own whose
// random generator it seeds from `seed`: rsl gives each thread a generator of its own, which
// the thread's first call of rsl::rng seeds and a later seeded call refuses, and CHOMP's
// stochastic descent draws from it (chomp_optimizer.cpp:568).
planning_interface::MotionPlanDetailedResponse chomp_answer(
    const planning_scene::PlanningSceneConstPtr& scene,
    const planning_interface::MotionPlanRequest& request,
    const robot_trajectory::RobotTrajectoryPtr& input, std::uint32_t seed) {
  planning_interface::MotionPlanDetailedResponse response;
  response.trajectory.push_back(input);
  std::exception_ptr error;
  std::thread([&] {
    try {
      rsl::rng(std::seed_seq{seed});
      const chomp::ChompPlanner planner;
      planner.solve(scene, request, plugin_parameters(), response);
    } catch (...) {
      error = std::current_exception();
    }
  }).join();
  if (error) {
    std::rethrow_exception(error);
  }
  return response;
}

}  // namespace

void seed_moveit(std::uint32_t seed) {
  {
    const std::lock_guard<std::mutex> held(moveit_seeds_lock());
    moveit_seeds().seed(seed);
  }
  // KDL's step off a singularity draws from Eigen's Random, which reads std::rand.
  std::srand(seed);
}

ArmGroup group_of(const moveit::core::RobotModel& model, const nlohmann::json& scenario) {
  return arm_group(model, scenario.at("group").get<std::string>());
}

moveit::core::RobotModelPtr arm_model(const nlohmann::json& scenario, Inputs& inputs) {
  auto model = description_model(inputs, scenario.at("description").get<std::string>());
  // A URDF bounds no acceleration, and the pipeline's time parameterization needs one,
  // as a moveit_config's joint_limits.yaml gives it.
  for (const auto* joint :
       model->getJointModelGroup(group_of(*model, scenario).name)->getActiveJointModels()) {
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

void load_kdl(const moveit::core::RobotModelPtr& model,
              const std::vector<std::string>& solver_groups) {
  start_ros();
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  std::vector<rclcpp::Parameter> solvers;
  for (const auto& name : solver_groups) {
    const std::string kinematics = "robot_description_kinematics." + name;
    solvers.emplace_back(kinematics + ".kinematics_solver",
                         "kdl_kinematics_plugin/KDLKinematicsPlugin");
    if (model->getJointModelGroup(name)->getActiveJointModels().size() < pose_freedoms) {
      solvers.emplace_back(kinematics + ".position_only_ik", true);
    }
  }
  options.parameter_overrides(solvers);
  auto node = std::make_shared<rclcpp::Node>("arm_planning_kinematics", options);
  auto loader = std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>(node);
  const auto load = loader->getLoaderFunction(std::make_shared<srdf::Model>(*model->getSRDF()));
  // The model calls the loader for each solver it allocates after this, as a pose space does for
  // its own, so the loader lives as long as the allocators, as RobotModelLoader keeps it.
  const moveit::core::SolverAllocatorFn allocator =
      [loader, load](const moveit::core::JointModelGroup* group) { return load(group); };
  std::map<std::string, moveit::core::SolverAllocatorFn> allocators;
  for (const auto& name : loader->getKnownGroups()) {
    allocators[name] = allocator;
  }
  model->setKinematicsAllocators(allocators);
  for (const auto& [name, timeout] : loader->getIKTimeout()) {
    if (model->hasJointModelGroup(name)) {
      model->getJointModelGroup(name)->setDefaultIKTimeout(timeout);
    }
  }
  for (const auto& name : solver_groups) {
    if (!model->getJointModelGroup(name)->getSolverInstance()) {
      throw std::runtime_error("MoveIt's KDL plugin loads no inverse kinematics for " + name);
    }
  }
}

std::vector<double> numbers(const nlohmann::json& values) {
  return values.get<std::vector<double>>();
}

planning_scene::PlanningScenePtr scene_of(const moveit::core::RobotModelPtr& model,
                                         const nlohmann::json& s, double voxel_size,
                                         const ArmGroup& group, double padding) {
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
  state.setJointGroupPositions(group.name, numbers(s.at("start")));
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
  for (const auto& held : s.at("attached")) {
    attach(held, held.at("link").get<std::string>());
  }
  auto& acm = scene->getAllowedCollisionMatrixNonConst();
  const auto& links = model->getLinkModelNamesWithCollisionGeometry();
  acm.setEntry(links, links, true);
  const auto& arm = *model->getJointModelGroup(group.name);
  std::vector<std::string> standing;
  for (const auto* link : model->getLinkModelsWithCollisionGeometry()) {
    if (!moves_with_group(link, arm)) {
      acm.setDefaultEntry(link->getName(), true);
      standing.push_back(link->getName());
    }
  }
  for (const auto& held : s.at("attached")) {
    const auto id = held.at("object").at("id").get<std::string>();
    const auto link = held.at("link").get<std::string>();
    if (moves_with_group(model->getLinkModel(link), arm)) {
      // An object an arm of the group holds meets the links that stand still, as it meets
      // the parts of the robot motion_planning_core's plan does not move.
      acm.setEntry(id, standing, false);
    } else {
      // An object an arm outside the group holds stands still with that arm: it meets the
      // arms the plan moves and what they hold, as an obstacle such an arm holds meets them.
      acm.setEntry(id, scene->getWorld()->getObjectIds(), true);
      acm.setEntry(id, standing, true);
    }
  }
  // A group of every arm checks each pair of the robot's own links that the SRDF does not
  // disable; a group of some of the arms checks none, as a plan of some arms checks its
  // bodies against the scene alone.
  if (group.holds_every_arm) {
    for (std::size_t i = 0; i < links.size(); ++i) {
      for (std::size_t j = i + 1; j < links.size(); ++j) {
        if (!disabled_pair(*model, links[i], links[j])) {
          acm.setEntry(links[i], links[j], false);
        }
      }
    }
  }
  for (const auto& pair : s.at("allowed")) {
    acm.setEntry(pair.at(0).get<std::string>(), pair.at(1).get<std::string>(), true);
  }
  return scene;
}

std::vector<Eigen::VectorXd> points_of(const nlohmann::json& waypoints) {
  std::vector<Eigen::VectorXd> points;
  for (const auto& waypoint : waypoints) {
    points.push_back(vector_of(numbers(waypoint)));
  }
  return points;
}

std::vector<Eigen::VectorXd> points_of(const robot_trajectory::RobotTrajectory& trajectory,
                                       const moveit::core::JointModelGroup& arm) {
  std::vector<Eigen::VectorXd> points;
  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
    std::vector<double> q;
    trajectory.getWayPoint(i).copyJointGroupPositions(&arm, q);
    points.push_back(vector_of(q));
  }
  return points;
}

ChompRefined chomp_refined(const planning_scene::PlanningScene& scene,
                           const robot_trajectory::RobotTrajectoryPtr& input, std::uint32_t seed) {
  const auto& arm = *scene.getRobotModel()->getJointModelGroup(input->getGroupName());
  // The scene CHOMP's plugin plans in: a diff of the scene, its collision environment the
  // hybrid one (chomp_plugin.cpp:94-95).
  const auto hybrid = scene.diff();
  hybrid->allocateCollisionDetector(
      collision_detection::CollisionDetectorAllocatorHybrid::create());
  // CHOMP takes a goal of joints alone (chomp_planner.cpp:97-103): the path's last waypoint,
  // each joint within 0 of it.
  planning_interface::MotionPlanRequest request;
  request.group_name = input->getGroupName();
  moveit::core::robotStateToRobotStateMsg(scene.getCurrentState(), request.start_state);
  request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints(input->getLastWayPoint(), &arm, 0.0, 0.0));
  const auto response = chomp_answer(hybrid, request, input, seed);
  ChompRefined refined{response.error_code.val, std::nullopt};
  // Where CHOMP's optimizer ran, its trajectory takes the input's place in the response,
  // whether it succeeds or not (chomp_planner.cpp:257-273).
  if (response.trajectory.size() == 1 && response.trajectory.front() != input) {
    refined.path = points_of(*response.trajectory.front(), arm);
  }
  return refined;
}

Probing probing_of(const nlohmann::json& scenario, const ArmGroup& group,
                   const moveit::core::RobotModel& model) {
  Probing probing;
  probing.group = group.name;
  probing.self = group.holds_every_arm;
  for (const auto& arm : group.arms) {
    probing.tip_links.push_back(arm.tip_link);
  }
  probing.spacing = scenario.at("probe_spacing").get<double>();
  probing.corner_tolerance = scenario.at("corner_tolerance").get<double>();
  const auto& joints = model.getJointModelGroup(group.name)->getActiveJointModelNames();
  probing.max_velocity.resize(static_cast<Eigen::Index>(joints.size()));
  probing.max_acceleration.resize(static_cast<Eigen::Index>(joints.size()));
  for (std::size_t j = 0; j < joints.size(); ++j) {
    const auto& bounds = model.getVariableBounds(joints[j]);
    probing.max_velocity[static_cast<Eigen::Index>(j)] = bounds.max_velocity_;
    probing.max_acceleration[static_cast<Eigen::Index>(j)] = bounds.max_acceleration_;
  }
  return probing;
}

Probed probe_of(const planning_scene::PlanningScene& scene, const Probing& probing) {
  Probed probed{probing.self};
  probed.held_targets = probing.held;
  for (const auto& tip_link : probing.tip_links) {
    std::vector<const moveit::core::AttachedBody*> held;
    const auto* tip = scene.getRobotModel()->getLinkModel(tip_link);
    scene.getCurrentState().getAttachedBodies(held, tip);
    if (!held.empty()) {
      probed.holding.insert(tip);
    }
  }
  return probed;
}

void probe(const planning_scene::PlanningScene& scene, moveit::core::RobotState& state,
           const moveit::core::JointModelGroup& arm, const Eigen::VectorXd& q, Probed& probed) {
  state.setJointGroupPositions(&arm, q);
  state.update();
  for (const auto& target : probed.held_targets) {
    probed.share_of_tolerance =
        std::max(probed.share_of_tolerance, share_of_tolerance(target, state));
  }
  if (scene.isStateColliding(state, arm.getName())) {
    ++probed.collisions;
  }
  // FCL's distance to an octomap a body touches is no measure: the planning scene's dense
  // probe read -2.3e190 of one. MoveIt reads -1 for a body in contact otherwise, so -1
  // stands for every distance below it.
  probed.clearance =
      std::min(probed.clearance, std::max(scene.distanceToCollisionUnpadded(state), -1.0));
  if (!probed.holding.empty()) {
    collision_detection::DistanceRequest request;
    request.active_components_only = &probed.holding;
    request.acm = &scene.getAllowedCollisionMatrix();
    collision_detection::DistanceResult result;
    scene.getCollisionEnvUnpadded()->distanceRobot(request, result, state);
    probed.held = std::min(probed.held, std::max(result.minimum_distance.distance, -1.0));
  }
  // The unpadded environment: the scene pads the robot's links against the world alone,
  // and its self check reads them unpadded (pad_self_collisions false). -1 stands for
  // every distance below it, as for the world.
  if (probed.self) {
    const double self = scene.getCollisionEnvUnpadded()->distanceSelf(
        state, scene.getAllowedCollisionMatrix());
    probed.self_clearance = std::min(probed.self_clearance, std::max(self, -1.0));
  }
}

std::size_t steps_of(double length, double spacing) {
  return static_cast<std::size_t>(std::max(std::ceil(length / spacing), 1.0));
}

namespace {

// The path through `points`, straight between them, probed as `probing` asks.
Probed probe_polyline(const planning_scene::PlanningScene& scene,
                      moveit::core::RobotState& state, const moveit::core::JointModelGroup& arm,
                      const std::vector<Eigen::VectorXd>& points, const Probing& probing) {
  Probed probed = probe_of(scene, probing);
  probe(scene, state, arm, points.front(), probed);
  for (std::size_t i = 1; i < points.size(); ++i) {
    const std::size_t steps = steps_of((points[i] - points[i - 1]).norm(), probing.spacing);
    for (std::size_t k = 1; k <= steps; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(steps);
      probe(scene, state, arm, points[i - 1] + (points[i] - points[i - 1]) * t, probed);
    }
  }
  return probed;
}

// The length of the path through `points`, straight between them, in the joints' L2 norm.
double polyline_length(const std::vector<Eigen::VectorXd>& points) {
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += (points[i] - points[i - 1]).norm();
  }
  return length;
}

}  // namespace

trajectory_processing::Path rounded_path(const std::vector<Eigen::VectorXd>& points,
                                         double corner_tolerance) {
  auto path = trajectory_processing::Path::create(points, corner_tolerance);
  if (!path) {
    throw std::runtime_error("MoveIt's time-optimal path does not round a path");
  }
  return std::move(*path);
}

nlohmann::json measured(const Probed& probed, double length,
                        const trajectory_processing::Path& rounded, const Probing& probing) {
  const double rounded_length = rounded.getLength();
  const std::size_t steps = steps_of(rounded_length, probing.spacing);
  const double step = rounded_length / static_cast<double>(steps);
  double turning = 0.0;
  double bending = 0.0;
  for (std::size_t k = 0; k < steps; ++k) {
    const double curvature =
        rounded.getCurvature((static_cast<double>(k) + 0.5) * step).norm();
    turning += curvature * step;
    bending += curvature * curvature * step;
  }
  const auto timed = trajectory_processing::Trajectory::create(rounded, probing.max_velocity,
                                                               probing.max_acceleration);
  return {{"collisions", probed.collisions},
          {"clearance", probed.clearance},
          {"held_clearance",
           probed.holding.empty() ? nlohmann::json(nullptr) : nlohmann::json(probed.held)},
          {"self_clearance",
           probed.self ? nlohmann::json(probed.self_clearance) : nlohmann::json(nullptr)},
          {"share_of_tolerance", probed.held_targets.empty()
                                     ? nlohmann::json(nullptr)
                                     : nlohmann::json(probed.share_of_tolerance)},
          {"length", length},
          {"turning", turning},
          {"bending", bending},
          {"duration", timed ? nlohmann::json(timed->getDuration()) : nlohmann::json(nullptr)}};
}

nlohmann::json polyline_measured(const planning_scene::PlanningScene& scene,
                                 moveit::core::RobotState& state,
                                 const moveit::core::JointModelGroup& arm,
                                 const std::vector<Eigen::VectorXd>& points,
                                 const Probing& probing) {
  return measured(probe_polyline(scene, state, arm, points, probing), polyline_length(points),
                  rounded_path(points, probing.corner_tolerance), probing);
}

// MoveIt's probe of motion_planning_core's path `waypoints`, rounded by MoveIt's
// time-optimal path at the probing's corner tolerance and probed along it as `probing` asks.
nlohmann::json ours_probed(const planning_scene::PlanningScene& scene,
                           const moveit::core::RobotModelPtr& model,
                           const nlohmann::json& waypoints, const Probing& probing) {
  const auto& arm = *model->getJointModelGroup(probing.group);
  const auto path = rounded_path(points_of(waypoints), probing.corner_tolerance);
  moveit::core::RobotState state(scene.getCurrentState());
  Probed probed = probe_of(scene, probing);
  const double length = path.getLength();
  const std::size_t steps = steps_of(length, probing.spacing);
  for (std::size_t k = 0; k <= steps; ++k) {
    const double at = length * static_cast<double>(k) / static_cast<double>(steps);
    probe(scene, state, arm, path.getConfig(at), probed);
  }
  return measured(probed, length, path, probing);
}

}  // namespace ros_reference
