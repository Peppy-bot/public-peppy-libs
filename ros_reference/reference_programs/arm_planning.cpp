#include "arm_planning.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
#include <rclcpp/rclcpp.hpp>

#include "arm_scene.hpp"

namespace ros_reference {
namespace {

constexpr const char* pipeline_namespace = "ompl";
// MoveIt's error codes of a plan that succeeds, and of one whose planned path fails
// MoveIt's own check: both ran the planner.
constexpr int moveit_success = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
constexpr int moveit_invalid_motion_plan = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;

// The configuration of the scenario's planner, by its name and its parameters: RRTConnect
// for `rrt_connect` and RRTstar for `rrt_star`, each at the parameters of MoveIt's
// default planner configurations (moveit2 2.15.2
// moveit_configs_utils/default_configs/ompl_defaults.yaml, RRTConnect and RRTstar).
struct PlannerConfig {
  std::string name;
  std::vector<rclcpp::Parameter> parameters;
};

PlannerConfig planner_config(const std::string& planner, const std::string& ns) {
  const auto config = [&](const std::string& name, const std::string& type,
                          std::vector<std::pair<std::string, rclcpp::ParameterValue>> values) {
    std::vector<rclcpp::Parameter> parameters{{ns + ".planner_configs." + name + ".type", type}};
    for (auto& [key, value] : values) {
      parameters.emplace_back(ns + ".planner_configs." + name + "." + key, std::move(value));
    }
    return PlannerConfig{name, parameters};
  };
  if (planner == "rrt_connect") {
    return config("RRTConnectkConfigDefault", "geometric::RRTConnect",
                  {{"range", rclcpp::ParameterValue(0.0)}});
  }
  if (planner == "rrt_star") {
    return config("RRTstarkConfigDefault", "geometric::RRTstar",
                  {{"range", rclcpp::ParameterValue(0.0)},
                   {"goal_bias", rclcpp::ParameterValue(0.05)},
                   {"delay_collision_checking", rclcpp::ParameterValue(1)}});
  }
  throw std::runtime_error("the scenario's planner is rrt_connect or rrt_star, and it is " +
                           planner);
}

// Whether MoveIt's CHOMP refines the path the planner answers, as the scenario's refiner
// says: `chomp`, or `none`.
bool refined_by_chomp(const std::string& refiner) {
  if (refiner == "chomp") {
    return true;
  }
  if (refiner != "none") {
    throw std::runtime_error("the scenario's refiner is none or chomp, and it is " + refiner);
  }
  return false;
}

// The pipeline: the OMPL planner the scenario names, ended as its termination
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

// Whether `goal` is a pose of each chain's tip link, and no configuration.
bool is_poses(const nlohmann::json& goal) {
  return !goal.empty() && goal.front().is_object();
}

// The orientations the scene `s` holds of the links of `model`.
std::vector<HeldTarget> held_targets_of(const nlohmann::json& s,
                                        const moveit::core::RobotModel& model) {
  std::vector<HeldTarget> targets;
  for (const auto& held : s.at("path_orientations")) {
    const auto orientation = numbers(held.at("orientation"));
    const auto free_axis = held.at("free_axis").get<std::string>();
    std::optional<Eigen::Vector3d> axis;
    if (free_axis == "x") {
      axis = Eigen::Vector3d::UnitX();
    } else if (free_axis == "y") {
      axis = Eigen::Vector3d::UnitY();
    } else if (free_axis == "z") {
      axis = Eigen::Vector3d::UnitZ();
    } else if (free_axis != "none") {
      throw std::runtime_error("free_axis is none, x, y or z, and it is " + free_axis);
    }
    targets.push_back({model.getLinkModel(held.at("link_name").get<std::string>()),
                       Eigen::Quaterniond(orientation.at(3), orientation.at(0), orientation.at(1),
                                          orientation.at(2)),
                       axis, held.at("tolerance_rad").get<double>()});
  }
  return targets;
}

// MoveIt's constraints of a pose `goal` of the link `tip_link` in the frame of `model`:
// its position within the radius `position_tolerance` and each component of the rotation
// vector from its target orientation within `orientation_tolerance`
// (constructGoalConstraints, ROTATION_VECTOR, moveit2 2.15.2 utils.cpp:220-226); a goal
// that names no orientation, its position alone.
moveit_msgs::msg::Constraints pose_constraints(const nlohmann::json& goal,
                                               const moveit::core::RobotModel& model,
                                               const std::string& tip_link) {
  const auto position = numbers(goal.at("position"));
  if (!goal.contains("orientation")) {
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = model.getModelFrame();
    point.point.x = position.at(0);
    point.point.y = position.at(1);
    point.point.z = position.at(2);
    return kinematic_constraints::constructGoalConstraints(
        tip_link, point, goal.at("position_tolerance").get<double>());
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = model.getModelFrame();
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

// MoveIt's constraints of the goal `goal`. A configuration is the goal state itself,
// each joint within 0 of it: every goal state that MoveIt's goal-sampling thread adds is
// that one state, however many it has added when the planner takes one. Poses, one of
// each chain's tip link of `tip_links` in their order, each as `pose_constraints` holds
// it, MoveIt samples by KDL's inverse kinematics, each goal sample's first attempt seeded
// from its working state, the start for the first sample.
moveit_msgs::msg::Constraints goal_constraints(const nlohmann::json& goal,
                                               const moveit::core::RobotState& start,
                                               const moveit::core::JointModelGroup& arm,
                                               const std::vector<std::string>& tip_links) {
  if (!is_poses(goal)) {
    moveit::core::RobotState state(start);
    state.setJointGroupPositions(&arm, numbers(goal));
    state.update();
    return kinematic_constraints::constructGoalConstraints(state, &arm, 0.0, 0.0);
  }
  if (goal.size() != tip_links.size()) {
    throw std::runtime_error("a goal of poses holds a pose of each chain's tip link");
  }
  const auto& model = *start.getRobotModel();
  auto constraints = pose_constraints(goal.front(), model, tip_links.front());
  for (std::size_t k = 1; k < tip_links.size(); ++k) {
    constraints = kinematic_constraints::mergeConstraints(
        constraints, pose_constraints(goal.at(k), model, tip_links.at(k)));
  }
  return constraints;
}

// MoveIt's path constraint of `held`, an orientation of a link in the model's frame with
// the rotation-vector tolerances the scenario names.
moveit_msgs::msg::OrientationConstraint path_orientation(const nlohmann::json& held,
                                                         const moveit::core::RobotModel& model) {
  const auto orientation = numbers(held.at("orientation"));
  const auto tolerances = numbers(held.at("moveit_tolerances"));
  moveit_msgs::msg::OrientationConstraint constraint;
  constraint.header.frame_id = model.getModelFrame();
  constraint.link_name = held.at("link_name").get<std::string>();
  constraint.orientation.x = orientation.at(0);
  constraint.orientation.y = orientation.at(1);
  constraint.orientation.z = orientation.at(2);
  constraint.orientation.w = orientation.at(3);
  constraint.absolute_x_axis_tolerance = tolerances.at(0);
  constraint.absolute_y_axis_tolerance = tolerances.at(1);
  constraint.absolute_z_axis_tolerance = tolerances.at(2);
  constraint.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
  constraint.weight = 1.0;
  return constraint;
}

// MoveIt's answer to a scene, planned within `time_limit` seconds by the planner
// `planner_id`, its path refined by CHOMP with `chomp`: the error code of the plan, or of
// CHOMP where it refines one, and the probe of the path when they planned one.
nlohmann::json moveit_answer(planning_pipeline::PlanningPipeline& planner,
                             const std::string& planner_id, bool chomp,
                             const planning_scene::PlanningScenePtr& scene,
                             const moveit::core::RobotModelPtr& model, const nlohmann::json& s,
                             double time_limit, const Probing& probing) {
  const auto& arm = *model->getJointModelGroup(group);
  // The scene's state: the arms at the scene's start, holding what the scene attaches.
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
  request.goal_constraints.push_back(
      goal_constraints(s.at("goal"), start, arm, probing.tip_links));
  for (const auto& held : s.at("path_orientations")) {
    request.path_constraints.orientation_constraints.push_back(path_orientation(held, *model));
  }

  const auto seed = s.at("seed").get<std::uint32_t>();
  const auto level = ompl::msg::getLogLevel();
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  ompl::RNG::setSeed(seed);
  ompl::msg::setLogLevel(level);
  seed_moveit(seed);
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
                            {"self_clearance", nullptr},
                            {"share_of_tolerance", nullptr},
                            {"length", nullptr},
                            {"turning", nullptr},
                            {"bending", nullptr},
                            {"duration", nullptr}};
  if (!planned || status != moveit_success || !response.trajectory) {
    return answer;
  }
  const auto probed = [&](const std::vector<Eigen::VectorXd>& points) {
    moveit::core::RobotState state(start);
    answer.update(polyline_measured(*scene, state, arm, points, probing));
    return answer;
  };
  if (!chomp) {
    return probed(points_of(*response.trajectory, arm));
  }
  // CHOMP refines the path the pipeline answers, timed and checked by its response adapters,
  // and the scene's answer is CHOMP's.
  const auto refined = chomp_refined(*scene, response.trajectory, seed);
  answer["status"] = refined.status;
  if (refined.status != moveit_success) {
    return answer;
  }
  return probed(refined.path.value());
}

}  // namespace

nlohmann::json arm_planning(const nlohmann::json& scenario, Inputs& inputs) {
  const ChainGroup chains = chains_of(scenario);
  const auto model = arm_model(scenario, chains, inputs);
  const PlannerConfig config =
      planner_config(scenario.at("planner").get<std::string>(), pipeline_namespace);
  const bool chomp = refined_by_chomp(scenario.at("refiner").get<std::string>());
  auto planner = pipeline(model, scenario, config);
  const auto& checked_pairs = scenario.at("checked_pairs");
  Probing probing = probing_of(scenario, chains, *model);
  // The probes measure the robot's own bodies apart where motion_planning_core's self
  // model checks pairs of them.
  probing.self = !checked_pairs.empty();
  const double voxel_size = scenario.at("voxel_size").get<double>();
  // MoveIt keeps the plans' clearance as the padding of the robot's links.
  const double padding = scenario.at("min_clearance").get<double>();
  const double time_limit = scenario.at("time_limit").get<double>();

  auto answers = nlohmann::json::array();
  for (const auto& s : scenario.at("scenes")) {
    // A scene of a pose goal loads MoveIt's inverse kinematics, which its constraint
    // sampler reads, as a solver of its own: its random restarts draw from a generator
    // that no scene before it drew from. A scene of a configuration loads none, and its
    // joint constraint reads none.
    if (is_poses(s.at("goal"))) {
      load_kdl(model, chains.solver_groups());
    }
    const auto scene = scene_of(model, s, voxel_size, checked_pairs, padding);
    const auto& ours = s.at("ours");
    Probing of_the_scene = probing;
    of_the_scene.held = held_targets_of(s, *model);
    answers.push_back({
        {"moveit",
         moveit_answer(*planner, config.name, chomp, scene, model, s, time_limit,
                       of_the_scene)},
        {"ours", ours.contains("waypoints")
                     ? ours_probed(*scene, model, ours.at("waypoints"), of_the_scene)
                     : nlohmann::json(nullptr)},
    });
  }
  return {{"kind", "arm_planning"}, {"scenes", answers}};
}

}  // namespace ros_reference
