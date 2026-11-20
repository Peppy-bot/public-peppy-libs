#include "smac_2d.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_smac_planner/a_star.hpp>
#include <nav2_smac_planner/collision_checker.hpp>
#include <nav2_smac_planner/node_2d.hpp>
#include <nav2_smac_planner/smoother.hpp>
#include <nav2_smac_planner/types.hpp>
#include <nav2_smac_planner/utils.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav_msgs/msg/path.hpp>

#include "costmap.hpp"
#include "scenario.hpp"

namespace ros_reference {

namespace {

using nav2_smac_planner::AStarAlgorithm;
using nav2_smac_planner::Node2D;

// The planner's parameters, each read from the scenario as SmacPlanner2D::configure reads
// it from its node, with the same type.
struct PlannerParameters {
  float tolerance;
  float cost_travel_multiplier;
  bool allow_unknown;
  int max_iterations;
  int max_on_approach_iterations;
  int terminal_checking_interval;
  double max_planning_time;
  bool use_final_approach_orientation;
};

// The least max_planning_time a scenario gives: createPath and smoothImpl read the steady
// clock against it, and no search of a scenario runs this long, so no clock ends one.
constexpr double least_max_planning_time_s = 1e9;

PlannerParameters planner_parameters(const nlohmann::json& planner) {
  const auto max_planning_time = planner.at("max_planning_time").get<double>();
  if (!(max_planning_time >= least_max_planning_time_s)) {
    throw std::runtime_error(
        "max_planning_time " + std::to_string(max_planning_time) +
        " s lets the steady clock end a search: give " +
        std::to_string(least_max_planning_time_s) + " s or more");
  }
  return {
      static_cast<float>(planner.at("tolerance").get<double>()),
      static_cast<float>(planner.at("cost_travel_multiplier").get<double>()),
      planner.at("allow_unknown").get<bool>(),
      planner.at("max_iterations").get<int>(),
      planner.at("max_on_approach_iterations").get<int>(),
      planner.at("terminal_checking_interval").get<int>(),
      max_planning_time,
      planner.at("use_final_approach_orientation").get<bool>(),
  };
}

// The smoother's parameters, as SmootherParams::get reads them, with holonomic_ true as
// SmacPlanner2D sets it.
nav2_smac_planner::SmootherParams smoother_parameters(const nlohmann::json& smoother,
                                                     int max_iterations) {
  nav2_smac_planner::SmootherParams params;
  params.tolerance_ = smoother.at("tolerance").get<double>();
  params.max_its_ = max_iterations;
  params.w_data_ = smoother.at("w_data").get<double>();
  params.w_smooth_ = smoother.at("w_smooth").get<double>();
  params.do_refinement_ = smoother.at("do_refinement").get<bool>();
  params.refinement_num_ = smoother.at("refinement_num").get<int>();
  params.holonomic_ = true;
  return params;
}

// One planner as SmacPlanner2D holds it after configure: its A*, collision checker and
// smoother, over the scenario's costmap.
class Planner {
 public:
  Planner(nav2_costmap_2d::Costmap2D* costmap, const PlannerParameters& parameters,
          const nav2_smac_planner::SmootherParams& smoother_params,
          const std::vector<geometry_msgs::msg::Point>& footprint)
      : costmap_(costmap),
        parameters_(parameters),
        footprint_(footprint),
        collision_checker_(nullptr, 1, nullptr),
        smoother_(smoother_params) {
    nav2_smac_planner::SearchInfo search_info;
    search_info.cost_penalty = parameters_.cost_travel_multiplier;
    collision_checker_.setCostmap(costmap_);
    collision_checker_.setFootprint(footprint_, true, 0.0);
    a_star_ = std::make_unique<AStarAlgorithm<Node2D>>(nav2_smac_planner::MotionModel::TWOD,
                                                      search_info);
    int max_iterations = parameters_.max_iterations;
    a_star_->initialize(parameters_.allow_unknown, max_iterations,
                        parameters_.max_on_approach_iterations,
                        parameters_.terminal_checking_interval, parameters_.max_planning_time,
                        0.0, 1);
    smoother_.initialize(1e-50);
  }

  // SmacPlanner2DT<Node2D>::createPlan, step for step, with the costmap not downsampled,
  // no viapoints and no cancel, and the robot's footprint, a polygon, given to the
  // smoother.
  nav_msgs::msg::Path create_plan(const geometry_msgs::msg::PoseStamped& start,
                                  const geometry_msgs::msg::PoseStamped& goal) {
    using namespace std::chrono;
    steady_clock::time_point a = steady_clock::now();
    nav2_costmap_2d::Costmap2D* costmap = costmap_;
    a_star_->setCollisionChecker(&collision_checker_);

    float mx_start, my_start, mx_goal, my_goal;
    if (!costmap->worldToMapContinuous(start.pose.position.x, start.pose.position.y, mx_start,
                                       my_start)) {
      throw nav2_core::StartOutsideMapBounds("start outside the map");
    }
    a_star_->setStart(mx_start, my_start, 0);
    if (!costmap->worldToMapContinuous(goal.pose.position.x, goal.pose.position.y, mx_goal,
                                       my_goal)) {
      throw nav2_core::GoalOutsideMapBounds("goal outside the map");
    }
    a_star_->setGoal(mx_goal, my_goal, 0);

    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    if (std::floor(mx_start) == std::floor(mx_goal) &&
        std::floor(my_start) == std::floor(my_goal)) {
      pose.pose = goal.pose;
      if (start.pose.orientation != goal.pose.orientation &&
          parameters_.use_final_approach_orientation) {
        pose.pose.orientation = start.pose.orientation;
      }
      plan.poses.push_back(pose);
      return plan;
    }

    Node2D::CoordinateVector path;
    int num_iterations = 0;
    if (!a_star_->createPath(path, num_iterations,
                             parameters_.tolerance / static_cast<float>(costmap->getResolution()),
                             [] { return false; })) {
      if (num_iterations == 1) {
        throw nav2_core::StartOccupied("Start occupied");
      }
      if (num_iterations < a_star_->getMaxIterations()) {
        throw nav2_core::NoValidPathCouldBeFound("no valid path found");
      }
      throw nav2_core::PlannerTimedOut("exceeded maximum iterations");
    }

    const bool reached_goal_cell = std::floor(path.front().x) == std::floor(mx_goal) &&
                                   std::floor(path.front().y) == std::floor(my_goal);
    plan.poses.reserve(path.size());
    for (int i = path.size() - 1; i >= 0; --i) {
      pose.pose = nav2_smac_planner::getWorldCoords(path[i].x, path[i].y, costmap);
      plan.poses.push_back(pose);
    }

    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    double time_remaining = parameters_.max_planning_time - static_cast<double>(time_span.count());
    smoother_.smooth(plan, costmap, time_remaining, footprint_);

    size_t plan_size = plan.poses.size();
    if (reached_goal_cell && plan_size > 0) {
      plan.poses.back().pose.position = goal.pose.position;
    }
    if (parameters_.use_final_approach_orientation) {
      if (plan_size == 1) {
        plan.poses.back().pose.orientation = start.pose.orientation;
      } else if (plan_size > 1) {
        double dx, dy, theta;
        auto last_pose = plan.poses.back().pose.position;
        auto approach_pose = plan.poses[plan_size - 2].pose.position;
        dx = last_pose.x - approach_pose.x;
        dy = last_pose.y - approach_pose.y;
        theta = atan2(dy, dx);
        plan.poses.back().pose.orientation =
            nav2_util::geometry_utils::orientationAroundZAxis(theta);
      }
    } else if (plan_size > 0) {
      plan.poses.back().pose.orientation = goal.pose.orientation;
    }
    return plan;
  }

 private:
  nav2_costmap_2d::Costmap2D* costmap_;
  PlannerParameters parameters_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  nav2_smac_planner::GridCollisionChecker collision_checker_;
  std::unique_ptr<AStarAlgorithm<Node2D>> a_star_;
  nav2_smac_planner::Smoother smoother_;
};

// The plan of `planner` from `start` to `goal`: "path" and its poses, or the failure
// Smac throws, by the name of its exception.
nlohmann::json planned(Planner& planner, const geometry_msgs::msg::PoseStamped& start,
                       const geometry_msgs::msg::PoseStamped& goal) {
  auto poses = nlohmann::json::array();
  try {
    for (const auto& pose : planner.create_plan(start, goal).poses) {
      poses.push_back(pose_read(pose.pose));
    }
  } catch (const nav2_core::StartOccupied&) {
    return {{"outcome", "start_occupied"}, {"poses", poses}};
  } catch (const nav2_core::GoalOccupied&) {
    return {{"outcome", "goal_occupied"}, {"poses", poses}};
  } catch (const nav2_core::NoValidPathCouldBeFound&) {
    return {{"outcome", "no_valid_path"}, {"poses", poses}};
  } catch (const nav2_core::PlannerTimedOut&) {
    return {{"outcome", "timed_out"}, {"poses", poses}};
  }
  return {{"outcome", "path"}, {"poses", poses}};
}

}  // namespace

nlohmann::json smac_2d(const nlohmann::json& scenario, Inputs&) {
  auto costmap = costmap_of(scenario.at("grid"), scenario.at("costs"));
  const auto footprint = footprint_of(scenario.at("footprint"));
  const auto& planner_json = scenario.at("planner");
  const auto& smoother_json = scenario.at("smoother");
  const auto parameters = planner_parameters(planner_json);
  const auto smoothed_params =
      smoother_parameters(smoother_json, smoother_json.at("max_iterations").get<int>());
  Planner raw(costmap.get(), parameters, smoother_parameters(smoother_json, 0), footprint);
  Planner smoothed(costmap.get(), parameters, smoothed_params, footprint);

  auto plans = nlohmann::json::array();
  for (const auto& entry : scenario.at("plans")) {
    const auto start = pose_stamped_of(entry.at("start"), "map");
    const auto goal = pose_stamped_of(entry.at("goal"), "map");
    const auto raw_plan = planned(raw, start, goal);
    const auto smoothed_plan = planned(smoothed, start, goal);
    plans.push_back({
        {"outcome", raw_plan.at("outcome")},
        {"raw", raw_plan.at("poses")},
        {"smoothed_outcome", smoothed_plan.at("outcome")},
        {"smoothed", smoothed_plan.at("poses")},
    });
  }
  return {
      {"kind", "smac_2d"},
      {"plans", plans},
  };
}

}  // namespace ros_reference
