#include "pure_pursuit.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_controller/plugins/feasible_path_handler.hpp>
#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_controller/plugins/simple_progress_checker.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>

#include "costmap.hpp"
#include "nav2_node.hpp"
#include "scenario.hpp"

namespace ros_reference {

namespace {

// The frames of the cost grid, the plan and the base's pose, and of the base.
constexpr const char* odometry_frame = "odom";
constexpr const char* base_frame = "base_link";
// The plugins' names, under which the node holds their parameters.
constexpr const char* controller_name = "FollowPath";
constexpr const char* goal_checker_name = "goal_checker";
constexpr const char* progress_checker_name = "progress_checker";
constexpr const char* path_handler_name = "path_handler";
// How long a transform may be missing before tf gives up, in seconds.
constexpr double transform_tolerance = 0.1;

// The cost map the controller and the path handler read: the scenario's grid as the
// master grid of a layered cost map of no layer, the scenario's footprint, unpadded, and
// the reference's tf buffer. The node is neither configured nor activated, so it runs no
// map update and no tf listener.
class GridCostmap : public nav2_costmap_2d::Costmap2DROS {
 public:
  GridCostmap(const nav2_costmap_2d::Costmap2D& grid, bool track_unknown_space,
              const std::vector<geometry_msgs::msg::Point>& footprint,
              const nav2::TransformBuffer::SharedPtr& tf)
      : Costmap2DROS("local_costmap", "/", false) {
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
        odometry_frame, false, track_unknown_space);
    layered_costmap_->resizeMap(grid.getSizeInCellsX(), grid.getSizeInCellsY(),
                                grid.getResolution(), grid.getOriginX(), grid.getOriginY());
    auto& master = *layered_costmap_->getCostmap();
    for (unsigned int y = 0; y < master.getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < master.getSizeInCellsX(); ++x) {
        master.setCost(x, y, grid.getCost(x, y));
      }
    }
    layered_costmap_->setFootprint(footprint);
    padded_footprint_ = std::make_shared<std::vector<geometry_msgs::msg::Point>>(footprint);
    unpadded_footprint_ = std::make_shared<std::vector<geometry_msgs::msg::Point>>(footprint);
    global_frame_ = odometry_frame;
    robot_base_frame_ = base_frame;
    transform_tolerance_ = transform_tolerance;
    tf_buffer_ = tf;
  }
};

// The base's pose in the odometry frame at `stamp_ns`, as tf holds it.
geometry_msgs::msg::TransformStamped base_transform(const geometry_msgs::msg::PoseStamped& pose) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header = pose.header;
  transform.child_frame_id = base_frame;
  transform.transform.translation.x = pose.pose.position.x;
  transform.transform.translation.y = pose.pose.position.y;
  transform.transform.translation.z = pose.pose.position.z;
  transform.transform.rotation = pose.pose.orientation;
  return transform;
}

geometry_msgs::msg::Twist twist_of(const nlohmann::json& axes) {
  const auto values = axes.get<std::vector<double>>();
  if (values.size() != 3) {
    throw std::runtime_error("a speed is not linear x, linear y and angular z");
  }
  geometry_msgs::msg::Twist twist;
  twist.linear.x = values[0];
  twist.linear.y = values[1];
  twist.angular.z = values[2];
  return twist;
}

nlohmann::json twist_read(const geometry_msgs::msg::Twist& twist) {
  return {twist.linear.x, twist.linear.y, twist.angular.z};
}

// The index in `plan` of the pose at `pose`'s position.
std::size_t index_in(const nav_msgs::msg::Path& plan, const geometry_msgs::msg::Pose& pose) {
  for (std::size_t i = 0; i < plan.poses.size(); ++i) {
    const auto& position = plan.poses[i].pose.position;
    if (position.x == pose.position.x && position.y == pose.position.y) {
      return i;
    }
  }
  throw std::runtime_error("the pruned plan holds a pose the plan does not");
}

}  // namespace

nlohmann::json pure_pursuit(const nlohmann::json& scenario, Inputs&) {
  const auto& controller_parameters = scenario.at("controller");
  const auto& goal_checker_parameters = scenario.at("goal_checker");
  const auto& progress_checker_parameters = scenario.at("progress_checker");
  const auto& path_handler_parameters = scenario.at("path_handler");
  auto node = nav2_node("pure_pursuit_reference",
                        {{controller_name, controller_parameters},
                         {goal_checker_name, goal_checker_parameters},
                         {progress_checker_name, progress_checker_parameters},
                         {path_handler_name, path_handler_parameters}},
                        {{"controller_frequency", scenario.at("controller_frequency")}});
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  const auto grid = costmap_of(scenario.at("grid"), scenario.at("costs"));
  const auto footprint = footprint_of(scenario.at("footprint"));
  const auto track_unknown_space = scenario.at("track_unknown_space").get<bool>();
  auto costmap = std::make_shared<GridCostmap>(*grid, track_unknown_space, footprint, tf);

  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController controller;
  controller.configure(node, controller_name, tf, costmap);
  check_declared(node, controller_name, controller_parameters);
  nav2_controller::SimpleGoalChecker goal_checker;
  goal_checker.initialize(node, goal_checker_name, costmap);
  check_declared(node, goal_checker_name, goal_checker_parameters);
  nav2_controller::SimpleProgressChecker progress_checker;
  progress_checker.initialize(node, progress_checker_name);
  check_declared(node, progress_checker_name, progress_checker_parameters);
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), path_handler_name, costmap, tf);
  check_declared(node, path_handler_name, path_handler_parameters);

  // ControllerServer::setPlannerPath with the scenario's plan.
  nav_msgs::msg::Path plan;
  plan.header.frame_id = odometry_frame;
  auto plan_read = nlohmann::json::array();
  for (const auto& entry : scenario.at("plan")) {
    plan.poses.push_back(pose_stamped_of(entry, odometry_frame));
    plan_read.push_back(pose_read(plan.poses.back().pose));
  }
  controller.newPathReceived(plan);
  path_handler.setPlan(plan);
  goal_checker.reset();
  progress_checker.reset();

  auto steps = nlohmann::json::array();
  for (const auto& step : scenario.at("steps")) {
    const auto at_ns = step.at("at_ns").get<std::int64_t>();
    set_ros_time(node->get_clock(), at_ns);
    auto pose = pose_stamped_of(step.at("pose"), odometry_frame);
    pose.header.stamp = rclcpp::Time(at_ns, RCL_ROS_TIME);
    tf->setTransform(base_transform(pose), "pure_pursuit_reference", false);
    const auto speed = twist_of(step.at("speed"));
    nlohmann::json record = {
        {"at_ns", at_ns},
        {"pose", pose_read(pose.pose)},
        {"speed", twist_read(speed)},
    };
    // ControllerServer::transformedPlanAndGoal, isGoalReached and
    // computeAndPublishVelocity, in that order.
    nav_msgs::msg::Path pruned;
    try {
      const auto [closest_point, pruned_plan_end] = path_handler.findPlanSegment(pose);
      pruned = path_handler.transformLocalPlan(closest_point, pruned_plan_end);
    } catch (const nav2_core::InvalidPath&) {
      record.update({{"pruned", nlohmann::json::array()},
                     {"outcome", "plan_off_grid"},
                     {"goal_reached", nullptr},
                     {"progress", nullptr},
                     {"twist", nlohmann::json::array()}});
      steps.push_back(record);
      continue;
    }
    record["pruned"] = {index_in(plan, pruned.poses.front().pose), pruned.poses.size()};
    const auto goal = path_handler.getTransformedGoal(pose.header.stamp);
    const bool reached = goal_checker.isGoalReached(pose.pose, goal.pose, speed, pruned);
    record["goal_reached"] = reached;
    if (reached) {
      record.update({{"outcome", "goal_reached"},
                     {"progress", nullptr},
                     {"twist", nlohmann::json::array()}});
      steps.push_back(record);
      continue;
    }
    record["progress"] = progress_checker.check(pose);
    try {
      const auto command =
          controller.computeVelocityCommands(pose, speed, &goal_checker, pruned, goal);
      record.update({{"outcome", "command"}, {"twist", twist_read(command.twist)}});
    } catch (const nav2_core::NoValidControl&) {
      record.update({{"outcome", "no_valid_control"}, {"twist", nlohmann::json::array()}});
    } catch (const nav2_core::ControllerException&) {
      record.update({{"outcome", "base_off_grid"}, {"twist", nlohmann::json::array()}});
    }
    steps.push_back(record);
  }

  return {
      {"kind", "pure_pursuit"},
      {"grid", grid_read(*grid)},
      {"costs", costs_read(*grid)},
      {"footprint", footprint_read(footprint)},
      {"inscribed_radius", costmap->getLayeredCostmap()->getInscribedRadius()},
      {"circumscribed_radius", costmap->getLayeredCostmap()->getCircumscribedRadius()},
      {"plan", plan_read},
      {"steps", steps},
  };
}

}  // namespace ros_reference
