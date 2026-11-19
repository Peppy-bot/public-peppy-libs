#include "collision_monitor.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav2_collision_monitor/collision_monitor_node.hpp>
#include <nav2_collision_monitor/pointcloud.hpp>
#include <nav2_collision_monitor/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "nav2_node.hpp"
#include "obstacles.hpp"
#include "private_member.hpp"
#include "scenario.hpp"

namespace ros_reference {

namespace {

// The point cloud source's data callback, in reach: the source takes a cloud only through
// the callback its subscription calls.
struct PointCloudAccess : nav2_collision_monitor::PointCloud {
  using PointCloud::dataCallback;
};

// The monitor with its configuration, its sources, its processing and its last action in
// reach: the node is configured and left inactive, with its tf listener stopped, so it
// runs no executor and no bond, and the reference calls its processing on its own thread.
// on_configure makes the tf listener (nav2_collision_monitor 1.5.2
// collision_monitor_node.cpp:52-53), and the monitor waits until the listener's thread
// spins wherever it destroys the listener: in configure, in on_cleanup, which on_configure
// calls when it refuses a parameter (:61, :96), and in its destructor, which an exception
// out of on_configure reaches.
class Monitor : public nav2_collision_monitor::CollisionMonitor {
 public:
  using CollisionMonitor::CollisionMonitor;

  ~Monitor() override { stop_tf_listener(); }

  void configure() {
    if (on_configure(get_current_state()) != nav2::CallbackReturn::SUCCESS) {
      throw std::runtime_error("the collision monitor refuses the scenario's parameters");
    }
    stop_tf_listener();
    process_active_ = true;
  }

  // The base frame, the frame of every cloud: the source reads its points untransformed.
  const std::string& base_frame() const { return base_frame_id_; }

  void take(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (sources_.size() != 1) {
      throw std::runtime_error("the monitor holds no single source");
    }
    auto source = std::dynamic_pointer_cast<nav2_collision_monitor::PointCloud>(sources_.front());
    if (!source) {
      throw std::runtime_error("the monitor's source is not a point cloud");
    }
    ((*source).*(&PointCloudAccess::dataCallback))(cloud);
  }

  nav2_collision_monitor::Action decide(const nav2_collision_monitor::Velocity& velocity) {
    process(velocity, std_msgs::msg::Header());
    return robot_action_prev_;
  }

  std::vector<nav2_collision_monitor::Point> polygon() const {
    if (polygons_.size() != 1) {
      throw std::runtime_error("the monitor holds no single polygon");
    }
    std::vector<nav2_collision_monitor::Point> points;
    polygons_.front()->getPolygon(points);
    return points;
  }

 protected:
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override {
    stop_tf_listener();
    return CollisionMonitor::on_cleanup(state);
  }

 private:
  void stop_tf_listener() {
    if (tf_listener_) {
      wait_until_spinning(*tf_listener_);
      tf_listener_.reset();
    }
  }
};

const char* action_name(nav2_collision_monitor::ActionType type) {
  switch (type) {
    case nav2_collision_monitor::DO_NOTHING:
      return "do_nothing";
    case nav2_collision_monitor::STOP:
      return "stop";
    case nav2_collision_monitor::SLOWDOWN:
      return "slowdown";
    case nav2_collision_monitor::APPROACH:
      return "approach";
    case nav2_collision_monitor::LIMIT:
      return "limit";
  }
  throw std::runtime_error("an action of no known type");
}

nav2_collision_monitor::Velocity velocity_of(const nlohmann::json& axes) {
  const auto values = axes.get<std::vector<double>>();
  if (values.size() != 3) {
    throw std::runtime_error("a velocity is not linear x, linear y and angular z");
  }
  return {values[0], values[1], values[2]};
}

nlohmann::json velocity_read(const nav2_collision_monitor::Velocity& velocity) {
  return {velocity.x, velocity.y, velocity.tw};
}

}  // namespace

nlohmann::json collision_monitor(const nlohmann::json& scenario, Inputs&) {
  start_ros();
  const auto& parameters = scenario.at("monitor");
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameter_overrides(parameters));

  auto polygon_read = nlohmann::json::array();
  auto cases = nlohmann::json::array();
  for (const auto& entry : scenario.at("cases")) {
    auto monitor = std::make_shared<Monitor>(options);
    monitor->configure();
    check_node_declared(monitor, parameters);
    polygon_read = nlohmann::json::array();
    for (const auto& point : monitor->polygon()) {
      polygon_read.push_back({point.x, point.y});
    }
    const auto now_ns = entry.at("now_ns").get<std::int64_t>();
    set_ros_time(monitor->get_clock(), now_ns);
    const auto& cloud_age = entry.at("cloud_age_ns");
    nlohmann::json record = {{"now_ns", now_ns}, {"cloud_age_ns", cloud_age}};
    if (!cloud_age.is_null()) {
      auto cloud = cloud_of(entry.at("points"), sensor_frame);
      cloud->header.frame_id = monitor->base_frame();
      cloud->header.stamp = rclcpp::Time(now_ns - cloud_age.get<std::int64_t>(), RCL_ROS_TIME);
      monitor->take(cloud);
    }
    const auto velocity = velocity_of(entry.at("velocity"));
    const auto action = monitor->decide(velocity);
    record.update({{"velocity", velocity_read(velocity)},
                   {"action", action_name(action.action_type)},
                   {"polygon_name", action.polygon_name},
                   {"allowed", velocity_read(action.req_vel)}});
    cases.push_back(record);
  }
  return {{"kind", "collision_monitor"}, {"polygon", polygon_read}, {"cases", cases}};
}

}  // namespace ros_reference
