#include "velocity_smoother.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_ros_common/qos_profiles.hpp>
#include <nav2_velocity_smoother/velocity_smoother.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav2_node.hpp"
#include "scenario.hpp"

namespace ros_reference {

namespace {

// The topic the smoother publishes its velocity on.
constexpr const char* smoothed_topic = "/cmd_vel_smoothed";

// The smoother with its configuration, its command callback and its timer's cycle in
// reach, and its publisher active: the node is configured and left inactive, so it runs
// no timer and no bond, and the reference calls each callback on its own thread.
class Smoother : public nav2_velocity_smoother::VelocitySmoother {
 public:
  using VelocitySmoother::VelocitySmoother;

  void configure() {
    if (on_configure(get_current_state()) != nav2::CallbackReturn::SUCCESS) {
      throw std::runtime_error("the velocity smoother refuses the scenario's parameters");
    }
    smoothed_cmd_pub_->on_activate();
  }

  void command(const geometry_msgs::msg::Twist& setpoint) {
    inputCommandCallback(std::make_shared<geometry_msgs::msg::Twist>(setpoint));
  }

  void cycle() { smootherTimer(); }
};

// The velocities the smoother publishes, read in the reference program's process: the
// subscription takes each from the publisher within the process, and the reference runs
// its callback on its own thread after each cycle.
class Published {
 public:
  Published() : node_(std::make_shared<rclcpp::Node>("velocity_smoother_reference_reader",
                                                     rclcpp::NodeOptions().use_intra_process_comms(true))) {
    subscription_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        smoothed_topic, nav2::qos::StandardTopicQoS(),
        [this](const geometry_msgs::msg::TwistStamped& message) { taken_.push_back(message); });
    executor_.add_node(node_);
  }

  // The velocity the last cycle published, if it published one.
  std::optional<geometry_msgs::msg::Twist> take() {
    executor_.spin_some();
    if (taken_.size() > 1) {
      throw std::runtime_error("a cycle of the smoother published more than one velocity");
    }
    std::optional<geometry_msgs::msg::Twist> velocity;
    if (!taken_.empty()) {
      velocity = taken_.front().twist;
    }
    taken_.clear();
    return velocity;
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<geometry_msgs::msg::TwistStamped> taken_;
};

geometry_msgs::msg::Twist twist_of(const nlohmann::json& axes) {
  const auto values = axes.get<std::vector<double>>();
  if (values.size() != 3) {
    throw std::runtime_error("a setpoint is not linear x, linear y and angular z");
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

nlohmann::json doubles_read(const std::vector<double>& values) {
  auto read = nlohmann::json::array();
  for (const auto value : values) {
    read.push_back(value);
  }
  return read;
}

}  // namespace

nlohmann::json velocity_smoother(const nlohmann::json& scenario, Inputs&) {
  start_ros();
  const auto& parameters = scenario.at("smoother");
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameter_overrides(parameters));
  options.use_intra_process_comms(true);
  auto smoother = std::make_shared<Smoother>(options);
  smoother->configure();
  check_node_declared(smoother, parameters);
  Published published;

  auto events = nlohmann::json::array();
  for (const auto& event : scenario.at("events")) {
    const auto at_ns = event.at("at_ns").get<std::int64_t>();
    set_ros_time(smoother->get_clock(), at_ns);
    const auto kind = event.at("event").get<std::string>();
    if (kind == "setpoint") {
      const auto setpoint = twist_of(event.at("twist"));
      smoother->command(setpoint);
      events.push_back({{"event", "setpoint"}, {"at_ns", at_ns}, {"twist", twist_read(setpoint)}});
      continue;
    }
    if (kind != "tick") {
      throw std::runtime_error("an event is neither a setpoint nor a tick: " + kind);
    }
    smoother->cycle();
    const auto velocity = published.take();
    events.push_back({{"event", "tick"},
                      {"at_ns", at_ns},
                      {"published", velocity.has_value()},
                      {"twist", velocity ? twist_read(*velocity) : nlohmann::json::array()}});
  }

  const auto parameter = [&](const char* name) { return smoother->get_parameter(name); };
  return {
      {"kind", "velocity_smoother"},
      {"smoother",
       {
           {"smoothing_frequency", parameter("smoothing_frequency").as_double()},
           {"feedback", parameter("feedback").as_string()},
           {"scale_velocities", parameter("scale_velocities").as_bool()},
           {"max_velocity", doubles_read(parameter("max_velocity").as_double_array())},
           {"min_velocity", doubles_read(parameter("min_velocity").as_double_array())},
           {"max_accel", doubles_read(parameter("max_accel").as_double_array())},
           {"max_decel", doubles_read(parameter("max_decel").as_double_array())},
           {"deadband_velocity", doubles_read(parameter("deadband_velocity").as_double_array())},
           {"velocity_timeout", parameter("velocity_timeout").as_double()},
       }},
      {"events", events},
  };
}

}  // namespace ros_reference
