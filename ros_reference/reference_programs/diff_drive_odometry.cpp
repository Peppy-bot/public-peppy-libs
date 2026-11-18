#include "diff_drive_odometry.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

// The odometry's header picks rcpputils' rolling mean by the macros of rcpputils/version.h,
// which it reads without including.
#include <rcpputils/version.h>
#include <diff_drive_controller/odometry.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json diff_drive_odometry(const nlohmann::json& scenario, Inputs&) {
  const auto& wheels = scenario.at("wheels");
  const auto wheel_separation = wheels.at("wheel_separation").get<double>();
  const auto left_wheel_radius = wheels.at("left_wheel_radius").get<double>();
  const auto right_wheel_radius = wheels.at("right_wheel_radius").get<double>();
  const auto window = wheels.at("velocity_rolling_window_size").get<std::size_t>();
  const auto period = scenario.at("period").get<double>();

  diff_drive_controller::Odometry odometry(window);
  odometry.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);

  auto updates = nlohmann::json::array();
  for (const auto& position : scenario.at("positions")) {
    const auto wheel = position.get<std::vector<double>>();
    if (wheel.size() != 2) {
      throw std::runtime_error("a wheel position is not the left and the right wheel's");
    }
    const bool integrated = odometry.update_from_pos(wheel[0], wheel[1], period);
    updates.push_back({
        {"positions", {wheel[0], wheel[1]}},
        {"integrated", integrated},
        {"pose", {odometry.getX(), odometry.getY(), odometry.getHeading()}},
        {"velocity", {odometry.getLinear(), odometry.getAngular()}},
    });
  }
  return {
      {"kind", "diff_drive_odometry"},
      {"wheels",
       {
           {"wheel_separation", wheel_separation},
           {"left_wheel_radius", left_wheel_radius},
           {"right_wheel_radius", right_wheel_radius},
           {"velocity_rolling_window_size", window},
       }},
      {"period", period},
      {"updates", updates},
  };
}

}  // namespace ros_reference
