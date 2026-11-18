// Diff drive odometry: the odometry of ros2_controllers 6.9.0's diff drive controller
// (diff_drive_controller::Odometry), with the scenario's wheel separation, wheel radii
// and rolling window, integrates each wheel position of the scenario by
// `update_from_pos` at the scenario's period, and gives the pose and the velocity after
// each update. The output holds the geometry, the period and the wheel positions as the
// reference read them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json diff_drive_odometry(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
