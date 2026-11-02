// Occupancy: MoveIt's point cloud updater inserts each point cloud of the scenario into
// MoveIt's occupancy map, with a sphere cut out as the robot's own body, and the map
// gives the log-odds of every voxel it holds. The output holds the updater's parameters,
// the sphere and each cloud as the reference read them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json occupancy(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
