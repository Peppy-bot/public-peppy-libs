// AMCL's global localization: Nav2's AMCL searches the map of each room of the scenario for
// the base over the room's drive, from no known pose, once a search, every random draw on
// drand48 seeded by the search. The map's grid is karto's grid of the room's map scans at
// their poses, as slam_toolbox publishes the grid of its map. The output holds each room's
// grid, and each search's amcl_pose after each scan that AMCL publishes one after: the mean
// of the filter's heaviest cluster (getMaxWeightHyp) and the covariance of the whole filter
// (`set->cov`).
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json amcl_global_localization(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
