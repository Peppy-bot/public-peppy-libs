// Map export: Nav2's map saver writes each occupancy grid of the scenario as a trinary map,
// with its free and occupied thresholds. The output holds, for each grid, the bytes of the
// image and the text of the description that `nav2_map_server::saveMapToFile` writes.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json map_export(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
