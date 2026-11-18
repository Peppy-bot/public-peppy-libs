// Global costmap: a Nav2 global cost map (nav2_costmap_2d 1.5.2) of a static layer, an
// obstacle layer and an inflation layer, updated once by LayeredCostmap::updateMap: the
// static layer takes the scenario's saved map as nav2_map_server's loadMapFromFile reads
// its image as a trinary map, the obstacle layer marks the scenario's cloud, seen from its
// sensor's pose, through one observation source with clearing off, and clears the cells
// under the scenario's footprint, padded as Costmap2DROS pads it, at the scenario's base
// pose, and the inflation layer inflates the result for the padded footprint. The output
// holds the map as map_io read it, the padded footprint, the cloud's points as the
// observation buffer transformed them into the map's frame in single precision, and the
// cost map's costs.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json global_costmap(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
