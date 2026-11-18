// Inflation: Nav2's inflation layer (nav2_costmap_2d 1.5.2) inflates the costs of each
// grid of the scenario for its footprint, padded as Costmap2DROS pads it, and the layer's
// parameters, over the whole grid, and gives the inflated costs. The output holds each
// grid, the padded footprint and the parameters as the reference read them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json inflation(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
