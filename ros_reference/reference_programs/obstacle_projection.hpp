// Obstacle projection: Nav2's obstacle layer (nav2_costmap_2d 1.5.2) marks, on the
// scenario's grid, the points of each cloud of the scenario, seen from its sensor's pose,
// through one observation source of the layer's height band with clearing off, then clears
// the cells under the scenario's footprint, padded as Costmap2DROS pads it, at the cloud's
// base pose, and gives the lethal cells and each point as the observation buffer
// transforms it into the grid's frame, in single precision. The output holds the grid, the
// padded footprint and the parameters as the reference read them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json obstacle_projection(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
