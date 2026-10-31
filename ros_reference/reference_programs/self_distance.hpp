// Self distance: MoveIt's FCL collision environment, posed at each configuration of the
// scenario, gives the distance between the collision meshes of each pair of links. The
// output names the pairs once, and holds for each configuration one distance per pair in
// their order.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json self_distance(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
