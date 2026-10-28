// Forward kinematics: MoveIt's robot state, posed at each configuration of each chain
// of the scenario, gives the global transform of each link that the chain moves.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json forward_kinematics(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
