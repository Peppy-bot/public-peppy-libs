// Orientation constraints: MoveIt's kinematic constraint decides, under rotation-vector
// tolerances about the axes of the target, whether a link at each configuration of the
// scenario keeps each target.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json orientation_constraints(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
