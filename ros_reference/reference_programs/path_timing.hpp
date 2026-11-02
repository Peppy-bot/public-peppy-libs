// Path timing: MoveIt's time-optimal trajectory generation (TOTG) blends the corners of
// each path of the scenario at its maximum deviation and times it under the scenario's
// limits, or refuses a path that turns back on itself.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json path_timing(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
