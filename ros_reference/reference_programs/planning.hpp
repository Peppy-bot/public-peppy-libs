// Planning: OMPL's geometric planners plan each plan of the scenario through a world of
// box obstacles in the unit cube, seeded and bounded by iterations, and simplify the
// path; a plan of PRM answers whether it found a path alone. The output holds the motion
// step, the worlds and the plans as the reference read them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json planning(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
