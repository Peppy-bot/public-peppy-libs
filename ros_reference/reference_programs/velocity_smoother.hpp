// Velocity smoother: Nav2's velocity smoother (nav2_velocity_smoother 1.5.2), configured
// with the scenario's parameters, takes each setpoint of the scenario through its command
// callback and runs its timer's cycle at each tick, each at its own time on the node's
// clock, and gives the velocity each cycle publishes, or that it publishes none. The
// output holds the parameters and the events as the reference read them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json velocity_smoother(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
