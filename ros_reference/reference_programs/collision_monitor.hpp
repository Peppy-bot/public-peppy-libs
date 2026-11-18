// Collision monitor: Nav2's collision monitor (nav2_collision_monitor 1.5.2), configured
// with the scenario's parameters, one footprint polygon in approach mode and one point
// cloud source, decides each case of the scenario in a monitor of its own, at the case's
// time on the node's clock: it takes the case's cloud, if the case has one, through the
// source's data callback, stamped the case's age before that time, and processes the
// case's velocity. The output holds the polygon as the reference read it and, for each
// case, its time, its cloud's age and its velocity as the reference read them, the action
// the monitor takes, the polygon or the reason that gave it, and the velocity it allows.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json collision_monitor(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
