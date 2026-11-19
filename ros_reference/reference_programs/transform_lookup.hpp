// Transform lookup: tf2's buffer (tf2::BufferCore of geometry2 0.45.10) takes the
// scenario's transforms in order, a static transform as tf2_ros's static broadcaster sends it, with
// every static transform sent before it, and answers each lookup of the scenario.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json transform_lookup(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
