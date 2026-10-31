// The collision geometry of a chain: MoveIt's robot model names the links with collision
// geometry that each chain of the scenario carries, and the center it grows each of
// their bodies about; its robot state, posed at each configuration of the chain, gives
// the global transform of each carried link; and its robot filter, the bodies grown by
// each padding of the scenario, tells whether each probe point lies inside the chain.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json chain_collision(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
