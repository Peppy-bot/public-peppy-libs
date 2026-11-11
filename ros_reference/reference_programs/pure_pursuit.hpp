// Pure pursuit: Nav2's regulated pure pursuit controller
// (nav2_regulated_pure_pursuit_controller 1.5.2), configured with the scenario's
// parameters on the scenario's cost grid and footprint, follows the scenario's plan as
// Nav2's controller server runs it (nav2_controller 1.5.2): at each step, at its own time
// on the node's clock with the base's pose in tf, the feasible path handler prunes the
// plan, the simple goal checker judges the goal, the simple progress checker the
// progress, and the controller computes the command. The output holds, for each step,
// the pruned plan, the goal checker's and the progress checker's verdicts, each null when
// the step ended before it, and the command or the exception that ended the step, with
// the grid, the footprint, the plan and each step's pose and speed as the reference read
// them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json pure_pursuit(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
