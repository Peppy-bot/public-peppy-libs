// Smac 2D: Nav2's Smac 2D planner (nav2_smac_planner 1.5.2) plans each start and goal of
// the scenario on its cost grid, by SmacPlanner2D::createPlan's steps on its own A*
// (AStarAlgorithm<Node2D>), collision checker and smoother: the path with the smoother
// off, and the path smoothed at the scenario's iteration bound. The output holds, for each
// start and goal, how each planner ended and its poses.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json smac_2d(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
