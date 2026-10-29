// Arm planning: MoveIt's planning pipeline, with OMPL's RRTConnect and its default
// adapters, plans each scene of the scenario among the scene's boxes of voxels as an
// octomap, to the goal state itself, seeded from the scene and bounded by the scenario's
// iterations, and MoveIt's collision check and distance probe each planned path:
// MoveIt's own, and the one motion_planning_core planned, its corners rounded as
// MoveIt's time-optimal trajectory generation rounds them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json arm_planning(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
