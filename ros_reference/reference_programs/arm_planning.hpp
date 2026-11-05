// Arm planning: MoveIt's planning pipeline, with the OMPL planner of the scenario's
// objective (RRTConnect for `any`, RRTstar for `shortest`) and its default adapters, plans
// each scene of the scenario among the scene's boxes of voxels as an octomap and its
// collision objects, holding the object it attaches to the arm's tip, the robot's links
// padded by the scenario's clearance against them, to the goal state itself, seeded from
// the scene and ended by the scenario's iterations or by its time limit alone, as its
// termination says, and MoveIt's collision check, padded, and unpadded distance probe each
// planned path: MoveIt's own, and the one motion_planning_core planned, its corners rounded
// as MoveIt's time-optimal trajectory generation rounds them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json arm_planning(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
