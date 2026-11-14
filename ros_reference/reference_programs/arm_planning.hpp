// Arm planning: MoveIt's planning pipeline, with the OMPL planner the scenario names
// (RRTConnect for `rrt_connect`, RRTstar for `rrt_star`) and its default adapters, plans each
// scene of the scenario among the scene's boxes of voxels as an octomap and its collision
// objects, holding the object it attaches to the arm's tip, the robot's links padded by the
// scenario's clearance against them, to the goal state itself or to a pose of each chain's
// tip link, whose goal sampler ends before the planner starts, seeded from the scene and
// ended by the scenario's iterations or by its time limit alone, as its termination says,
// MoveIt's CHOMP refining the planned path where the scenario's refiner is `chomp`, and
// MoveIt's collision check, padded, and unpadded distance probe each planned path: MoveIt's
// own, and the one motion_planning_core planned, its corners rounded as MoveIt's
// time-optimal trajectory generation rounds them.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json arm_planning(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
