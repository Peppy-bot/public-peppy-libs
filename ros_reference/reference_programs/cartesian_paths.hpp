// Cartesian paths: MoveIt's Cartesian interpolator, as move_group's compute_cartesian_path
// service runs it, takes a link of the arm of the scenario's one chain along straight lines
// through each case's poses, each the link's pose at the case's start moved by an offset, in
// steps of the scenario's step, with KDL as the inverse kinematics, loaded for each case and
// seeded from it. Where a case avoids collisions, the service's state validity check holds
// each state clear of the case's boxes of voxels as an octomap and its collision objects, the
// robot's links padded by the scenario's clearance against them. The output gives, for each
// case, the fraction of its lines covered, and the link's pose and the joints at each state.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json cartesian_paths(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
