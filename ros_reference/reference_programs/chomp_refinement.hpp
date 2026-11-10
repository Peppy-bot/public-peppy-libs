// CHOMP refinement: MoveIt's CHOMP optimizer, at the parameters that MoveIt's CHOMP plugin
// loads by default, refines the input path of each scene of the scenario, its initial
// trajectory the input path (`fillTrajectory`), among the scene's boxes of voxels as an
// octomap and its collision objects, in the scene's hybrid collision environment, as the
// plugin builds it, its random generator seeded from the scene. MoveIt's collision check,
// the robot's links padded by the scenario's clearance, and its unpadded distances probe the
// input path and CHOMP's, each straight between its waypoints, and MoveIt's time-optimal
// path, rounding each at the scenario's corner tolerance, measures its turning, its bending
// and its duration. The output gives, for each scene, CHOMP's error code, its waypoints
// where its optimizer ran, and both probes.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json chomp_refinement(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
