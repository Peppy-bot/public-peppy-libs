// Planning scene: MoveIt's planning scene, its octomap the scenario's voxels from the
// start, takes the scenario's calls of motion_planning's scene members in order, as
// collision object and attached collision object messages and entries of its allowed
// collision matrix, each call of apply_attached_collision_object with the joints it names
// at their positions. The output gives the scene's objects, its attached objects and its
// allowed pairs that name an object as its planning scene message holds them, and names
// once the pairs FCL measures: each body of the robot, a link or an attached object, and
// each object or the octomap, and each attached object and each link it may not touch.
// For each configuration it holds one signed distance per pair in their order, and
// whether one body of the pair lies wholly inside the other's closed mesh where FCL finds
// them apart. It gives the waypoints of each path of the scenario that MoveIt's path check
// finds invalid, and, for each padding, whether each probe lies inside an object the calls
// leave standing, as the octomap updater's shape mask grows it.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

nlohmann::json planning_scene(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
