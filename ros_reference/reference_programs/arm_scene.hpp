// An arm of a scenario's chains in a scene, as MoveIt holds it: what the references of arm
// planning and of Cartesian paths share. The robot model whose group holds the chains, its
// KDL inverse kinematics, the seeds of MoveIt's own random generators, and the planning
// scene of a scene of the scenario.
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <nlohmann/json.hpp>

#include "robot.hpp"
#include "scenario.hpp"

namespace ros_reference {

// The planning group of a scenario's chains.
inline constexpr const char* group = "arm";

// How many of MoveIt's random generators the reference program's constructor has built.
std::atomic<std::uint64_t>& moveit_generators();

// Seeds the generator of the seeds of MoveIt's own random generators, each built after this
// call drawing from it, and std::rand, from `seed`.
void seed_moveit(std::uint32_t seed);

// The group "arm" of the scenario's `chains`, each by its `name` from its `base_link` to its
// `tip_link`.
ChainGroup chains_of(const nlohmann::json& scenario);

// The robot model of the scenario, whose group "arm" holds its `chains`, its joints the
// chains' `joints` in their order, each bounded in acceleration.
moveit::core::RobotModelPtr arm_model(const nlohmann::json& scenario, const ChainGroup& chains,
                                      Inputs& inputs);

// MoveIt's KDL inverse kinematics on `solver_groups`, a ChainGroup's, which a pose goal's
// constraint sampler reads. It loads as move_group loads a moveit_config's kinematics.yaml
// that names only the solver: at the search resolution and the timeout that
// moveit_ros_planning's kinematics_parameters.yaml gives by default (0.1 and 0.05 s), the
// timeout set on each group as RobotModelLoader::loadKinematicsSolvers sets it (moveit2
// 2.15.2 robot_model_loader.cpp:310-318), which the constraint sampler gives each IK call
// (default_constraint_samplers.cpp:268).
void load_kdl(const moveit::core::RobotModelPtr& model,
              const std::vector<std::string>& solver_groups);

std::vector<double> numbers(const nlohmann::json& values);

// The scene `s`: its boxes, each the voxels from its first key to its last on every axis,
// in an octomap of `voxel_size`; its objects; each object it attaches to a grasp link with
// its touch links, at its pose in the link's frame, the arms at the scene's start, which
// stays the scene's state, an arm outside the group at its zero configuration; and its
// allowed pairs. Every link with collision geometry is padded by `padding` against the
// octomap and the objects, and the robot meets itself unpadded, as a collision request's
// defaults pad_environment_collisions and pad_self_collisions ask. An attached object
// takes the padding of its link (moveit2 2.15.2 collision_env_fcl.cpp:195-196), and a
// grasp link has no collision geometry, which setPadding pads alone
// (collision_env.cpp:127-133), so its padding is 0 (collision_env.cpp:166-176): MoveIt
// keeps a held object unpadded from the objects. Collisions between the robot's own links
// are allowed but those of `checked_pairs`, the pairs motion_planning_core's self model
// checks, and so are those of every link that does not move with the group's joints, the
// body, the arms' bases, the head camera, and an arm outside the group and its gripper,
// but against an object an arm of the group holds.
planning_scene::PlanningScenePtr scene_of(const moveit::core::RobotModelPtr& model,
                                         const nlohmann::json& s, double voxel_size,
                                         const nlohmann::json& checked_pairs, double padding);

}  // namespace ros_reference
