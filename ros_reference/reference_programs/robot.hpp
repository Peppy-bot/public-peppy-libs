// The MoveIt robot model of a scenario's URDF, and what the references read from it:
// what every reference that poses a robot shares.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

// The robot model of the description at `description`, a directory of the repository that
// holds `<name>.urdf`, `<name>.srdf` and the collision meshes under `meshes/`, `<name>` the
// directory's last part: MoveIt's model of the URDF and the SRDF as they are, each read
// through `inputs`, its collision meshes loaded from the files of the same names under
// `meshes/`.
moveit::core::RobotModelPtr description_model(Inputs& inputs, const std::string& description);

// An arm of a description: an SRDF group of one chain whose tip an end effector stands on,
// its active joints in the chain's order, and the chain's tip.
struct Arm {
  std::string name;
  std::vector<std::string> joints;
  std::string tip_link;
};

// A planning group of a description and its arms: the group itself when it is an arm, else
// its subgroups, each an arm; and whether it holds every arm of the description.
struct ArmGroup {
  std::string name;
  std::vector<Arm> arms;
  bool holds_every_arm;

  // The groups that carry an inverse kinematics solver: the arms, whose union MoveIt samples
  // a goal of the group from (moveit2 2.15.2 constraint_sampler_manager.cpp,
  // UnionConstraintSampler, for a group of several).
  std::vector<std::string> solver_groups() const;
};

// The planning group `name` of `model`'s SRDF and its arms; a group that is neither an arm
// nor a group of arms is refused.
ArmGroup arm_group(const moveit::core::RobotModel& model, const std::string& name);

// The arm `name` of `model`'s SRDF; a group that is no arm is refused.
Arm the_arm(const moveit::core::RobotModel& model, const std::string& name);

// Whether the SRDF of `model` disables the collisions of `a` and `b`.
bool disabled_pair(const moveit::core::RobotModel& model, const std::string& a,
                   const std::string& b);

// The child link of `joint`.
std::string child_link(const moveit::core::RobotModel& model, const std::string& joint);

// Refuses a chain whose path from the URDF root to `tip_link` holds a movable joint that
// `joints` does not name: MoveIt stands such a joint at its default position, which a
// parity test does not know.
void require_named_path(const moveit::core::RobotModel& model, const std::string& tip_link,
                        const std::vector<std::string>& joints);

// The global transform of `link` at `state`: its translation, and its rotation matrix row
// by row.
nlohmann::json link_pose(const moveit::core::RobotState& state, const std::string& link);

}  // namespace ros_reference
