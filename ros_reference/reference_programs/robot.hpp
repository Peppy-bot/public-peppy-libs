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

// The robot model of a URDF, with an SRDF that names no group and no virtual joint, so
// the URDF root stands fixed at the origin of the model frame.
moveit::core::RobotModelPtr robot_model(const std::string& urdf_text);

// The robot model of the URDF at `urdf`, whose collision meshes MoveIt loads from the
// files of the same names in `mesh_directory`, each read through `inputs`; a URDF that
// names a collision mesh with no mesh directory is refused. Both paths are relative to
// the repository root.
moveit::core::RobotModelPtr robot_model_with_meshes(
    Inputs& inputs, const std::string& urdf,
    const std::optional<std::string>& mesh_directory);

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
