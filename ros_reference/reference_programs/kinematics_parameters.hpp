// The parameters of a group's inverse kinematics on a node, which MoveIt's kinematics plugin
// loader reads under robot_description_kinematics.<group> (moveit2 2.15.2
// kinematics_plugin_loader.cpp:260), and the one the reference's KDL plugin reads beside
// them (pass_bounded_kdl.cpp).
#pragma once

#include <string>

namespace ros_reference {

// The passes of KDL's descent loop after which a search of the group starts no descent.
inline constexpr const char* ik_search_passes_parameter = "ik_search_passes";

// The name of the parameter `key` of `group`'s inverse kinematics.
inline std::string kinematics_parameter(const std::string& group, const std::string& key) {
  return "robot_description_kinematics." + group + "." + key;
}

}  // namespace ros_reference
