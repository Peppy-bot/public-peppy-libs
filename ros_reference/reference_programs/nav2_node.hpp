// The node a Nav2 plugin reads its parameters from, in the reference program's process:
// it holds the scenario's parameters of each plugin under the plugin's name, and the
// reference checks that the plugin declared exactly those, so no parameter of a plugin
// runs on a default the scenario does not name. The node declares the globals the
// scenario gives (the static layer's use_maximum and cost thresholds, the obstacle
// layer's track_unknown_space) as given; a layer reads a global with get_parameter and
// keeps its member's default for one the scenario leaves out. A Nav2 node that declares
// its own parameters takes the scenario's as its overrides, and the reference checks that
// it declared exactly those too. The clock of a node reads the time a reference sets on
// it.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nav2_ros_common/lifecycle_node.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros_reference {

// The parameter overrides of `parameters`: each member under its own name, an object's
// members nested under "<member>.<parameter>". A member is a bool, a whole number, a
// number, a string, or an array of numbers or of strings, an empty array being one of
// strings.
std::vector<rclcpp::Parameter> parameter_overrides(const nlohmann::json& parameters);

// The node `name` that holds, for each member of `plugins`, a plugin's name and an object
// of its parameters, each parameter under "<plugin>.<parameter>", an object's members
// nested under "<plugin>.<parameter>.<member>", for the plugin to declare; and each member
// of `globals` declared under its own name, as Costmap2DROS declares the parameters its
// layers read. The node is neither configured nor activated.
nav2::LifecycleNode::SharedPtr nav2_node(const std::string& name, const nlohmann::json& plugins,
                                         const nlohmann::json& globals);

// Throws unless the parameters `node` holds under "<plugin>." with a value are exactly
// those that `parameters`, the plugin's object of `plugins`, names.
void check_declared(const nav2::LifecycleNode::SharedPtr& node, const std::string& plugin,
                    const nlohmann::json& parameters);

// Throws unless the parameters `node` declared with a value, beyond those every Nav2 node
// declares and rclcpp's QoS overrides, are exactly those that `parameters`, the node's
// overrides, names.
void check_node_declared(const nav2::LifecycleNode::SharedPtr& node,
                         const nlohmann::json& parameters);

// Sets `clock`, the ROS clock of a node, to `stamp_ns`, so the node's now() reads it.
void set_ros_time(const rclcpp::Clock::SharedPtr& clock, std::int64_t stamp_ns);

}  // namespace ros_reference
