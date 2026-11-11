#include "nav2_node.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "scenario.hpp"

namespace ros_reference {

namespace {

rclcpp::ParameterValue value_of(const std::string& name, const nlohmann::json& value) {
  if (value.is_boolean()) {
    return rclcpp::ParameterValue(value.get<bool>());
  }
  if (value.is_number_integer()) {
    return rclcpp::ParameterValue(value.get<std::int64_t>());
  }
  if (value.is_number_float()) {
    return rclcpp::ParameterValue(value.get<double>());
  }
  if (value.is_string()) {
    return rclcpp::ParameterValue(value.get<std::string>());
  }
  throw std::runtime_error("the parameter " + name + " is not a bool, a number or a string");
}

// Appends each parameter of `parameters` under `prefix`, an object's members one level
// deeper, to `names` and `overrides`.
void flatten(const std::string& prefix, const nlohmann::json& parameters,
             std::set<std::string>& names, std::vector<rclcpp::Parameter>& overrides) {
  for (const auto& [key, value] : parameters.items()) {
    const auto name = prefix + "." + key;
    if (value.is_object()) {
      flatten(name, value, names, overrides);
      continue;
    }
    names.insert(name);
    overrides.emplace_back(name, value_of(name, value));
  }
}

}  // namespace

nav2::LifecycleNode::SharedPtr nav2_node(const std::string& name, const nlohmann::json& plugins,
                                         const nlohmann::json& globals) {
  start_ros();
  std::set<std::string> names;
  std::vector<rclcpp::Parameter> overrides;
  for (const auto& [plugin, parameters] : plugins.items()) {
    flatten(plugin, parameters, names, overrides);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  auto node = std::make_shared<nav2::LifecycleNode>(name, "", options);
  for (const auto& [key, value] : globals.items()) {
    node->declare_parameter(key, value_of(key, value));
  }
  return node;
}

void check_declared(const nav2::LifecycleNode::SharedPtr& node, const std::string& plugin,
                    const nlohmann::json& parameters) {
  std::set<std::string> given;
  std::vector<rclcpp::Parameter> unused;
  flatten(plugin, parameters, given, unused);
  const auto listed = node->list_parameters({plugin}, 0).names;
  const std::set<std::string> declared(listed.begin(), listed.end());
  for (const auto& name : declared) {
    if (!given.count(name)) {
      throw std::runtime_error("the plugin declares " + name +
                               ", which the scenario does not set: give it in the scenario");
    }
  }
  for (const auto& name : given) {
    if (!declared.count(name)) {
      throw std::runtime_error("the scenario sets " + name +
                               ", which the plugin does not declare: remove it");
    }
  }
}

}  // namespace ros_reference
