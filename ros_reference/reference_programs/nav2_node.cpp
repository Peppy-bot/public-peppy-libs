#include "nav2_node.hpp"

#include <algorithm>
#include <mutex>
#include <set>
#include <stdexcept>

#include "scenario.hpp"

namespace ros_reference {

namespace {

// The prefix of the parameters rclcpp declares for a publisher's or subscription's QoS.
constexpr const char* qos_overrides_prefix = "qos_overrides.";

rclcpp::ParameterValue array_value_of(const std::string& name, const nlohmann::json& value) {
  if (value.empty()) {
    return rclcpp::ParameterValue(std::vector<std::string>());
  }
  if (std::all_of(value.begin(), value.end(), [](const auto& v) { return v.is_number(); })) {
    return rclcpp::ParameterValue(value.get<std::vector<double>>());
  }
  if (std::all_of(value.begin(), value.end(), [](const auto& v) { return v.is_string(); })) {
    return rclcpp::ParameterValue(value.get<std::vector<std::string>>());
  }
  throw std::runtime_error("the parameter " + name + " is not an array of numbers or of strings");
}

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
  if (value.is_array()) {
    return array_value_of(name, value);
  }
  throw std::runtime_error("the parameter " + name +
                           " is not a bool, a number, a string or an array");
}

// Appends each parameter of `parameters` under `prefix`, or under its own name when the
// prefix is empty, an object's members one level deeper, to `names` and `overrides`.
void flatten(const std::string& prefix, const nlohmann::json& parameters,
             std::set<std::string>& names, std::vector<rclcpp::Parameter>& overrides) {
  for (const auto& [key, value] : parameters.items()) {
    const auto name = prefix.empty() ? key : prefix + "." + key;
    if (value.is_object()) {
      flatten(name, value, names, overrides);
      continue;
    }
    names.insert(name);
    overrides.emplace_back(name, value_of(name, value));
  }
}

std::set<std::string> names_of(const nlohmann::json& parameters) {
  std::set<std::string> names;
  std::vector<rclcpp::Parameter> unused;
  flatten("", parameters, names, unused);
  return names;
}

// Throws unless `declared` and `given` name the same parameters.
void check_same(const std::set<std::string>& declared, const std::set<std::string>& given,
                const std::string& who) {
  for (const auto& name : declared) {
    if (!given.count(name)) {
      throw std::runtime_error(who + " declares " + name +
                               ", which the scenario does not set: give it in the scenario");
    }
  }
  for (const auto& name : given) {
    if (!declared.count(name)) {
      throw std::runtime_error("the scenario sets " + name + ", which " + who +
                               " does not declare: remove it");
    }
  }
}

// Whether the parameter `name` of `node` holds a value. A plugin that tries a deprecated
// name before its own declares it with none, and reads no default from it.
bool holds_value(const nav2::LifecycleNode::SharedPtr& node, const std::string& name) {
  try {
    node->get_parameter(name);
    return true;
  } catch (const rclcpp::exceptions::ParameterUninitializedException&) {
    return false;
  }
}

// The parameters `node` holds under `prefixes`, every one for none, that hold a value,
// rclcpp's QoS overrides left out.
std::set<std::string> declared_names(const nav2::LifecycleNode::SharedPtr& node,
                                     const std::vector<std::string>& prefixes) {
  const auto listed = node->list_parameters(prefixes, 0).names;
  std::set<std::string> names;
  for (const auto& name : listed) {
    if (name.rfind(qos_overrides_prefix, 0) != 0 && holds_value(node, name)) {
      names.insert(name);
    }
  }
  return names;
}

}  // namespace

std::vector<rclcpp::Parameter> parameter_overrides(const nlohmann::json& parameters) {
  std::set<std::string> names;
  std::vector<rclcpp::Parameter> overrides;
  flatten("", parameters, names, overrides);
  return overrides;
}

nav2::LifecycleNode::SharedPtr nav2_node(const std::string& name, const nlohmann::json& plugins,
                                         const nlohmann::json& globals) {
  start_ros();
  rclcpp::NodeOptions options;
  options.parameter_overrides(parameter_overrides(plugins));
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
  check_same(declared_names(node, {plugin}), given, "the plugin");
}

void check_node_declared(const nav2::LifecycleNode::SharedPtr& node,
                         const nlohmann::json& parameters) {
  // Every Nav2 node declares its lifecycle and bond parameters; a node of no plugin
  // declares those alone.
  const auto common = declared_names(
      std::make_shared<nav2::LifecycleNode>(node->get_name() + std::string("_without_plugins"),
                                            "", rclcpp::NodeOptions()),
      {});
  std::set<std::string> own;
  for (const auto& name : declared_names(node, {})) {
    if (!common.count(name)) {
      own.insert(name);
    }
  }
  check_same(own, names_of(parameters), std::string("the node ") + node->get_name());
}

void set_ros_time(const rclcpp::Clock::SharedPtr& clock, std::int64_t stamp_ns) {
  std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
  rcl_clock_t* handle = clock->get_clock_handle();
  if (rcl_enable_ros_time_override(handle) != RCL_RET_OK ||
      rcl_set_ros_time_override(handle, stamp_ns) != RCL_RET_OK) {
    throw std::runtime_error("a node's clock does not take the time " +
                             std::to_string(stamp_ns) + " ns");
  }
}

}  // namespace ros_reference
