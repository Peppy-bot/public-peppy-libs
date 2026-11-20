// The inverse kinematics plugin that the references load for each arm: MoveIt's KDL plugin
// (moveit2 2.15.2 kdl_kinematics_plugin.cpp), its search ended by the passes of KDL's descent
// loop. KDL's search descends from the seed, then from configurations it draws, until a
// descent converges to a configuration its caller accepts or its timeout, a clock, runs out
// (:369-409). This plugin runs each descent as KDL's search of one attempt, the search of a
// timeout of 0 (:265-266), draws each later start as KDL draws it (:70-74), from a robot
// state of its own, and starts no descent once the search's descents have run the passes of
// its group's kinematics parameter ik_search_passes for each default timeout its timeout
// holds, so a search gives the same answer on every machine: a search of the default timeout
// runs those passes, one of a timeout of 0 runs one descent, as KDL's search of one attempt
// does, and the search of twice the default timeout that MoveIt's PoseModelStateSpace runs
// where one attempt finds no configuration (moveit2 2.15.2 pose_model_state_space.cpp:241-245)
// runs twice those passes. KDL reports the passes of a descent only in its debug line "Result
// <code> after <i> iterations" (:493-494), so the plugin turns the KDL logger's debug lines
// on, reads that line, and drops every line of that logger below a warning: KDL logs one at
// the info level on each search of a position alone (:346), which the plugin runs once per
// descent.

#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/utils/logger.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcutils/logging.h>

#include "kinematics_parameters.hpp"

namespace ros_reference {
namespace {

// The timeout, in seconds, that a search's passes stand for: moveit_ros_planning's
// kinematics_solver_timeout default (kinematics_parameters.yaml), which load_kdl sets on each
// arm and the searches of the constraint samplers carry.
constexpr double kdl_timeout = 0.05;
// KDL's result of a descent that ran all of its max_solver_iterations passes (:493).
constexpr int ran_every_pass = -3;

// The passes of each descent that KDL reported on this thread since the plugin last took
// them.
thread_local std::vector<std::size_t> reported_descents;

const std::string& kdl_logger_name() {
  static const std::string name =
      moveit::getLogger("moveit.kinematics.kdl_kinematics_plugin").get_name();
  return name;
}

rcutils_logging_output_handler_t& forwarded_handler() {
  static rcutils_logging_output_handler_t handler = nullptr;
  return handler;
}

// The program's log output handler: it reads KDL's report of each descent out of the KDL
// logger's lines below a warning, drops those lines, and forwards every other line.
void read_kdl_lines(const rcutils_log_location_t* location, int severity, const char* name,
                    rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
  if (severity >= RCUTILS_LOG_SEVERITY_WARN || name == nullptr || kdl_logger_name() != name) {
    forwarded_handler()(location, severity, name, timestamp, format, args);
    return;
  }
  // KDL streams the report, which rclcpp logs as "%s" (rclcpp/logging.hpp:176).
  if (std::strcmp(format, "%s") != 0) {
    return;
  }
  va_list message_args;
  va_copy(message_args, *args);
  const char* message = va_arg(message_args, const char*);
  va_end(message_args);
  int result = 0;
  unsigned int iterations = 0;
  if (std::sscanf(message, "Result %d after %u iterations", &result, &iterations) != 2) {
    return;
  }
  // A descent that converges or stalls at the loop's index i ran i + 1 passes.
  reported_descents.push_back(result == ran_every_pass ? iterations : iterations + 1);
}

// The passes of a search of `group_name` that the kinematics parameter ik_search_passes of
// `node` gives the group: a whole number above 0.
std::size_t search_passes_of(const rclcpp::Node& node, const std::string& group_name) {
  const std::string name = kinematics_parameter(group_name, ik_search_passes_parameter);
  rclcpp::Parameter parameter;
  if (!node.get_parameter(name, parameter) ||
      parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
      parameter.as_int() < 1) {
    throw std::runtime_error("the kinematics parameter " + name +
                             " gives no whole number of passes above 0");
  }
  return static_cast<std::size_t>(parameter.as_int());
}

void read_kdl_reports() {
  static std::once_flag installed;
  std::call_once(installed, [] {
    forwarded_handler() = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(read_kdl_lines);
    if (rcutils_logging_set_logger_level(kdl_logger_name().c_str(),
                                         RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK) {
      throw std::runtime_error("rcutils sets no debug level on the logger " +
                               kdl_logger_name());
    }
  });
}

}  // namespace

class PassBoundedKdl : public kdl_kinematics_plugin::KDLKinematicsPlugin {
 public:
  using KDLKinematicsPlugin::searchPositionIK;

  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames,
                  double search_discretization) override {
    read_kdl_reports();
    if (!KDLKinematicsPlugin::initialize(node, robot_model, group_name, base_frame, tip_frames,
                                         search_discretization)) {
      return false;
    }
    search_passes_ = search_passes_of(*node, group_name);
    starts_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    return true;
  }

  // Every other search of KDL's plugin calls this one (:266, :278, :288, :299).
  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state, double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override {
    const double default_timeouts = timeout / kdl_timeout;
    if (!(default_timeouts >= 0.0) || default_timeouts != std::floor(default_timeouts)) {
      throw std::runtime_error("a search of " + getGroupName() + " carries the timeout " +
                               std::to_string(timeout) + " s, and the plugin bounds a search of " +
                               "a whole number of times " + std::to_string(kdl_timeout) +
                               " s alone");
    }
    const std::size_t search_passes = static_cast<std::size_t>(default_timeouts) * search_passes_;
    if (!consistency_limits.empty()) {
      throw std::runtime_error("a search of " + getGroupName() +
                               " carries consistency limits, which the plugin does not bound");
    }
    std::vector<double> start = ik_seed_state;
    std::size_t passes = 0;
    while (true) {
      reported_descents.clear();
      const bool found =
          KDLKinematicsPlugin::searchPositionIK(ik_pose, start, 0.0, consistency_limits, solution,
                                                solution_callback, error_code, options);
      // KDL refuses an uninitialized plugin or a seed of another size before it descends
      // (:311-323).
      if (reported_descents.empty() && error_code.val == error_code.NO_IK_SOLUTION) {
        return false;
      }
      if (reported_descents.size() != 1) {
        throw std::runtime_error("KDL's search of one attempt reported " +
                                 std::to_string(reported_descents.size()) + " descents");
      }
      passes += reported_descents.front();
      if (found || passes >= search_passes) {
        return found;
      }
      start = drawn_start();
    }
  }

 private:
  // A start drawn inside the group's bounds, as KDL's getRandomConfiguration draws one.
  std::vector<double> drawn_start() const {
    const auto* group = robot_model_->getJointModelGroup(getGroupName());
    starts_->setToRandomPositions(group);
    std::vector<double> start;
    starts_->copyJointGroupPositions(group, start);
    return start;
  }

  // The passes of KDL's descent loop (CartToJnt, :435-491), for each default timeout a
  // search's timeout holds, after which the search starts no descent.
  std::size_t search_passes_ = 0;
  moveit::core::RobotStatePtr starts_;
};

}  // namespace ros_reference

PLUGINLIB_EXPORT_CLASS(ros_reference::PassBoundedKdl, kinematics::KinematicsBase)
