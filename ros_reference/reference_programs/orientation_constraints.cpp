#include "orientation_constraints.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/transforms/transforms.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

#include "robot.hpp"
#include "scenario.hpp"

namespace ros_reference {

namespace {

// The fixed frame that stands at a case's target. MoveIt's `decide` turns the rotation
// vector from the target into the axes of the constraint's frame
// (`OrientationConstraint::decide`, moveit2 2.15.2 kinematic_constraint.cpp:765-769), so
// with this frame and the identity as the message's orientation, each tolerance bounds the
// rotation about an axis of the target, the link's own at the target.
const std::string target_frame = "target";

// Whether MoveIt's constraint keeps `link` at `state` within `tolerances` about the axes of
// the target `orientation`, a quaternion [x, y, z, w] in the model frame.
bool accepted(const moveit::core::RobotModelPtr& model, const moveit::core::RobotState& state,
              const std::string& link, const std::vector<double>& orientation,
              const std::array<double, 3>& tolerances) {
  moveit::core::Transforms transforms(model->getModelFrame());
  Eigen::Isometry3d at_target = Eigen::Isometry3d::Identity();
  at_target.linear() =
      Eigen::Quaterniond(orientation.at(3), orientation.at(0), orientation.at(1), orientation.at(2))
          .toRotationMatrix();
  transforms.setTransform(at_target, target_frame);
  moveit_msgs::msg::OrientationConstraint message;
  message.header.frame_id = target_frame;
  message.link_name = link;
  message.orientation.w = 1.0;
  message.absolute_x_axis_tolerance = tolerances[0];
  message.absolute_y_axis_tolerance = tolerances[1];
  message.absolute_z_axis_tolerance = tolerances[2];
  message.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
  message.weight = 1.0;
  kinematic_constraints::OrientationConstraint constraint(model);
  if (!constraint.configure(message, transforms)) {
    throw std::runtime_error("MoveIt does not configure the orientation constraint of " + link);
  }
  return constraint.decide(state).satisfied;
}

// The place of the axis `free_axis` names among x, y and z, none for "none".
std::optional<std::size_t> free_place(const std::string& free_axis) {
  if (free_axis == "none") {
    return std::nullopt;
  }
  if (free_axis == "x") {
    return 0;
  }
  if (free_axis == "y") {
    return 1;
  }
  if (free_axis == "z") {
    return 2;
  }
  throw std::runtime_error("free_axis is none, x, y or z, and it is " + free_axis);
}

// `bound` about each axis, and pi about the free axis, which a rotation vector never passes.
std::array<double, 3> tolerances(double bound, std::optional<std::size_t> free) {
  std::array<double, 3> each{bound, bound, bound};
  if (free) {
    each[*free] = std::numbers::pi;
  }
  return each;
}

}  // namespace

nlohmann::json orientation_constraints(const nlohmann::json& scenario, Inputs& inputs) {
  const auto model = description_model(inputs, scenario.at("description").get<std::string>());
  const auto arm = the_arm(*model, scenario.at("arm").get<std::string>());
  const auto& joints = arm.joints;
  const auto& link = arm.tip_link;
  require_named_path(*model, link, joints);
  moveit::core::RobotState state(model);
  auto answers = nlohmann::json::array();
  for (const auto& entry : scenario.at("cases")) {
    const auto positions = entry.at("positions").get<std::vector<double>>();
    if (positions.size() != joints.size()) {
      throw std::runtime_error("a case does not hold one position per joint");
    }
    state.setToDefaultValues();
    for (std::size_t i = 0; i < joints.size(); ++i) {
      state.setVariablePosition(joints[i], positions[i]);
    }
    state.update();
    const auto orientation = entry.at("orientation").get<std::vector<double>>();
    const auto tolerance = entry.at("tolerance_rad").get<double>();
    const auto free = free_place(entry.at("free_axis").get<std::string>());
    const double rejecting =
        free ? std::numbers::pi * std::sin(tolerance / 2.0) : tolerance;
    answers.push_back(
        {{"accepted_within_inner",
          accepted(model, state, link, orientation,
                   tolerances(tolerance / std::numbers::sqrt3, free))},
         {"accepted_within_outer",
          accepted(model, state, link, orientation, tolerances(rejecting, free))}});
  }
  return {{"kind", "orientation_constraints"}, {"cases", answers}};
}

}  // namespace ros_reference
