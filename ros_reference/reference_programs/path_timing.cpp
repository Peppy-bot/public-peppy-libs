#include "path_timing.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

namespace ros_reference {

namespace {

// The time step TOTG integrates at in its own time parameterization
// (`DEFAULT_TIMESTEP`, moveit2 2.15.2).
constexpr double time_step = 1e-3;

Eigen::VectorXd vector_of(const nlohmann::json& values) {
  const auto numbers = values.get<std::vector<double>>();
  Eigen::VectorXd vector(static_cast<Eigen::Index>(numbers.size()));
  for (std::size_t i = 0; i < numbers.size(); ++i) {
    vector[static_cast<Eigen::Index>(i)] = numbers[i];
  }
  return vector;
}

nlohmann::json numbers_of(const Eigen::VectorXd& vector) {
  auto out = nlohmann::json::array();
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    out.push_back(vector[i]);
  }
  return out;
}

}  // namespace

nlohmann::json path_timing(const nlohmann::json& scenario, Inputs&) {
  auto paths = nlohmann::json::array();
  for (const auto& path : scenario.at("paths")) {
    const auto name = path.at("name").get<std::string>();
    std::vector<Eigen::VectorXd> waypoints;
    auto echoed_waypoints = nlohmann::json::array();
    for (const auto& waypoint : path.at("waypoints")) {
      waypoints.push_back(vector_of(waypoint));
      echoed_waypoints.push_back(numbers_of(waypoints.back()));
    }
    const auto max_deviation = path.at("max_deviation").get<double>();
    const auto max_velocity = vector_of(path.at("max_velocity"));
    const auto max_acceleration = vector_of(path.at("max_acceleration"));
    const auto fractions = vector_of(path.at("fractions"));
    nlohmann::json output = {
        {"name", name},
        {"waypoints", echoed_waypoints},
        {"max_deviation", max_deviation},
        {"max_velocity", numbers_of(max_velocity)},
        {"max_acceleration", numbers_of(max_acceleration)},
        {"fractions", numbers_of(fractions)},
    };
    const auto created = trajectory_processing::Path::create(waypoints, max_deviation);
    if (!created) {
      output["refused"] = true;
      paths.push_back(output);
      continue;
    }
    const double length = created->getLength();
    auto configs = nlohmann::json::array();
    for (Eigen::Index i = 0; i < fractions.size(); ++i) {
      configs.push_back(numbers_of(created->getConfig(fractions[i] * length)));
    }
    const auto timed = trajectory_processing::Trajectory::create(
        *created, max_velocity, max_acceleration, time_step);
    if (!timed) {
      throw std::runtime_error(name + ": TOTG times no trajectory along the path");
    }
    output["length"] = length;
    output["configs"] = configs;
    output["duration"] = timed->getDuration();
    paths.push_back(output);
  }
  return {{"kind", "path_timing"}, {"paths", paths}};
}

}  // namespace ros_reference
