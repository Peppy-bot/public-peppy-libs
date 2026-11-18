// The files that the reference of a room dataset scenario writes under the directory that
// ROS_REFERENCE_RESULTS names, and what its preparation and its runs share with the AMCL
// global localization reference.
#pragma once

#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace ros_reference::room_dataset_files {

// The frames of every room dataset reference: the scans stand in the base frame, whose
// origin is on the floor below the camera and whose x axis is the camera's optical axis
// laid on the floor.
inline constexpr const char* map_frame = "map";
inline constexpr const char* odometry_frame = "odom";
inline constexpr const char* base_frame = "base_footprint";

inline constexpr std::int64_t nanoseconds_per_second = 1'000'000'000;

// A stamp of a message, from nanoseconds.
builtin_interfaces::msg::Time stamp_of_nanoseconds(std::int64_t stamp_ns);

// A pose on the floor: a position (m) and a heading (rad).
struct PlanarPose {
  double x;
  double y;
  double yaw;
};

// `angle` wrapped into [-pi, pi].
double wrapped(double angle);

// The scan parameters that pointcloud_to_laserscan wrote into each scan, in the single
// precision of the message.
struct ScanParameters {
  float angle_min;
  float angle_max;
  float angle_increment;
  float range_min;
  float range_max;
  float scan_time;
};

// A kept depth frame: its stamp (ns), the base's pose in the ground truth's frame, and the
// ranges of its scan.
struct PreparedScan {
  std::int64_t stamp_ns;
  PlanarPose truth;
  std::vector<float> ranges;
};

struct Prepared {
  ScanParameters parameters;
  std::vector<PreparedScan> scans;
};

// The directory of the results of the scenario `name`.
std::filesystem::path results_of(const std::string& name);

// A pose as the JSON array [x, y, yaw], and back.
nlohmann::json pose_json(const PlanarPose& pose);
PlanarPose pose_of(const nlohmann::json& values);

// The prepared inputs of a scenario: `prepared.json`, and the odometry of each seed,
// `odometry_<seed>.json`, one pose a kept scan.
void write_prepared(const std::filesystem::path& results, const Prepared& prepared);
Prepared read_prepared(const std::filesystem::path& results);
void write_odometry(const std::filesystem::path& results, int seed,
                    const std::vector<PlanarPose>& poses);
std::vector<PlanarPose> read_odometry(const std::filesystem::path& results, int seed);

// The scenario's parameters of a node, with the parameters that this program sets.
rclcpp::NodeOptions options_of(const nlohmann::json& parameters,
                               std::vector<rclcpp::Parameter> program_parameters);

// Throws unless `node` declared each of the scenario's parameters `parameters`, which it
// then read.
template <typename Node>
void check_declared(const Node& node, const nlohmann::json& parameters, const char* reference) {
  for (const auto& [name, value] : parameters.items()) {
    if (!node.has_parameter(name)) {
      throw std::runtime_error(std::string(reference) + " declares no parameter " + name);
    }
  }
}

// The writer of the tf transforms that the runs put in each node's buffer.
inline constexpr const char* tf_authority = "room_dataset";

// The base's pose in the odometry frame at `stamp_ns`, as a transform of `base_frame` in
// `odometry_frame`.
geometry_msgs::msg::TransformStamped odometry_transform(std::int64_t stamp_ns,
                                                        const PlanarPose& pose);

// The scan `scan` as pointcloud_to_laserscan published it.
sensor_msgs::msg::LaserScan::SharedPtr laser_scan_of(const ScanParameters& parameters,
                                                     const PreparedScan& scan);

// The poses that a run recorded: the index of each scan among the kept scans, and its
// pose in the map frame.
struct RecordedPose {
  std::size_t scan;
  PlanarPose pose;
};

// The file of the run `run` of `seed`: `<run>_<seed>.json`.
std::filesystem::path run_file(const std::filesystem::path& results, const std::string& run,
                               int seed);
std::vector<RecordedPose> read_recorded_poses(const std::filesystem::path& file);

// The base name, without extension, of the map that `seed` of a mapping scenario made: its
// pose graph (`.posegraph`, `.data`) and its grid (`.yaml`, `.pgm`).
std::filesystem::path map_of(const std::filesystem::path& results, int seed);

}  // namespace ros_reference::room_dataset_files
