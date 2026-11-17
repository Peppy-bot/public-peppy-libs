#include "room_dataset_files.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "scenario.hpp"

namespace ros_reference::room_dataset_files {

namespace {

nlohmann::json read_document(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot read " + path.string());
  }
  return nlohmann::json::parse(file);
}

// The ranges of a scan: each finite range widened to a double, a JSON number that
// write_json writes as its shortest decimal, and each mark of REP 117, which JSON has no
// number for, as the string "inf", "-inf" or "nan".
nlohmann::json ranges_json(const std::vector<float>& ranges) {
  auto out = nlohmann::json::array();
  for (const float range : ranges) {
    if (std::isnan(range)) {
      out.push_back("nan");
    } else if (std::isinf(range)) {
      out.push_back(range > 0 ? "inf" : "-inf");
    } else {
      out.push_back(static_cast<double>(range));
    }
  }
  return out;
}

// The single-precision value whose widening `number` holds.
float narrowed(const nlohmann::json& number) {
  const auto wide = number.get<double>();
  const auto narrow = static_cast<float>(wide);
  if (static_cast<double>(narrow) != wide) {
    throw std::runtime_error(number.dump() + " is not a single-precision value");
  }
  return narrow;
}

// A range of ranges_json.
float range_of(const nlohmann::json& range) {
  if (!range.is_string()) {
    return narrowed(range);
  }
  const auto mark = range.get<std::string>();
  if (mark == "inf") {
    return std::numeric_limits<float>::infinity();
  }
  if (mark == "-inf") {
    return -std::numeric_limits<float>::infinity();
  }
  if (mark == "nan") {
    return std::numeric_limits<float>::quiet_NaN();
  }
  throw std::runtime_error(range.dump() + " is neither a range nor a mark of REP 117");
}

}  // namespace

std::filesystem::path results_of(const std::string& name) {
  return directory_named_by("ROS_REFERENCE_RESULTS") / name;
}

builtin_interfaces::msg::Time stamp_of_nanoseconds(std::int64_t stamp_ns) {
  if (stamp_ns < 0 ||
      stamp_ns / nanoseconds_per_second > std::numeric_limits<std::int32_t>::max()) {
    throw std::runtime_error("the stamp " + std::to_string(stamp_ns) +
                             " ns lies outside the seconds that a message's stamp holds");
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(stamp_ns / nanoseconds_per_second);
  stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % nanoseconds_per_second);
  return stamp;
}

double wrapped(double angle) { return std::remainder(angle, 2.0 * M_PI); }

nlohmann::json pose_json(const PlanarPose& pose) { return {pose.x, pose.y, pose.yaw}; }

PlanarPose pose_of(const nlohmann::json& values) {
  return {values.at(0).get<double>(), values.at(1).get<double>(), values.at(2).get<double>()};
}

void write_prepared(const std::filesystem::path& results, const Prepared& prepared) {
  const auto& p = prepared.parameters;
  auto scans = nlohmann::json::array();
  for (const auto& scan : prepared.scans) {
    scans.push_back({{"stamp_ns", scan.stamp_ns},
                     {"truth", pose_json(scan.truth)},
                     {"ranges", ranges_json(scan.ranges)}});
  }
  write_json(results / "prepared.json",
             {{"laser_scan",
               {{"angle_min", p.angle_min},
                {"angle_max", p.angle_max},
                {"angle_increment", p.angle_increment},
                {"range_min", p.range_min},
                {"range_max", p.range_max},
                {"scan_time", p.scan_time}}},
              {"scans", scans}});
}

Prepared read_prepared(const std::filesystem::path& results) {
  const auto document = read_document(results / "prepared.json");
  const auto& p = document.at("laser_scan");
  Prepared prepared{
      .parameters = {.angle_min = narrowed(p.at("angle_min")),
                     .angle_max = narrowed(p.at("angle_max")),
                     .angle_increment = narrowed(p.at("angle_increment")),
                     .range_min = narrowed(p.at("range_min")),
                     .range_max = narrowed(p.at("range_max")),
                     .scan_time = narrowed(p.at("scan_time"))},
      .scans = {}};
  for (const auto& scan : document.at("scans")) {
    std::vector<float> ranges;
    for (const auto& range : scan.at("ranges")) {
      ranges.push_back(range_of(range));
    }
    prepared.scans.push_back({.stamp_ns = scan.at("stamp_ns").get<std::int64_t>(),
                              .truth = pose_of(scan.at("truth")),
                              .ranges = std::move(ranges)});
  }
  return prepared;
}

void write_odometry(const std::filesystem::path& results, int seed,
                    const std::vector<PlanarPose>& poses) {
  auto lines = nlohmann::json::array();
  for (const auto& pose : poses) {
    lines.push_back(pose_json(pose));
  }
  write_json(run_file(results, "odometry", seed), {{"seed", seed}, {"poses", lines}});
}

std::vector<PlanarPose> read_odometry(const std::filesystem::path& results, int seed) {
  const auto document = read_document(run_file(results, "odometry", seed));
  std::vector<PlanarPose> poses;
  for (const auto& pose : document.at("poses")) {
    poses.push_back(pose_of(pose));
  }
  return poses;
}

sensor_msgs::msg::LaserScan::SharedPtr laser_scan_of(const ScanParameters& parameters,
                                                     const PreparedScan& scan) {
  auto message = std::make_shared<sensor_msgs::msg::LaserScan>();
  message->header.frame_id = base_frame;
  message->header.stamp = stamp_of_nanoseconds(scan.stamp_ns);
  message->angle_min = parameters.angle_min;
  message->angle_max = parameters.angle_max;
  message->angle_increment = parameters.angle_increment;
  message->time_increment = 0.0f;
  message->scan_time = parameters.scan_time;
  message->range_min = parameters.range_min;
  message->range_max = parameters.range_max;
  message->ranges = scan.ranges;
  return message;
}

std::filesystem::path run_file(const std::filesystem::path& results, const std::string& run,
                               int seed) {
  return results / (run + "_" + std::to_string(seed) + ".json");
}

std::vector<RecordedPose> read_recorded_poses(const std::filesystem::path& file) {
  const auto document = read_document(file);
  std::vector<RecordedPose> poses;
  for (const auto& pose : document.at("poses")) {
    poses.push_back({pose.at("scan").get<std::size_t>(), pose_of(pose.at("pose"))});
  }
  return poses;
}

std::filesystem::path map_of(const std::filesystem::path& results) { return results / "map"; }

rclcpp::Parameter parameter_of(const std::string& name, const nlohmann::json& value) {
  if (value.is_boolean()) {
    return {name, value.get<bool>()};
  }
  if (value.is_number_integer()) {
    return {name, value.get<std::int64_t>()};
  }
  if (value.is_number_float()) {
    return {name, value.get<double>()};
  }
  if (value.is_string()) {
    return {name, value.get<std::string>()};
  }
  throw std::runtime_error("the parameter " + name + " is not a bool, a number or a string");
}

}  // namespace ros_reference::room_dataset_files
