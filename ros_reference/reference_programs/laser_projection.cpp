#include "laser_projection.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "point_cloud.hpp"
#include "private_member.hpp"

namespace ros_reference {

namespace {

using pointcloud_to_laserscan::PointCloudToLaserScanNode;

// pointcloud_to_laserscan takes a cloud only from its subscription, through its private
// cloudCallback; no public method reaches it. The reference calls it on each cloud itself,
// on its own thread, through the tag below and PrivateMember (private_member.hpp).
struct CloudCallback {
  using type = void (PointCloudToLaserScanNode::*)(sensor_msgs::msg::PointCloud2::ConstSharedPtr);
  friend type pointer(CloudCallback);
};
template struct PrivateMember<CloudCallback, &PointCloudToLaserScanNode::cloudCallback>;

// The frame of the clouds of a laser projection scenario, and so of their scans.
constexpr const char* cloud_frame = "scan";

// The angles and the range limits that the node wrote into a scan.
nlohmann::json header_read(const sensor_msgs::msg::LaserScan& scan) {
  return {{"angle_min", scan.angle_min},
          {"angle_max", scan.angle_max},
          {"angle_increment", scan.angle_increment},
          {"range_min", scan.range_min},
          {"range_max", scan.range_max}};
}

}  // namespace

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

float narrowed(const nlohmann::json& number) {
  const auto wide = number.get<double>();
  const auto narrow = static_cast<float>(wide);
  if (static_cast<double>(narrow) != wide) {
    throw std::runtime_error(number.dump() + " is not a single-precision value");
  }
  return narrow;
}

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

ScanProjector::ScanProjector(const nlohmann::json& laser_scan) {
  std::vector<rclcpp::Parameter> overrides;
  for (const auto& [name, value] : laser_scan.items()) {
    overrides.push_back(parameter_of(name, value));
  }
  overrides.emplace_back("target_frame", "");
  projector_ = std::make_shared<PointCloudToLaserScanNode>(
      rclcpp::NodeOptions().use_intra_process_comms(true).parameter_overrides(overrides));
  for (const auto& [name, value] : laser_scan.items()) {
    if (!projector_->has_parameter(name)) {
      throw std::runtime_error("pointcloud_to_laserscan declares no parameter " + name);
    }
  }
  taker_ = std::make_shared<rclcpp::Node>("scan_taker",
                                          rclcpp::NodeOptions().use_intra_process_comms(true));
  subscription_ = taker_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::UniquePtr scan) { taken_ = std::move(scan); });
  executor_.add_node(taker_);
}

sensor_msgs::msg::LaserScan ScanProjector::project(
    sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  taken_.reset();
  (projector_.get()->*pointer(CloudCallback{}))(cloud);
  executor_.spin_some();
  if (!taken_) {
    throw std::runtime_error("pointcloud_to_laserscan published no scan for a cloud");
  }
  return *taken_;
}

nlohmann::json laser_projection(const nlohmann::json& scenario, Inputs&) {
  start_ros();
  const auto& laser_scan = scenario.at("laser_scan");
  auto header = nlohmann::json();
  auto ranges = nlohmann::json::array();
  {
    ScanProjector projector(laser_scan);
    for (const auto& points : scenario.at("clouds")) {
      const auto scan = projector.project(cloud_of(points, cloud_frame));
      if (header.is_null()) {
        header = header_read(scan);
      } else if (header_read(scan) != header) {
        throw std::runtime_error("pointcloud_to_laserscan wrote the scans of one scenario "
                                 "with other angles or range limits");
      }
      ranges.push_back(ranges_json(scan.ranges));
    }
  }
  rclcpp::shutdown();
  return {{"kind", "laser_projection"}, {"scan", header}, {"ranges", ranges}};
}

}  // namespace ros_reference
