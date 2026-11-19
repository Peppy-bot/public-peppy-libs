// Laser projection: pointcloud_to_laserscan projects each point cloud of the scenario to a
// 2-D scan, with the scenario's height band, angles and range limits and no target frame.
// The output holds the parameters as the node read them, and each cloud, with its points in
// the single precision in which the node read them, and its scan.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "scenario.hpp"

namespace ros_reference {

// pointcloud_to_laserscan's node, which projects a cloud to a scan with the parameters
// `laser_scan` and no target frame, and the node that takes each scan it publishes. Both
// use intra-process communication, so publishing a scan puts it in the subscription's queue
// before cloudCallback returns, and spinning the taking node's own executor once hands it
// over.
class ScanProjector {
 public:
  explicit ScanProjector(const nlohmann::json& laser_scan);

  sensor_msgs::msg::LaserScan project(sensor_msgs::msg::PointCloud2::SharedPtr cloud);

 private:
  std::shared_ptr<pointcloud_to_laserscan::PointCloudToLaserScanNode> projector_;
  rclcpp::Node::SharedPtr taker_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  sensor_msgs::msg::LaserScan::UniquePtr taken_;
};

// The ranges of a scan: each finite range widened to a double, a JSON number that
// write_json writes as its shortest decimal, and each mark of REP 117, which JSON has no
// number for, as the string "inf", "-inf" or "nan".
nlohmann::json ranges_json(const std::vector<float>& ranges);

// A range of ranges_json, and back.
float range_of(const nlohmann::json& range);

// The single-precision value whose widening `number` holds. Throws on a number that no
// float widens to.
float narrowed(const nlohmann::json& number);

nlohmann::json laser_projection(const nlohmann::json& scenario, Inputs& inputs);

}  // namespace ros_reference
