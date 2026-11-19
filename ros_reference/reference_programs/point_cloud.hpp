// The point clouds that the references hand to ROS code: a PointCloud2 of float x, y, z,
// and its points as a reference output carries them.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace ros_reference {

// A PointCloud2 of float x, y, z with `header`, holding `points` in their order.
sensor_msgs::msg::PointCloud2::SharedPtr float_cloud(const std::vector<Eigen::Vector3f>& points,
                                                     const std_msgs::msg::Header& header);

// The points of a scenario's cloud, each [x, y, z], in the frame `frame_id`, as a
// PointCloud2 of float x, y, z.
sensor_msgs::msg::PointCloud2::SharedPtr cloud_of(const nlohmann::json& points,
                                                  const std::string& frame_id);

// The points of `cloud` as ROS code reads them, in single precision: each point [x, y, z],
// each coordinate widened to a double, in their order.
nlohmann::json points_read(const sensor_msgs::msg::PointCloud2& cloud);

}  // namespace ros_reference
