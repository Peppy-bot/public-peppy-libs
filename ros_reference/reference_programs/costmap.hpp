// What the Nav2 references share: a scenario's grid as Nav2's Costmap2D, a scenario's
// footprint as Nav2's polygon, a scenario's pose as Nav2's stamped pose, and each of them
// as a reference output holds it.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nlohmann/json.hpp>

namespace ros_reference {

// The Costmap2D of `grid`, its size in cells, its resolution and the corner of its first
// cell, every cell free.
std::unique_ptr<nav2_costmap_2d::Costmap2D> free_costmap_of(const nlohmann::json& grid);

// The Costmap2D of `grid` with `costs`, one row of cells a member from the first row.
std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_of(const nlohmann::json& grid,
                                                      const nlohmann::json& costs);

// The footprint of `points`, each x and y in meters.
std::vector<geometry_msgs::msg::Point> footprint_of(const nlohmann::json& points);

// The footprint of `points` padded by `padding` meters, as Costmap2DROS pads the robot's
// footprint before its layers read it (padFootprint): each x and y moved by the padding
// away from the axes.
std::vector<geometry_msgs::msg::Point> padded_footprint_of(const nlohmann::json& points,
                                                           double padding);

// The pose of `pose`, x and y in meters and a heading in radians, in `frame`, its
// quaternion made from the heading as nav2_util's orientationAroundZAxis makes it.
geometry_msgs::msg::PoseStamped pose_stamped_of(const nlohmann::json& pose,
                                                const std::string& frame);

// The x, y and heading of `pose`, the heading read back from its quaternion, as a
// reference output holds them.
nlohmann::json pose_read(const geometry_msgs::msg::Pose& pose);

// The size, resolution and origin of `costmap`, as a reference output holds them.
nlohmann::json grid_read(const nav2_costmap_2d::Costmap2D& costmap);

// The costs of `costmap`, one row of cells a member from the first row.
nlohmann::json costs_read(const nav2_costmap_2d::Costmap2D& costmap);

// The points of `footprint`, as a reference output holds them.
nlohmann::json footprint_read(const std::vector<geometry_msgs::msg::Point>& footprint);

}  // namespace ros_reference
