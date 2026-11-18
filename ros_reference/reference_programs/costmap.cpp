#include "costmap.hpp"

#include <stdexcept>
#include <string>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <tf2/utils.hpp>

#include "scenario.hpp"

namespace ros_reference {

std::unique_ptr<nav2_costmap_2d::Costmap2D> free_costmap_of(const nlohmann::json& grid) {
  const auto origin = grid.at("origin").get<std::vector<double>>();
  if (origin.size() != 2) {
    throw std::runtime_error("a grid's origin is not x and y");
  }
  return std::make_unique<nav2_costmap_2d::Costmap2D>(
      grid.at("size_x").get<unsigned int>(), grid.at("size_y").get<unsigned int>(),
      grid.at("resolution").get<double>(), origin[0], origin[1], nav2_costmap_2d::FREE_SPACE);
}

std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_of(const nlohmann::json& grid,
                                                      const nlohmann::json& costs) {
  auto costmap = free_costmap_of(grid);
  const auto size_x = costmap->getSizeInCellsX();
  const auto size_y = costmap->getSizeInCellsY();
  if (costs.size() != size_y) {
    throw std::runtime_error("the costs hold " + std::to_string(costs.size()) +
                             " rows of a grid of " + std::to_string(size_y));
  }
  for (unsigned int y = 0; y < size_y; ++y) {
    const auto row = costs.at(y).get<std::vector<unsigned int>>();
    if (row.size() != size_x) {
      throw std::runtime_error("cost row " + std::to_string(y) + " holds " +
                               std::to_string(row.size()) + " cells of " +
                               std::to_string(size_x));
    }
    for (unsigned int x = 0; x < size_x; ++x) {
      if (row[x] > 255) {
        throw std::runtime_error("a cost is above 255");
      }
      costmap->setCost(x, y, static_cast<unsigned char>(row[x]));
    }
  }
  return costmap;
}

std::vector<geometry_msgs::msg::Point> footprint_of(const nlohmann::json& points) {
  std::vector<geometry_msgs::msg::Point> footprint;
  for (const auto& point : points) {
    const auto p = point.get<std::vector<double>>();
    if (p.size() != 2) {
      throw std::runtime_error("a footprint point is not x and y");
    }
    geometry_msgs::msg::Point pt;
    pt.x = p[0];
    pt.y = p[1];
    footprint.push_back(pt);
  }
  return footprint;
}

std::vector<geometry_msgs::msg::Point> padded_footprint_of(const nlohmann::json& points,
                                                           double padding) {
  auto footprint = footprint_of(points);
  nav2_costmap_2d::padFootprint(footprint, padding);
  return footprint;
}

geometry_msgs::msg::PoseStamped pose_stamped_of(const nlohmann::json& pose,
                                                const std::string& frame) {
  const auto p = pose.get<std::vector<double>>();
  if (p.size() != 3) {
    throw std::runtime_error("a pose is not x, y and yaw");
  }
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = frame;
  stamped.pose.position.x = p[0];
  stamped.pose.position.y = p[1];
  stamped.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(p[2]);
  return stamped;
}

nlohmann::json pose_read(const geometry_msgs::msg::Pose& pose) {
  return {pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
}

nlohmann::json grid_read(const nav2_costmap_2d::Costmap2D& costmap) {
  return {
      {"size_x", costmap.getSizeInCellsX()},
      {"size_y", costmap.getSizeInCellsY()},
      {"resolution", costmap.getResolution()},
      {"origin", {costmap.getOriginX(), costmap.getOriginY()}},
  };
}

nlohmann::json costs_read(const nav2_costmap_2d::Costmap2D& costmap) {
  auto rows = nlohmann::json::array();
  for (unsigned int y = 0; y < costmap.getSizeInCellsY(); ++y) {
    auto row = nlohmann::json::array();
    for (unsigned int x = 0; x < costmap.getSizeInCellsX(); ++x) {
      row.push_back(static_cast<unsigned int>(costmap.getCost(x, y)));
    }
    rows.push_back(row);
  }
  return rows;
}

nlohmann::json footprint_read(const std::vector<geometry_msgs::msg::Point>& footprint) {
  auto points = nlohmann::json::array();
  for (const auto& point : footprint) {
    points.push_back({point.x, point.y});
  }
  return points;
}

}  // namespace ros_reference
