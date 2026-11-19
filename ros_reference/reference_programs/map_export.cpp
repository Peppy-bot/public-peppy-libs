#include "map_export.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace ros_reference {

namespace {

// The occupancy grid message of a grid of the scenario: its size, the side of a cell, the
// corner of its first cell and the heading of the grid, and each cell row by row from the
// first, -1 for unknown and 0 to 100 for the rest.
nav_msgs::msg::OccupancyGrid grid_of(const nlohmann::json& grid) {
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = grid.at("width").get<std::uint32_t>();
  map.info.height = grid.at("height").get<std::uint32_t>();
  map.info.resolution = grid.at("resolution").get<float>();
  const auto origin = grid.at("origin").get<std::vector<double>>();
  if (origin.size() != 3) {
    throw std::runtime_error("a grid's origin is not [x, y, yaw]");
  }
  map.info.origin.position.x = origin[0];
  map.info.origin.position.y = origin[1];
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, origin[2]);
  map.info.origin.orientation.x = orientation.x();
  map.info.origin.orientation.y = orientation.y();
  map.info.origin.orientation.z = orientation.z();
  map.info.origin.orientation.w = orientation.w();
  for (const auto& cell : grid.at("cells")) {
    map.data.push_back(cell.get<std::int8_t>());
  }
  return map;
}

std::string file_text(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("the map saver wrote no " + path.string());
  }
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

}  // namespace

nlohmann::json map_export(const nlohmann::json& scenario, Inputs&) {
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() / "ros_reference_map_export";
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);
  nav2_map_server::SaveParameters parameters;
  parameters.map_file_name = (directory / "map").string();
  parameters.image_format = "pgm";
  parameters.free_thresh = scenario.at("free_thresh").get<double>();
  parameters.occupied_thresh = scenario.at("occupied_thresh").get<double>();
  parameters.mode = nav2_map_server::MapMode::Trinary;
  auto maps = nlohmann::json::array();
  for (const auto& grid : scenario.at("grids")) {
    if (!nav2_map_server::saveMapToFile(grid_of(grid), parameters)) {
      throw std::runtime_error("the map saver refused a grid");
    }
    const auto image = file_text(directory / "map.pgm");
    auto bytes = nlohmann::json::array();
    for (const unsigned char byte : image) {
      bytes.push_back(static_cast<int>(byte));
    }
    maps.push_back({{"image", bytes}, {"description", file_text(directory / "map.yaml")}});
  }
  std::filesystem::remove_all(directory);
  return {{"kind", "map_export"}, {"maps", maps}};
}

}  // namespace ros_reference
