#include "global_costmap.hpp"

#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/static_layer.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.hpp>

#include "costmap.hpp"
#include "nav2_node.hpp"
#include "obstacles.hpp"
#include "scenario.hpp"

namespace ros_reference {

namespace {

constexpr const char* static_name = "static_layer";
constexpr const char* obstacle_name = "obstacle_layer";
constexpr const char* inflation_name = "inflation_layer";

// The static layer, with its map callback in reach: the layer takes a map only through
// the callback its subscription calls.
class StaticLayerOfOneMap : public nav2_costmap_2d::StaticLayer {
 public:
  void take(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map) { incomingMap(map); }
};

// The rows of `image`, top first, each pixel an 8-bit gray value, as a binary PGM at
// `path`, the file map_io reads.
void write_pgm(const std::filesystem::path& path, const nlohmann::json& image) {
  const auto height = image.size();
  const auto width = image.at(0).size();
  std::ofstream file(path, std::ios::binary);
  file << "P5\n" << width << ' ' << height << "\n255\n";
  for (const auto& row : image) {
    const auto pixels = row.get<std::vector<unsigned int>>();
    if (pixels.size() != width) {
      throw std::runtime_error("the image's rows differ in length");
    }
    for (const auto pixel : pixels) {
      if (pixel > 255) {
        throw std::runtime_error("a pixel is above 255");
      }
      file.put(static_cast<char>(pixel));
    }
  }
  file.close();
  if (!file) {
    throw std::runtime_error("cannot write " + path.string());
  }
}

// The saved map of `map` as map_io loads it: its image as a trinary map at the
// resolution and origin `map` gives, in the grid's frame.
nav_msgs::msg::OccupancyGrid::SharedPtr saved_map(const nlohmann::json& map) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("ros_reference_map_" + std::to_string(::getpid()) + ".pgm");
  write_pgm(path, map.at("image"));
  nav2_map_server::LoadParameters load;
  load.image_file_name = path.string();
  load.resolution = map.at("resolution").get<double>();
  load.origin = map.at("origin").get<std::vector<double>>();
  load.free_thresh = map.at("free_thresh").get<double>();
  load.occupied_thresh = map.at("occupied_thresh").get<double>();
  load.mode = nav2_map_server::map_mode_from_string(map.at("mode").get<std::string>());
  load.negate = map.at("negate").get<bool>();
  auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  nav2_map_server::loadMapFromFile(load, *grid);
  std::filesystem::remove(path);
  grid->header.frame_id = grid_frame;
  return grid;
}

// The values of `map`, -1 for unknown, 0 to 100 for known, one row of cells a member from
// the first row.
nlohmann::json map_read(const nav_msgs::msg::OccupancyGrid& map) {
  auto rows = nlohmann::json::array();
  for (unsigned int y = 0; y < map.info.height; ++y) {
    auto row = nlohmann::json::array();
    for (unsigned int x = 0; x < map.info.width; ++x) {
      row.push_back(static_cast<int>(map.data[y * map.info.width + x]));
    }
    rows.push_back(row);
  }
  return rows;
}

}  // namespace

nlohmann::json global_costmap(const nlohmann::json& scenario, Inputs&) {
  const auto& globals = scenario.at("globals");
  const nlohmann::json plugins = {
      {static_name, scenario.at("static")},
      {obstacle_name, scenario.at("obstacle")},
      {inflation_name, scenario.at("inflation")},
  };
  auto node = nav2_node("global_costmap_reference", plugins, globals);
  const auto map = saved_map(scenario.at("map"));

  nav2_costmap_2d::LayeredCostmap layered(grid_frame, false,
                                          globals.at("track_unknown_space").get<bool>());
  layered.resizeMap(map->info.width, map->info.height, map->info.resolution,
                    map->info.origin.position.x, map->info.origin.position.y);
  tf2_ros::Buffer tf(node->get_clock());
  auto static_layer = std::make_shared<StaticLayerOfOneMap>();
  auto obstacle_layer = std::make_shared<ObstacleLayerOfOneSource>();
  auto inflation_layer = std::make_shared<nav2_costmap_2d::InflationLayer>();
  layered.addPlugin(static_layer);
  static_layer->initialize(&layered, static_name, &tf, node, nullptr);
  layered.addPlugin(obstacle_layer);
  obstacle_layer->initialize(&layered, obstacle_name, &tf, node, nullptr);
  layered.addPlugin(inflation_layer);
  inflation_layer->initialize(&layered, inflation_name, &tf, node, nullptr);
  for (const auto& [name, parameters] : plugins.items()) {
    check_declared(node, name, parameters);
  }
  const auto footprint = padded_footprint_of(scenario.at("footprint"),
                                             scenario.at("footprint_padding").get<double>());
  layered.setFootprint(footprint);

  static_layer->take(map);
  const auto& entry = scenario.at("cloud");
  const auto transform = sensor_transform(entry.at("sensor"));
  tf.setTransform(transform, "global_costmap_reference", true);
  const auto cloud = cloud_of(entry.at("points"));
  obstacle_layer->take(cloud);
  const auto base = scenario.at("base").get<std::vector<double>>();
  if (base.size() != 3) {
    throw std::runtime_error("a base's pose is not x, y and yaw");
  }
  layered.updateMap(base[0], base[1], base[2]);

  const auto& master = *layered.getCostmap();
  return {
      {"kind", "global_costmap"},
      {"map", map_read(*map)},
      {"grid", grid_read(master)},
      {"footprint", footprint_read(footprint)},
      {"points", points_in_grid_frame(tf, *cloud)},
      {"costs", costs_read(master)},
  };
}

}  // namespace ros_reference
