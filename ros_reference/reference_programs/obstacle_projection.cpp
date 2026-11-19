#include "obstacle_projection.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <tf2_ros/buffer.hpp>

#include "costmap.hpp"
#include "nav2_node.hpp"
#include "obstacles.hpp"
#include "scenario.hpp"

namespace ros_reference {

namespace {

// The name of the layer, under which the node holds its parameters.
constexpr const char* layer_name = "obstacle";

}  // namespace

nlohmann::json obstacle_projection(const nlohmann::json& scenario, Inputs&) {
  const auto& parameters = scenario.at("obstacle");
  const auto& globals = scenario.at("globals");
  auto node = nav2_node("obstacle_projection_reference", {{layer_name, parameters}}, globals);
  const auto given = free_costmap_of(scenario.at("grid"));
  const auto footprint = padded_footprint_of(scenario.at("footprint"),
                                             scenario.at("footprint_padding").get<double>());

  auto clouds = nlohmann::json::array();
  for (const auto& entry : scenario.at("clouds")) {
    // A layered costmap of the obstacle layer alone, of the scenario's grid, and a tf
    // buffer that holds the sensor's pose alone.
    nav2_costmap_2d::LayeredCostmap layered(grid_frame, false,
                                            globals.at("track_unknown_space").get<bool>());
    layered.resizeMap(given->getSizeInCellsX(), given->getSizeInCellsY(),
                      given->getResolution(), given->getOriginX(), given->getOriginY());
    layered.setFootprint(footprint);
    tf2_ros::Buffer tf(node->get_clock());
    auto layer = std::make_shared<ObstacleLayerOfOneSource>();
    layer->initialize(&layered, layer_name, &tf, node, nullptr);
    check_declared(node, layer_name, parameters);

    const auto transform = sensor_transform(entry.at("sensor"));
    tf.setTransform(transform, "obstacle_projection_reference", true);
    const auto cloud = cloud_of(entry.at("points"), sensor_frame);
    layer->take(cloud);

    // The layer marks the cloud's points with the base at its pose, then clears the
    // cells under the padded footprint there, as a layered costmap's update runs it.
    const auto base = entry.at("base").get<std::vector<double>>();
    if (base.size() != 3) {
      throw std::runtime_error("a base's pose is not x, y and yaw");
    }
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    layer->updateBounds(base[0], base[1], base[2], &min_x, &min_y, &max_x, &max_y);
    layer->updateCosts(*layered.getCostmap(), 0, 0,
                       static_cast<int>(layered.getCostmap()->getSizeInCellsX()),
                       static_cast<int>(layered.getCostmap()->getSizeInCellsY()));

    auto lethal = nlohmann::json::array();
    for (unsigned int y = 0; y < layer->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < layer->getSizeInCellsX(); ++x) {
        if (layer->getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE) {
          lethal.push_back({x, y});
        }
      }
    }
    clouds.push_back({
        {"points", points_in_grid_frame(tf, *cloud)},
        {"lethal", lethal},
    });
  }
  const auto height = [&](const std::string& key) {
    return node->get_parameter(std::string(layer_name) + "." + key).as_double();
  };
  return {
      {"kind", "obstacle_projection"},
      {"grid", grid_read(*given)},
      {"footprint", footprint_read(footprint)},
      {"layer_band", {height("min_obstacle_height"), height("max_obstacle_height")}},
      {"source_band", {height("depth.min_obstacle_height"), height("depth.max_obstacle_height")}},
      {"clouds", clouds},
  };
}

}  // namespace ros_reference
