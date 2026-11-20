#include "inflation.hpp"

#include <memory>

#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

#include "costmap.hpp"
#include "nav2_node.hpp"
#include "scenario.hpp"

namespace ros_reference {

namespace {

// The name of the layer, under which the node holds its parameters.
constexpr const char* layer_name = "inflation";

}  // namespace

nlohmann::json inflation(const nlohmann::json& scenario, Inputs&) {
  const auto& parameters = scenario.at("inflation");
  auto node = nav2_node("inflation_reference", {{layer_name, parameters}}, nlohmann::json::object());
  const auto footprint = padded_footprint_of(scenario.at("footprint"),
                                             scenario.at("footprint_padding").get<double>());

  auto grids = nlohmann::json::array();
  for (const auto& case_json : scenario.at("grids")) {
    const auto given = costmap_of(case_json.at("grid"), case_json.at("costs"));
    // The layered costmap of the inflation layer alone, of the grid's size, resolution and
    // origin, with the footprint, from which the layer takes its inscribed radius.
    nav2_costmap_2d::LayeredCostmap layered("map", false, true);
    layered.resizeMap(given->getSizeInCellsX(), given->getSizeInCellsY(), given->getResolution(),
                      given->getOriginX(), given->getOriginY());
    auto layer = std::make_shared<nav2_costmap_2d::InflationLayer>();
    layer->initialize(&layered, layer_name, nullptr, node, nullptr);
    check_declared(node, layer_name, parameters);
    layered.addPlugin(layer);
    layered.setFootprint(footprint);

    auto& master = *layered.getCostmap();
    for (unsigned int y = 0; y < master.getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < master.getSizeInCellsX(); ++x) {
        master.setCost(x, y, given->getCost(x, y));
      }
    }
    layer->updateCosts(master, 0, 0, static_cast<int>(master.getSizeInCellsX()),
                       static_cast<int>(master.getSizeInCellsY()));
    grids.push_back({
        {"inscribed_radius", layered.getInscribedRadius()},
        {"inflated", costs_read(master)},
    });
  }
  return {
      {"kind", "inflation"},
      {"footprint", footprint_read(footprint)},
      {"grids", grids},
  };
}

}  // namespace ros_reference
