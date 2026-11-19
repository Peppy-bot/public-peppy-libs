#include "amcl_global_localization.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <karto_sdk/Karto.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
// toolbox_types.hpp defines MAP_IDX, which visualization_utils.hpp reads.
#include <slam_toolbox/toolbox_types.hpp>
#include <slam_toolbox/visualization_utils.hpp>

#include "amcl_map_cspace.hpp"
#include "amcl_node.hpp"
#include "room_dataset_files.hpp"

namespace ros_reference {

namespace {

using namespace room_dataset_files;

// The scenario's lidar: its rays a scan, and the parameters of its scans, AMCL reading no
// scan time.
struct Lidar {
  std::size_t rays;
  ScanParameters parameters;
};

Lidar lidar_of(const nlohmann::json& laser) {
  const auto angle_min = laser.at("angle_min").get<float>();
  const auto angle_increment = laser.at("angle_increment").get<float>();
  const auto rays = laser.at("rays").get<std::size_t>();
  if (rays < 2) {
    throw std::runtime_error("an AMCL global localization lidar has fewer than two rays");
  }
  return {rays,
          {angle_min, angle_min + static_cast<float>(rays - 1) * angle_increment,
           angle_increment, laser.at("range_min").get<float>(),
           laser.at("range_max").get<float>(), 0.0f}};
}

std::vector<float> ranges_of(const nlohmann::json& scan, const Lidar& lidar) {
  auto ranges = scan.at("ranges").get<std::vector<float>>();
  if (ranges.size() != lidar.rays) {
    throw std::runtime_error("a scan of an AMCL global localization room holds " +
                             std::to_string(ranges.size()) + " ranges of the lidar's " +
                             std::to_string(lidar.rays));
  }
  return ranges;
}

// The laser of the map's scans, as slam_toolbox's LaserAssistant::makeLaser makes one of a
// scan of the lidar: at the base's origin, facing ahead, its range threshold the lidar's
// maximum range, and no 360-degree laser.
karto::LaserRangeFinder* map_laser(const ScanParameters& parameters) {
  auto* laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
      karto::LaserRangeFinder_Custom,
      karto::Name("Custom Described Lidar : amcl_global_localization"));
  laser->SetOffsetPose(karto::Pose2(0.0, 0.0, 0.0));
  laser->SetMinimumRange(parameters.range_min);
  laser->SetMaximumRange(parameters.range_max);
  laser->SetMinimumAngle(parameters.angle_min);
  laser->SetMaximumAngle(parameters.angle_max);
  laser->SetAngularResolution(parameters.angle_increment);
  laser->SetIs360Laser(false);
  laser->SetRangeThreshold(parameters.range_max);
  karto::SensorManager::GetInstance()->RegisterSensor(laser);
  return laser;
}

// karto's grid of the room's map scans, each at its pose, as slam_toolbox publishes it.
nav_msgs::msg::OccupancyGrid map_grid(const nlohmann::json& room, const nlohmann::json& grid,
                                      const Lidar& lidar, karto::LaserRangeFinder* laser) {
  karto::LocalizedRangeScanVector scans;
  for (const auto& map_scan : room.at("map_scans")) {
    const auto ranges = ranges_of(map_scan, lidar);
    auto* scan = new karto::LocalizedRangeScan(
        laser->GetName(), karto::RangeReadingsVector(ranges.begin(), ranges.end()));
    const auto pose = pose_of(map_scan.at("pose"));
    scan->SetOdometricPose(karto::Pose2(pose.x, pose.y, pose.yaw));
    scan->SetCorrectedPose(karto::Pose2(pose.x, pose.y, pose.yaw));
    scans.push_back(scan);
  }
  std::unique_ptr<karto::OccupancyGrid> occupancy(karto::OccupancyGrid::CreateFromScans(
      scans, grid.at("resolution").get<double>(), grid.at("min_pass_through").get<kt_int32u>(),
      grid.at("occupancy_threshold").get<double>()));
  for (auto* scan : scans) {
    delete scan;
  }
  if (!occupancy) {
    throw std::runtime_error("karto builds no grid of a room's map scans");
  }
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = map_frame;
  map.info.resolution = grid.at("resolution").get<double>();
  map.info.origin.orientation.w = 1.0;
  vis_utils::toNavMap(occupancy.get(), map);
  return map;
}

// The character of a cell of slam_toolbox's grid: '#' occupied, '.' free, '?' unknown.
char cell_character(std::int8_t value) {
  switch (value) {
    case 100:
      return '#';
    case 0:
      return '.';
    case -1:
      return '?';
    default:
      throw std::runtime_error("slam_toolbox's grid holds a cell of " + std::to_string(value));
  }
}

// The grid's origin (m), its size in cells, and its cells, a row a string from the lowest y.
nlohmann::json grid_json(const nav_msgs::msg::OccupancyGrid& map) {
  auto rows = nlohmann::json::array();
  for (std::uint32_t y = 0; y < map.info.height; ++y) {
    std::string row;
    for (std::uint32_t x = 0; x < map.info.width; ++x) {
      row += cell_character(map.data[y * map.info.width + x]);
    }
    rows.push_back(row);
  }
  return {{"origin", {map.info.origin.position.x, map.info.origin.position.y}},
          {"width", map.info.width},
          {"height", map.info.height},
          {"rows", rows}};
}

// The amcl_pose of a search after each scan of the room's drive that AMCL publishes one
// after: the scan's index, the pose [x, y, yaw], and the covariance, as the room dataset's
// runs record an amcl_pose.
nlohmann::json searched(const nlohmann::json& search, const nlohmann::json& room,
                        const nav_msgs::msg::OccupancyGrid& map, const Lidar& lidar) {
  const auto node = amcl_with(search.at("amcl"));
  node->receive_map(map);
  const auto updates_before = amcl_map_cspace_updates;
  node->start_global_localization(search.at("seed").get<int>());
  auto estimates = nlohmann::json::array();
  const auto& scans = room.at("scans");
  for (std::size_t i = 0; i < scans.size(); ++i) {
    const auto& scan = scans[i];
    const auto stamp_ns = scan.at("stamp_ns").get<std::int64_t>();
    const PreparedScan prepared{stamp_ns, {}, ranges_of(scan, lidar)};
    if (const auto estimate = node->receive(stamp_ns, pose_of(scan.at("odometry")),
                                            laser_scan_of(lidar.parameters, prepared))) {
      const auto& c = estimate->covariance;
      estimates.push_back(
          {{"scan", i},
           {"pose", pose_json(estimate->pose)},
           {"covariance", {{"xx", c.xx}, {"xy", c.xy}, {"yy", c.yy}, {"yaw", c.yaw}}}});
    }
  }
  if (amcl_map_cspace_updates == updates_before) {
    throw std::runtime_error("AMCL built its likelihood field without the program's "
                             "map_update_cspace");
  }
  return {{"name", search.at("name")}, {"estimates", estimates}};
}

}  // namespace

nlohmann::json amcl_global_localization(const nlohmann::json& scenario, Inputs&) {
  start_ros();
  const auto lidar = lidar_of(scenario.at("laser"));
  auto* laser = map_laser(lidar.parameters);
  auto rooms = nlohmann::json::array();
  for (const auto& room : scenario.at("rooms")) {
    const auto map = map_grid(room, scenario.at("grid"), lidar, laser);
    auto searches = nlohmann::json::array();
    for (const auto& search : room.at("searches")) {
      searches.push_back(searched(search, room, map, lidar));
    }
    rooms.push_back({{"name", room.at("name")}, {"grid", grid_json(map)}, {"searches", searches}});
  }
  return {{"kind", "amcl_global_localization"}, {"rooms", rooms}};
}

}  // namespace ros_reference
