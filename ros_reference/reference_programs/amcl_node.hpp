// Nav2's AMCL node as the references drive it, in the reference program's process: AMCL
// configured as on_configure configures it, without the executor that it spins there, and
// taking scans as on_activate lets it, its clock and its tf buffer at each scan's stamp.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <nav2_amcl/amcl_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "room_dataset_files.hpp"

namespace ros_reference {

class GlobalLocalizationReference : public nav2_amcl::AmclNode {
 public:
  using AmclNode::AmclNode;

  // initTransforms makes a tf listener that spins an executor on a thread of its own
  // (nav2_amcl 1.5.2 amcl_node.cpp:1382), which dies with the node, canceling that
  // executor and joining the thread (tf2_ros 0.45.10 transform_listener.cpp:67-72): the
  // reference waits until the executor spins.
  ~GlobalLocalizationReference() override;

  void prepare();

  void receive_map(const nav_msgs::msg::OccupancyGrid& map);

  // Seeds drand48, which AMCL alone draws from, and asks for global localization.
  void start_global_localization(int seed);

  // The covariance of the whole filter that amcl_pose carries: of x and y, and the
  // circular variance of the heading.
  struct Covariance {
    double xx;
    double xy;
    double yy;
    double yaw;
  };

  // An amcl_pose that AMCL published, and whether the filter's convergence test held then.
  struct Estimate {
    room_dataset_files::PlanarPose pose;
    Covariance covariance;
    bool converged;
  };

  // The amcl_pose that AMCL published after `scan`, the base at `odometry` in the odometry
  // frame at `stamp_ns`, if it published one: publishAmclPose stamps each pose with its
  // scan's stamp and keeps the pose it publishes in last_published_pose_ (amcl_node.cpp:833,
  // :856), and laserReceived publishes after a resample, after the first scan of a global
  // localization, and after each scan until it published its first pose (:546-556, :590).
  std::optional<Estimate> receive(std::int64_t stamp_ns,
                                  const room_dataset_files::PlanarPose& odometry,
                                  const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);
};

// AMCL with the scenario's `parameters`, each of which it declares, in the references' map,
// odometry and base frames, broadcasting no transform, prepared to take a map.
std::shared_ptr<GlobalLocalizationReference> amcl_with(const nlohmann::json& parameters);

}  // namespace ros_reference
