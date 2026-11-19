#include "amcl_node.hpp"

#include <cstdlib>
#include <memory>
#include <stdexcept>

#include <std_srvs/srv/empty.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav2_node.hpp"
#include "private_member.hpp"

namespace ros_reference {

using namespace room_dataset_files;

GlobalLocalizationReference::~GlobalLocalizationReference() {
  if (tf_listener_) {
    wait_until_spinning(*tf_listener_);
  }
}

void GlobalLocalizationReference::prepare() {
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  initParameters();
  initTransforms();
  initParticleFilter();
  initLaserScan();
  initMessageFilters();
  initPubSub();
  initServices();
  initOdometry();
  active_ = true;
}

void GlobalLocalizationReference::receive_map(const nav_msgs::msg::OccupancyGrid& map) {
  mapReceived(std::make_shared<const nav_msgs::msg::OccupancyGrid>(map));
  if (!first_map_received_) {
    throw std::runtime_error("AMCL refuses the map");
  }
}

void GlobalLocalizationReference::start_global_localization(int seed) {
  srand48(seed);
  globalLocalizationCallback(nullptr, std::make_shared<std_srvs::srv::Empty::Request>(),
                             std::make_shared<std_srvs::srv::Empty::Response>());
}

std::optional<GlobalLocalizationReference::Estimate> GlobalLocalizationReference::receive(
    std::int64_t stamp_ns, const PlanarPose& odometry,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) {
  set_ros_time(get_clock(), stamp_ns);
  tf_buffer_->setTransform(odometry_transform(stamp_ns, odometry), tf_authority, false);
  laserReceived(scan);
  if (last_published_pose_.header.stamp != scan->header.stamp) {
    return std::nullopt;
  }
  const auto& published = last_published_pose_.pose;
  const auto& position = published.pose.position;
  const auto& covariance = published.covariance;
  return Estimate{{position.x, position.y, tf2::getYaw(published.pose.orientation)},
                  {covariance[0], covariance[1], covariance[7], covariance[35]},
                  pf_->converged != 0};
}

std::shared_ptr<GlobalLocalizationReference> amcl_with(const nlohmann::json& parameters) {
  auto node = std::make_shared<GlobalLocalizationReference>(
      options_of(parameters, {{"base_frame_id", base_frame},
                              {"odom_frame_id", odometry_frame},
                              {"global_frame_id", map_frame},
                              {"tf_broadcast", false}}));
  node->prepare();
  check_declared(*node, parameters, "AMCL");
  return node;
}

}  // namespace ros_reference
