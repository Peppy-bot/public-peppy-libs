// What the references of Nav2's obstacle layer share: the layer of one observation
// source, set up on a layered costmap from a node's parameters, and the clouds it marks,
// each from its sensor's pose, which the reference places in the layer's tf buffer.
#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>

#include "point_cloud.hpp"

namespace ros_reference {

// The frame of the grid, the layered costmap's global frame.
inline constexpr const char* grid_frame = "map";

// The frame of a cloud's sensor.
inline constexpr const char* sensor_frame = "sensor";

// The obstacle layer, with its one observation buffer in reach: the layer takes a cloud
// only through the callback its subscription calls, with the buffer it bound there.
class ObstacleLayerOfOneSource : public nav2_costmap_2d::ObstacleLayer {
 public:
  // Gives `cloud` to the layer's callback, as the tf message filter of its subscription
  // gives a cloud whose transform the buffer holds.
  void take(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
};

// The pose of the sensor frame in the grid's frame, from a scenario's sensor, its position
// and its orientation as a unit quaternion [x, y, z, w], as a static transform.
geometry_msgs::msg::TransformStamped sensor_transform(const nlohmann::json& sensor);

// The points of `cloud` transformed into the grid's frame by tf2, as the observation
// buffer transforms them, each coordinate in single precision.
nlohmann::json points_in_grid_frame(tf2_ros::Buffer& tf, const sensor_msgs::msg::PointCloud2& cloud);

}  // namespace ros_reference
