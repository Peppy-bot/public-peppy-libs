#include "obstacles.hpp"

#include <stdexcept>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "scenario.hpp"

namespace ros_reference {

namespace {

std::vector<double> numbers(const nlohmann::json& values, std::size_t count, const char* what) {
  auto out = values.get<std::vector<double>>();
  if (out.size() != count) {
    throw std::runtime_error(std::string(what) + " does not hold " + std::to_string(count) +
                             " numbers");
  }
  return out;
}

}  // namespace

void ObstacleLayerOfOneSource::take(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) {
  if (observation_buffers_.size() != 1) {
    throw std::runtime_error("the obstacle layer holds no single observation source");
  }
  pointCloud2Callback(cloud, observation_buffers_.front());
}

geometry_msgs::msg::TransformStamped sensor_transform(const nlohmann::json& sensor) {
  const auto p = numbers(sensor.at("position"), 3, "a sensor position");
  const auto q = numbers(sensor.at("orientation"), 4, "a sensor orientation");
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = grid_frame;
  transform.child_frame_id = sensor_frame;
  transform.transform.translation.x = p[0];
  transform.transform.translation.y = p[1];
  transform.transform.translation.z = p[2];
  transform.transform.rotation.x = q[0];
  transform.transform.rotation.y = q[1];
  transform.transform.rotation.z = q[2];
  transform.transform.rotation.w = q[3];
  return transform;
}

sensor_msgs::msg::PointCloud2::SharedPtr cloud_of(const nlohmann::json& points) {
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = sensor_frame;
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(*cloud, "z");
  for (const auto& point : points) {
    const auto p = numbers(point, 3, "a point");
    *x = static_cast<float>(p[0]);
    *y = static_cast<float>(p[1]);
    *z = static_cast<float>(p[2]);
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

nlohmann::json points_in_grid_frame(tf2_ros::Buffer& tf, const sensor_msgs::msg::PointCloud2& cloud) {
  sensor_msgs::msg::PointCloud2 global;
  tf.transform(cloud, global, grid_frame, tf2::durationFromSec(0.0));
  auto points = nlohmann::json::array();
  for (sensor_msgs::PointCloud2ConstIterator<float> point(global, "x"); point != point.end();
       ++point) {
    points.push_back({point[0], point[1], point[2]});
  }
  return points;
}

}  // namespace ros_reference
