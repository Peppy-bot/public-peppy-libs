#include "point_cloud.hpp"

#include <stdexcept>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ros_reference {

sensor_msgs::msg::PointCloud2::SharedPtr float_cloud(const std::vector<Eigen::Vector3f>& points,
                                                     const std_msgs::msg::Header& header) {
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header = header;
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(*cloud, "z");
  for (const auto& point : points) {
    *x = point.x();
    *y = point.y();
    *z = point.z();
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2::SharedPtr cloud_of(const nlohmann::json& points,
                                                  const std::string& frame_id) {
  std::vector<Eigen::Vector3f> single;
  for (const auto& point : points) {
    const auto p = point.get<std::vector<double>>();
    if (p.size() != 3) {
      throw std::runtime_error("a point does not hold 3 numbers");
    }
    single.emplace_back(static_cast<float>(p[0]), static_cast<float>(p[1]),
                        static_cast<float>(p[2]));
  }
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  return float_cloud(single, header);
}

nlohmann::json points_read(const sensor_msgs::msg::PointCloud2& cloud) {
  auto points = nlohmann::json::array();
  for (sensor_msgs::PointCloud2ConstIterator<float> point(cloud, "x"); point != point.end();
       ++point) {
    points.push_back(nlohmann::json::array({static_cast<double>(point[0]),
                                            static_cast<double>(point[1]),
                                            static_cast<double>(point[2])}));
  }
  return points;
}

}  // namespace ros_reference
