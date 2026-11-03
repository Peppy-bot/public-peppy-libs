#include "occupancy.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.hpp>
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.hpp>

#include "scenario.hpp"

namespace ros_reference {

namespace {

using occupancy_map_monitor::PointCloudOctomapUpdater;

// The updater takes a point cloud only from its subscription, through its private
// cloudMsgCallback, which finds the sensor in its private tf buffer, tf_buffer_, which
// the buffer's own listener fills from /tf; no public method reaches either. The
// reference puts each sensor's pose in that buffer and calls the callback on each cloud
// itself, on its own thread, so the updater's own code inserts it. The access rules do
// not apply to the names in an explicit instantiation, so the instantiation of
// PrivateMember for each tag below defines `pointer`, which gives that one member.
template <typename Tag, typename Tag::type member>
struct PrivateMember {
  friend typename Tag::type pointer(Tag) { return member; }
};

struct CloudCallback {
  using type =
      void (PointCloudOctomapUpdater::*)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&);
  friend type pointer(CloudCallback);
};
template struct PrivateMember<CloudCallback, &PointCloudOctomapUpdater::cloudMsgCallback>;

struct TfBuffer {
  using type = std::shared_ptr<tf2_ros::Buffer> PointCloudOctomapUpdater::*;
  friend type pointer(TfBuffer);
};
template struct PrivateMember<TfBuffer, &PointCloudOctomapUpdater::tf_buffer_>;

constexpr const char* map_frame = "map";
constexpr const char* sensor_frame = "sensor";

// The sensor's name, under which the node holds the updater's parameters, as MoveIt's
// sensors parameter names a sensor.
constexpr const char* sensor_name = "depth_sensor";

// OctoMap keys a coordinate from 2^15, the key of the voxel whose low corner is 0, on a
// tree of depth 16.
constexpr int key_of_zero = 32768;

std::vector<double> numbers(const nlohmann::json& values, std::size_t count, const char* what) {
  auto out = values.get<std::vector<double>>();
  if (out.size() != count) {
    throw std::runtime_error(std::string(what) + " does not hold " + std::to_string(count) +
                             " numbers");
  }
  return out;
}

// The node of the updater, which holds the scenario's range and padding as the updater's
// parameters, under the sensor's name: the updater reads every point of every cloud, and
// publishes no filtered cloud.
rclcpp::Node::SharedPtr node_of(const nlohmann::json& scenario) {
  const std::string prefix = std::string(sensor_name) + ".";
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
      rclcpp::Parameter(prefix + "point_cloud_topic", "points"),
      rclcpp::Parameter(prefix + "filtered_cloud_topic", ""),
      rclcpp::Parameter(prefix + "max_range", scenario.at("max_range").get<double>()),
      rclcpp::Parameter(prefix + "padding_offset", scenario.at("padding_offset").get<double>()),
      rclcpp::Parameter(prefix + "padding_scale", scenario.at("padding_scale").get<double>()),
      rclcpp::Parameter(prefix + "point_subsample", 1),
      rclcpp::Parameter(prefix + "max_update_rate", 0.0),
  });
  return std::make_shared<rclcpp::Node>("occupancy_reference", options);
}

// The pose of the sensor frame in the map frame, as a static transform.
geometry_msgs::msg::TransformStamped sensor_transform(const nlohmann::json& sensor) {
  const auto p = numbers(sensor.at("position"), 3, "a sensor position");
  const auto q = numbers(sensor.at("orientation"), 4, "a sensor orientation");
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = map_frame;
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

// The sensor's pose as the transform holds it.
nlohmann::json pose_read(const geometry_msgs::msg::TransformStamped& transform) {
  const auto& t = transform.transform.translation;
  const auto& r = transform.transform.rotation;
  return {
      {"position", {t.x, t.y, t.z}},
      {"orientation", {r.x, r.y, r.z, r.w}},
  };
}

Eigen::Isometry3d isometry(const geometry_msgs::msg::TransformStamped& transform) {
  const auto& t = transform.transform;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z));
  pose.rotate(Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z));
  return pose;
}

// The points of a cloud, in the sensor frame, as a PointCloud2 of float x, y, z.
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

// The points of `cloud` as the updater reads them, in single precision.
nlohmann::json points_read(const sensor_msgs::msg::PointCloud2& cloud) {
  auto points = nlohmann::json::array();
  for (sensor_msgs::PointCloud2ConstIterator<float> point(cloud, "x"); point != point.end();
       ++point) {
    points.push_back(nlohmann::json::array(
        {static_cast<double>(point[0]), static_cast<double>(point[1]),
         static_cast<double>(point[2])}));
  }
  return points;
}

}  // namespace

nlohmann::json occupancy(const nlohmann::json& scenario, Inputs&) {
  start_ros();
  auto node = node_of(scenario);
  // The monitor as the planning scene monitor builds it, with a tf buffer and the map
  // frame. It logs that no 3D sensor plugin is defined: it loads the updaters that its
  // `sensors` parameter names, and the node names none, since the reference sets up its
  // updater itself, as the monitor sets up an updater it loads.
  occupancy_map_monitor::OccupancyMapMonitor monitor(
      node, std::make_shared<tf2_ros::Buffer>(node->get_clock()), map_frame,
      scenario.at("voxel_size").get<double>());

  PointCloudOctomapUpdater updater;
  updater.setMonitor(&monitor);
  updater.initialize(node);
  if (!updater.setParams(sensor_name)) {
    throw std::runtime_error("the updater does not read its parameters from the node");
  }

  // The robot's body: a sphere in the map frame, which the updater cuts out of every
  // cloud, placed in the cloud's frame by the transform cache.
  const auto& cut_out = scenario.at("cut_out");
  const auto center = numbers(cut_out.at("center"), 3, "the cut-out's center");
  const auto radius = cut_out.at("radius").get<double>();
  const auto handle = updater.excludeShape(std::make_shared<shapes::Sphere>(radius));
  if (handle == 0) {
    throw std::runtime_error("the updater holds no shape mask to cut the sphere out with");
  }
  Eigen::Isometry3d map_h_sensor = Eigen::Isometry3d::Identity();
  updater.setTransformCacheCallback(
      [&](const std::string&, const rclcpp::Time&, occupancy_map_monitor::ShapeTransformCache& cache) {
        Eigen::Isometry3d map_h_body = Eigen::Isometry3d::Identity();
        map_h_body.translate(Eigen::Vector3d(center[0], center[1], center[2]));
        cache[handle] = map_h_sensor.inverse() * map_h_body;
        return true;
      });

  auto clouds = nlohmann::json::array();
  for (const auto& scan : scenario.at("clouds")) {
    const auto transform = sensor_transform(scan.at("sensor"));
    map_h_sensor = isometry(transform);
    (updater.*pointer(TfBuffer{}))->setTransform(transform, "occupancy_reference", true);
    const auto cloud = cloud_of(scan.at("points"));
    (updater.*pointer(CloudCallback{}))(cloud);
    clouds.push_back({{"sensor", pose_read(transform)}, {"points", points_read(*cloud)}});
  }

  // Every voxel the map holds, its pruned regions expanded to single voxels.
  auto tree = monitor.getOcTreePtr();
  tree->expand();
  auto voxels = nlohmann::json::array();
  for (auto leaf = tree->begin_leafs(), end = tree->end_leafs(); leaf != end; ++leaf) {
    if (leaf.getDepth() != tree->getTreeDepth()) {
      throw std::runtime_error("an expanded tree holds a leaf above the voxel level");
    }
    const auto key = leaf.getKey();
    voxels.push_back({static_cast<int>(key[0]) - key_of_zero,
                      static_cast<int>(key[1]) - key_of_zero,
                      static_cast<int>(key[2]) - key_of_zero,
                      static_cast<double>(leaf->getLogOdds())});
  }
  // The updater's parameters as the node holds them, where setParams read them.
  const auto parameter = [&](const char* key) {
    return node->get_parameter(std::string(sensor_name) + "." + key).as_double();
  };
  return {
      {"kind", "occupancy"},
      {"voxel_size", tree->getResolution()},
      {"max_range", parameter("max_range")},
      {"padding_offset", parameter("padding_offset")},
      {"padding_scale", parameter("padding_scale")},
      {"cut_out", {{"center", center}, {"radius", radius}}},
      {"clouds", clouds},
      {"voxels", voxels},
  };
}

}  // namespace ros_reference
