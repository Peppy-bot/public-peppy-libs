#include "chain_collision.hpp"

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <moveit/robot_state/robot_state.hpp>

#include "robot.hpp"
#include "scenario.hpp"

namespace ros_reference {
namespace {

// The links with collision geometry whose pose the chain's `joints` decide: each link
// whose walk up toward the root reaches a joint of `joints` across fixed joints and
// across the movable joints of the path from the root to `tip_link` that `joints` does
// not name, which stand at 0, in the order of the model's links.
std::vector<std::string> carried_links(const moveit::core::RobotModel& model,
                                       const std::vector<std::string>& joints,
                                       const std::string& tip_link) {
  std::vector<const moveit::core::JointModel*> path;
  for (const auto* joint = model.getLinkModel(tip_link)->getParentJointModel(); joint != nullptr;
       joint = joint->getParentLinkModel() == nullptr
                   ? nullptr
                   : joint->getParentLinkModel()->getParentJointModel()) {
    path.push_back(joint);
  }
  std::vector<std::string> carried;
  for (const auto* link : model.getLinkModelsWithCollisionGeometry()) {
    for (const auto* joint = link->getParentJointModel(); joint != nullptr;
         joint = joint->getParentLinkModel() == nullptr
                     ? nullptr
                     : joint->getParentLinkModel()->getParentJointModel()) {
      if (std::find(joints.begin(), joints.end(), joint->getName()) != joints.end()) {
        carried.push_back(link->getName());
        break;
      }
      const bool on_path = std::find(path.begin(), path.end(), joint) != path.end();
      if (joint->getType() != moveit::core::JointModel::FIXED && !on_path) {
        break;
      }
    }
  }
  return carried;
}

// The body of each collision shape of `links`, each with the transform from its link's
// frame, as MoveIt's robot filter (point_containment_filter::ShapeMask) holds them: a
// convex mesh for a mesh, and the primitive itself for a box, a cylinder or a sphere.
std::vector<std::pair<bodies::BodyPtr, Eigen::Isometry3d>> shape_bodies(
    const moveit::core::RobotModel& model, const std::vector<std::string>& links) {
  std::vector<std::pair<bodies::BodyPtr, Eigen::Isometry3d>> shapes;
  for (const auto& link : links) {
    const auto* link_model = model.getLinkModel(link);
    const auto& origins = link_model->getCollisionOriginTransforms();
    for (std::size_t i = 0; i < link_model->getShapes().size(); ++i) {
      shapes.emplace_back(bodies::createBodyFromShape(link_model->getShapes()[i].get()),
                          origins[i]);
    }
  }
  return shapes;
}

// For each body of `shapes`, in their order, the center that MoveIt grows it about, in
// its link's frame: the mean of a convex mesh's vertices, or a primitive's origin.
nlohmann::json growth_centers(
    const moveit::core::RobotModel& model, const std::vector<std::string>& links,
    std::vector<std::pair<bodies::BodyPtr, Eigen::Isometry3d>>& shapes) {
  auto centers = nlohmann::json::array();
  std::size_t shape = 0;
  for (const auto& link : links) {
    for (std::size_t i = 0; i < model.getLinkModel(link)->getShapes().size(); ++i) {
      auto& [body, origin] = shapes[shape++];
      body->setScaleDirty(1.0);
      body->setPaddingDirty(0.0);
      body->setPose(origin);
      bodies::BoundingSphere sphere;
      body->computeBoundingSphere(sphere);
      centers.push_back(
          {{"link", link},
           {"center", {sphere.center.x(), sphere.center.y(), sphere.center.z()}}});
    }
  }
  return centers;
}

}  // namespace

nlohmann::json chain_collision(const nlohmann::json& scenario, Inputs& inputs) {
  std::optional<std::string> mesh_directory;
  if (scenario.contains("meshes")) {
    mesh_directory = scenario.at("meshes").get<std::string>();
  }
  const auto model =
      robot_model_with_meshes(inputs, scenario.at("urdf").get<std::string>(), mesh_directory);
  moveit::core::RobotState state(model);
  auto chains = nlohmann::json::array();
  for (const auto& chain : scenario.at("chains")) {
    const auto joints = chain.at("joints").get<std::vector<std::string>>();
    require_named_path(*model, chain.at("tip_link").get<std::string>(), joints);
    const auto links = carried_links(*model, joints, chain.at("tip_link").get<std::string>());
    auto shapes = shape_bodies(*model, links);
    const auto centers = growth_centers(*model, links, shapes);
    auto configurations = nlohmann::json::array();
    for (const auto& configuration : chain.at("configurations")) {
      const auto positions = configuration.at("positions").get<std::vector<double>>();
      if (positions.size() != joints.size()) {
        throw std::runtime_error("a configuration of " + chain.at("name").get<std::string>() +
                                 " does not hold one position per joint");
      }
      state.setToDefaultValues();
      for (std::size_t i = 0; i < joints.size(); ++i) {
        state.setVariablePosition(joints[i], positions[i]);
      }
      state.update();
      auto poses = nlohmann::json::array();
      for (const auto& link : links) {
        poses.push_back(link_pose(state, link));
      }
      auto echoed_positions = nlohmann::json::array();
      for (const auto position : positions) {
        echoed_positions.push_back(position);
      }
      std::vector<Eigen::Vector3d> points;
      auto echoed_points = nlohmann::json::array();
      for (const auto& point : configuration.at("points")) {
        const auto link = point.at("link").get<std::string>();
        const auto position = point.at("position").get<std::vector<double>>();
        if (position.size() != 3) {
          throw std::runtime_error("a probe point of " + link + " does not hold three coordinates");
        }
        points.push_back(state.getGlobalLinkTransform(link) *
                         Eigen::Vector3d(position[0], position[1], position[2]));
        echoed_points.push_back(
            {{"link", link},
             {"position", {position[0], position[1], position[2]}}});
      }
      auto inside = nlohmann::json::array();
      for (const auto& padding : scenario.at("paddings")) {
        std::size_t shape = 0;
        for (const auto& link : links) {
          const auto& transform = state.getGlobalLinkTransform(link);
          for (std::size_t i = 0; i < model->getLinkModel(link)->getShapes().size(); ++i) {
            auto& [body, origin] = shapes[shape++];
            body->setScaleDirty(padding.at("scale").get<double>());
            body->setPaddingDirty(padding.at("offset").get<double>());
            body->setPose(transform * origin);
          }
        }
        auto held = nlohmann::json::array();
        for (const auto& point : points) {
          bool contained = false;
          for (const auto& [body, origin] : shapes) {
            contained = contained || body->containsPoint(point);
          }
          held.push_back(contained);
        }
        inside.push_back(held);
      }
      configurations.push_back({{"positions", echoed_positions},
                                {"points", echoed_points},
                                {"links", poses},
                                {"inside", inside}});
    }
    chains.push_back({{"name", chain.at("name")},
                      {"links", links},
                      {"bodies", centers},
                      {"configurations", configurations}});
  }
  auto paddings = nlohmann::json::array();
  for (const auto& padding : scenario.at("paddings")) {
    paddings.push_back({{"scale", padding.at("scale").get<double>()},
                        {"offset", padding.at("offset").get<double>()}});
  }
  return {{"kind", "chain_collision"}, {"paddings", paddings}, {"chains", chains}};
}

}  // namespace ros_reference
