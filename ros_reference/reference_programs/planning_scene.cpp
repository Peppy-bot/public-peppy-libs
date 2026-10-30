#include "planning_scene.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <octomap/OcTree.h>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "robot.hpp"

namespace ros_reference {
namespace {

using shape_msgs::msg::SolidPrimitive;

// The name motion_planning gives the voxel map in a pair.
const std::string occupancy_map = "occupancy_map";

// OctoMap's key of the voxel whose low corner is 0, on each axis of a tree of depth 16.
constexpr int key_of_zero = 32768;

// The name of `name` in MoveIt's allowed collision matrix, which names the voxel map as
// its octomap.
std::string moveit_name(const std::string& name) {
  return name == occupancy_map ? planning_scene::PlanningScene::OCTOMAP_NS : name;
}

// The name motion_planning gives the entry `name` of MoveIt's allowed collision matrix.
std::string contract_name(const std::string& name) {
  return name == planning_scene::PlanningScene::OCTOMAP_NS ? occupancy_map : name;
}

geometry_msgs::msg::Pose pose_of(const nlohmann::json& fields) {
  const auto p = fields.at("position").get<std::vector<double>>();
  const auto q = fields.at("orientation").get<std::vector<double>>();
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.at(0);
  pose.position.y = p.at(1);
  pose.position.z = p.at(2);
  pose.orientation.x = q.at(0);
  pose.orientation.y = q.at(1);
  pose.orientation.z = q.at(2);
  pose.orientation.w = q.at(3);
  return pose;
}

nlohmann::json pose_json(const geometry_msgs::msg::Pose& pose) {
  return {{"position", {pose.position.x, pose.position.y, pose.position.z}},
          {"orientation",
           {pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w}}};
}

const std::map<std::string, std::uint8_t> primitive_types = {
    {"box", SolidPrimitive::BOX},
    {"sphere", SolidPrimitive::SPHERE},
    {"cylinder", SolidPrimitive::CYLINDER},
};

// The collision object message that adds the object of `entry`, an entry of
// apply_collision_objects as motion_planning writes it, with its shapes, in `frame`.
moveit_msgs::msg::CollisionObject object_added(const nlohmann::json& entry,
                                               const std::string& frame) {
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = entry.at("id").get<std::string>();
  object.pose = pose_of(entry);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  for (const auto& shape : entry.at("shapes")) {
    const auto kind = shape.at("kind").get<std::string>();
    if (kind == "mesh") {
      shape_msgs::msg::Mesh mesh;
      const auto coordinates = shape.at("mesh_vertices").get<std::vector<double>>();
      for (std::size_t i = 0; i + 2 < coordinates.size(); i += 3) {
        geometry_msgs::msg::Point vertex;
        vertex.x = coordinates[i];
        vertex.y = coordinates[i + 1];
        vertex.z = coordinates[i + 2];
        mesh.vertices.push_back(vertex);
      }
      const auto indices = shape.at("mesh_triangles").get<std::vector<std::uint32_t>>();
      for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
        shape_msgs::msg::MeshTriangle triangle;
        triangle.vertex_indices = {indices[i], indices[i + 1], indices[i + 2]};
        mesh.triangles.push_back(triangle);
      }
      object.meshes.push_back(mesh);
      object.mesh_poses.push_back(pose_of(shape));
      continue;
    }
    SolidPrimitive primitive;
    primitive.type = primitive_types.at(kind);
    for (const double dimension : shape.at("dimensions").get<std::vector<double>>()) {
      primitive.dimensions.push_back(dimension);
    }
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose_of(shape));
  }
  return object;
}

// Sets each joint `positions` names to its position in the scene's current state, after
// refusing positions that leave a movable joint on the path to `link` unnamed.
void pose_joints(planning_scene::PlanningScene& scene, const std::string& link,
                 const nlohmann::json& positions) {
  std::vector<std::string> joints;
  for (const auto& [joint, position] : positions.items()) {
    joints.push_back(joint);
  }
  require_named_path(*scene.getRobotModel(), link, joints);
  auto& state = scene.getCurrentStateNonConst();
  for (const auto& [joint, position] : positions.items()) {
    state.setVariablePosition(joint, position.get<double>());
  }
  state.update();
}

// The collision object message of one entry of apply_collision_objects: an add, a move
// or a removal.
moveit_msgs::msg::CollisionObject object_message(const nlohmann::json& entry,
                                                 const std::string& frame) {
  const auto operation = entry.at("operation").get<std::string>();
  if (operation == "add") {
    return object_added(entry, frame);
  }
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame;
  object.id = entry.at("id").get<std::string>();
  if (operation == "move") {
    object.pose = pose_of(entry);
    object.operation = moveit_msgs::msg::CollisionObject::MOVE;
    return object;
  }
  if (operation == "remove") {
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    return object;
  }
  throw std::runtime_error("no operation of an object is " + operation);
}

// Marks each voxel of `keys` occupied in `tree`, the octomap of `scene`, which then holds
// the tree as it stands.
void mark(planning_scene::PlanningScene& scene, const std::shared_ptr<octomap::OcTree>& tree,
          const nlohmann::json& keys) {
  for (const auto& key : keys) {
    const auto k = key.get<std::vector<int>>();
    tree->updateNode(octomap::OcTreeKey(static_cast<uint16_t>(k.at(0) + key_of_zero),
                                        static_cast<uint16_t>(k.at(1) + key_of_zero),
                                        static_cast<uint16_t>(k.at(2) + key_of_zero)),
                     true);
  }
  auto& world = *scene.getWorldNonConst();
  world.removeObject(planning_scene::PlanningScene::OCTOMAP_NS);
  world.addToObject(planning_scene::PlanningScene::OCTOMAP_NS,
                    std::make_shared<const shapes::OcTree>(tree), Eigen::Isometry3d::Identity());
}

// Applies one call of the scenario: a batch of apply_collision_objects, a call of
// set_allowed_collision, one of apply_attached_collision_object, made with the joints it
// names at their positions, or voxels of the octomap marked occupied, as a depth frame
// marks them.
void apply(planning_scene::PlanningScene& scene, const nlohmann::json& call,
           const std::string& frame, const std::shared_ptr<octomap::OcTree>& tree) {
  const auto op = call.at("op").get<std::string>();
  if (op == "mark") {
    mark(scene, tree, call.at("keys"));
    return;
  }
  if (op == "objects") {
    for (const auto& entry : call.at("objects")) {
      if (!scene.processCollisionObjectMsg(object_message(entry, frame))) {
        throw std::runtime_error("MoveIt refused the object " + entry.dump());
      }
    }
    return;
  }
  if (op == "allow") {
    const auto name = call.at("name").get<std::string>();
    for (const auto& other : call.at("other_names").get<std::vector<std::string>>()) {
      scene.getAllowedCollisionMatrixNonConst().setEntry(name, moveit_name(other),
                                                         call.at("allowed").get<bool>());
    }
    return;
  }
  if (op != "attached") {
    throw std::runtime_error("no call is " + op);
  }
  const auto link = call.at("link_name").get<std::string>();
  pose_joints(scene, link, call.at("positions"));
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.object.id = call.at("object_id").get<std::string>();
  attached.object.header.frame_id = frame;
  const auto operation = call.at("operation").get<std::string>();
  if (operation == "attach") {
    attached.link_name = link;
    attached.touch_links = call.at("touch_links").get<std::vector<std::string>>();
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  } else if (operation == "detach") {
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  } else {
    throw std::runtime_error("no operation of an attached object is " + operation);
  }
  if (!scene.processAttachedCollisionObjectMsg(attached)) {
    throw std::runtime_error("MoveIt refused the attached object " + call.dump());
  }
}

nlohmann::json shapes_json(const std::vector<SolidPrimitive>& primitives,
                           const std::vector<geometry_msgs::msg::Pose>& primitive_poses,
                           const std::vector<shape_msgs::msg::Mesh>& meshes,
                           const std::vector<geometry_msgs::msg::Pose>& mesh_poses) {
  auto shapes = nlohmann::json::array();
  for (std::size_t i = 0; i < primitives.size(); ++i) {
    const auto kind = std::find_if(primitive_types.begin(), primitive_types.end(),
                                   [&](const auto& entry) {
                                     return entry.second == primitives[i].type;
                                   });
    auto dimensions = nlohmann::json::array();
    for (const double dimension : primitives[i].dimensions) {
      dimensions.push_back(dimension);
    }
    auto shape = pose_json(primitive_poses[i]);
    shape["kind"] = kind->first;
    shape["dimensions"] = dimensions;
    shape["mesh_vertices"] = nlohmann::json::array();
    shape["mesh_triangles"] = nlohmann::json::array();
    shapes.push_back(shape);
  }
  for (std::size_t i = 0; i < meshes.size(); ++i) {
    auto vertices = nlohmann::json::array();
    for (const auto& vertex : meshes[i].vertices) {
      vertices.push_back(vertex.x);
      vertices.push_back(vertex.y);
      vertices.push_back(vertex.z);
    }
    auto triangles = nlohmann::json::array();
    for (const auto& triangle : meshes[i].triangles) {
      for (const auto index : triangle.vertex_indices) {
        triangles.push_back(index);
      }
    }
    auto shape = pose_json(mesh_poses[i]);
    shape["kind"] = "mesh";
    shape["dimensions"] = nlohmann::json::array();
    shape["mesh_vertices"] = vertices;
    shape["mesh_triangles"] = triangles;
    shapes.push_back(shape);
  }
  return shapes;
}

// The scene as its planning scene message holds it: the objects and the attached objects,
// each sorted by id, and the allowed pairs that name an object, by motion_planning's
// names, each sorted, in sorted order.
nlohmann::json scene_json(const planning_scene::PlanningScene& scene) {
  moveit_msgs::msg::PlanningScene message;
  scene.getPlanningSceneMsg(message);
  std::map<std::string, nlohmann::json> objects;
  for (const auto& object : message.world.collision_objects) {
    auto entry = pose_json(object.pose);
    entry["id"] = object.id;
    entry["shapes"] =
        shapes_json(object.primitives, object.primitive_poses, object.meshes, object.mesh_poses);
    objects[object.id] = entry;
  }
  std::map<std::string, nlohmann::json> attached;
  for (const auto& body : message.robot_state.attached_collision_objects) {
    auto entry = pose_json(body.object.pose);
    entry["id"] = body.object.id;
    entry["link"] = body.link_name;
    auto touch_links = body.touch_links;
    std::sort(touch_links.begin(), touch_links.end());
    entry["touch_links"] = touch_links;
    entry["shapes"] = shapes_json(body.object.primitives, body.object.primitive_poses,
                                  body.object.meshes, body.object.mesh_poses);
    attached[body.object.id] = entry;
  }
  const auto names_an_object = [&](const std::string& name) {
    return objects.contains(name) || attached.contains(name);
  };
  std::vector<std::pair<std::string, std::string>> pairs;
  const auto& matrix = message.allowed_collision_matrix;
  for (std::size_t i = 0; i < matrix.entry_names.size(); ++i) {
    for (std::size_t j = i + 1; j < matrix.entry_names.size(); ++j) {
      const auto a = contract_name(matrix.entry_names[i]);
      const auto b = contract_name(matrix.entry_names[j]);
      if (matrix.entry_values[i].enabled[j] && (names_an_object(a) || names_an_object(b))) {
        pairs.push_back(std::minmax(a, b));
      }
    }
  }
  std::sort(pairs.begin(), pairs.end());
  auto objects_json = nlohmann::json::array();
  for (const auto& [id, entry] : objects) {
    objects_json.push_back(entry);
  }
  auto attached_json = nlohmann::json::array();
  for (const auto& [id, entry] : attached) {
    attached_json.push_back(entry);
  }
  auto pairs_json = nlohmann::json::array();
  for (const auto& [a, b] : pairs) {
    pairs_json.push_back({a, b});
  }
  return {{"objects", objects_json}, {"attached", attached_json}, {"allowed", pairs_json}};
}

// FCL's smallest signed distance of each pair, by the pair's names in sorted order.
using Distances = std::map<std::pair<std::string, std::string>, double>;

// Merges into `distances` the smallest signed distance of each pair `result` measured.
void collect(const collision_detection::DistanceResult& result, Distances& distances) {
  for (const auto& [names, measured] : result.distances) {
    const std::pair<std::string, std::string> key = std::minmax(names.first, names.second);
    auto& nearest =
        distances.try_emplace(key, std::numeric_limits<double>::infinity()).first->second;
    for (const auto& data : measured) {
      nearest = std::min(nearest, data.distance);
    }
  }
}

// Whether the pair `names` is two objects attached to one link of `state`.
bool on_one_link(const moveit::core::RobotState& state,
                 const std::pair<std::string, std::string>& names) {
  if (!state.hasAttachedBody(names.first) || !state.hasAttachedBody(names.second)) {
    return false;
  }
  return state.getAttachedBody(names.first)->getAttachedLinkName() ==
         state.getAttachedBody(names.second)->getAttachedLinkName();
}

// FCL's smallest signed distance, at the scene's current state, of each pair of a body of
// the robot, a link or an attached object, and an object, and of each attached object
// and each link it may not touch; two objects attached to one link are no pair.
Distances measure(const planning_scene::PlanningScene& scene) {
  collision_detection::DistanceRequest request;
  request.type = collision_detection::DistanceRequestType::ALL;
  request.enable_signed_distance = true;
  request.acm = &scene.getAllowedCollisionMatrix();
  const auto& state = scene.getCurrentState();
  Distances distances;
  collision_detection::DistanceResult robot_world;
  scene.getCollisionEnv()->distanceRobot(request, robot_world, state);
  collect(robot_world, distances);
  collision_detection::DistanceResult self;
  scene.getCollisionEnv()->distanceSelf(request, self, state);
  Distances self_distances;
  collect(self, self_distances);
  for (const auto& [names, distance] : self_distances) {
    if ((state.hasAttachedBody(names.first) || state.hasAttachedBody(names.second)) &&
        !on_one_link(state, names)) {
      distances[names] = distance;
    }
  }
  return distances;
}

// The pairs of two objects attached to one link, which MoveIt's collision check always
// lets touch (collision_common.cpp: bodies attached to the same link should not
// collide): FCL's signed distance of each, and whether MoveIt's collision check of the
// robot against itself finds the two in contact.
nlohmann::json one_link_pairs(const planning_scene::PlanningScene& scene) {
  const auto& state = scene.getCurrentState();
  collision_detection::DistanceRequest request;
  request.type = collision_detection::DistanceRequestType::ALL;
  request.enable_signed_distance = true;
  request.acm = &scene.getAllowedCollisionMatrix();
  collision_detection::DistanceResult self;
  scene.getCollisionEnv()->distanceSelf(request, self, state);
  Distances distances;
  collect(self, distances);
  collision_detection::CollisionRequest check;
  check.contacts = true;
  check.max_contacts = 10000;
  check.max_contacts_per_pair = 1;
  collision_detection::CollisionResult found;
  scene.checkSelfCollision(check, found, state);
  auto pairs = nlohmann::json::array();
  for (const auto& [names, distance] : distances) {
    if (!on_one_link(state, names)) {
      continue;
    }
    const bool contact =
        found.contacts.contains(names) || found.contacts.contains({names.second, names.first});
    pairs.push_back({{"pair", {names.first, names.second}}, {"distance", distance},
                     {"contact", contact}});
  }
  return pairs;
}

// A body of the scene: its shapes, its pieces, each with its pose in the model frame.
struct Body {
  std::vector<shapes::ShapeConstPtr> pieces;
  EigenSTL::vector_Isometry3d poses;
};

// The body `name` of the scene at its current state: a link, an attached object or an
// object.
Body body_of(const planning_scene::PlanningScene& scene, const std::string& name) {
  const auto& state = scene.getCurrentState();
  const auto& model = *scene.getRobotModel();
  if (model.hasLinkModel(name)) {
    const auto* link = model.getLinkModel(name);
    Body body{link->getShapes(), {}};
    for (std::size_t i = 0; i < body.pieces.size(); ++i) {
      body.poses.push_back(state.getCollisionBodyTransform(link, i));
    }
    return body;
  }
  if (state.hasAttachedBody(name)) {
    const auto* attached = state.getAttachedBody(name);
    return {attached->getShapes(), attached->getGlobalCollisionBodyTransforms()};
  }
  const auto object = scene.getWorld()->getObject(name);
  if (!object) {
    throw std::runtime_error("FCL measured " + name + ", which is no body of the scene");
  }
  return {object->shapes_, object->global_shape_poses_};
}

// The directions the inside test casts its rays along: none along an axis or a diagonal,
// where the edges and the vertices of a modeled mesh line up.
const std::array<Eigen::Vector3d, 3> ray_directions = {
    Eigen::Vector3d(0.3772, 0.6427, 0.6667).normalized(),
    Eigen::Vector3d(-0.5836, 0.2381, 0.7764).normalized(),
    Eigen::Vector3d(0.1690, -0.8920, 0.4193).normalized(),
};
// The least distance along a ray at which a crossing counts, and the least determinant
// of a ray against a triangle's plane: below it the ray runs along the plane.
constexpr double ray_eps = 1e-12;

// Whether the ray from `origin` along `direction` crosses the triangle of the corners
// `a`, `b` and `c` (Moller and Trumbore).
bool crosses(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
             const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
  const Eigen::Vector3d ab = b - a;
  const Eigen::Vector3d ac = c - a;
  const Eigen::Vector3d p = direction.cross(ac);
  const double det = ab.dot(p);
  if (std::abs(det) < ray_eps) {
    return false;
  }
  const Eigen::Vector3d to_origin = origin - a;
  const double u = to_origin.dot(p) / det;
  if (u < 0.0 || u > 1.0) {
    return false;
  }
  const Eigen::Vector3d q = to_origin.cross(ab);
  const double v = direction.dot(q) / det;
  if (v < 0.0 || u + v > 1.0) {
    return false;
  }
  return ac.dot(q) / det > ray_eps;
}

// Whether `point`, in the model frame, lies inside the closed mesh `mesh` that `pose`
// places: along most of the ray directions, the ray from it crosses the mesh's triangles
// an odd number of times.
bool inside_mesh(const shapes::Mesh& mesh, const Eigen::Isometry3d& pose,
                 const Eigen::Vector3d& point) {
  const Eigen::Vector3d local = pose.inverse() * point;
  const auto vertex = [&](unsigned int index) {
    return Eigen::Vector3d(mesh.vertices[3 * index], mesh.vertices[3 * index + 1],
                           mesh.vertices[3 * index + 2]);
  };
  std::size_t odd = 0;
  for (const auto& direction : ray_directions) {
    const Eigen::Vector3d along = pose.linear().transpose() * direction;
    std::size_t crossings = 0;
    for (unsigned int t = 0; t < mesh.triangle_count; ++t) {
      const unsigned int* corners = mesh.triangles + 3 * t;
      if (crosses(local, along, vertex(corners[0]), vertex(corners[1]), vertex(corners[2]))) {
        ++crossings;
      }
    }
    odd += crossings % 2;
  }
  return 2 * odd > ray_directions.size();
}

// A point of `body`: the first vertex of its first shape, a mesh, or the center of its
// first shape, a primitive.
Eigen::Vector3d point_of(const Body& body) {
  if (body.pieces.empty()) {
    throw std::runtime_error("FCL measured a body of no shape");
  }
  const auto& shape = *body.pieces.front();
  const auto& pose = body.poses.front();
  if (shape.type != shapes::MESH) {
    return pose.translation();
  }
  const auto& mesh = static_cast<const shapes::Mesh&>(shape);
  return pose * Eigen::Vector3d(mesh.vertices[0], mesh.vertices[1], mesh.vertices[2]);
}

// Whether `point`, in the model frame, lies inside one of the closed meshes of `body`.
bool inside_body(const Body& body, const Eigen::Vector3d& point) {
  for (std::size_t i = 0; i < body.pieces.size(); ++i) {
    const auto& shape = *body.pieces[i];
    if (shape.type == shapes::MESH &&
        inside_mesh(static_cast<const shapes::Mesh&>(shape), body.poses[i], point)) {
      return true;
    }
  }
  return false;
}

// Whether one of the bodies `a` and `b`, whose surfaces FCL finds apart, lies wholly
// inside a closed mesh of the other: a point of it does.
bool one_inside_the_other(const Body& a, const Body& b) {
  return inside_body(b, point_of(a)) || inside_body(a, point_of(b));
}

// MoveIt's cut-out of the scene's objects and attached objects from a cloud, as the
// octomap updater's shape mask holds them (the planning scene monitor excludes both): each
// shape of each a body of geometric_shapes, a convex mesh for a mesh and the primitive
// itself otherwise, at the scene's current state, scaled and padded by each padding of
// `spec`; for each padding, whether each probe of `spec`, in the model frame, lies
// inside one.
nlohmann::json cut_out(const planning_scene::PlanningScene& scene, const nlohmann::json& spec) {
  std::vector<std::pair<bodies::BodyPtr, Eigen::Isometry3d>> shaped;
  for (const auto& id : scene.getWorld()->getObjectIds()) {
    if (id == planning_scene::PlanningScene::OCTOMAP_NS) {
      continue;
    }
    const auto object = scene.getWorld()->getObject(id);
    for (std::size_t i = 0; i < object->shapes_.size(); ++i) {
      shaped.emplace_back(bodies::createBodyFromShape(object->shapes_[i].get()),
                          object->global_shape_poses_[i]);
    }
  }
  std::vector<const moveit::core::AttachedBody*> attached;
  scene.getCurrentState().getAttachedBodies(attached);
  for (const auto* body : attached) {
    const auto& poses = body->getGlobalCollisionBodyTransforms();
    for (std::size_t i = 0; i < body->getShapes().size(); ++i) {
      shaped.emplace_back(bodies::createBodyFromShape(body->getShapes()[i].get()), poses[i]);
    }
  }
  auto inside = nlohmann::json::array();
  for (const auto& padding : spec.at("paddings")) {
    for (auto& [body, pose] : shaped) {
      body->setScaleDirty(padding.at("scale").get<double>());
      body->setPaddingDirty(padding.at("offset").get<double>());
      body->setPose(pose);
    }
    auto row = nlohmann::json::array();
    for (const auto& probe : spec.at("probes")) {
      const auto p = probe.get<std::vector<double>>();
      const Eigen::Vector3d point(p.at(0), p.at(1), p.at(2));
      row.push_back(std::any_of(shaped.begin(), shaped.end(), [&](const auto& entry) {
        return entry.first->containsPoint(point);
      }));
    }
    inside.push_back(row);
  }
  return {{"inside", inside}};
}

// Whether `link` moves with one of `joints`: one of them lies on its way up to the root,
// as the left hand's fingers hang below the left arm's joints.
bool moves_with(const moveit::core::LinkModel* link, const std::set<std::string>& joints) {
  for (const moveit::core::JointModel* joint = link->getParentJointModel(); joint != nullptr;) {
    if (joints.contains(joint->getName())) {
      return true;
    }
    const moveit::core::LinkModel* parent = joint->getParentLinkModel();
    joint = parent == nullptr ? nullptr : parent->getParentJointModel();
  }
  return false;
}

// The scene as MoveIt's path check of the left arm reads it: a child of `scene` whose
// allowed collision matrix lets every two links touch, as the robot's own body is
// motion_planning_core's self model's to check, and lets each link that `joints` do not
// move, and each object attached to one, touch the objects, the octomap and those links:
// the rest of the robot stands still through a path of the arm.
planning_scene::PlanningScenePtr checking_the_arm(const planning_scene::PlanningScene& scene,
                                                 const std::set<std::string>& joints) {
  auto child = scene.diff();
  auto& acm = child->getAllowedCollisionMatrixNonConst();
  const auto& model = *scene.getRobotModel();
  const auto& links = model.getLinkModelNamesWithCollisionGeometry();
  acm.setEntry(links, links, true);
  std::vector<std::string> standing;
  for (const auto* link : model.getLinkModelsWithCollisionGeometry()) {
    if (!moves_with(link, joints)) {
      standing.push_back(link->getName());
    }
  }
  std::vector<const moveit::core::AttachedBody*> attached;
  scene.getCurrentState().getAttachedBodies(attached);
  std::vector<std::string> still = standing;
  for (const auto* body : attached) {
    if (!moves_with(body->getAttachedLink(), joints)) {
      still.push_back(body->getName());
    }
  }
  std::vector<std::string> world = scene.getWorld()->getObjectIds();
  for (const auto& name : still) {
    acm.setEntry(name, world, true);
    acm.setEntry(name, standing, true);
  }
  return child;
}

// The step of the dense probe of a path, in radians of each joint: a joint of OpenArm v2's
// arm, at most 0.8 m from the farthest point of what it turns, moves a point at most
// 0.8 mm a step.
constexpr double dense_step_rad = 1e-3;

// MoveIt's dense probe of `trajectory`, straight in the joints between its waypoints: at
// each step of `dense_step_rad` in the joint that moves most, the collision check and the
// distance to the objects and the octomap; whether a probe is in collision, and the least
// distance of a path in contact nowhere, -1 for one in collision, where FCL's distance to
// an octomap it touches is no measure.
nlohmann::json dense_probe(const planning_scene::PlanningScene& scene,
                           const robot_trajectory::RobotTrajectory& trajectory) {
  bool colliding = false;
  double least = std::numeric_limits<double>::infinity();
  moveit::core::RobotState state(trajectory.getWayPoint(0));
  for (std::size_t w = 0; w + 1 < trajectory.getWayPointCount(); ++w) {
    const auto& from = trajectory.getWayPoint(w);
    const auto& to = trajectory.getWayPoint(w + 1);
    double widest = 0.0;
    for (std::size_t v = 0; v < from.getVariableCount(); ++v) {
      widest = std::max(widest, std::abs(to.getVariablePosition(v) - from.getVariablePosition(v)));
    }
    const auto steps = static_cast<std::size_t>(std::ceil(widest / dense_step_rad));
    for (std::size_t s = 0; s <= steps; ++s) {
      from.interpolate(to, steps == 0 ? 0.0 : static_cast<double>(s) / steps, state);
      state.update();
      colliding = colliding || scene.isStateColliding(state, "", false);
      if (!colliding) {
        least = std::min(least, scene.distanceToCollision(state));
      }
    }
  }
  return {{"colliding", colliding}, {"clearance", colliding ? -1.0 : least}};
}

// MoveIt's check of each path of the scenario, the left arm through its waypoints from the
// scene's current state: the indices of the waypoints it finds invalid, and its dense
// probe of the path; and both again with the robot's links padded by `padding` against
// the objects and the octomap, the robot meeting itself unpadded.
nlohmann::json checked_paths(const planning_scene::PlanningScene& scene,
                             const nlohmann::json& paths, double padding) {
  std::set<std::string> joints;
  for (const auto& [joint, position] : paths.at(0).at(0).items()) {
    joints.insert(joint);
  }
  const auto checking = checking_the_arm(scene, joints);
  const auto padded = checking->diff();
  padded->getCollisionEnvNonConst()->setPadding(padding);
  auto checked = nlohmann::json::array();
  for (const auto& path : paths) {
    robot_trajectory::RobotTrajectory trajectory(scene.getRobotModel());
    for (const auto& waypoint : path) {
      moveit::core::RobotState state(scene.getCurrentState());
      for (const auto& [joint, position] : waypoint.items()) {
        state.setVariablePosition(joint, position.get<double>());
      }
      state.update();
      trajectory.addSuffixWayPoint(state, 0.0);
    }
    std::vector<std::size_t> invalid;
    checking->isPathValid(trajectory, "", false, &invalid);
    std::vector<std::size_t> invalid_padded;
    padded->isPathValid(trajectory, "", false, &invalid_padded);
    checked.push_back(
        {{"invalid", invalid},
         {"dense", dense_probe(*checking, trajectory)},
         {"padded",
          {{"invalid", invalid_padded},
           {"colliding", dense_probe(*padded, trajectory).at("colliding")}}}});
  }
  return checked;
}

}  // namespace

nlohmann::json planning_scene(const nlohmann::json& scenario, Inputs& inputs) {
  const auto model = robot_model_with_meshes(inputs, scenario.at("urdf").get<std::string>(),
                                             scenario.at("meshes").get<std::string>());
  const auto tip_link = scenario.at("tip_link").get<std::string>();
  // Owned by a shared pointer, as a child scene of it needs (PlanningScene::diff).
  const auto owned = std::make_shared<planning_scene::PlanningScene>(model);
  auto& scene = *owned;
  scene.getCurrentStateNonConst().setToDefaultValues();
  // The voxel map, from the start: an octomap of the scenario's voxels, each occupied,
  // which the calls leave as it is.
  const auto& octomap = scenario.at("octomap");
  auto tree = std::make_shared<octomap::OcTree>(octomap.at("voxel_size").get<double>());
  for (const auto& key : octomap.at("keys")) {
    const auto k = key.get<std::vector<int>>();
    tree->updateNode(octomap::OcTreeKey(static_cast<uint16_t>(k.at(0) + key_of_zero),
                                        static_cast<uint16_t>(k.at(1) + key_of_zero),
                                        static_cast<uint16_t>(k.at(2) + key_of_zero)),
                     true);
  }
  scene.getWorldNonConst()->addToObject(planning_scene::PlanningScene::OCTOMAP_NS,
                                        std::make_shared<const shapes::OcTree>(tree),
                                        Eigen::Isometry3d::Identity());
  const auto frame = model->getModelFrame();
  for (const auto& call : scenario.at("operations")) {
    apply(scene, call, frame, tree);
  }
  // The cut-out at the state the calls leave, before the configurations pose the arm.
  const auto cut = cut_out(scene, scenario.at("cut_out"));
  std::vector<std::pair<std::string, std::string>> pairs;
  auto configurations = nlohmann::json::array();
  for (const auto& configuration : scenario.at("configurations")) {
    pose_joints(scene, tip_link, configuration);
    std::vector<std::pair<std::string, std::string>> measured;
    auto nearest = nlohmann::json::array();
    auto inside = nlohmann::json::array();
    for (const auto& [names, distance] : measure(scene)) {
      measured.push_back(names);
      nearest.push_back(distance);
      inside.push_back(distance > 0.0 && one_inside_the_other(body_of(scene, names.first),
                                                              body_of(scene, names.second)));
    }
    if (configurations.empty()) {
      pairs = measured;
    } else if (measured != pairs) {
      throw std::runtime_error("FCL measured another set of pairs at another configuration");
    }
    auto echoed_positions = nlohmann::json::object();
    for (const auto& [joint, position] : configuration.items()) {
      echoed_positions[joint] = position.get<double>();
    }
    configurations.push_back(
        {{"positions", echoed_positions}, {"distances", nearest}, {"inside", inside}});
  }
  auto names = nlohmann::json::array();
  for (const auto& [a, b] : pairs) {
    const std::string first = contract_name(a);
    const std::string second = contract_name(b);
    names.push_back({std::min(first, second), std::max(first, second)});
  }
  return {{"kind", "planning_scene"},
          {"scene", scene_json(scene)},
          {"pairs", names},
          {"configurations", configurations},
          {"paths", checked_paths(scene, scenario.at("paths"), scenario.at("path_padding"))},
          {"cut_out", cut},
          {"one_link_pairs", one_link_pairs(scene)}};
}

}  // namespace ros_reference
