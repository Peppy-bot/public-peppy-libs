// The runs of a room dataset scenario, each in a process of its own: every node's
// callbacks are called on this program's thread, no executor spins but each tf listener's
// own, which spins the listener's subscriptions on a thread of its own, no node activates,
// the odometry enters each node's tf buffer at each scan's stamp, and each node's clock
// reads each scan's stamp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_amcl/amcl_node.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <slam_toolbox/slam_toolbox_localization.hpp>
#include <slam_toolbox/slam_toolbox_sync.hpp>
#include <slam_toolbox/visualization_utils.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "amcl_map_cspace.hpp"
#include "amcl_node.hpp"
#include "nav2_node.hpp"
#include "private_member.hpp"
#include "room_dataset.hpp"
#include "room_dataset_files.hpp"
#include "scenario.hpp"

namespace ros_reference {

namespace {

using namespace room_dataset_files;

// The members of slam_toolbox's Ceres solver plugin up to its solver options, in their
// order in solvers/ceres_solver.hpp of slam_toolbox 2.10.0, which the package does not
// install. Configure sets 50 solver threads, whose partial sums of the cost Ceres adds in
// the order in which the threads finish; the runs set one, after checking that the
// options hold every value that Configure sets, so the layout is the plugin's.
struct CeresSolverLayout : karto::ScanSolver {
  karto::ScanSolver::IdPoseVector corrections;
  ceres::Solver::Options options;
};

void use_one_solver_thread(karto::ScanSolver* solver) {
  if (std::string(typeid(*solver).name()).find("CeresSolver") == std::string::npos) {
    throw std::runtime_error("slam_toolbox's solver is not its Ceres solver plugin");
  }
  auto& options = reinterpret_cast<CeresSolverLayout*>(solver)->options;
  if (options.num_threads != 50 || options.function_tolerance != 1e-3 ||
      options.gradient_tolerance != 1e-6 || options.parameter_tolerance != 1e-3 ||
      options.initial_trust_region_radius != 1e4 || options.max_trust_region_radius != 1e8 ||
      options.min_lm_diagonal != 1e-6 || options.max_lm_diagonal != 1e32 ||
      options.max_num_consecutive_invalid_steps != 3) {
    throw std::runtime_error("slam_toolbox's Ceres solver holds options other than those of "
                             "slam_toolbox 2.10.0");
  }
  options.num_threads = 1;
}

PlanarPose planar(const karto::Pose2& pose) {
  return {pose.GetX(), pose.GetY(), pose.GetHeading()};
}

std::vector<rclcpp::Parameter> slam_toolbox_frames(const char* mode) {
  return {{"mode", mode},
          {"odom_frame", odometry_frame},
          {"map_frame", map_frame},
          {"base_frame", base_frame},
          {"use_map_saver", false},
          {"use_lifecycle_manager", false}};
}

// A mode of slam_toolbox, `Toolbox`, set up for the runs.
template <typename Toolbox>
class SlamToolboxReference : public Toolbox {
 public:
  using Toolbox::Toolbox;

  // on_configure makes a tf listener that spins an executor on a thread of its own
  // (slam_toolbox 2.10.0 slam_toolbox_common.cpp:134), which ~SlamToolbox destroys
  // (:305), canceling that executor and joining the thread (tf2_ros 0.45.10
  // transform_listener.cpp:67-72): the reference waits until the executor spins.
  ~SlamToolboxReference() override {
    if (this->tfL_) {
      wait_until_spinning(*this->tfL_);
    }
  }

  // Sets up what on_activate sets up and the runs read: the scan's interfaces, and the
  // identity as the transform from a continued session's odometry, which every scan's
  // odometry passes through; and the solver on one thread.
  void prepare() {
    use_one_solver_thread(this->solver_.get());
    this->setROSInterfaces();
    this->reprocessing_transform_.setIdentity();
  }
};

// slam_toolbox's synchronous mode: laserCallback queues a scan that it should process, and
// the run loop adds each queued scan; this takes the loop's place.
class MappingReference : public SlamToolboxReference<slam_toolbox::SynchronousSlamToolbox> {
 public:
  using SlamToolboxReference::SlamToolboxReference;

  // The scan that `scan` added to the map, or none.
  karto::LocalizedRangeScan* add(std::int64_t stamp_ns, const PlanarPose& odometry,
                                 const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) {
    set_ros_time(get_clock(), stamp_ns);
    tf_->setTransform(odometry_transform(stamp_ns, odometry), tf_authority, false);
    laserCallback(scan);
    karto::LocalizedRangeScan* added = nullptr;
    while (!q_.empty()) {
      toolbox_types::PosedScan posed = q_.front();
      q_.pop();
      boost::mutex::scoped_lock lock(smapper_mutex_);
      karto::LaserRangeFinder* laser = getLaser(posed.scan);
      if (laser == nullptr) {
        throw std::runtime_error("slam_toolbox made no laser of a scan");
      }
      added = addScanImpl(laser, posed.scan, posed.pose);
    }
    return added;
  }

  // Saves the pose graph as the serialize_map service does, and the grid as the map topic
  // carries it, through nav2_map_server's map saver.
  void save_map(const std::filesystem::path& base_name, const nlohmann::json& map_saver) {
    auto request = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
    auto response = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Response>();
    request->filename = base_name.string();
    serializePoseGraphCallback(nullptr, request, response);
    if (response->result != response->RESULT_SUCCESS) {
      throw std::runtime_error("slam_toolbox does not serialize its pose graph");
    }
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = map_frame;
    grid.info.resolution = resolution_;
    grid.info.origin.orientation.w = 1.0;
    {
      boost::mutex::scoped_lock lock(smapper_mutex_);
      std::unique_ptr<karto::OccupancyGrid> occupancy(smapper_->getOccupancyGrid(resolution_));
      if (!occupancy) {
        throw std::runtime_error("slam_toolbox gives no grid of its map");
      }
      vis_utils::toNavMap(occupancy.get(), grid);
    }
    nav2_map_server::SaveParameters save;
    save.map_file_name = base_name.string();
    save.image_format = "pgm";
    save.free_thresh = map_saver.at("free_thresh").get<double>();
    save.occupied_thresh = map_saver.at("occupied_thresh").get<double>();
    save.mode = nav2_map_server::MapMode::Trinary;
    if (!nav2_map_server::saveMapToFile(grid, save)) {
      throw std::runtime_error("nav2_map_server does not save the grid");
    }
  }
};

// The covariance of a match that slam_toolbox publishes with a scan's pose on its pose
// topic: of x and y (m^2), and of the heading (rad^2), with position_covariance_scale and
// yaw_covariance_scale at their default of 1.
struct MatchCovariance {
  double xx;
  double xy;
  double yy;
  double yaw;
};

// A scan that slam_toolbox's localization added: its pose in the map, and the covariance of
// its match.
struct Localized {
  PlanarPose pose;
  MatchCovariance covariance;
};

// slam_toolbox's localization mode: laserCallback adds a scan that it should process
// against the loaded map, and this records the pose of each scan that it adds, with the
// covariance of its match.
class LocalizationReference
    : public SlamToolboxReference<slam_toolbox::LocalizationSlamToolbox> {
 public:
  using SlamToolboxReference::SlamToolboxReference;

  // Loads the pose graph at `base_name` to localize from `start`, as loadPoseGraphByParams
  // does with the parameters map_file_name and map_start_pose.
  void load_map(const std::filesystem::path& base_name, const PlanarPose& start) {
    auto request = std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();
    auto response = std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Response>();
    request->filename = base_name.string();
    request->match_type = slam_toolbox::srv::DeserializePoseGraph::Request::LOCALIZE_AT_POSE;
    request->initial_pose.position.x = start.x;
    request->initial_pose.position.y = start.y;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, start.yaw);
    request->initial_pose.orientation = tf2::toMsg(rotation);
    deserializePoseGraphCallback(nullptr, request, response);
    if (response->result != response->RESULT_SUCCESS) {
      throw std::runtime_error("slam_toolbox does not load the map " + base_name.string());
    }
  }

  // The pose of `scan` in the map and the covariance of its match, when slam_toolbox added
  // it.
  std::optional<Localized> localize(std::int64_t stamp_ns, const PlanarPose& odometry,
                                    const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) {
    set_ros_time(get_clock(), stamp_ns);
    tf_->setTransform(odometry_transform(stamp_ns, odometry), tf_authority, false);
    added_.reset();
    laserCallback(scan);
    return added_;
  }

 protected:
  karto::LocalizedRangeScan* addScan(karto::LaserRangeFinder* laser,
                                     const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
                                     karto::Pose2& odometry) override {
    karto::LocalizedRangeScan* added = LocalizationSlamToolbox::addScan(laser, scan, odometry);
    if (added != nullptr) {
      added_ = Localized{planar(added->GetCorrectedPose()), match_covariance(added)};
    }
    return added;
  }

 private:
  // The covariance of the match of `scan`: karto's AddEdges links the scan to the scan
  // before it first (Mapper.cpp, MapperGraph::AddEdges and LinkScans), and labels that
  // edge with the covariance that ProcessLocalization and ProcessAgainstNodesNearBy hand
  // slam_toolbox to publish. The identity, as slam_toolbox publishes it, for a scan that
  // karto linked to no scan before it.
  MatchCovariance match_covariance(karto::LocalizedRangeScan* scan) {
    boost::mutex::scoped_lock lock(smapper_mutex_);
    const auto& vertices = smapper_->getMapper()->GetGraph()->GetVertices();
    const auto& by_id = vertices.at(scan->GetSensorName());
    const auto vertex = by_id.find(scan->GetStateId());
    if (vertex == by_id.end()) {
      throw std::runtime_error("karto's graph holds no vertex of an added scan");
    }
    for (auto* edge : vertex->second->GetEdges()) {
      if (edge->GetTarget() != vertex->second) {
        continue;
      }
      auto* link = dynamic_cast<karto::LinkInfo*>(edge->GetLabel());
      if (link == nullptr) {
        throw std::runtime_error("an edge of karto's graph carries no link");
      }
      const karto::Matrix3& c = link->GetCovariance();
      return {c(0, 0), c(0, 1), c(1, 1), c(2, 2)};
    }
    return {1.0, 0.0, 1.0, 1.0};
  }

  std::optional<Localized> added_;
};

// The pose of each processed scan, by the index of the scan among the kept scans.
nlohmann::json recorded(const std::vector<RecordedPose>& poses) {
  auto lines = nlohmann::json::array();
  for (const auto& pose : poses) {
    lines.push_back({{"scan", pose.scan}, {"pose", pose_json(pose.pose)}});
  }
  return lines;
}

void map(const nlohmann::json& scenario, int seed, const std::filesystem::path& results) {
  const auto& parameters = scenario.at("slam_toolbox");
  auto node = std::make_shared<MappingReference>(
      options_of(parameters, slam_toolbox_frames("mapping")));
  node->configure();
  node->prepare();
  const auto prepared = read_prepared(results);
  const auto odometry = read_odometry(results, seed);
  std::vector<std::pair<std::size_t, karto::LocalizedRangeScan*>> added;
  for (std::size_t i = 0; i < prepared.scans.size(); ++i) {
    const auto& scan = prepared.scans[i];
    if (auto* processed = node->add(scan.stamp_ns, odometry[i],
                                     laser_scan_of(prepared.parameters, scan))) {
      added.emplace_back(i, processed);
    }
  }
  // Each scan's pose after the last optimization of the pose graph.
  std::vector<RecordedPose> poses;
  for (const auto& [index, scan] : added) {
    poses.push_back({index, planar(scan->GetCorrectedPose())});
  }
  // slam_toolbox declares the laser's parameters when it makes the laser of the first scan.
  check_declared(*node, parameters, "slam_toolbox");
  node->save_map(map_of(results, seed), scenario.at("task").at("mapping").at("map_saver"));
  write_json(run_file(results, "mapping", seed), {{"seed", seed}, {"poses", recorded(poses)}});
}

// The pose of the base at the first kept scan of `prepared` in the map that `map_seed` of a
// mapping scenario made: the ground truth pose, moved by the rigid motion that best aligns,
// in the least-squares sense, the ground truth of the map's scans with their poses in it
// (Eigen's umeyama, without scaling).
PlanarPose start_in_map(const Prepared& prepared, const std::filesystem::path& map_results,
                        int map_seed) {
  const auto map_prepared = read_prepared(map_results);
  const auto map_poses = read_recorded_poses(run_file(map_results, "mapping", map_seed));
  Eigen::Matrix2Xd truth(2, map_poses.size()), in_map(2, map_poses.size());
  for (std::size_t k = 0; k < map_poses.size(); ++k) {
    const auto& t = map_prepared.scans.at(map_poses[k].scan).truth;
    truth.col(static_cast<Eigen::Index>(k)) << t.x, t.y;
    in_map.col(static_cast<Eigen::Index>(k)) << map_poses[k].pose.x, map_poses[k].pose.y;
  }
  const Eigen::Matrix3d map_from_truth = Eigen::umeyama(truth, in_map, false);
  const auto& first = prepared.scans.front().truth;
  const Eigen::Vector3d position = map_from_truth * Eigen::Vector3d(first.x, first.y, 1.0);
  const double turn = std::atan2(map_from_truth(1, 0), map_from_truth(0, 0));
  return {position.x(), position.y(), wrapped(first.yaw + turn)};
}

// The results of `map_scenario`, whose seed `seed` made the map that a run of `seed`
// localizes in.
std::filesystem::path map_results_of(const nlohmann::json& map_scenario, int seed) {
  const auto& seeds = map_scenario.at("seeds");
  if (std::find(seeds.begin(), seeds.end(), seed) == seeds.end()) {
    throw std::runtime_error(map_scenario.at("name").get<std::string>() + " maps no seed " +
                             std::to_string(seed));
  }
  return results_of(map_scenario.at("name").get<std::string>());
}

void localize(const nlohmann::json& scenario, int seed, const std::filesystem::path& results,
              const nlohmann::json& map_scenario) {
  const auto map_results = map_results_of(map_scenario, seed);
  const auto& parameters = scenario.at("slam_toolbox");
  auto node = std::make_shared<LocalizationReference>(
      options_of(parameters, slam_toolbox_frames("localization")));
  node->configure();
  node->prepare();
  const auto prepared = read_prepared(results);
  const auto odometry = read_odometry(results, seed);
  const auto start = start_in_map(prepared, map_results, seed);
  node->load_map(map_of(map_results, seed), start);
  std::vector<RecordedPose> poses;
  for (std::size_t i = 0; i < prepared.scans.size(); ++i) {
    const auto& scan = prepared.scans[i];
    if (const auto localized = node->localize(scan.stamp_ns, odometry[i],
                                              laser_scan_of(prepared.parameters, scan))) {
      poses.push_back({i, localized->pose});
    }
  }
  check_declared(*node, parameters, "slam_toolbox");
  write_json(run_file(results, "localization", seed),
             {{"seed", seed}, {"start", pose_json(start)}, {"poses", recorded(poses)}});
}

// The kept scans of `prepared` and the odometry `odometry` spliced from two places, as the
// parity test of mapping_core splices them: the first half of the scans, then the scans
// from the one, among the first half of the rest, whose ground truth lies farthest from the
// last of the first half (the first of equals), a scan period after it, their odometry moved
// rigidly to go on from it, as a base carried to another place reads its odometry.
struct Spliced {
  Prepared prepared;
  std::vector<PlanarPose> odometry;
  // The index of the first scan after the splice, and of the scan of `prepared` it is.
  std::size_t splice;
  std::size_t far;
};

PlanarPose composed(const PlanarPose& a, const PlanarPose& b) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw};
}

// `b` in the frame of `a`.
PlanarPose relative(const PlanarPose& a, const PlanarPose& b) {
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  const double dx = b.x - a.x, dy = b.y - a.y;
  return {c * dx + s * dy, -s * dx + c * dy, b.yaw - a.yaw};
}

Spliced spliced(const Prepared& prepared, const std::vector<PlanarPose>& odometry) {
  const std::size_t half = prepared.scans.size() / 2;
  const auto& last = prepared.scans[half - 1];
  const std::size_t rest = prepared.scans.size() - half;
  std::size_t far = half;
  double farthest = 0.0;
  for (std::size_t index = half; index < half + rest / 2; ++index) {
    const auto& truth = prepared.scans[index].truth;
    const double distance = std::hypot(truth.x - last.truth.x, truth.y - last.truth.y);
    if (distance > farthest) {
      far = index;
      farthest = distance;
    }
  }
  const std::int64_t period_ns = prepared.scans[1].stamp_ns - prepared.scans[0].stamp_ns;
  Spliced result{{prepared.parameters, {}}, {}, half, far};
  for (std::size_t index = 0; index < half; ++index) {
    result.prepared.scans.push_back(prepared.scans[index]);
    result.odometry.push_back(odometry[index]);
  }
  const auto& carried = odometry[half - 1];
  const auto& from = odometry[far];
  for (std::size_t index = far; index < prepared.scans.size(); ++index) {
    auto scan = prepared.scans[index];
    scan.stamp_ns = last.stamp_ns + period_ns + (scan.stamp_ns - prepared.scans[far].stamp_ns);
    result.prepared.scans.push_back(scan);
    result.odometry.push_back(composed(carried, relative(from, odometry[index])));
  }
  return result;
}

// slam_toolbox's localization on the scans and the odometry of `seed` spliced from two
// places of the map: the pose and the covariance of the match of each scan it adds.
void localize_spliced(const nlohmann::json& scenario, int seed,
                      const std::filesystem::path& results, const nlohmann::json& map_scenario) {
  const auto map_results = map_results_of(map_scenario, seed);
  const auto& parameters = scenario.at("slam_toolbox");
  auto node = std::make_shared<LocalizationReference>(
      options_of(parameters, slam_toolbox_frames("localization")));
  node->configure();
  node->prepare();
  const auto run = spliced(read_prepared(results), read_odometry(results, seed));
  const auto start = start_in_map(run.prepared, map_results, seed);
  node->load_map(map_of(map_results, seed), start);
  auto answers = nlohmann::json::array();
  for (std::size_t i = 0; i < run.prepared.scans.size(); ++i) {
    const auto& scan = run.prepared.scans[i];
    if (const auto localized = node->localize(scan.stamp_ns, run.odometry[i],
                                              laser_scan_of(run.prepared.parameters, scan))) {
      const auto& c = localized->covariance;
      answers.push_back(
          {{"scan", i},
           {"pose", pose_json(localized->pose)},
           {"covariance", {{"xx", c.xx}, {"xy", c.xy}, {"yy", c.yy}, {"yaw", c.yaw}}}});
    }
  }
  check_declared(*node, parameters, "slam_toolbox");
  write_json(run_file(results, "spliced_localization", seed),
             {{"seed", seed},
              {"start", pose_json(start)},
              {"splice", run.splice},
              {"far", run.far},
              {"poses", answers}});
}

void localize_globally(const nlohmann::json& scenario, int seed,
                       const std::filesystem::path& results,
                       const nlohmann::json& map_scenario) {
  const auto map_results = map_results_of(map_scenario, seed);
  const auto& parameters = scenario.at("task").at("localization").at("amcl");
  const auto node = amcl_with(parameters);
  nav_msgs::msg::OccupancyGrid map;
  const auto yaml = map_of(map_results, seed).string() + ".yaml";
  if (nav2_map_server::loadMapFromYaml(yaml, map) != nav2_map_server::LOAD_MAP_SUCCESS) {
    throw std::runtime_error("nav2_map_server does not load " + yaml);
  }
  map.header.frame_id = map_frame;
  node->receive_map(map);
  node->start_global_localization(seed);
  const auto prepared = read_prepared(results);
  const auto odometry = read_odometry(results, seed);
  auto lines = nlohmann::json::array();
  for (std::size_t i = 0; i < prepared.scans.size(); ++i) {
    const auto& scan = prepared.scans[i];
    if (const auto estimate = node->receive(scan.stamp_ns, odometry[i],
                                            laser_scan_of(prepared.parameters, scan))) {
      const auto& covariance = estimate->covariance;
      lines.push_back({{"scan", i},
                       {"pose", pose_json(estimate->pose)},
                       {"covariance",
                        {{"xx", covariance.xx},
                         {"xy", covariance.xy},
                         {"yy", covariance.yy},
                         {"yaw", covariance.yaw}}},
                       {"converged", estimate->converged}});
    }
  }
  if (amcl_map_cspace_updates == 0) {
    throw std::runtime_error("AMCL built its likelihood field without the program's "
                             "map_update_cspace");
  }
  write_json(run_file(results, "global_localization", seed), {{"seed", seed}, {"poses", lines}});
}

}  // namespace

int room_dataset_run(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "usage: ros_reference " << room_dataset_run_command
              << " <mapping|localization|spliced_localization|global_localization> "
                 "<scenario.json> <seed> "
                 "<repository root>\n";
    return 2;
  }
  const std::string run = argv[2];
  try {
    Inputs inputs(argv[5]);
    const auto scenario = nlohmann::json::parse(inputs.read(argv[3]));
    const int seed = std::stoi(argv[4]);
    const auto results = results_of(scenario.at("name").get<std::string>());
    start_ros();
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);
    if (run == "mapping") {
      map(scenario, seed, results);
    } else {
      const auto map_scenario = nlohmann::json::parse(inputs.read(
          "ros_reference/scenarios/" +
          scenario.at("task").at("localization").at("map_scenario").get<std::string>() + ".json"));
      if (run == "localization") {
        localize(scenario, seed, results, map_scenario);
      } else if (run == "spliced_localization") {
        localize_spliced(scenario, seed, results, map_scenario);
      } else if (run == "global_localization") {
        localize_globally(scenario, seed, results, map_scenario);
      } else {
        throw std::runtime_error("no run is named " + run);
      }
    }
    rclcpp::shutdown();
  } catch (const std::exception& error) {
    std::cerr << argv[3] << " " << run << " " << argv[4] << ": " << error.what() << '\n';
    return 1;
  }
  return 0;
}

}  // namespace ros_reference
