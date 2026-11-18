#include "room_dataset.hpp"

#include <spawn.h>
#include <sys/wait.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <array>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <png.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "laser_projection.hpp"
#include "point_cloud.hpp"
#include "room_dataset_files.hpp"
#include "scenario.hpp"

extern char** environ;

namespace ros_reference {

namespace {

using namespace room_dataset_files;

// A stamp of the TUM files, seconds with a decimal fraction, in nanoseconds, read from
// its digits.
std::int64_t stamp_of(const std::string& text) {
  const auto point = text.find('.');
  const auto seconds = std::stoll(text.substr(0, point));
  auto fraction = point == std::string::npos ? std::string() : text.substr(point + 1);
  if (fraction.size() > 9 ||
      !std::all_of(fraction.begin(), fraction.end(), [](char c) { return std::isdigit(c); })) {
    throw std::runtime_error(text + " is not a stamp of seconds and at most 9 decimals");
  }
  fraction.resize(9, '0');
  return seconds * nanoseconds_per_second + std::stoll(fraction);
}

// A pose of the ground truth: the camera's optical frame in the motion capture frame.
struct CameraPose {
  std::int64_t stamp_ns;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

// The lines of groundtruth.txt: a stamp, a position and a unit quaternion [x, y, z, w].
std::vector<CameraPose> read_ground_truth(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::vector<CameraPose> poses;
  for (std::string line; std::getline(file, line);) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream fields(line);
    std::string stamp;
    double tx, ty, tz, qx, qy, qz, qw;
    if (!(fields >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
      throw std::runtime_error(path.string() + " holds a line that is not a pose: " + line);
    }
    poses.push_back({stamp_of(stamp), {tx, ty, tz}, Eigen::Quaterniond(qw, qx, qy, qz)});
  }
  if (poses.size() < 2 || !std::is_sorted(poses.begin(), poses.end(), [](auto& a, auto& b) {
        return a.stamp_ns < b.stamp_ns;
      })) {
    throw std::runtime_error(path.string() + " holds fewer than two poses or is out of order");
  }
  return poses;
}

// The depth frames of depth/, each named by its stamp, in the order of their stamps.
struct DepthFrame {
  std::int64_t stamp_ns;
  std::filesystem::path path;
};

std::vector<DepthFrame> list_depth_frames(const std::filesystem::path& directory) {
  std::vector<DepthFrame> frames;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    if (entry.path().extension() == ".png") {
      frames.push_back({stamp_of(entry.path().stem().string()), entry.path()});
    }
  }
  std::sort(frames.begin(), frames.end(),
            [](const auto& a, const auto& b) { return a.stamp_ns < b.stamp_ns; });
  return frames;
}

// A 16-bit grayscale PNG: its width, height and samples, row by row.
struct DepthImage {
  int width;
  int height;
  std::vector<std::uint16_t> samples;
};

// libpng reports an error by a longjmp to the setjmp of the function that called it. Each
// of these two functions holds that setjmp and no object with a destructor, so the
// longjmp leaves no destructor uncalled; each answers whether libpng read without error.
bool read_png_header(png_structp png, png_infop info, FILE* file) {
  if (setjmp(png_jmpbuf(png))) {
    return false;
  }
  png_init_io(png, file);
  png_read_info(png, info);
  return true;
}

bool read_png_rows(png_structp png, png_bytepp rows) {
  if (setjmp(png_jmpbuf(png))) {
    return false;
  }
  // PNG stores a 16-bit sample most significant byte first.
  png_set_swap(png);
  png_read_image(png, rows);
  return true;
}

// libpng's read and info structures, which it destroys together.
class PngReader {
 public:
  PngReader()
      : png_(png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr)),
        info_(png_ == nullptr ? nullptr : png_create_info_struct(png_)) {}
  ~PngReader() { png_destroy_read_struct(&png_, &info_, nullptr); }
  PngReader(const PngReader&) = delete;
  PngReader& operator=(const PngReader&) = delete;

  png_structp png() const { return png_; }
  png_infop info() const { return info_; }

 private:
  png_structp png_;
  png_infop info_;
};

DepthImage read_depth_image(const std::filesystem::path& path) {
  // A depth frame that libpng cannot read is damaged or missing in the dataset cache,
  // which run.sh --datasets checks and extracts again.
  const auto refused = [&](const std::string& why) {
    return std::runtime_error(path.string() + " " + why +
                              "; ros_reference/run.sh --datasets extracts the dataset again");
  };
  const std::unique_ptr<FILE, int (*)(FILE*)> file(std::fopen(path.c_str(), "rb"), std::fclose);
  if (!file) {
    throw refused("cannot be read");
  }
  const PngReader reader;
  if (reader.info() == nullptr) {
    throw std::runtime_error("libpng allocates no structure to read " + path.string());
  }
  if (!read_png_header(reader.png(), reader.info(), file.get())) {
    throw refused("is not a PNG that libpng reads");
  }
  if (png_get_bit_depth(reader.png(), reader.info()) != 16 ||
      png_get_color_type(reader.png(), reader.info()) != PNG_COLOR_TYPE_GRAY) {
    throw refused("is not a 16-bit grayscale PNG");
  }
  DepthImage image{static_cast<int>(png_get_image_width(reader.png(), reader.info())),
                   static_cast<int>(png_get_image_height(reader.png(), reader.info())),
                   {}};
  image.samples.resize(static_cast<std::size_t>(image.width) * image.height);
  std::vector<png_bytep> rows(image.height);
  for (int v = 0; v < image.height; ++v) {
    rows[v] = reinterpret_cast<png_bytep>(image.samples.data() +
                                          static_cast<std::size_t>(v) * image.width);
  }
  if (!read_png_rows(reader.png(), rows.data())) {
    throw refused("holds rows that libpng does not read");
  }
  return image;
}

// The rotation of the camera's body frame (x forward, y left, z up) from its optical frame
// (x right, y down, z forward).
Eigen::Matrix3d body_from_optical() {
  Eigen::Matrix3d rotation;
  rotation << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  return rotation;
}

// The camera's mount on the base, fitted once from the ground truth: the floor normal is
// the normal of the plane that best fits the camera's positions, which ride at one height
// above the floor; the height is the median distance of the camera from the plane through
// the motion capture origin with that normal; the pitch (positive tilts the optical axis
// down) and the roll are the medians of the camera's attitude against the normal.
struct Mount {
  Eigen::Vector3d floor_normal;
  double height;
  double pitch;
  double roll;

  // The camera's optical frame in the base frame.
  Eigen::Isometry3d base_from_optical() const {
    const Eigen::Matrix3d base_from_body =
        (Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = base_from_body * body_from_optical();
    pose.translation() = Eigen::Vector3d(0.0, 0.0, height);
    return pose;
  }
};

double median(std::vector<double> values) {
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

Mount fit_mount(const std::vector<CameraPose>& truth) {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& pose : truth) {
    mean += pose.position;
  }
  mean /= static_cast<double>(truth.size());
  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (const auto& pose : truth) {
    const Eigen::Vector3d d = pose.position - mean;
    scatter += d * d.transpose();
  }
  // The eigenvector of the smallest eigenvalue, turned to point up.
  Eigen::Vector3d normal = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(scatter).eigenvectors().col(0);
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  const Eigen::Quaterniond floor_from_world =
      Eigen::Quaterniond::FromTwoVectors(normal, Eigen::Vector3d::UnitZ());
  std::vector<double> heights, pitches, rolls;
  for (const auto& pose : truth) {
    heights.push_back(normal.dot(pose.position));
    const Eigen::Matrix3d floor_from_body =
        floor_from_world.toRotationMatrix() * pose.orientation.toRotationMatrix() *
        body_from_optical().transpose();
    const double heading = std::atan2(floor_from_body(1, 0), floor_from_body(0, 0));
    const Eigen::Matrix3d base_from_body =
        Eigen::AngleAxisd(-heading, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        floor_from_body;
    pitches.push_back(std::atan2(-base_from_body(2, 0),
                                 std::hypot(base_from_body(2, 1), base_from_body(2, 2))));
    rolls.push_back(std::atan2(base_from_body(2, 1), base_from_body(2, 2)));
  }
  return {normal, median(heights), median(pitches), median(rolls)};
}

// The base's pose in the motion capture frame's floor coordinates: the camera's position
// and the heading of its optical axis.
PlanarPose base_pose_of(const CameraPose& pose) {
  const Eigen::Vector3d forward = pose.orientation * Eigen::Vector3d::UnitZ();
  return {pose.position.x(), pose.position.y(), std::atan2(forward.y(), forward.x())};
}

// The base's pose at `stamp_ns`, interpolated between the ground truth's poses on either
// side, or none when `stamp_ns` lies outside the ground truth or the poses on either side
// lie more than `max_gap_ns` apart.
std::optional<PlanarPose> truth_at(const std::vector<CameraPose>& truth, std::int64_t stamp_ns,
                                   std::int64_t max_gap_ns) {
  const auto after = std::lower_bound(
      truth.begin(), truth.end(), stamp_ns,
      [](const CameraPose& pose, std::int64_t stamp) { return pose.stamp_ns < stamp; });
  if (after == truth.end()) {
    return std::nullopt;
  }
  if (after->stamp_ns == stamp_ns) {
    return base_pose_of(*after);
  }
  if (after == truth.begin()) {
    return std::nullopt;
  }
  const auto before = std::prev(after);
  if (after->stamp_ns - before->stamp_ns > max_gap_ns) {
    return std::nullopt;
  }
  const double s = static_cast<double>(stamp_ns - before->stamp_ns) /
                   static_cast<double>(after->stamp_ns - before->stamp_ns);
  const auto a = base_pose_of(*before);
  const auto b = base_pose_of(*after);
  return PlanarPose{a.x + s * (b.x - a.x), a.y + s * (b.y - a.y),
                    wrapped(a.yaw + s * wrapped(b.yaw - a.yaw))};
}

// The depth frames that the scans come from: the first frame at or after each multiple of
// the scan period from the first frame's stamp.
std::vector<DepthFrame> one_frame_a_period(const std::vector<DepthFrame>& frames,
                                           std::int64_t period_ns) {
  std::vector<DepthFrame> kept;
  std::int64_t next_ns = frames.front().stamp_ns;
  for (const auto& frame : frames) {
    if (frame.stamp_ns < next_ns) {
      continue;
    }
    kept.push_back(frame);
    while (next_ns <= frame.stamp_ns) {
      next_ns += period_ns;
    }
  }
  return kept;
}

// The pinhole intrinsics of the depth camera and the depth sample of one meter.
struct Camera {
  double fx;
  double fy;
  double cx;
  double cy;
  double counts_per_meter;
};

// The points of a depth frame in the base frame, as a PointCloud2 of float x, y, z: each
// pixel with a depth, put through the pinhole model and the mount in double precision.
sensor_msgs::msg::PointCloud2::SharedPtr cloud_of(const DepthImage& image, const Camera& camera,
                                                  const Eigen::Isometry3d& base_from_optical,
                                                  std::int64_t stamp_ns) {
  std::vector<Eigen::Vector3f> points;
  for (int v = 0; v < image.height; ++v) {
    for (int u = 0; u < image.width; ++u) {
      const auto sample = image.samples[static_cast<std::size_t>(v) * image.width + u];
      if (sample == 0) {
        continue;
      }
      const double z = sample / camera.counts_per_meter;
      const Eigen::Vector3d optical((u - camera.cx) * z / camera.fx,
                                    (v - camera.cy) * z / camera.fy, z);
      points.push_back((base_from_optical * optical).cast<float>());
    }
  }
  std_msgs::msg::Header header;
  header.frame_id = base_frame;
  header.stamp = stamp_of_nanoseconds(stamp_ns);
  return float_cloud(points, header);
}

// The scan parameters as cloudCallback writes the scenario's into a scan, in single
// precision.
ScanParameters scan_parameters_of(const nlohmann::json& laser_scan) {
  const auto single = [&](const char* name) {
    return static_cast<float>(laser_scan.at(name).get<double>());
  };
  return {single("angle_min"), single("angle_max"), single("angle_increment"),
          single("range_min"), single("range_max"), single("scan_time")};
}

// Throws unless the scan holds as many ranges as slam_toolbox's laser expects, and a
// point at `angle_max` falls in its last range: cloudCallback holds the ceiling of the
// angle span over the increment and puts a point in the range of the floor of its angle's
// span over the increment, and karto's laser expects the rounded span over the increment,
// and one.
void check_scan_size(const ScanParameters& p) {
  const auto ranges = static_cast<std::uint32_t>(
      std::ceil((p.angle_max - p.angle_min) / p.angle_increment));
  const double span = (static_cast<double>(p.angle_max) - p.angle_min) / p.angle_increment;
  const int last = static_cast<int>(span);
  const double karto_ranges = (span >= 0.0 ? std::floor(span + 0.5) : std::ceil(span - 0.5)) + 1;
  if (last < 0 || static_cast<std::uint32_t>(last) + 1 != ranges ||
      karto_ranges != static_cast<double>(ranges)) {
    throw std::runtime_error("the scan's angles give " + std::to_string(ranges) +
                             " ranges, which slam_toolbox's laser or the last range of a "
                             "point at angle_max does not match");
  }
}

// The difference of two angles, wrapped into [-pi, pi], as AMCL's angleutils computes it.
double angle_diff(double a, double b) {
  a = std::atan2(std::sin(a), std::cos(a));
  b = std::atan2(std::sin(b), std::cos(b));
  const double d1 = a - b;
  const double d2 = d1 > 0 ? -(2 * M_PI - std::fabs(d1)) : 2 * M_PI - std::fabs(d1);
  return std::fabs(d1) < std::fabs(d2) ? d1 : d2;
}

// The odometry of a seed: the base's pose at each kept scan, from the first ground truth
// pose on, each motion between two kept scans sampled as AMCL's differential motion model
// samples a particle's motion (DifferentialMotionModel::odometryUpdate of Nav2 1.5.2): a
// turn, a move and a turn, each with Gaussian noise whose variance the scenario's alpha1
// to alpha4 scale, the first turn 0 for a move under 1 cm, and a turn's noise grown by its
// distance from 0 or from pi, so a move backward has small turns.
std::vector<PlanarPose> noisy_odometry(const std::vector<PlanarPose>& truth,
                                       const nlohmann::json& noise, int seed) {
  const double alpha1 = noise.at("alpha1").get<double>();
  const double alpha2 = noise.at("alpha2").get<double>();
  const double alpha3 = noise.at("alpha3").get<double>();
  const double alpha4 = noise.at("alpha4").get<double>();
  std::mt19937_64 generator(static_cast<std::uint64_t>(seed));
  // A motion of no turn and no move draws no noise.
  const auto gaussian = [&](double deviation) {
    return deviation > 0.0 ? std::normal_distribution<double>(0.0, deviation)(generator) : 0.0;
  };
  // A turn's distance from 0 or from pi.
  const auto turn_noise = [](double turn) {
    return std::min(std::fabs(angle_diff(turn, 0.0)), std::fabs(angle_diff(turn, M_PI)));
  };
  constexpr double shortest_move_with_a_bearing_m = 0.01;
  std::vector<PlanarPose> odometry{truth.front()};
  for (std::size_t i = 1; i < truth.size(); ++i) {
    const auto& a = truth[i - 1];
    const auto& b = truth[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double move = std::sqrt(dx * dx + dy * dy);
    const double first_turn =
        move < shortest_move_with_a_bearing_m ? 0.0 : angle_diff(std::atan2(dy, dx), a.yaw);
    const double second_turn = angle_diff(angle_diff(b.yaw, a.yaw), first_turn);
    const double first_noise = turn_noise(first_turn);
    const double second_noise = turn_noise(second_turn);
    const double noisy_first = angle_diff(
        first_turn,
        gaussian(std::sqrt(alpha1 * first_noise * first_noise + alpha2 * move * move)));
    const double noisy_move =
        move - gaussian(std::sqrt(alpha3 * move * move + alpha4 * first_noise * first_noise +
                                  alpha4 * second_noise * second_noise));
    const double noisy_second = angle_diff(
        second_turn,
        gaussian(std::sqrt(alpha1 * second_noise * second_noise + alpha2 * move * move)));
    const auto& last = odometry.back();
    odometry.push_back({last.x + noisy_move * std::cos(last.yaw + noisy_first),
                        last.y + noisy_move * std::sin(last.yaw + noisy_first),
                        wrapped(last.yaw + noisy_first + noisy_second)});
  }
  return odometry;
}

// Runs `run` of `seed` in a process of its own, and fails when it fails.
void run_in_own_process(const std::string& run, const std::string& scenario_path, int seed,
                        const std::string& repository_root) {
  const std::string seed_text = std::to_string(seed);
  std::vector<std::string> arguments{"ros_reference", room_dataset_run_command, run,
                                     scenario_path, seed_text, repository_root};
  std::vector<char*> argv;
  for (auto& argument : arguments) {
    argv.push_back(argument.data());
  }
  argv.push_back(nullptr);
  pid_t child;
  if (posix_spawn(&child, "/proc/self/exe", nullptr, nullptr, argv.data(), environ) != 0) {
    throw std::runtime_error("cannot start the " + run + " run of seed " + seed_text);
  }
  int status = 0;
  if (waitpid(child, &status, 0) != child || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    throw std::runtime_error("the " + run + " run of seed " + seed_text + " failed");
  }
}

// What a source of scans prepares: the kept scans with their ground truth, the base's pose
// at each kept scan to which each seed's odometry adds its noise, and what the output says of
// the source.
struct SourcePrepared {
  Prepared prepared;
  std::vector<PlanarPose> odometry;
  nlohmann::json described;
};

// A TUM RGB-D sequence: one depth frame a scan period, each projected to a scan by
// pointcloud_to_laserscan from the base frame of the mount fitted from the ground truth, a
// frame inside a gap of the ground truth dropped. Each seed's odometry adds its noise to
// the ground truth.
SourcePrepared tum_rgbd(const nlohmann::json& source, Inputs& inputs) {
  const auto sequence = source.at("sequence").get<std::string>();
  const auto& camera_json = source.at("camera");
  const Camera camera{camera_json.at("fx").get<double>(), camera_json.at("fy").get<double>(),
                      camera_json.at("cx").get<double>(), camera_json.at("cy").get<double>(),
                      camera_json.at("depth_counts_per_meter").get<double>()};
  const auto period_ns = static_cast<std::int64_t>(
      std::llround(source.at("scan_period_s").get<double>() * nanoseconds_per_second));
  const auto max_gap_ns = static_cast<std::int64_t>(
      std::llround(source.at("max_ground_truth_gap_s").get<double>() * nanoseconds_per_second));
  const auto truth = read_ground_truth(inputs.dataset(sequence, "groundtruth.txt"));
  const auto frames = list_depth_frames(inputs.dataset(sequence, "depth"));
  if (frames.empty()) {
    throw std::runtime_error("the dataset " + sequence + " holds no depth frame");
  }
  const auto mount = fit_mount(truth);
  const auto base_from_optical = mount.base_from_optical();
  const auto parameters = scan_parameters_of(source.at("laser_scan"));
  check_scan_size(parameters);

  start_ros();
  std::optional<ScanProjector> projector(std::in_place, source.at("laser_scan"));
  Prepared prepared{};
  auto dropped = nlohmann::json::array();
  for (const auto& frame : one_frame_a_period(frames, period_ns)) {
    const auto pose = truth_at(truth, frame.stamp_ns, max_gap_ns);
    if (!pose) {
      dropped.push_back(frame.stamp_ns);
      continue;
    }
    const auto scan = projector->project(
        cloud_of(read_depth_image(frame.path), camera, base_from_optical, frame.stamp_ns));
    const ScanParameters written{scan.angle_min, scan.angle_max, scan.angle_increment,
                                 scan.range_min, scan.range_max, scan.scan_time};
    if (std::memcmp(&written, &parameters, sizeof written) != 0) {
      throw std::runtime_error("pointcloud_to_laserscan wrote other scan parameters than the "
                               "scenario's");
    }
    prepared.scans.push_back({frame.stamp_ns, *pose, scan.ranges});
  }
  projector.reset();
  rclcpp::shutdown();
  prepared.parameters = parameters;
  std::vector<PlanarPose> truth_at_scans;
  for (const auto& scan : prepared.scans) {
    truth_at_scans.push_back(scan.truth);
  }
  return {std::move(prepared),
          std::move(truth_at_scans),
          {{"tum_rgbd",
            {{"sequence", sequence},
             {"mount",
              {{"floor_normal",
                {mount.floor_normal.x(), mount.floor_normal.y(), mount.floor_normal.z()}},
               {"height", mount.height},
               {"pitch", mount.pitch},
               {"roll", mount.roll}}},
             {"depth_frames", frames.size()},
             {"dropped_scans", dropped}}}}};
}

// lidar_2d's range of a ray the sensor could not measure, or of a surface nearer than its
// nearest range, and of a ray that met nothing within its farthest range.
constexpr float unmeasured_mark_m = 0.0f;
constexpr float no_return_mark_m = 10000.0f;

// Each record of `json`, an array of arrays of N numbers.
template <std::size_t N>
std::vector<std::array<double, N>> records_of(const nlohmann::json& json, const char* what) {
  std::vector<std::array<double, N>> records;
  for (const auto& record : json) {
    const auto values = record.get<std::vector<double>>();
    if (values.size() != N) {
      throw std::runtime_error(std::string(what) + " does not hold " + std::to_string(N) +
                               " numbers");
    }
    std::array<double, N> fixed{};
    std::copy(values.begin(), values.end(), fixed.begin());
    records.push_back(fixed);
  }
  return records;
}

// The base's pose at `t` (s) on a route of timed poses [t, x, y, yaw], in the order of
// their times: the two poses around `t` interpolated, the heading by its shortest turn.
PlanarPose on_route(const std::vector<std::array<double, 4>>& route, double t) {
  const auto after = std::lower_bound(
      route.begin(), route.end(), t,
      [](const std::array<double, 4>& pose, double time) { return pose[0] < time; });
  if (after == route.end() || (after == route.begin() && (*after)[0] != t)) {
    throw std::runtime_error("the drive's route does not cover " + std::to_string(t) + " s");
  }
  if ((*after)[0] == t) {
    return {(*after)[1], (*after)[2], (*after)[3]};
  }
  const auto& a = *std::prev(after);
  const auto& b = *after;
  const double s = (t - a[0]) / (b[0] - a[0]);
  return {a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2]),
          wrapped(a[3] + s * wrapped(b[3] - a[3]))};
}

// The distance from `from` along the heading `angle` to the nearest of the walls, each
// [x0, y0, x1, y1], or infinity when the ray meets none.
double distance_to_walls(const std::vector<std::array<double, 4>>& walls, const PlanarPose& from,
                         double angle) {
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto& [x0, y0, x1, y1] : walls) {
    const double ex = x1 - x0;
    const double ey = y1 - y0;
    const double denominator = dx * ey - dy * ex;
    if (denominator == 0.0) {
      continue;
    }
    const double wx = x0 - from.x;
    const double wy = y0 - from.y;
    const double along_ray = (wx * ey - wy * ex) / denominator;
    const double along_wall = (wx * dy - wy * dx) / denominator;
    if (along_ray >= 0.0 && along_wall >= 0.0 && along_wall <= 1.0) {
      nearest = std::min(nearest, along_ray);
    }
  }
  return nearest;
}

// A lidar drive, which the README describes: the scans that the source keeps, each ray cast
// at its own time from the base's pose on the drive's route, the lidar above the base
// frame's origin facing the way the base faces, its range in single precision, or
// lidar_2d's mark 0 for a wall nearer than the nearest range, or 10000 for no wall within
// the farthest range. Each seed's odometry adds its noise to the route's pose at each kept
// scan.
SourcePrepared lidar_drive(const nlohmann::json& source, Inputs& inputs) {
  const auto drive = nlohmann::json::parse(inputs.read(source.at("drive").get<std::string>()));
  const auto& lidar = drive.at("lidar");
  const auto rays = lidar.at("rays").get<std::size_t>();
  const auto number = [&](const char* name) { return lidar.at(name).get<double>(); };
  const double min_angle = number("min_angle_rad");
  const double increment = number("angle_increment_rad");
  const double time_increment = number("time_increment_s");
  const double min_range = number("min_range_m");
  const double max_range = number("max_range_m");
  const double period = number("scan_period_s");
  const auto walls = records_of<4>(drive.at("walls"), "a wall");
  const auto route = records_of<4>(drive.at("route"), "a timed pose of the route");
  const auto kept = source.at("scans").get<std::vector<std::size_t>>();
  if (rays < 2 || kept.size() != 2 || kept[0] >= kept[1]) {
    throw std::runtime_error("a lidar drive casts at least two rays a scan, and its source keeps "
                             "[first, end) of its scans");
  }
  const ScanParameters parameters{static_cast<float>(min_angle),
                                  static_cast<float>(min_angle + (rays - 1) * increment),
                                  static_cast<float>(increment),
                                  static_cast<float>(min_range),
                                  static_cast<float>(max_range),
                                  static_cast<float>(period)};
  Prepared prepared{parameters, {}};
  std::vector<PlanarPose> truth_at_scans;
  for (std::size_t k = kept[0]; k < kept[1]; ++k) {
    const double stamp_s = static_cast<double>(k) * period;
    std::vector<float> ranges;
    for (std::size_t i = 0; i < rays; ++i) {
      const auto base = on_route(route, stamp_s + static_cast<double>(i) * time_increment);
      const double hit =
          distance_to_walls(walls, base, base.yaw + min_angle + static_cast<double>(i) * increment);
      ranges.push_back(hit >= max_range   ? no_return_mark_m
                       : hit < min_range ? unmeasured_mark_m
                                         : static_cast<float>(hit));
    }
    const auto truth = on_route(route, stamp_s);
    prepared.scans.push_back(
        {std::llround(stamp_s * nanoseconds_per_second), truth, std::move(ranges)});
    truth_at_scans.push_back(truth);
  }
  return {std::move(prepared), std::move(truth_at_scans), {{"lidar_drive", nlohmann::json::object()}}};
}

}  // namespace

nlohmann::json room_dataset(const nlohmann::json& scenario, Inputs& inputs) {
  // The scenario's source and task: objects of one member each.
  const auto& source_json = scenario.at("source");
  const auto source = source_json.size() == 1 ? source_json.begin().key() : std::string();
  const auto& task_json = scenario.at("task");
  const auto task = task_json.size() == 1 ? task_json.begin().key() : std::string();
  const auto seeds = scenario.at("seeds").get<std::vector<int>>();
  if (seeds.empty() || (task != "mapping" && task != "localization") ||
      (source != "tum_rgbd" && source != "lidar_drive")) {
    throw std::runtime_error("a room dataset scenario names seeds, a source, tum_rgbd or "
                             "lidar_drive, and a task, mapping or localization");
  }
  // The scenario's file holds the scenario under its own name, which names its results.
  const auto name = scenario.at("name").get<std::string>();
  const std::string scenario_path = "ros_reference/scenarios/" + name + ".json";
  if (nlohmann::json::parse(inputs.read(scenario_path)) != scenario) {
    throw std::runtime_error(scenario_path + " does not hold the scenario named " + name);
  }
  auto prepared_source = source == "tum_rgbd" ? tum_rgbd(source_json.at(source), inputs)
                                              : lidar_drive(source_json.at(source), inputs);
  const auto& prepared = prepared_source.prepared;
  if (prepared.scans.size() < 2) {
    throw std::runtime_error(name + " gives fewer than two scans");
  }

  const auto results = results_of(name);
  std::filesystem::remove_all(results);
  std::filesystem::create_directories(results);
  write_prepared(results, prepared);
  for (const int seed : seeds) {
    write_odometry(results, seed,
                   noisy_odometry(prepared_source.odometry, scenario.at("odometry_noise"), seed));
  }

  if (task == "localization") {
    const auto map_scenario = nlohmann::json::parse(
        inputs.read("ros_reference/scenarios/" +
                    task_json.at("localization").at("map_scenario").get<std::string>() + ".json"));
    if (!map_scenario.at("task").contains("mapping")) {
      throw std::runtime_error(name + " localizes in a scenario that maps no room");
    }
  }
  // A localization on a lidar drive runs on the drive spliced from two places too.
  auto runs = task == "mapping" ? std::vector<std::string>{"mapping"}
                                : std::vector<std::string>{"localization", "global_localization"};
  if (task == "localization" && source == "lidar_drive") {
    runs.push_back("spliced_localization");
  }
  for (const auto& run : runs) {
    for (const int seed : seeds) {
      run_in_own_process(run, scenario_path, seed, inputs.repository_root().string());
    }
  }

  // The repository files that the reference read, which a run of the harness without the
  // datasets keeps the digests of.
  std::vector<std::string> read = inputs.paths();
  std::sort(read.begin(), read.end());
  read.erase(std::unique(read.begin(), read.end()), read.end());
  return {
      {"kind", "room_dataset"},
      {"source", prepared_source.described},
      {"task", task},
      {"inputs", read},
      {"datasets_read", inputs.datasets_read()},
      {"scans", prepared.scans.size()},
      {"seeds", seeds},
      {"runs", runs},
  };
}

}  // namespace ros_reference
