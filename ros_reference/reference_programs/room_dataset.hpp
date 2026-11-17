// Room dataset: slam_toolbox maps a room, and slam_toolbox and Nav2's AMCL localize in that
// map, on 2-D scans that pointcloud_to_laserscan projects from the depth frames of a TUM
// RGB-D sequence, with odometry made from its ground truth and seeded noise.
//
// The reference of a scenario prepares its sequence in the program's own process: it
// keeps one depth frame each scan period, drops a frame inside a gap of the ground truth,
// fits the camera's mount on the base from the ground truth, projects each kept frame to
// a scan, and draws the noisy odometry of each seed. It writes those prepared inputs under
// the directory that ROS_REFERENCE_RESULTS names, at `<scenario>/`. Each run then has a
// process of its own, which this program starts as `ros_reference room_dataset_run`: a
// mapping scenario maps once a seed, and the first seed's run also saves its pose graph
// and exports its grid as nav2_map_server writes it; a localization scenario localizes in
// the first seed's map of the mapping scenario that it names, with slam_toolbox once a
// seed and with AMCL's global localization once a seed. Each run writes its poses beside
// the prepared inputs. The output holds what the reference read and prepared; the
// `statistics` program of the ros_reference crate scores the runs.
#pragma once

#include <nlohmann/json.hpp>

#include "scenario.hpp"

namespace ros_reference {

// The first argument by which this program runs one run of a room dataset scenario.
inline constexpr const char* room_dataset_run_command = "room_dataset_run";

nlohmann::json room_dataset(const nlohmann::json& scenario, Inputs& inputs);

// One run in this process: `ros_reference room_dataset_run <run> <scenario.json> <seed>
// <repository root>`, where <run> is mapping, localization or global_localization.
int room_dataset_run(int argc, char** argv);

}  // namespace ros_reference
