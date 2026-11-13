# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the room dataset scenarios of the ROS reference harness.

Each scenario reads one TUM RGB-D sequence of a Kinect on a Pioneer, named as
ros_reference/datasets.txt lists it: pioneer_slam and pioneer_360, which hold loops,
map the room; pioneer_slam2 and pioneer_slam3, recorded in the same motion capture
frame, localize in the first seed's map of pioneer_slam. Every scenario runs SEEDS, keeps
one depth frame each SCAN_PERIOD_S, and projects it with the ROS default intrinsics that
TUM recommends, through pointcloud_to_laserscan with the height band and the angles of
LASER_SCAN: ANGLE_MAX lies a quarter increment into the last of BEAMS ranges, so
pointcloud_to_laserscan's index of a point at ANGLE_MAX stays inside the scan, and
slam_toolbox's laser, which expects the rounded angle span over the increment, and one,
expects BEAMS ranges.
slam_toolbox takes its synchronous mapping parameters and its localization parameters as
its configuration files mapper_params_online_sync.yaml and mapper_params_localization.yaml
of slam_toolbox 2.10.0 give them, but for the laser's ranges, and AMCL the parameters of
Nav2's nav2_params.yaml. Run it from any directory:

    uv run ros_reference/tools/draw_room_dataset.py
"""

from drawing import options, write

MAPPING = ("pioneer_slam", "pioneer_360")
LOCALIZATION = ("pioneer_slam2", "pioneer_slam3")
MAP_SEQUENCE = "pioneer_slam"
SEEDS = list(range(10))
SCAN_PERIOD_S = 0.2
MAX_GROUND_TRUTH_GAP_S = 0.1
CAMERA = {"fx": 525.0, "fy": 525.0, "cx": 319.5, "cy": 239.5, "depth_counts_per_meter": 5000.0}
ANGLE_MIN = -0.55
ANGLE_INCREMENT = 0.005
BEAMS = 220
RANGE_MIN = 0.45
# The Kinect's usable range.
RANGE_MAX = 8.0
LASER_SCAN = {
    "min_height": 0.15,
    "max_height": 1.5,
    "angle_min": ANGLE_MIN,
    "angle_max": ANGLE_MIN + (BEAMS - 0.75) * ANGLE_INCREMENT,
    "angle_increment": ANGLE_INCREMENT,
    "scan_time": SCAN_PERIOD_S,
    "range_min": RANGE_MIN,
    "range_max": RANGE_MAX,
    "use_inf": True,
}
# The variance coefficients of AMCL's differential motion model, which draws the odometry
# noise: a standard deviation of 5% of a turn and 0.02 rad a meter in each turn, and of 5%
# of a move and 0.01 m a radian of the turns in the move.
ODOMETRY_NOISE = {"alpha1": 0.0025, "alpha2": 0.0004, "alpha3": 0.0025, "alpha4": 0.0001}
SLAM_TOOLBOX = {
    "solver_plugin": "solver_plugins::CeresSolver",
    "ceres_linear_solver": "SPARSE_NORMAL_CHOLESKY",
    "ceres_preconditioner": "SCHUR_JACOBI",
    "ceres_trust_strategy": "LEVENBERG_MARQUARDT",
    "ceres_dogleg_type": "TRADITIONAL_DOGLEG",
    "ceres_loss_function": "None",
    "scan_topic": "/scan",
    "debug_logging": False,
    "throttle_scans": 1,
    "resolution": 0.05,
    "restamp_tf": False,
    "min_laser_range": RANGE_MIN,
    "max_laser_range": RANGE_MAX,
    "minimum_time_interval": 0.5,
    "transform_timeout": 0.2,
    "tf_buffer_duration": 30.0,
    "enable_interactive_mode": False,
    "use_scan_matching": True,
    "use_scan_barycenter": True,
    "minimum_travel_distance": 0.5,
    "minimum_travel_heading": 0.5,
    "check_min_dist_and_heading_precisely": False,
    "scan_buffer_size": 10,
    "scan_buffer_maximum_scan_distance": 10.0,
    "link_match_minimum_response_fine": 0.1,
    "link_scan_maximum_distance": 1.5,
    "loop_search_maximum_distance": 3.0,
    "do_loop_closing": True,
    "loop_match_minimum_chain_size": 10,
    "loop_match_maximum_variance_coarse": 3.0,
    "loop_match_minimum_response_coarse": 0.35,
    "loop_match_minimum_response_fine": 0.45,
    "correlation_search_space_dimension": 0.5,
    "correlation_search_space_resolution": 0.01,
    "correlation_search_space_smear_deviation": 0.1,
    "loop_search_space_dimension": 8.0,
    "loop_search_space_resolution": 0.05,
    "loop_search_space_smear_deviation": 0.03,
    "distance_variance_penalty": 0.5,
    "angle_variance_penalty": 1.0,
    "fine_search_angle_offset": 0.00349,
    "coarse_search_angle_offset": 0.349,
    "coarse_angle_resolution": 0.0349,
    "minimum_angle_penalty": 0.9,
    "minimum_distance_penalty": 0.5,
    "use_response_expansion": True,
    "min_pass_through": 2,
    "occupancy_threshold": 0.1,
}
# What mapper_params_localization.yaml sets otherwise.
SLAM_TOOLBOX_LOCALIZATION = {**SLAM_TOOLBOX, "scan_buffer_size": 3, "loop_match_minimum_chain_size": 3}
MAP_SAVER = {"free_thresh": 0.25, "occupied_thresh": 0.65}
AMCL = {
    "alpha1": 0.2,
    "alpha2": 0.2,
    "alpha3": 0.2,
    "alpha4": 0.2,
    "alpha5": 0.2,
    "beam_skip_distance": 0.5,
    "beam_skip_error_threshold": 0.9,
    "beam_skip_threshold": 0.3,
    "do_beamskip": False,
    "lambda_short": 0.1,
    "laser_likelihood_max_dist": 2.0,
    "laser_max_range": 100.0,
    "laser_min_range": -1.0,
    "laser_model_type": "likelihood_field",
    "max_beams": 60,
    "max_particles": 2000,
    "min_particles": 500,
    "pf_err": 0.05,
    "pf_z": 0.99,
    "recovery_alpha_fast": 0.0,
    "recovery_alpha_slow": 0.0,
    "resample_interval": 1,
    "robot_model_type": "nav2_amcl::DifferentialMotionModel",
    "sigma_hit": 0.2,
    "transform_tolerance": 1.0,
    "update_min_a": 0.2,
    "update_min_d": 0.25,
    "z_hit": 0.5,
    "z_max": 0.05,
    "z_rand": 0.5,
    "z_short": 0.05,
}
# How near the ground truth an estimate of AMCL lies to count as localized: 0.5 m, the
# distance from the particles' mean along x and along y within which AMCL's convergence
# test holds every particle, and 0.35 rad.
LOCALIZED_WITHIN = {"position_m": 0.5, "yaw_rad": 0.35}
# The largest standard deviations of AMCL's whole filter at which the global localization
# of room_map (ROBOTCTRL-62) takes AMCL's estimate as localized, provisional values.
SEARCH_LIMITS = {"position_std_m": 0.3, "yaw_std_rad": 0.3}


def name_of(sequence):
    return f"room_dataset_{sequence}"


def scenario(sequence, slam_toolbox, task):
    return {
        "kind": "room_dataset",
        "name": name_of(sequence),
        "sequence": sequence,
        "seeds": SEEDS,
        "scan_period_s": SCAN_PERIOD_S,
        "max_ground_truth_gap_s": MAX_GROUND_TRUTH_GAP_S,
        "camera": CAMERA,
        "laser_scan": LASER_SCAN,
        "odometry_noise": ODOMETRY_NOISE,
        "slam_toolbox": slam_toolbox,
        "task": task,
    }


def main():
    options(__doc__, 0, "unused: the scenarios read datasets, and draw nothing").parse_args()
    for sequence in MAPPING:
        write(name_of(sequence), scenario(sequence, SLAM_TOOLBOX, {"mapping": {"map_saver": MAP_SAVER}}))
    for sequence in LOCALIZATION:
        task = {
            "localization": {
                "map_scenario": name_of(MAP_SEQUENCE),
                "amcl": AMCL,
                "localized_within": LOCALIZED_WITHIN,
                "search_limits": SEARCH_LIMITS,
            }
        }
        write(name_of(sequence), scenario(sequence, SLAM_TOOLBOX_LOCALIZATION, task))


if __name__ == "__main__":
    main()
