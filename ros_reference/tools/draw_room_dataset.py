# /// script
# requires-python = ">=3.12,<3.14"
# dependencies = ["robot_description_py"]
#
# [tool.uv.sources]
# robot_description_py = { path = "../../robot_description_py" }
# ///
"""Writes the room dataset scenarios of the ROS reference harness.

Each scenario reads one TUM RGB-D sequence of a Kinect on a Pioneer, named as
ros_reference/datasets.txt lists it: pioneer_slam and pioneer_360, which hold loops, map
the room; pioneer_slam2 and pioneer_slam3, recorded in the same motion capture frame,
localize in pioneer_slam's maps, a seed in the map that the same seed made. Every scenario
runs SEEDS, keeps one depth frame each SCAN_PERIOD_S, and projects it with the ROS default
intrinsics that TUM recommends, through pointcloud_to_laserscan with the height band and
the angles of LASER_SCAN: ANGLE_MAX lies a quarter increment into the last of BEAMS
ranges, so pointcloud_to_laserscan's index of a point at ANGLE_MAX stays inside the scan,
and slam_toolbox's laser, which expects the rounded angle span over the increment, and
one, expects BEAMS ranges.
slam_toolbox takes its synchronous mapping parameters and its localization parameters as
its configuration files mapper_params_online_sync.yaml and mapper_params_localization.yaml
of slam_toolbox 2.10.0 give them, but for the laser's ranges, and AMCL the parameters of
Nav2's nav2_params.yaml.

It also writes the lidar drive DRIVE, which the README describes, and its scenarios: its
first lap maps the room, and its second lap localizes in the first lap's maps from three
places, each with the lidar's range limits. Run it from any directory:

    uv run ros_reference/tools/draw_room_dataset.py
"""

import math

from drawing import REPOSITORY, options, segment_distance, write, write_input

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

# The lidar drive: an RPLIDAR S3 as the sllidar_ros2 driver gives its scans, 3240 rays from
# -pi a turn round, swept in a scan period, each ray TIME_INCREMENT after the one before; on
# a base that drives DRIVE_LAPS laps of ROUTE through L_ROOM, turning in place at each
# waypoint at DRIVE_TURN_RATE and driving between them at DRIVE_SPEED.
DRIVE = "ros_reference/drives/l_room.json"
DRIVE_RAYS = 3240
DRIVE_SCAN_PERIOD_S = 0.1
DRIVE_LIDAR = {
    "rays": DRIVE_RAYS,
    "min_angle_rad": -math.pi,
    "angle_increment_rad": 2.0 * math.pi / (DRIVE_RAYS - 1),
    "time_increment_s": DRIVE_SCAN_PERIOD_S / (DRIVE_RAYS - 1),
    "min_range_m": 0.05,
    "max_range_m": 40.0,
    "scan_period_s": DRIVE_SCAN_PERIOD_S,
}
# The room's outline, an L of an 8 m by 3 m arm and a 5 m by 3 m one, and five boxes that
# make each place of it look unlike the others, each a polygon of its corners (m), and a
# doorway in the outline's east wall, through which a ray meets nothing within the lidar's
# range and reads lidar_2d's no-return mark.
L_ROOM = [
    [(0.0, 0.0), (8.0, 0.0), (8.0, 3.0), (5.0, 3.0), (5.0, 6.0), (0.0, 6.0)],
    [(2.0, 0.0), (2.8, 0.0), (2.8, 0.6), (2.0, 0.6)],
    [(6.5, 2.2), (7.3, 2.2), (7.3, 3.0), (6.5, 3.0)],
    [(4.3, 1.4), (5.0, 1.4), (5.0, 1.8), (4.3, 1.8)],
    [(0.0, 4.2), (0.7, 4.2), (0.7, 5.4), (0.0, 5.4)],
    [(3.2, 4.6), (3.6, 4.6), (3.6, 5.0), (3.2, 5.0)],
]
DOORWAY = ((8.0, 1.8), (8.0, 2.6))
# A closed route through both arms, at least ROUTE_CLEARANCE_M from every wall and box.
ROUTE_CLEARANCE_M = 0.25
ROUTE = [
    (1.2, 2.4),
    (5.8, 2.4),
    (7.5, 1.0),
    (3.6, 0.9),
    (1.2, 1.0),
    (1.4, 3.6),
    (2.2, 5.5),
    (4.4, 5.5),
    (4.4, 3.4),
]
DRIVE_LAPS = 2
DRIVE_SPEED_M_S = 0.3
DRIVE_TURN_RATE_RAD_S = 0.5
# Where the drive's localizations start, as shares of the second lap: each search runs at
# least half a lap.
DRIVE_LOCALIZATION_STARTS = {f"lap2_at_{round(100 * share):02d}": share for share in (0.0, 0.2, 0.4)}
# The largest standard deviations of AMCL's whole filter that end the search on the drive,
# provisional: at Nav2's alphas, in drawn turned L-shaped rooms, 0.45 is the smallest limit
# in steps of 0.05 at which every room's search handed over, and 0.5 one step more.
DRIVE_SEARCH_LIMITS = {"position_std_m": 0.5, "yaw_std_rad": 0.5}


def name_of(sequence):
    return f"room_dataset_{sequence}"


def scenario(name, source, slam_toolbox, task):
    return {
        "kind": "room_dataset",
        "name": name,
        "source": source,
        "seeds": SEEDS,
        "odometry_noise": ODOMETRY_NOISE,
        "slam_toolbox": slam_toolbox,
        "task": task,
    }


def tum_rgbd(sequence):
    return {
        "tum_rgbd": {
            "sequence": sequence,
            "scan_period_s": SCAN_PERIOD_S,
            "max_ground_truth_gap_s": MAX_GROUND_TRUTH_GAP_S,
            "camera": CAMERA,
            "laser_scan": LASER_SCAN,
        }
    }


def localization(map_scenario, search_limits):
    return {
        "localization": {
            "map_scenario": map_scenario,
            "amcl": AMCL,
            "localized_within": LOCALIZED_WITHIN,
            "search_limits": search_limits,
        }
    }


MAPPING_TASK = {"mapping": {"map_saver": MAP_SAVER}}


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def timed_route():
    """The base's poses where it starts and ends each turn and each straight move of the
    drive, [t (s), x, y, yaw], and the time it comes back to the route's start at the end of
    each lap. The base starts at the first waypoint facing the second."""
    x, y = ROUTE[0]
    yaw = math.atan2(ROUTE[1][1] - y, ROUTE[1][0] - x)
    t = 0.0
    poses = [[t, x, y, yaw]]
    lap_ends = []
    for _ in range(DRIVE_LAPS):
        for target in ROUTE[1:] + ROUTE[:1]:
            heading = math.atan2(target[1] - y, target[0] - x)
            turn = wrapped(heading - yaw)
            if turn != 0.0:
                t += abs(turn) / DRIVE_TURN_RATE_RAD_S
                yaw = heading
                poses.append([t, x, y, yaw])
            t += math.hypot(target[0] - x, target[1] - y) / DRIVE_SPEED_M_S
            x, y = target
            poses.append([t, x, y, yaw])
        lap_ends.append(t)
    return poses, lap_ends


def cut(wall, opening):
    """The parts of the wall `wall`, (a, b), that the opening `opening`, a part of it, leaves
    standing."""
    a, b = wall
    p, q = opening
    along = math.dist(a, b)
    if not math.isclose(math.dist(a, p) + math.dist(p, q) + math.dist(q, b), along):
        return [wall]
    return [part for part in ((a, p), (q, b)) if math.dist(*part) > 0.0]


def drive():
    """The drive's file, and the index of the scan at the end of each lap."""
    poses, lap_ends = timed_route()
    sides = [(polygon[i], polygon[(i + 1) % len(polygon)]) for polygon in L_ROOM for i in range(len(polygon))]
    walls = [[*a, *b] for side in sides for a, b in cut(side, DOORWAY)]
    assert len(walls) == len(sides) + 1, "the doorway cuts one wall in two"
    legs = list(zip(ROUTE, ROUTE[1:] + ROUTE[:1]))
    clearance = min(segment_distance(*leg, wall[:2], wall[2:]) for leg in legs for wall in walls)
    assert clearance >= ROUTE_CLEARANCE_M, f"the route passes {clearance} m from a wall"
    return {"lidar": DRIVE_LIDAR, "walls": walls, "route": poses}, [
        int(end / DRIVE_SCAN_PERIOD_S) for end in lap_ends
    ]


def write_drive_scenarios():
    """The drive's file, and its scenarios: the first lap maps the room, back at the
    route's start, and the second lap localizes in the first lap's maps from each share of
    DRIVE_LOCALIZATION_STARTS to the drive's last scan, whose last ray the route covers."""
    definition, lap_end_scans = drive()
    path = REPOSITORY / DRIVE
    path.parent.mkdir(exist_ok=True)
    write_input(path, definition)
    scans = int((definition["route"][-1][0] - DRIVE_SCAN_PERIOD_S) / DRIVE_SCAN_PERIOD_S) + 1
    ranges = {"min_laser_range": DRIVE_LIDAR["min_range_m"], "max_laser_range": DRIVE_LIDAR["max_range_m"]}
    mapping = name_of("l_room")
    first_lap = [0, lap_end_scans[0] + 1]
    write(mapping, scenario(mapping, {"lidar_drive": {"drive": DRIVE, "scans": first_lap}}, {**SLAM_TOOLBOX, **ranges}, MAPPING_TASK))
    lap_scans = lap_end_scans[1] - lap_end_scans[0]
    for start_name, share in DRIVE_LOCALIZATION_STARTS.items():
        name = name_of(f"l_room_{start_name}")
        kept = [lap_end_scans[0] + round(share * lap_scans), scans]
        write(
            name,
            scenario(
                name,
                {"lidar_drive": {"drive": DRIVE, "scans": kept}},
                {**SLAM_TOOLBOX_LOCALIZATION, **ranges},
                localization(mapping, DRIVE_SEARCH_LIMITS),
            ),
        )


def main():
    options(__doc__, 0, "unused: the scenarios read datasets or the drive, and draw no set").parse_args()
    for sequence in MAPPING:
        write(name_of(sequence), scenario(name_of(sequence), tum_rgbd(sequence), SLAM_TOOLBOX, MAPPING_TASK))
    for sequence in LOCALIZATION:
        write(
            name_of(sequence),
            scenario(
                name_of(sequence),
                tum_rgbd(sequence),
                SLAM_TOOLBOX_LOCALIZATION,
                localization(name_of(MAP_SEQUENCE), SEARCH_LIMITS),
            ),
        )
    write_drive_scenarios()


if __name__ == "__main__":
    main()
