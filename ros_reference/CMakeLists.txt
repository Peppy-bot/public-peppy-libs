# The reference program of the ROS reference harness and the library of its plugins of
# MoveIt's base classes. The Dockerfile builds them in the reference image, against the ROS
# packages that the image holds.
cmake_minimum_required(VERSION 3.28)
project(ros_reference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS serialization thread)
find_package(Ceres REQUIRED)
find_package(chomp_motion_planner REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(octomap REQUIRED)
find_package(ompl REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PNG REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)
find_package(random_numbers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rsl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(srdfdom REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

# moveit_planners_ompl 2.15.2 installs its headers under include/moveit_planners, and its
# target names include/moveit_planners_ompl, a directory the package does not install.
set_target_properties(moveit_planners_ompl::moveit_ompl_interface PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${moveit_planners_ompl_DIR}/../../../include/moveit_planners")

# The libraries of slam_toolbox's synchronous and localization modes, of the libraries
# that it exports, which hold its RViz plugin too.
foreach(library toolbox_common sync_slam_toolbox localization_slam_toolbox kartoSlamToolbox)
  find_library(SLAM_TOOLBOX_${library} ${library}
    PATHS ${slam_toolbox_DIR}/../../../lib NO_DEFAULT_PATH REQUIRED)
  list(APPEND SLAM_TOOLBOX_MODE_LIBRARIES ${SLAM_TOOLBOX_${library}})
endforeach()

# pointcloud_to_laserscan exports neither its library nor its headers' directory.
find_library(POINTCLOUD_TO_LASERSCAN_LIBRARY pointcloud_to_laserscan
  PATHS ${pointcloud_to_laserscan_DIR}/../../../lib NO_DEFAULT_PATH REQUIRED)
find_path(POINTCLOUD_TO_LASERSCAN_INCLUDE_DIR pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp
  PATHS ${pointcloud_to_laserscan_DIR}/../../../include/pointcloud_to_laserscan/include
  NO_DEFAULT_PATH REQUIRED)

add_executable(ros_reference
  reference_programs/main.cpp
  reference_programs/amcl_map_cspace.cpp
  reference_programs/scenario.cpp
  reference_programs/robot.cpp
  reference_programs/forward_kinematics.cpp
  reference_programs/chain_collision.cpp
  reference_programs/self_distance.cpp
  reference_programs/path_timing.cpp
  reference_programs/planning.cpp
  reference_programs/occupancy.cpp
  reference_programs/orientation_constraints.cpp
  reference_programs/arm_scene.cpp
  reference_programs/arm_planning.cpp
  reference_programs/cartesian_paths.cpp
  reference_programs/chomp_refinement.cpp
  reference_programs/planning_scene.cpp
  reference_programs/room_dataset.cpp
  reference_programs/room_dataset_files.cpp
  reference_programs/room_dataset_runs.cpp
)
target_link_libraries(ros_reference PRIVATE
  Boost::serialization
  Boost::thread
  chomp_motion_planner::chomp_motion_planner
  geometric_shapes::geometric_shapes
  moveit_core::moveit_collision_detection
  moveit_core::moveit_collision_distance_field
  moveit_core::moveit_collision_detection_fcl
  moveit_core::moveit_kinematic_constraints
  moveit_core::moveit_planning_interface
  moveit_core::moveit_planning_scene
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  moveit_core::moveit_robot_trajectory
  moveit_core::moveit_trajectory_processing
  moveit_planners_ompl::moveit_ompl_interface
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  moveit_ros_perception::moveit_pointcloud_octomap_updater_core
  moveit_ros_planning::moveit_kinematics_plugin_loader
  nav2_amcl::amcl_core
  nav2_map_server::map_io
  nlohmann_json::nlohmann_json
  ${OCTOMAP_LIBRARIES}
  ompl::ompl
  pluginlib::pluginlib
  PNG::PNG
  ${POINTCLOUD_TO_LASERSCAN_LIBRARY}
  random_numbers::random_numbers
  rclcpp::rclcpp
  rsl::rsl
  ${sensor_msgs_TARGETS}
  ${SLAM_TOOLBOX_MODE_LIBRARIES}
  Ceres::ceres
  srdfdom::srdfdom
  ${std_srvs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  urdf::urdf
)
# The planning reference walks a motion with the arithmetic of the Rust world that its
# parity test plans through, each multiply and add rounded on its own on every
# architecture, as Rust rounds them.
set_source_files_properties(reference_programs/planning.cpp PROPERTIES COMPILE_OPTIONS
  -ffp-contract=off)
target_include_directories(ros_reference PRIVATE
  ${OCTOMAP_INCLUDE_DIRS}
  ${POINTCLOUD_TO_LASERSCAN_INCLUDE_DIR}
  ${slam_toolbox_INCLUDE_DIRS}
)
# arm_scene.cpp defines the default constructor of random_numbers' generator, which MoveIt's
# samplers and its KDL plugin construct theirs by, and seeds it from the scene, and
# amcl_map_cspace.cpp defines map_update_cspace, which AMCL's sensor models call; the
# program exports them, so MoveIt's and AMCL's libraries bind to them.
target_link_options(ros_reference PRIVATE
  "LINKER:--export-dynamic-symbol=_ZN14random_numbers21RandomNumberGeneratorC1Ev"
  "LINKER:--export-dynamic-symbol=_ZN14random_numbers21RandomNumberGeneratorC2Ev"
  "LINKER:--export-dynamic-symbol=map_update_cspace")

# The library of MoveIt's KDL plugin, which ros_reference/PassBoundedKdl derives from, found
# in moveit_kinematics' lib directory: moveit_kinematics 2.15.2 exports no target of it.
find_library(MOVEIT_KDL_KINEMATICS_PLUGIN moveit_kdl_kinematics_plugin
  PATHS ${moveit_kinematics_DIR}/../../../lib NO_DEFAULT_PATH REQUIRED)

# The plugins of MoveIt's base classes that the references load through pluginlib, which
# moveit_plugins.xml names.
add_library(ros_reference_moveit_plugins SHARED
  reference_programs/pass_bounded_kdl.cpp
  reference_programs/union_in_name_order.cpp
)
target_link_libraries(ros_reference_moveit_plugins PRIVATE
  ${MOVEIT_KDL_KINEMATICS_PLUGIN}
  moveit_core::moveit_constraint_samplers
  moveit_core::moveit_kinematics_base
  moveit_core::moveit_planning_scene
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  moveit_core::moveit_utils
  moveit_kinematics::kdl_kinematics_parameters
  ${orocos_kdl_LIBRARIES}
  pluginlib::pluginlib
  rcutils::rcutils
)

install(TARGETS ros_reference)
install(TARGETS ros_reference_moveit_plugins LIBRARY DESTINATION lib)
# The ament index entries by which pluginlib finds the plugins at the install prefix, which
# the image puts on AMENT_PREFIX_PATH: the package ros_reference, and its plugins of
# moveit_core's base classes.
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ament_index/packages/ros_reference "")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ament_index/moveit_core__pluginlib__plugin/ros_reference
  "share/ros_reference/moveit_plugins.xml\n")
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ament_index/
  DESTINATION share/ament_index/resource_index)
install(FILES reference_programs/package.xml reference_programs/moveit_plugins.xml
  DESTINATION share/ros_reference)
