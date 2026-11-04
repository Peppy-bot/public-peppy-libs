# The reference program of the ROS reference harness. The Dockerfile builds it in the
# reference image, against the ROS packages that the image holds.
cmake_minimum_required(VERSION 3.28)
project(ros_reference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(geometric_shapes REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(octomap REQUIRED)
find_package(ompl REQUIRED)
find_package(random_numbers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)

add_executable(ros_reference
  reference_programs/main.cpp
  reference_programs/scenario.cpp
  reference_programs/robot.cpp
  reference_programs/forward_kinematics.cpp
  reference_programs/chain_collision.cpp
  reference_programs/self_distance.cpp
  reference_programs/path_timing.cpp
  reference_programs/planning.cpp
  reference_programs/occupancy.cpp
  reference_programs/arm_planning.cpp
  reference_programs/planning_scene.cpp
)
target_link_libraries(ros_reference PRIVATE
  geometric_shapes::geometric_shapes
  moveit_core::moveit_collision_detection
  moveit_core::moveit_collision_detection_fcl
  moveit_core::moveit_kinematic_constraints
  moveit_core::moveit_planning_scene
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  moveit_core::moveit_trajectory_processing
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  moveit_ros_perception::moveit_pointcloud_octomap_updater_core
  moveit_ros_planning::moveit_kinematics_plugin_loader
  moveit_ros_planning::moveit_planning_pipeline
  nlohmann_json::nlohmann_json
  ${OCTOMAP_LIBRARIES}
  ompl::ompl
  random_numbers::random_numbers
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  srdfdom::srdfdom
  urdf::urdf
)
# The planning reference walks a motion with the arithmetic of the Rust world that its
# parity test plans through, each multiply and add rounded on its own on every
# architecture, as Rust rounds them.
set_source_files_properties(reference_programs/planning.cpp PROPERTIES COMPILE_OPTIONS
  -ffp-contract=off)
target_include_directories(ros_reference PRIVATE ${OCTOMAP_INCLUDE_DIRS})
# The arm planning reference defines the default constructor of random_numbers'
# generator, which MoveIt's samplers construct theirs by, and seeds it from the scene;
# the program exports it, so MoveIt's libraries bind to it.
target_link_options(ros_reference PRIVATE
  "LINKER:--export-dynamic-symbol=_ZN14random_numbers21RandomNumberGeneratorC1Ev"
  "LINKER:--export-dynamic-symbol=_ZN14random_numbers21RandomNumberGeneratorC2Ev")
install(TARGETS ros_reference)
