# The reference program of the ROS reference harness. The Dockerfile builds it in the
# reference image, against the ROS packages that the image holds.
cmake_minimum_required(VERSION 3.28)
project(ros_reference LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(geometric_shapes REQUIRED)
find_package(moveit_core REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)

add_executable(ros_reference
  reference_programs/main.cpp
  reference_programs/scenario.cpp
  reference_programs/robot.cpp
  reference_programs/forward_kinematics.cpp
  reference_programs/chain_collision.cpp
  reference_programs/self_distance.cpp
)
target_link_libraries(ros_reference PRIVATE
  geometric_shapes::geometric_shapes
  moveit_core::moveit_collision_detection
  moveit_core::moveit_collision_detection_fcl
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  nlohmann_json::nlohmann_json
  srdfdom::srdfdom
  urdf::urdf
)
install(TARGETS ros_reference)
