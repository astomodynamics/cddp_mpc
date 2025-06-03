cmake_minimum_required(VERSION 3.14)
project(
  cddp_mpc
  VERSION 0.1.0
  DESCRIPTION "CDDP-MPC module for ROS2"
  HOMEPAGE_URL  "https://github.com/astomodynamics/cddp_mpc"
)

set(CMAKE_CXX_STANDARD 17) # Enforce C++17 as the minimum standard
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Enforce C++17 as the minimum standard
set(ABSL_PROPAGATE_CXX_STD ON) # Enforce C++17 for absl
set(CMAKE_BUILD_TYPE "Release") # Set the build type to Release by default

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Enable FetchContent for downloading dependencies
include(FetchContent)

# CDDP
FetchContent_Declare(
  cddp
  GIT_REPOSITORY https://github.com/astomodynamics/cddp-cpp
  GIT_TAG origin/master
)
FetchContent_MakeAvailable(cddp)

# Build the MPC node.
add_executable(mpc_node src/mpc_node.cpp)
ament_target_dependencies(mpc_node 
  rclcpp 
  geometry_msgs 
  nav_msgs
  Eigen3
)

target_link_libraries(mpc_node 
  cddp
)

# Build the unicycle model library.
# Assuming unicycle.hpp is in include/ and unicycle.cpp is in src/
add_library(unicycle_lib
  src/unicycle.cpp
)
target_include_directories(unicycle_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(unicycle_lib rclcpp geometry_msgs)

# Build the unicycle robot node.
add_executable(unicycle_robot_node
  src/unicycle_robot_node.cpp
)
target_link_libraries(unicycle_robot_node unicycle_lib)
ament_target_dependencies(unicycle_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Build the teleop keyboard node.
add_executable(teleop_keyboard_node
  src/teleop_keyboard.cpp
)
ament_target_dependencies(teleop_keyboard_node rclcpp geometry_msgs)

# Build the hardcoded map node.
add_executable(hardcoded_map_node src/hardcoded_map_node.cpp)

ament_target_dependencies(hardcoded_map_node rclcpp nav_msgs geometry_msgs)


install(TARGETS 
  mpc_node
  unicycle_robot_node
  teleop_keyboard_node
  hardcoded_map_node
  unicycle_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
