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

# Enable FetchContent for downloading dependencies
include(FetchContent)

# CDDP
FetchContent_Declare(
  cddp
  GIT_REPOSITORY https://github.com/astomodynamics/cddp-cpp
  GIT_TAG origin/master
)
FetchContent_MakeAvailable(cddp)


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

install(TARGETS 
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
