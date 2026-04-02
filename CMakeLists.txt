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
find_package(px4_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Enable FetchContent for downloading dependencies
include(FetchContent)

set(CDDP_LOCAL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../cddp-cpp")
if(EXISTS "${CDDP_LOCAL_SOURCE_DIR}/CMakeLists.txt")
  message(STATUS "Using local cddp-cpp source from ${CDDP_LOCAL_SOURCE_DIR}")
  set(CDDP_CPP_BUILD_TESTS OFF CACHE BOOL "Disable cddp-cpp tests in parent build" FORCE)
  set(CDDP_CPP_BUILD_EXAMPLES OFF CACHE BOOL "Disable cddp-cpp examples in parent build" FORCE)
  add_subdirectory("${CDDP_LOCAL_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/cddp-cpp-build" EXCLUDE_FROM_ALL)
else()
  FetchContent_Declare(
    cddp
    GIT_REPOSITORY https://github.com/astomodynamics/cddp-cpp
    GIT_TAG origin/master
  )
  FetchContent_MakeAvailable(cddp)
endif()

add_executable(px4_mpc_node src/px4_mpc_node.cpp)
target_include_directories(px4_mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(px4_mpc_node
  rclcpp
  px4_msgs
  std_srvs
  Eigen3
)
target_link_libraries(px4_mpc_node
  cddp
)

install(TARGETS 
  px4_mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_px4_utils test/test_px4_utils.cpp)
  if(TARGET test_px4_utils)
    target_include_directories(test_px4_utils PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    ament_target_dependencies(test_px4_utils Eigen3)
  endif()
endif()

ament_package()
