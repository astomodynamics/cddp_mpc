cmake_minimum_required(VERSION 3.14)
project(cddp_mpc)


set(CMAKE_CXX_STANDARD 17) # Enforce C++17 as the minimum standard
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Enforce C++17 as the minimum standard
set(ABSL_PROPAGATE_CXX_STD ON) # Enforce C++17 as the minimum standard


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# Find the include path for CDDP's header files
find_path(CDDP_INCLUDE_DIR NAMES CDDP.hpp
          PATHS /home/tom/github/CDDP-cpp/include/cddp)

# Find the compiled library file
find_library(CDDP_LIBRARY NAMES cddp
             PATHS /home/tom/github/CDDP-cpp/build)

# Enable FetchContent for downloading dependencies
include(FetchContent)

# OSQP-CPP
if (NOT TARGET osqp-cpp OR NOT TARGET osqpstatic)
    message(STATUS "cddp_mpc_pkg: `osqp-cpp` targets not found. Attempting to fetch contents...")
    FetchContent_Declare(
    osqp-cpp
    GIT_REPOSITORY https://github.com/google/osqp-cpp
    GIT_TAG master
  )
  FetchContent_MakeAvailable(osqp-cpp)
else()
    message(STATUS "cddp_mpc_pkg: `osqp-cpp` targets found.")
    find_package(osqp-cpp REQUIRED)
endif()

# Include the headers path
include_directories(${CDDP_INCLUDE_DIR})

set(CDDP_SRC /home/tom/github/CDDP-cpp/src/cddp_core/CDDPProblem.cpp)

add_executable(mpc_node src/mpc_node.cpp ${CDDP_SRC})
ament_target_dependencies(mpc_node 
  rclcpp 
  geometry_msgs 
  nav_msgs
  Eigen3
)

target_include_directories(mpc_node PUBLIC ${CDDP_INCLUDE_DIRS})
target_link_libraries(mpc_node ${CDDP_LIBRARY} osqp-cpp)



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
