cmake_minimum_required(VERSION 3.14)
project(cddp_mpc_pkg)


set(CMAKE_CXX_STANDARD 17) # Enforce C++17 as the minimum standard
set(CMAKE_CXX_STANDARD_REQUIRED ON) 


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)


# Find the include path for CDDP's header files
find_path(CDDP_INCLUDE_DIR NAMES CDDP.hpp
          PATHS /home/tom/github/CDDP-cpp/include/cddp)

# Find the compiled library file
find_library(CDDP_LIBRARY NAMES cddp
             PATHS /home/tom/github/CDDP-cpp/build/lib)

# Include the headers path
include_directories(${CDDP_INCLUDE_DIR})

add_executable(mpc_node src/mpc_node.cpp)
ament_target_dependencies(mpc_node 
  rclcpp 
  geometry_msgs 
  Eigen3
)
target_link_libraries(mpc_node ${CDDP_LIBRARY})


install(TARGETS 
  mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
