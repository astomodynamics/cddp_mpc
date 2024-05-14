cmake_minimum_required(VERSION 3.14)
project(cddp_mpc)


set(CMAKE_CXX_STANDARD 17) # Enforce C++17 as the minimum standard
set(CMAKE_CXX_STANDARD_REQUIRED ON) 


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


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

add_library(cddp src/cddp_core/CDDPProblem.cpp)
target_link_libraries(cddp PUBLIC 
    Eigen3::Eigen 
    PRIVATE osqp-cpp ${CMAKE_DL_LIBS})

target_include_directories(cddp PUBLIC 
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" 
    include/cddp)

add_executable(mpc_node src/mpc_node.cpp)
ament_target_dependencies(mpc_node 
  rclcpp 
  geometry_msgs 
  nav_msgs
  Eigen3
)

target_include_directories(mpc_node PUBLIC 
    ${CDDP_INCLUDE_DIRS} 
    )
target_link_directories(mpc_node PUBLIC include/cddp)
target_link_libraries(mpc_node osqp-cpp)



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
