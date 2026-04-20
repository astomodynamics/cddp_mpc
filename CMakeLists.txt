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
option(BUILD_LEGACY_GROUND_DEMOS "Build archived pre-PX4 ground-robot demo nodes" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
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

add_executable(
  px4_mpc_node
  src/px4_mpc_node.cpp
  src/indi_rotational_compensator.cpp
  src/reference_manager.cpp
  src/reference_provider.cpp
)
target_include_directories(px4_mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(px4_mpc_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  px4_msgs
  sensor_msgs
  std_srvs
  visualization_msgs
  Eigen3
)
target_link_libraries(px4_mpc_node
  cddp
)

add_executable(px4_visualizer src/px4_visualizer.cpp)
target_include_directories(px4_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(px4_visualizer
  diagnostic_msgs
  Eigen3
  geometry_msgs
  interactive_markers
  nav_msgs
  px4_msgs
  rclcpp
  visualization_msgs
)

add_executable(gamepad_teleop_node src/gamepad_teleop_node.cpp)
ament_target_dependencies(gamepad_teleop_node
  geometry_msgs
  rclcpp
  sensor_msgs
)

if(BUILD_LEGACY_GROUND_DEMOS)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  add_library(unicycle_lib
    src/unicycle.cpp
  )
  target_include_directories(unicycle_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(unicycle_lib rclcpp geometry_msgs)

  add_executable(legacy_mpc_node src/mpc_node.cpp)
  target_include_directories(legacy_mpc_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(legacy_mpc_node
    rclcpp
    geometry_msgs
    nav_msgs
    Eigen3
  )
  target_link_libraries(legacy_mpc_node cddp)

  add_executable(legacy_car_mpc_node src/car_mpc_node.cpp)
  target_include_directories(legacy_car_mpc_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(legacy_car_mpc_node
    rclcpp
    geometry_msgs
    nav_msgs
    Eigen3
  )
  target_link_libraries(legacy_car_mpc_node cddp)

  add_executable(legacy_unicycle_robot_node
    src/unicycle_robot_node.cpp
  )
  target_include_directories(legacy_unicycle_robot_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(legacy_unicycle_robot_node unicycle_lib)
  ament_target_dependencies(legacy_unicycle_robot_node
    rclcpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
  )

  add_executable(legacy_teleop_keyboard_node
    src/teleop_keyboard.cpp
  )
  ament_target_dependencies(legacy_teleop_keyboard_node rclcpp geometry_msgs)

  add_executable(legacy_hardcoded_map_node src/hardcoded_map_node.cpp)
  ament_target_dependencies(legacy_hardcoded_map_node rclcpp nav_msgs geometry_msgs)
endif()

install(TARGETS
  gamepad_teleop_node
  px4_mpc_node
  px4_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_LEGACY_GROUND_DEMOS)
  install(TARGETS
    legacy_mpc_node
    legacy_car_mpc_node
    legacy_unicycle_robot_node
    legacy_teleop_keyboard_node
    legacy_hardcoded_map_node
    unicycle_lib
    DESTINATION lib/${PROJECT_NAME}
  )
endif()


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY examples
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY legacy
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY standalone
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY video
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(Threads REQUIRED)
  find_path(GTEST_INCLUDE_DIR gtest/gtest.h REQUIRED)
  find_library(GTEST_LIBRARY gtest REQUIRED)
  find_library(GTEST_MAIN_LIBRARY gtest_main REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_executable(test_px4_utils test/test_px4_utils.cpp)
  if(TARGET test_px4_utils)
    target_include_directories(test_px4_utils PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${GTEST_INCLUDE_DIR}
    )
    ament_target_dependencies(test_px4_utils Eigen3)
    target_link_libraries(test_px4_utils
      ${GTEST_LIBRARY}
      ${GTEST_MAIN_LIBRARY}
      Threads::Threads
    )
    add_test(NAME test_px4_utils COMMAND test_px4_utils)
  endif()

  add_executable(test_controller_logic test/test_controller_logic.cpp)
  if(TARGET test_controller_logic)
    target_include_directories(test_controller_logic PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${GTEST_INCLUDE_DIR}
    )
    ament_target_dependencies(test_controller_logic Eigen3)
    target_link_libraries(test_controller_logic
      ${GTEST_LIBRARY}
      ${GTEST_MAIN_LIBRARY}
      Threads::Threads
    )
    add_test(NAME test_controller_logic COMMAND test_controller_logic)
  endif()

  add_executable(test_error_state_quadrotor_rate test/test_error_state_quadrotor_rate.cpp)
  if(TARGET test_error_state_quadrotor_rate)
    target_include_directories(test_error_state_quadrotor_rate PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${GTEST_INCLUDE_DIR}
    )
    ament_target_dependencies(test_error_state_quadrotor_rate Eigen3)
    target_link_libraries(test_error_state_quadrotor_rate
      cddp
      ${GTEST_LIBRARY}
      ${GTEST_MAIN_LIBRARY}
      Threads::Threads
    )
    add_test(NAME test_error_state_quadrotor_rate COMMAND test_error_state_quadrotor_rate)
  endif()

  add_executable(test_error_state_quadrotor_thrust test/test_error_state_quadrotor_thrust.cpp)
  if(TARGET test_error_state_quadrotor_thrust)
    target_include_directories(test_error_state_quadrotor_thrust PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${GTEST_INCLUDE_DIR}
    )
    ament_target_dependencies(test_error_state_quadrotor_thrust Eigen3)
    target_link_libraries(test_error_state_quadrotor_thrust
      cddp
      ${GTEST_LIBRARY}
      ${GTEST_MAIN_LIBRARY}
      Threads::Threads
    )
    add_test(NAME test_error_state_quadrotor_thrust COMMAND test_error_state_quadrotor_thrust)
  endif()

  add_executable(
    test_reference_provider
    test/test_reference_provider.cpp
    src/reference_provider.cpp
  )
  if(TARGET test_reference_provider)
    target_include_directories(test_reference_provider PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${GTEST_INCLUDE_DIR}
    )
    ament_target_dependencies(test_reference_provider Eigen3)
    target_link_libraries(test_reference_provider
      ${GTEST_LIBRARY}
      ${GTEST_MAIN_LIBRARY}
      Threads::Threads
    )
    add_test(NAME test_reference_provider COMMAND test_reference_provider)
  endif()

  add_executable(
    test_reference_manager
    test/test_reference_manager.cpp
    src/reference_manager.cpp
  )
  if(TARGET test_reference_manager)
    target_include_directories(test_reference_manager PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${GTEST_INCLUDE_DIR}
    )
    ament_target_dependencies(test_reference_manager Eigen3)
    target_link_libraries(test_reference_manager
      ${GTEST_LIBRARY}
      ${GTEST_MAIN_LIBRARY}
      Threads::Threads
    )
    add_test(NAME test_reference_manager COMMAND test_reference_manager)
  endif()

  add_executable(
    test_indi_rotational_compensator
    test/test_indi_rotational_compensator.cpp
    src/indi_rotational_compensator.cpp
  )
  if(TARGET test_indi_rotational_compensator)
    target_include_directories(test_indi_rotational_compensator PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${GTEST_INCLUDE_DIR}
    )
    ament_target_dependencies(test_indi_rotational_compensator Eigen3)
    target_link_libraries(test_indi_rotational_compensator
      ${GTEST_LIBRARY}
      ${GTEST_MAIN_LIBRARY}
      Threads::Threads
    )
    add_test(NAME test_indi_rotational_compensator COMMAND test_indi_rotational_compensator)
  endif()

  add_test(
    NAME test_validate_takeoff_hover_py
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_validate_takeoff_hover.py
  )

  add_test(
    NAME test_calibrate_hover_mass_py
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_calibrate_hover_mass.py
  )

  add_test(
    NAME test_px4_data_collector_py
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_px4_data_collector.py
  )

  add_test(
    NAME test_sitl_config_defaults_py
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sitl_config_defaults.py
  )
endif()

ament_package()
