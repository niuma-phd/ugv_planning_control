cmake_minimum_required(VERSION 3.8)
project(ugv_gps_waypoint_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ugv_subject2_mvp REQUIRED)

add_library(${PROJECT_NAME}_core src/gps_waypoint_core.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(gps_waypoint_controller_node src/gps_waypoint_controller_node.cpp)
target_link_libraries(gps_waypoint_controller_node ${PROJECT_NAME}_core)
ament_target_dependencies(
  gps_waypoint_controller_node
  geometry_msgs rclcpp sensor_msgs std_msgs ugv_subject2_mvp)

install(TARGETS
  ${PROJECT_NAME}_core
  gps_waypoint_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_gps_waypoint_core test/test_gps_waypoint_core.cpp)
  target_link_libraries(test_gps_waypoint_core ${PROJECT_NAME}_core)
  ament_add_pytest_test(test_gps_subject2_launch test/test_gps_subject2_launch.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
