cmake_minimum_required(VERSION 3.8)
project(ugv_subject2_mvp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(pure_pursuit_controller src/pure_pursuit_controller.cpp)
target_include_directories(pure_pursuit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(stall_boost_controller src/stall_boost_controller.cpp)
target_include_directories(stall_boost_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(waypoint_file_loader src/waypoint_file_loader.cpp)
target_include_directories(waypoint_file_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(waypoint_controller_node src/waypoint_controller_node.cpp)
target_link_libraries(waypoint_controller_node
  pure_pursuit_controller stall_boost_controller waypoint_file_loader)
ament_target_dependencies(waypoint_controller_node
  geometry_msgs nav_msgs rclcpp rcl_interfaces tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS pure_pursuit_controller stall_boost_controller
  waypoint_file_loader waypoint_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pure_pursuit_controller test/test_pure_pursuit_controller.cpp)
  target_link_libraries(test_pure_pursuit_controller pure_pursuit_controller)
  ament_add_gtest(test_waypoint_file_loader test/test_waypoint_file_loader.cpp)
  target_link_libraries(test_waypoint_file_loader waypoint_file_loader)
  ament_add_gtest(test_stall_boost_controller test/test_stall_boost_controller.cpp)
  target_link_libraries(test_stall_boost_controller stall_boost_controller)
endif()

ament_export_include_directories(include)
ament_export_libraries(
  pure_pursuit_controller stall_boost_controller waypoint_file_loader)
ament_package()
