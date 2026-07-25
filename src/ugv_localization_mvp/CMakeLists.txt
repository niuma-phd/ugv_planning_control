cmake_minimum_required(VERSION 3.8)
project(ugv_localization_mvp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/odom_guard_core.cpp
  src/ros_time.cpp
  src/transform_math.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}_core builtin_interfaces geometry_msgs tf2 tf2_geometry_msgs)

add_executable(lio_odom_adapter_node src/lio_odom_adapter_node.cpp)
target_link_libraries(lio_odom_adapter_node ${PROJECT_NAME}_core)
ament_target_dependencies(lio_odom_adapter_node rclcpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(map_odom_manager_node src/map_odom_manager_node.cpp)
target_link_libraries(map_odom_manager_node ${PROJECT_NAME}_core)
ament_target_dependencies(
  map_odom_manager_node rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(odom_guard_node src/odom_guard_node.cpp)
target_link_libraries(odom_guard_node ${PROJECT_NAME}_core)
ament_target_dependencies(odom_guard_node rclcpp nav_msgs std_msgs std_srvs)

install(TARGETS
  ${PROJECT_NAME}_core
  lio_odom_adapter_node
  map_odom_manager_node
  odom_guard_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odom_guard_core test/test_odom_guard_core.cpp)
  target_link_libraries(test_odom_guard_core ${PROJECT_NAME}_core)
  ament_add_gtest(test_transform_math test/test_transform_math.cpp)
  target_link_libraries(test_transform_math ${PROJECT_NAME}_core)
  ament_target_dependencies(test_transform_math geometry_msgs tf2 tf2_geometry_msgs)
  ament_add_gtest(test_ros_time test/test_ros_time.cpp)
  target_link_libraries(test_ros_time ${PROJECT_NAME}_core)
  ament_target_dependencies(test_ros_time builtin_interfaces)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(builtin_interfaces geometry_msgs tf2 tf2_geometry_msgs)
ament_package()
