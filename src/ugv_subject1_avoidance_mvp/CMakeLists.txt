cmake_minimum_required(VERSION 3.8)
project(ugv_subject1_avoidance_mvp)

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
find_package(std_msgs REQUIRED)

add_library(local_avoidance src/local_avoidance.cpp)
target_include_directories(local_avoidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(local_avoidance_node src/local_avoidance_node.cpp)
target_link_libraries(local_avoidance_node local_avoidance)
ament_target_dependencies(local_avoidance_node geometry_msgs nav_msgs rclcpp std_msgs)

install(TARGETS local_avoidance local_avoidance_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_local_avoidance test/test_local_avoidance.cpp)
  target_link_libraries(test_local_avoidance local_avoidance)
endif()

ament_export_include_directories(include)
ament_export_libraries(local_avoidance)
ament_package()
