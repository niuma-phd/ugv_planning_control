cmake_minimum_required(VERSION 3.8)
project(ugv_subject2_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lio_process_supervisor
    test/test_lio_process_supervisor.py)
  ament_add_pytest_test(test_lidar_extrinsics_launch
    test/test_lidar_extrinsics_launch.py)
  ament_add_pytest_test(test_convert_wgs84_waypoints
    test/test_convert_wgs84_waypoints.py)
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(PROGRAMS
  scripts/lio_process_supervisor.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
