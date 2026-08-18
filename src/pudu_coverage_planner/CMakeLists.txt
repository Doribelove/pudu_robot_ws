cmake_minimum_required(VERSION 3.8)
project(pudu_coverage_planner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/area_coverage.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config include launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_coverage_geometry test/test_coverage_geometry.py)
  ament_add_pytest_test(
    test_area_coverage_helpers test/test_area_coverage_helpers.py)
endif()

ament_package()
