cmake_minimum_required(VERSION 3.8)
project(pudu_coverage_planner)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config include launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
