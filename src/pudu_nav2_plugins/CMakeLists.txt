cmake_minimum_required(VERSION 3.8)
project(pudu_nav2_plugins)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
