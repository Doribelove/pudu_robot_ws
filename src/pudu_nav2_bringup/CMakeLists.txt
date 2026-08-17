cmake_minimum_required(VERSION 3.8)
project(pudu_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY behavior_trees config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
