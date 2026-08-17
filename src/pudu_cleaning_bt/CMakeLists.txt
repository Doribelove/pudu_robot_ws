cmake_minimum_required(VERSION 3.8)
project(pudu_cleaning_bt)

find_package(ament_cmake REQUIRED)

install(DIRECTORY behavior_trees config include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
