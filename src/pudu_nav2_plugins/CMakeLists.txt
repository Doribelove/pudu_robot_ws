cmake_minimum_required(VERSION 3.8)
project(pudu_nav2_plugins)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

add_library(pudu_back_up_to_free_space SHARED
  src/back_up_to_free_space.cpp
)
target_include_directories(pudu_back_up_to_free_space PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pudu_back_up_to_free_space
  geometry_msgs
  nav2_behaviors
  nav2_core
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  tf2
)

pluginlib_export_plugin_description_file(nav2_core pudu_nav2_plugins.xml)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS pudu_back_up_to_free_space
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES pudu_nav2_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(pudu_back_up_to_free_space)
ament_export_dependencies(
  geometry_msgs
  nav2_behaviors
  nav2_core
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  tf2
)

ament_package()
