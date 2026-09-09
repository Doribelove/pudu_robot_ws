cmake_minimum_required(VERSION 3.8)
project(two_a_v1_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(two_a_v1_r2_planner SHARED src/two_a_v1_r2_planner.cpp)
target_include_directories(two_a_v1_r2_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(two_a_v1_r2_planner
  geometry_msgs nav2_core nav2_costmap_2d nav_msgs pluginlib rclcpp
  rclcpp_lifecycle tf2_ros)
target_compile_features(two_a_v1_r2_planner PUBLIC cxx_std_17)
pluginlib_export_plugin_description_file(nav2_core two_a_v1_r2_planner_plugin.xml)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS two_a_v1_r2_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)
install(FILES two_a_v1_r2_planner_plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch behavior_trees DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/online_r2_bridge scripts/prepare_path_bank
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_contracts test/test_contracts.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_export_include_directories(include)
ament_export_libraries(two_a_v1_r2_planner)
ament_package()
