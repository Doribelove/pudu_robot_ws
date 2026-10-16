cmake_minimum_required(VERSION 3.8)
project(three_d_v1_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenSSL REQUIRED)

add_library(frozen_r1_planner SHARED src/frozen_r1_planner.cpp src/online_r1_planner.cpp)
target_include_directories(frozen_r1_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(frozen_r1_planner
  geometry_msgs nav2_core nav2_costmap_2d nav2_util nav_msgs pluginlib rclcpp
  rclcpp_lifecycle std_msgs tf2 tf2_ros)
target_link_libraries(frozen_r1_planner OpenSSL::Crypto)
target_compile_features(frozen_r1_planner PUBLIC cxx_std_17)
pluginlib_export_plugin_description_file(nav2_core frozen_r1_planner_plugin.xml)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS frozen_r1_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)
install(FILES frozen_r1_planner_plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch rviz behavior_trees DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/prepare_path_bank
  scripts/run_dynamic_scenarios
  scripts/cmd_vel_guard
  scripts/truth_localization
  scripts/seq8_mission
  scripts/online_r1_bridge
  scripts/moving_obstacle
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_contracts test/test_contracts.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_guard test/test_guard.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_mission_state test/test_mission_state.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_runtime_safety test/test_runtime_safety.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_ros_guards test/test_ros_guards.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_online_bridge test/test_online_bridge.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_moving_obstacle test/test_moving_obstacle.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_export_include_directories(include)
ament_export_libraries(frozen_r1_planner)
ament_package()
