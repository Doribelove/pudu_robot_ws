cmake_minimum_required(VERSION 3.8)
project(trilayer_costmap_bridge)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/costmap_bridge DESTINATION lib/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME full_snapshot_dds_smoke COMMAND ${CMAKE_COMMAND} -E env
    ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/smoke_ros.py)
  add_test(NAME p6_delta_dds_smoke COMMAND ${CMAKE_COMMAND} -E env
    ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/delta_ros.py)
  set_tests_properties(full_snapshot_dds_smoke p6_delta_dds_smoke PROPERTIES TIMEOUT 30)
endif()
ament_package()
