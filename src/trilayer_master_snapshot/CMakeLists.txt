cmake_minimum_required(VERSION 3.16)
project(trilayer_master_snapshot)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenSSL REQUIRED)
add_library(${PROJECT_NAME} SHARED src/master_snapshot.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME} rclcpp nav2_costmap_2d nav2_msgs nav2_util pluginlib)
target_link_libraries(${PROJECT_NAME} OpenSSL::Crypto)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(FILES LICENSE README.md PROVENANCE.json DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  set(test_stage "${CMAKE_CURRENT_BINARY_DIR}/test_install")
  configure_file(test/stage_install.cmake.in stage_install.cmake @ONLY)
  add_test(NAME master_private_stage_setup COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/stage_install.cmake)
  set_tests_properties(master_private_stage_setup PROPERTIES FIXTURES_SETUP master_private_stage TIMEOUT 30)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(std_srvs REQUIRED)
  find_package(std_msgs REQUIRED)
  add_executable(costmap_test_host test/costmap_host.cpp)
  target_compile_features(costmap_test_host PRIVATE cxx_std_17)
  ament_target_dependencies(costmap_test_host rclcpp nav2_costmap_2d nav2_util std_srvs std_msgs)
  install(TARGETS costmap_test_host RUNTIME DESTINATION lib/${PROJECT_NAME})
  add_test(NAME master_layer_lifecycle_contract COMMAND ${CMAKE_COMMAND} -E env
    ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1
    AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/smoke_ros.py
    --host $<TARGET_FILE:costmap_test_host>)
  add_test(NAME p6_exact_ack_master_contract COMMAND ${CMAKE_COMMAND} -E env
    ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1
    AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/p6_ack/real_master_ack.py
    --host $<TARGET_FILE:costmap_test_host>)
  set_tests_properties(master_layer_lifecycle_contract p6_exact_ack_master_contract PROPERTIES TIMEOUT 40 FIXTURES_REQUIRED master_private_stage
    ENVIRONMENT "TRILAYER_TEST_PREFIX=${test_stage}")
endif()
ament_package()
