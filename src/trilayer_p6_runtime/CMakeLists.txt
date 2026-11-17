cmake_minimum_required(VERSION 3.16)
project(trilayer_p6_runtime)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
foreach(extension nav2_effective_costmap endpoint_geometry_r3)
  Python3_add_library(_${extension} MODULE WITH_SOABI src/${extension}.cpp)
  target_compile_features(_${extension} PRIVATE cxx_std_17)
  install(TARGETS _${extension} LIBRARY DESTINATION
    share/${PROJECT_NAME}/resources/p6/base/runtime/arena_evaluation)
endforeach()
find_package(rclcpp REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
add_executable(curve_nav2_stack src/native/nav2_stack/main.cpp)
ament_target_dependencies(curve_nav2_stack rclcpp nav2_map_server nav2_planner nav2_lifecycle_manager)
install(TARGETS curve_nav2_stack RUNTIME DESTINATION share/${PROJECT_NAME}/resources/p6/base/runtime/base/native)
foreach(helper cold_dstar curve_rectangles curve_rectangles_center failure_endpoint failure_heading goal_dijkstra pose_batch pose_groups request_ack request_edges request_membership tail_astar v12_failure_heading v12_grid_astar v12_palette v13_astar v13_dstar)
  add_library(${helper} SHARED src/native/${helper}.cpp)
  target_compile_features(${helper} PRIVATE cxx_std_17)
  target_compile_options(${helper} PRIVATE -O3 -ffp-contract=off -fno-fast-math)
  install(TARGETS ${helper} LIBRARY DESTINATION share/${PROJECT_NAME}/resources/p6/base/runtime/base/native)
endforeach()
install(DIRECTORY patches DESTINATION share/${PROJECT_NAME})
install(DIRECTORY licenses DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
install(FILES LICENSE README.md SOURCE_PROVENANCE.json LICENSE_SCOPE.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  set(test_stage "${CMAKE_CURRENT_BINARY_DIR}/test_install")
  configure_file(test/stage_install.cmake.in stage_install.cmake @ONLY)
  add_test(NAME p6_private_stage_setup COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/stage_install.cmake)
  set_tests_properties(p6_private_stage_setup PROPERTIES FIXTURES_SETUP p6_private_stage TIMEOUT 30)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_library(request_ack_scalar_test SHARED src/native/request_ack.cpp)
  target_compile_features(request_ack_scalar_test PRIVATE cxx_std_17)
  target_compile_options(request_ack_scalar_test PRIVATE -O3 -ffp-contract=off -fno-fast-math)
  target_compile_definitions(request_ack_scalar_test PRIVATE TRILAYER_ACK_FORCE_SCALAR)
  add_test(NAME p6_ack_portable_scalar_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ack_portability.py
    $<TARGET_FILE:request_ack> $<TARGET_FILE:request_ack_scalar_test>)
  set_tests_properties(p6_ack_portable_scalar_contract PROPERTIES TIMEOUT 20)
  add_test(NAME p6_import_contract COMMAND ${CMAKE_COMMAND} -E env
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_import.py
    ${test_stage}/share/${PROJECT_NAME}/resources)
  set_tests_properties(p6_import_contract PROPERTIES TIMEOUT 30)
  add_test(NAME p6_api_validation_contract COMMAND ${CMAKE_COMMAND} -E env
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_api_validation.py)
  set_tests_properties(p6_api_validation_contract PROPERTIES TIMEOUT 10)
  add_test(NAME p6_semantic_validation_contract COMMAND ${CMAKE_COMMAND} -E env
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_semantic_validation.py)
  set_tests_properties(p6_semantic_validation_contract PROPERTIES TIMEOUT 10)

  add_test(NAME p6_research_guards_contract COMMAND ${CMAKE_COMMAND} -E env
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_research_guards.py
    ${test_stage}/share/${PROJECT_NAME}/resources)
  set_tests_properties(p6_research_guards_contract PROPERTIES TIMEOUT 30)


  set(TRILAYER_P6_TEST_SOLVER_ROOT "" CACHE PATH "Explicit source-built OSQP/QDLDL/Clarabel site-packages for isolated service tests")
  if(TRILAYER_P6_TEST_SOLVER_ROOT)
    add_test(NAME p6_dynamic_session_contract COMMAND ${CMAKE_COMMAND} -E env
      AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
      ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_dynamic_session.py
      --solver-root ${TRILAYER_P6_TEST_SOLVER_ROOT})
    set_tests_properties(p6_dynamic_session_contract PROPERTIES TIMEOUT 70 RUN_SERIAL TRUE RESOURCE_LOCK p6_domain184)
    find_program(TIMEOUT_EXECUTABLE timeout REQUIRED)
    foreach(session_mode map semantic)
      add_test(NAME p6_public_session_${session_mode}_contract COMMAND ${CMAKE_COMMAND} -E env
        AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
        PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
        ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=179 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
        ${TIMEOUT_EXECUTABLE} --signal=INT --kill-after=5s 60s
        ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_public_session.py
        ${TRILAYER_P6_TEST_SOLVER_ROOT} ${session_mode})
      set_tests_properties(p6_public_session_${session_mode}_contract PROPERTIES TIMEOUT 70 RUN_SERIAL TRUE RESOURCE_LOCK p6_domain179)
    endforeach()
    foreach(case straight shifted small_budget obstacle)
      add_test(NAME p6_service_${case}_contract COMMAND ${CMAKE_COMMAND} -E env
        AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
        PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
        ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=179 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
        ${TIMEOUT_EXECUTABLE} --signal=INT --kill-after=5s 60s
        ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_service_request.py
        --case ${case} --solver-root ${TRILAYER_P6_TEST_SOLVER_ROOT})
      set_tests_properties(p6_service_${case}_contract PROPERTIES TIMEOUT 70 RUN_SERIAL TRUE RESOURCE_LOCK p6_domain179)
    endforeach()
    add_test(NAME p6_cleanup_failure_path_contract COMMAND ${CMAKE_COMMAND} -E env
      AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
      ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=179 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
      ${TIMEOUT_EXECUTABLE} --signal=INT --kill-after=5s 60s
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cleanup_failure_path.py
      ${TRILAYER_P6_TEST_SOLVER_ROOT})
    set_tests_properties(p6_cleanup_failure_path_contract PROPERTIES TIMEOUT 70 RUN_SERIAL TRUE RESOURCE_LOCK p6_domain179)
    add_test(NAME p6_semantic_interrupt_cleanup_contract COMMAND ${CMAKE_COMMAND} -E env
      AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
      ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=179 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
      ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_interrupt_cleanup.py
      ${TRILAYER_P6_TEST_SOLVER_ROOT})
    set_tests_properties(p6_semantic_interrupt_cleanup_contract PROPERTIES TIMEOUT 35 RUN_SERIAL TRUE RESOURCE_LOCK p6_domain179)
    foreach(semantic_case baseline one_way_forward one_way_reverse forbidden_transition parking_valid parking_missing parking_anchor_mismatch parking_wrong_side)
      add_test(NAME p6_semantic_${semantic_case}_contract COMMAND ${CMAKE_COMMAND} -E env
        AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
        PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
        ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=179 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
        ${TIMEOUT_EXECUTABLE} --signal=INT --kill-after=5s 60s
        ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_semantic_service.py
        ${TRILAYER_P6_TEST_SOLVER_ROOT} ${semantic_case})
      set_tests_properties(p6_semantic_${semantic_case}_contract PROPERTIES TIMEOUT 70 RUN_SERIAL TRUE RESOURCE_LOCK p6_domain179)
    endforeach()
  endif()

  get_property(package_tests DIRECTORY PROPERTY TESTS)
  foreach(test_name IN LISTS package_tests)
    if(NOT test_name STREQUAL "p6_private_stage_setup")
      set_tests_properties(${test_name} PROPERTIES FIXTURES_REQUIRED p6_private_stage)
    endif()
  endforeach()
endif()
ament_package()
