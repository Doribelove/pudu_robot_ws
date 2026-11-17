cmake_minimum_required(VERSION 3.16)
project(trilayer_global_planner)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/plan DESTINATION lib/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME global_cli_contract COMMAND ${CMAKE_COMMAND} -E env
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cli.py)
  set_tests_properties(global_cli_contract PROPERTIES TIMEOUT 30)
  set(TRILAYER_ENTRY_TEST_SOLVER_ROOT "" CACHE PATH "Explicit source-built solver site-packages for actual entry tests")
  if(TRILAYER_ENTRY_TEST_SOLVER_ROOT)
    foreach(case obstacle small_budget custom timeout)
      add_test(NAME entry_${case}_contract COMMAND ${CMAKE_COMMAND} -E env
        PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
        ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_entry_runtime.py
        --solver-root ${TRILAYER_ENTRY_TEST_SOLVER_ROOT} --case ${case})
      set_tests_properties(entry_${case}_contract PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE RESOURCE_LOCK trilayer_domain_181)
    endforeach()
  endif()
endif()
ament_package()
