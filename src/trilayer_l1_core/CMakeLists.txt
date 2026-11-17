cmake_minimum_required(VERSION 3.16)
project(trilayer_l1_core)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(FILES README.md LICENSE SOURCE_PROVENANCE.json SEMANTIC_PROVENANCE.json THIRD_PARTY_SCOPE.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME l1_grid_contract COMMAND ${CMAKE_COMMAND} -E env
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_grid.py)
  add_test(NAME l1_semantic_contract COMMAND ${CMAKE_COMMAND} -E env
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_semantic.py)
  set_tests_properties(l1_grid_contract l1_semantic_contract PROPERTIES TIMEOUT 30)
endif()
ament_package()
