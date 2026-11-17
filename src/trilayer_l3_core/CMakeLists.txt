cmake_minimum_required(VERSION 3.8)
project(trilayer_l3_core)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
add_library(semantic_sweep SHARED src/swept_rect.cpp)
add_library(curve_rectangles SHARED src/curve_rectangles.cpp)
foreach(target semantic_sweep curve_rectangles)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -O3 -ffp-contract=off -fno-fast-math)
  install(TARGETS ${target} LIBRARY DESTINATION lib/${PROJECT_NAME})
endforeach()
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE PROVENANCE.md DEPENDENCIES.md GEOMETRY_PROVENANCE.json THIRD_PARTY_SCOPE.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(test_stage "${CMAKE_CURRENT_BINARY_DIR}/test_install")
  configure_file(test/stage_install.cmake.in stage_install.cmake @ONLY)
  add_test(NAME l3_private_stage_setup COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/stage_install.cmake)
  set_tests_properties(l3_private_stage_setup PROPERTIES FIXTURES_SETUP l3_private_stage TIMEOUT 30)

  set(TRILAYER_L3_SOLVER_DEPS "" CACHE PATH "Optional explicit frozen solver dependencies for validation only")
  set(solver_args)
  if(TRILAYER_L3_SOLVER_DEPS)
    list(APPEND solver_args --solver-deps ${TRILAYER_L3_SOLVER_DEPS})
  endif()
  add_test(NAME l3_numeric_contract COMMAND ${CMAKE_COMMAND} -E env
    AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_numeric.py ${solver_args})
  add_test(NAME l3_optimize_certify_contract COMMAND ${CMAKE_COMMAND} -E env
    AMENT_PREFIX_PATH=${test_stage}:$ENV{AMENT_PREFIX_PATH}
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
    ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_optimize.py ${solver_args})
  set_tests_properties(l3_numeric_contract l3_optimize_certify_contract PROPERTIES TIMEOUT 30 FIXTURES_REQUIRED l3_private_stage)
endif()
ament_package()
