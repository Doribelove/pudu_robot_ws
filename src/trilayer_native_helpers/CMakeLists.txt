cmake_minimum_required(VERSION 3.16)
project(trilayer_native_helpers)
find_package(ament_cmake REQUIRED)
foreach(helper v13_dstar v13_segment_cells v13_extract_path)
  add_library(${helper} SHARED src/${helper}.cpp)
  target_compile_features(${helper} PRIVATE cxx_std_17)
  target_compile_options(${helper} PRIVATE -O3 -ffp-contract=off -fno-fast-math -Wall -Wextra)
  # Keep legacy file names only inside this package's private directory.
  install(TARGETS ${helper} LIBRARY DESTINATION lib/${PROJECT_NAME})
endforeach()
install(FILES LICENSE SOURCE_PROVENANCE.json DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME native_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_native.py
    --library-dir $<TARGET_FILE_DIR:v13_dstar>)
endif()
ament_package()
