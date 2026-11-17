cmake_minimum_required(VERSION 3.16)
project(trilayer_clarabel_vendor)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
find_program(TRILAYER_CLARABEL_CARGO NAMES cargo-1.85 cargo DOC "Explicit offline Cargo >=1.84 executable")
find_program(TRILAYER_CLARABEL_RUSTC NAMES rustc-1.85 rustc DOC "Explicit offline Rust >=1.84 executable")
if(NOT TRILAYER_CLARABEL_CARGO OR NOT TRILAYER_CLARABEL_RUSTC)
  message(FATAL_ERROR "Rust/Cargo >=1.84 required. Provide TRILAYER_CLARABEL_CARGO and TRILAYER_CLARABEL_RUSTC. No toolchain download is performed.")
endif()
set(TRILAYER_CLARABEL_TOOLCHAIN_LIBRARY_PATH "" CACHE STRING "Explicit offline shared-library path for extracted toolchain packages; normally empty")
foreach(tool IN ITEMS CARGO RUSTC)
  execute_process(COMMAND ${CMAKE_COMMAND} -E env "LD_LIBRARY_PATH=${TRILAYER_CLARABEL_TOOLCHAIN_LIBRARY_PATH}" "${TRILAYER_CLARABEL_${tool}}" --version OUTPUT_VARIABLE v RESULT_VARIABLE rc)
  if(NOT rc EQUAL 0 OR NOT v MATCHES "([0-9]+\\.[0-9]+\\.[0-9]+)")
    message(FATAL_ERROR "Could not inspect ${tool} version")
  endif()
  if(CMAKE_MATCH_1 VERSION_LESS "1.84.0")
    message(FATAL_ERROR "${tool} >=1.84 required; found ${v}. Default unversioned Jammy toolchain is insufficient; this package does not download a replacement.")
  endif()
endforeach()
set(TRILAYER_CLARABEL_JOBS "2" CACHE STRING "Bounded Cargo build job count")
set(vendor_work "${CMAKE_CURRENT_BINARY_DIR}/vendor_work")
add_custom_command(OUTPUT "${vendor_work}/complete.json"
 COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_vendor.py"
 --source-root "${CMAKE_CURRENT_SOURCE_DIR}" --work "${vendor_work}"
 --cargo "${TRILAYER_CLARABEL_CARGO}" --rustc "${TRILAYER_CLARABEL_RUSTC}" --jobs "${TRILAYER_CLARABEL_JOBS}" --toolchain-library-path "${TRILAYER_CLARABEL_TOOLCHAIN_LIBRARY_PATH}"
 DEPENDS scripts/build_vendor.py SOURCE_MANIFEST.json provenance/FILTER_MANIFEST.json
 sources/clarabel-0.11.1-filtered-python-linux.tar.gz VERBATIM)
add_custom_target(vendor_extension ALL DEPENDS "${vendor_work}/complete.json")
install(DIRECTORY "${vendor_work}/python/" DESTINATION "${PYTHON_INSTALL_DIR}")
install(FILES README.md LICENSE SOURCE_MANIFEST.json DESTINATION share/${PROJECT_NAME})
install(FILES "${vendor_work}/complete.json" DESTINATION share/${PROJECT_NAME} RENAME BUILD_PROVENANCE.json)
install(DIRECTORY third_party_notices/ DESTINATION share/${PROJECT_NAME}/third_party_notices)
install(DIRECTORY supplemental_notices/ DESTINATION share/${PROJECT_NAME}/supplemental_notices)
install(DIRECTORY provenance/ DESTINATION share/${PROJECT_NAME}/provenance)
configure_file(env/vendor_pythonpath.dsv.in vendor_pythonpath.dsv @ONLY)
ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/vendor_pythonpath.dsv")
if(BUILD_TESTING)
 enable_testing()
 # Default tests consume this build's products, never an older installed copy.
 add_test(NAME build_clarabel_contract COMMAND ${CMAKE_COMMAND} -E env
  PYTHONNOUSERSITE=1 "PYTHONPATH=${vendor_work}/python"
  "TRILAYER_EXPECTED_VENDOR_PREFIX=${vendor_work}/python"
  ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_solver.py)
 set_tests_properties(build_clarabel_contract PROPERTIES TIMEOUT 30)
 option(TRILAYER_VENDOR_TEST_INSTALLED "Also test this package after explicit installation (not for pre-install Debian build tests)" OFF)
 if(TRILAYER_VENDOR_TEST_INSTALLED)
  add_test(NAME installed_clarabel_contract COMMAND ${CMAKE_COMMAND} -E env
   PYTHONNOUSERSITE=1 "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}"
   "TRILAYER_EXPECTED_VENDOR_PREFIX=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}"
   ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_solver.py)
  set_tests_properties(installed_clarabel_contract PROPERTIES TIMEOUT 30)
 endif()
endif()
ament_package()
