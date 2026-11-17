cmake_minimum_required(VERSION 3.16)
project(trilayer_osqp_vendor)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
set(TRILAYER_OSQP_BUILD_PYTHONPATH "" CACHE PATH "Explicit offline build-only Python dependency directory, e.g. extracted Jammy setuptools-scm")
option(TRILAYER_OSQP_EXPERIMENTAL_ARM64 "Explicitly allow unvalidated Linux aarch64 source build" OFF)
set(osqp_arch_args "")
if(TRILAYER_OSQP_EXPERIMENTAL_ARM64)
  list(APPEND osqp_arch_args --allow-experimental-arm64)
endif()
set(vendor_work "${CMAKE_CURRENT_BINARY_DIR}/vendor_work")
add_custom_command(OUTPUT "${vendor_work}/complete.json"
  COMMAND ${CMAKE_COMMAND} -E env PYTHONNOUSERSITE=1
    "PYTHONPATH=${TRILAYER_OSQP_BUILD_PYTHONPATH}"
    ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_vendor.py"
    --source-root "${CMAKE_CURRENT_SOURCE_DIR}" --work "${vendor_work}" ${osqp_arch_args}
  DEPENDS scripts/build_vendor.py SOURCES_AND_LICENSES.json
    sources/osqp-0.6.7.post3.tar.gz sources/qdldl-0.1.7.post5.tar.gz
  VERBATIM)
add_custom_target(vendor_extensions ALL DEPENDS "${vendor_work}/complete.json")
install(DIRECTORY "${vendor_work}/python/" DESTINATION "${PYTHON_INSTALL_DIR}")
install(FILES README.md LICENSE SOURCES_AND_LICENSES.json DESTINATION share/${PROJECT_NAME})
install(FILES "${vendor_work}/complete.json" DESTINATION share/${PROJECT_NAME} RENAME BUILD_PROVENANCE.json)
install(DIRECTORY licenses/ DESTINATION share/${PROJECT_NAME}/licenses)
configure_file(env/vendor_pythonpath.dsv.in vendor_pythonpath.dsv @ONLY)
ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/vendor_pythonpath.dsv")
if(BUILD_TESTING)
 enable_testing()
 # Default tests consume this build's products, never an older installed copy.
 add_test(NAME build_osqp_qdldl_contract COMMAND ${CMAKE_COMMAND} -E env
  PYTHONNOUSERSITE=1 "PYTHONPATH=${vendor_work}/python"
  "TRILAYER_EXPECTED_VENDOR_PREFIX=${vendor_work}/python"
  ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_solvers.py)
 set_tests_properties(build_osqp_qdldl_contract PROPERTIES TIMEOUT 30)
 option(TRILAYER_VENDOR_TEST_INSTALLED "Also test this package after explicit installation (not for pre-install Debian build tests)" OFF)
 if(TRILAYER_VENDOR_TEST_INSTALLED)
  add_test(NAME installed_osqp_qdldl_contract COMMAND ${CMAKE_COMMAND} -E env
   PYTHONNOUSERSITE=1 "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}"
   "TRILAYER_EXPECTED_VENDOR_PREFIX=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}"
   ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_solvers.py)
  set_tests_properties(installed_osqp_qdldl_contract PROPERTIES TIMEOUT 30)
 endif()
endif()
ament_package()
