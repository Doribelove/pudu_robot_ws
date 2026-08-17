cmake_minimum_required(VERSION 3.8)
project(pudu_navigation_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

install(DIRECTORY config launch scenarios test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
