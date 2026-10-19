# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target nav2_msgs::nav2_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${nav2_msgs_TARGETS}.
if(nav2_msgs_TARGETS AND NOT TARGET nav2_msgs::nav2_msgs)
  add_library(nav2_msgs::nav2_msgs INTERFACE IMPORTED)
  set_target_properties(nav2_msgs::nav2_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${nav2_msgs_TARGETS}")
endif()
