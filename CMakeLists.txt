cmake_minimum_required(VERSION 3.8)
project(mrs_vins_republisher)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPENDENCIES
  tf2
  tf2_ros
  mrs_lib
  pluginlib
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  ${mrs_lib_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(MrsVinsRepublisher_component SHARED 
  src/VinsRepublisher.cpp
)

# Apply dependencies to the target
ament_target_dependencies(MrsVinsRepublisher_component
  ${DEPENDENCIES}
)

# Register the component
rclcpp_components_register_node(
  MrsVinsRepublisher_component
  PLUGIN "vins_republisher::VinsRepublisher"
  EXECUTABLE MrsVinsRepublisher
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS MrsVinsRepublisher_component
  EXPORT export_MrsVinsRepublisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install headers if needed
#install(
#  DIRECTORY include/
#  DESTINATION include/
#)

ament_export_targets(
  export_MrsVinsRepublisher HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()