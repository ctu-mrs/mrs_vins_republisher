cmake_minimum_required(VERSION 3.8)
project(mrs_vins_republisher)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPENDENCIES
  tf2
  tf2_ros
  mrs_lib
  pluginlib
  rclcpp_components
)

set(LIBRARIES
  MrsVinsRepublisher
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  ${mrs_lib_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(MrsVinsRepublisher_component SHARED src/VinsRepublisher.cpp)

#target_link_libraries(${PROJECT_NAME}_single_node_component
#  ${LOCAL_MVIMPACT_LIBRARIES}
#  ${LOCAL_TOOLKIT_LIBRARIES}
#)

rclcpp_components_register_node(
    MrsVinsRepublisher_component
    PLUGIN "mrs_vins_republisher::VinsRepublisher"
    EXECUTABLE MrsVinsRepublisher
)

# Apply dependencies to the target
ament_target_dependencies(MrsVinsRepublisher_component
  ${DEPENDENCIES}
)

# Link OpenCV libraries
#target_link_libraries(MrsVinsRepublisher_component
#  ${OpenCV_LIBRARIES}
#  ${PCL_LIBRARIES}
#  ${flame_LIBRARIES}
# ${Sophus_LIBRARIES}
#)

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

ament_export_targets(
  export_MrsVinsRepublisher
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
