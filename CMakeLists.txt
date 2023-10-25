cmake_minimum_required(VERSION 2.8.3)
project(vins_republisher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  mrs_lib
  nodelet
)

catkin_package(
 # INCLUDE_DIRS include
 LIBRARIES VinsRepublisher
 CATKIN_DEPENDS roscpp tf2 tf2_ros mrs_lib
 # DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(VinsRepublisher src/VinsRepublisher.cpp)

# add_executable(VinsRepublisher src/VinsRepublisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(VinsRepublisher
  ${catkin_LIBRARIES}
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
