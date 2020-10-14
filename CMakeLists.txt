cmake_minimum_required(VERSION 2.8.3)
project(vins_republish)

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
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vins_republish
 CATKIN_DEPENDS roscpp tf2 tf2_ros mrs_lib
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(VinsRepublisher src/republish.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(VinsRepublisher
  ${catkin_LIBRARIES}
)
