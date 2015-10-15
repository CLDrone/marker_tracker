cmake_minimum_required(VERSION 2.8.3)
project(marker_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(marker_tracker_node src/marker_tracker_node.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(marker_tracker_node
   ${catkin_LIBRARIES}
 )

