cmake_minimum_required(VERSION 2.8.3)
project(mappingturtles)

## Find catkin macros and libraries
find_package(catkin REQUIRED 
  geometry_msgs
  nav_msgs
  roscpp
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
# ground truth publisher
add_executable(ground_truth_publisher src/ground_truth_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ground_truth_publisher
   ${catkin_LIBRARIES}
)
