cmake_minimum_required(VERSION 2.8.3)
project(radar_odometry_ros)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  tf
  cv_bridge

  std_msgs
  novatel_oem7_msgs
  autoku_msgs
  autoku_utils
  pcl_conversions
  jsk_recognition_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS}
  ../../../interface
)

# # Define an option to enable or disable TBB
# option(USE_TBB "Enable TBB support" OFF)

# if(USE_TBB)
#   # find_package(TBB REQUIRED)
#   add_definitions(-DUSE_TBB)
#   # include_directories(${TBB_INCLUDE_DIRS})
#   # link_directories(${TBB_LIBRARY_DIRS})
#   # set(TBB_LIBS ${TBB_LIBRARIES})
# endif()

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/radar_odometry.cpp
#   src/algorithm/ego_motion_compensation.cpp
#   src/algorithm/registration.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} 
    src/radar_odometry_node.cpp
    src/radar_odometry.cpp
    src/algorithm/adaptive_threshold.cpp
    src/algorithm/ego_motion_compensation.cpp
    src/algorithm/registration.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    autoku_msgs_generate_messages_cpp
    novatel_oem7_msgs_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)