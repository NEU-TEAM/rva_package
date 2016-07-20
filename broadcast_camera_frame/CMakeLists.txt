cmake_minimum_required(VERSION 2.8.3)
project(broadcast_camera_frame)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2_ros
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES broadcast_camera_frame
#  CATKIN_DEPENDS roscpp tf tf2_ros
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
)

# Declare a cpp executable
 add_executable(broadcast_camera_frame src/broadcast_camera_frame.cpp)

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
 add_dependencies(broadcast_camera_frame broadcast_camera_frame_generate_messages_cpp)

# Specify libraries to link a library or executable target against
 target_link_libraries(broadcast_camera_frame
   ${catkin_LIBRARIES}
 )

