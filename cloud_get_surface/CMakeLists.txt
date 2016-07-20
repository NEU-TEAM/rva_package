cmake_minimum_required(VERSION 2.8.3)
project(cloud_get_surface)

set(PROJECT_SRCS src/cloud_get_surface.cpp src/utilities.cpp src/findplane.cpp)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  Point_Cloud_Vector.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(cloud_get_surface ${PROJECT_SRCS})

#Very important for generate message
add_dependencies(cloud_get_surface cloud_get_surface_generate_message_cpp)

 target_link_libraries(cloud_get_surface
   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
 )
