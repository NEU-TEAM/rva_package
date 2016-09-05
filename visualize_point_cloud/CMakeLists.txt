cmake_minimum_required(VERSION 2.8.3)
project(visualize_point_cloud)

set(PROJECT_SRCS src/visualize_point_cloud.cpp )

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
  tf
  cloud_get_surface
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visualize_point_cloud
#  CATKIN_DEPENDS pcl_ros roscpp std_msgs tf
#  DEPENDS system_lib
)

find_package(PCL 1.7 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(visualize_point_cloud ${PROJECT_SRCS})
add_dependencies(visualize_point_cloud ${catkin_EXPORTED_TARGETS})

target_link_libraries(visualize_point_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})




