cmake_minimum_required(VERSION 2.8.3)
project(publish_point_cloud)

set(PROJECT_SRCS src/getsourcecloud.cpp src/publish_point_cloud.cpp)

#Comment this if don't want debug
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  tf
  cv_bridge
  sensor_msgs
  image_transport
  image_geometry
  dynamic_reconfigure
)

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/ppcParameters.cfg)

catkin_package()

include_directories(
  include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(publish_point_cloud ${PROJECT_SRCS})

# Specify libraries to link a library or executable target against
 target_link_libraries(publish_point_cloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

# make sure configure headers are built before any node using them
add_dependencies(publish_point_cloud publish_point_cloud_gencfg)

