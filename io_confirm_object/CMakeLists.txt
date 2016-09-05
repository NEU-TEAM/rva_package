cmake_minimum_required(VERSION 2.8.3)
project(io_confirm_object)

set(PROJECT_SRCS src/io_confirm_object.cpp)

#Comment this if don't want debug
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  cloud_get_object
  rva_central_control
  recognize_with_vgg
)

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES io_confirm_object
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_generation pcl_ros roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(io_confirm_object ${PROJECT_SRCS})
add_dependencies(io_confirm_object ${catkin_EXPORTED_TARGETS})

 target_link_libraries(io_confirm_object ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
