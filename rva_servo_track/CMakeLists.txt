cmake_minimum_required(VERSION 2.8.3)
project(rva_servo_track)

set(PROJECT_SRCS src/rva_servo_track.cpp src/colortrack.cpp src/camshift.cpp src/utilities)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  rva_central_control
  recognize_with_vgg
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

add_executable(rva_servo_track ${PROJECT_SRCS})
add_dependencies(rva_servo_track ${catkin_EXPORTED_TARGETS})

target_link_libraries(rva_servo_track ${catkin_LIBRARIES} ${OpenCV_LIBS})

