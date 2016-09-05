cmake_minimum_required(VERSION 2.8.3)
project(rva_central_control)

set(PROJECT_SRCS src/rva_central_control.cpp src/param.cpp src/request.cpp)

#Comment this if don't want debug
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  message_generation
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  recognize_with_vgg
)

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   target.msg
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs
 )

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/globalParam.cfg)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
  recognize_with_vgg
)

include_directories(${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(rva_central_control ${PROJECT_SRCS})

#Very important for generate message
add_dependencies(rva_central_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(rva_central_control ${PROJECT_NAME}_gencfg)

target_link_libraries(rva_central_control ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
