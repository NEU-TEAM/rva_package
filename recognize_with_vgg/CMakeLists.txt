cmake_minimum_required(VERSION 2.8.3)
project(recognize_with_vgg)

set(PROJECT_SRCS src/recognize_with_vgg.cpp src/vgg_process src/utilities.cpp)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cloud_get_object
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  tf
  message_generation
)

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)

find_package(Caffe REQUIRED)

find_package(PCL 1.7 REQUIRED)

add_message_files(
   FILES
   Object_Pixels_Boundaries.msg
   VGG_Recognized_Object.msg
   Target_Recognized_Object.msg
)

 generate_messages(
   DEPENDENCIES
   cloud_get_object
   sensor_msgs
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS 
   message_runtime
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${Caffe_DEFINITIONS} ${PCL_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Caffe_INCLUDE_DIRS}
)

add_executable(recognize_with_vgg ${PROJECT_SRCS})

#Very important for generate message
add_dependencies(recognize_with_vgg recognize_with_vgg_generate_message_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(recognize_with_vgg ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Caffe_LIBRARIES} ${PCL_LIBRARIES})

