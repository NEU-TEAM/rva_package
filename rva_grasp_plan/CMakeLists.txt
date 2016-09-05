cmake_minimum_required(VERSION 2.8.3)
project(rva_grasp_plan)

set(PROJECT_SRCS src/rva_grasp_plan.cpp src/makeplan)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  pcl_ros
  recognize_with_vgg
  cloud_get_object
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

add_message_files(
   FILES
   Grasp_Info.msg
)

 generate_messages(
   DEPENDENCIES
   recognize_with_vgg
   geometry_msgs
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS
   message_runtime
   recognize_with_vgg
   geometry_msgs
   std_msgs
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable(rva_grasp_plan ${PROJECT_SRCS})

#Very important for generate message
add_dependencies(rva_grasp_plan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rva_grasp_plan ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
