cmake_minimum_required(VERSION 2.8.3)
project(cloud_get_object)

set(PROJECT_SRCS src/cloud_get_object.cpp src/findobject.cpp)

#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  pcl_ros
  roscpp
  std_msgs
  tf
  message_generation
  cloud_get_surface
)

find_package(PCL 1.7 REQUIRED)

add_message_files(
   FILES
   Object_Image_Pixels.msg
   Objects_Pixels_Vector.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(cloud_get_object ${PROJECT_SRCS})

#Very important for generate message
add_dependencies(cloud_get_object cloud_get_object_generate_message_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(cloud_get_object ${catkin_LIBRARIES} ${PCL_LIBRARIES})
