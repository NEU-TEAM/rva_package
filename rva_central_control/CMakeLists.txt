cmake_minimum_required(VERSION 2.8.3)
project(rva_central_control)

set(PROJECT_SRCS src/rva_central_control.cpp src/param.cpp src/request.cpp)

#Comment this if don't want debug
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
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

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(CATKIN_DEPENDS message_runtime recognize_with_vgg)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(rva_central_control
#   src/${PROJECT_NAME}/rva_central_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(rva_central_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(rva_central_control_node src/rva_central_control_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rva_central_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(rva_central_control_node
#   ${catkin_LIBRARIES}
# )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(rva_central_control ${PROJECT_SRCS})

#Very important for generate message
add_dependencies(rva_central_control rva_central_control_generate_message_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(rva_central_control ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
