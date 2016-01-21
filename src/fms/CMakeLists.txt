cmake_minimum_required(VERSION 2.8.3)
project(fms)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  serial
)

 add_message_files(
   FILES
   Mess.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fms
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs serial
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fms_mavtx src/fms_mavtx.cpp)
target_link_libraries(fms_mavtx ${catkin_LIBRARIES})
add_dependencies(fms_mavtx fms_generate_messages_cpp)

add_executable(fms_mavrx src/fms_mavrx.cpp)
target_link_libraries(fms_mavrx ${catkin_LIBRARIES})
add_dependencies(fms_mavrx fms_generate_messages_cpp)

add_executable(fms_kernel src/fms_kernel.cpp)
target_link_libraries(fms_kernel ${catkin_LIBRARIES})
add_dependencies(fms_kernel fms_generate_messages_cpp)
