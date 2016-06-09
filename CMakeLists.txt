cmake_minimum_required(VERSION 2.8.3)
project(can_bus_ros_version)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings -g ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-l:src/libicsneo.a ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-lftdi1 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-lpthread ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
    FILES
    CanBusMsgStamped.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_bus_node
  src/can_bus_node.cpp
  src/cicsneoVI.cpp
  src/icsneoLinuxAPI.cpp
  src/OCriticalSection.cpp
  src/OEvent.cpp
  src/OSAbstraction.cpp
  src/OThread.cpp
#  src/can_bus.cpp
#  src/CFTDILib.cpp
)
#add_library(libicsneo STATIC IMPORTED)
#SET_TARGET_PROPERTIES(libicsneo PROPERTIES
#    IMPORTED_LOCATION /home/rcac/catkin_ws/src/can_bus/can_bus_ros_version/src/libicsneo.a)
#find_library(LIB_LOCATION ./src/libicsneo.a)
#target_link_libraries(can_bus_node ${CMAKE_SOURCE_DIR}/can_bus/can_bus_ros_version/src/libicsneo.a)
target_link_libraries(can_bus_node ${catkin_LIBRARIES})
target_link_libraries(can_bus_node ftdi1)
