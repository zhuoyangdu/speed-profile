cmake_minimum_required(VERSION 2.8.3)
project(planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  common
  )

find_package(Boost REQUIRED COMPONENTS system)

INCLUDE(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/environment_config.proto
                                            proto/planning_debug.proto
                                            proto/planning_config.proto)
ADD_LIBRARY(proto ${PROTO_HDRS} ${PROTO_SRCS})

ADD_DEFINITIONS(-std=c++11)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime visualization_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
)

add_executable(planning_node
               src/planning_node.cpp
               src/planning_visualization.cpp
               src/route.cpp
               src/rrt.cpp
               src/spline.cpp
               src/obstacles.cpp)

add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                  ${catkin_EXPORTED_TARGETS})

target_link_libraries(planning_node
   ${catkin_LIBRARIES}
   proto ${PROTOBUF_LIBRARY}
)

