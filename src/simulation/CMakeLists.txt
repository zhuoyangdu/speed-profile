cmake_minimum_required(VERSION 2.8.3)

project(simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sumo
  common
)

ADD_DEFINITIONS(-std=c++11)

add_subdirectory(proto)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS}
                    ${sumo_INCLUDES_DIRS})

add_executable(simulation_node src/simulation_node.cpp)

target_link_libraries(simulation_node
  ${catkin_LIBRARIES}
  simulation_proto
)

