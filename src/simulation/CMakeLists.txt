cmake_minimum_required(VERSION 3.4)

project(simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  common
)

ADD_DEFINITIONS(-std=c++11)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS}
                    ${sumo_INCLUDES_DIRS}
)

# add_executable(simulation_node src/simulation_node.cpp)

#target_link_libraries(simulation_node
#  ${catkin_LIBRARIES}
#)


