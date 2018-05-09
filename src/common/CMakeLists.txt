cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

ADD_DEFINITIONS(-std=c++11)

add_message_files(
    FILES
    Pose.msg
    DynamicObstacle.msg
    ObstacleMap.msg
    Trajectory.msg
    )

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
