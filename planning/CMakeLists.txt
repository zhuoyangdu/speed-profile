cmake_minimum_required(VERSION 2.8.3)
project(planning)

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

add_executable(planning_node
               src/rrt/planning_node.cpp
               src/rrt/rrt.cpp
               src/rrt/spline.cpp
               src/rrt/obstacles.cpp)

add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(planning_node
   ${catkin_LIBRARIES}
)

