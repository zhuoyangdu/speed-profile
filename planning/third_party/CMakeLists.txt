cmake_minimum_required(VERSION 2.8.3)
project(third_party)

find_package(catkin REQUIRED)

add_subdirectory(sumo)

catkin_package(
    LIBRARIES third_party
    CATKIN_DEPENDS
 )
