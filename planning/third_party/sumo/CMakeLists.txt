cmake_minimum_required(VERSION 2.8.3)
project(sumo)

find_package(catkin REQUIRED)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
 )

include_directories(
    include
    src/
    src/foreign/tcpip/
    src/traci-server/
)

add_library(sumo src/utils/traci/TraCIAPI.cpp
                 src/foreign/tcpip/socket.cpp
                 src/foreign/tcpip/storage.cpp
                 )

