cmake_minimum_required(VERSION 2.8.3)
project(sumo)

ADD_DEFINITIONS(-std=c++11)

set(sumo_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES sumo
    # CFG_EXTRAS ${CMAKE_SOURCE_DIR}/cmake/flags.cmake
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE SUMO_SRC ${CMAKE_CURRENT_SOURCE_DIR}
    FOLLOW_SYMLINKS ./src/*.cpp)

add_library(sumo ${SUMO_SRC})

target_link_libraries(sumo ${catkin_LIBRARIES})

install(TARGETS sumo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable(testclient
                ${SUMO_SRC})

