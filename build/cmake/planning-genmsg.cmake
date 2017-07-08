# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "planning: 4 messages, 0 services")

set(MSG_I_FLAGS "-Iplanning:/home/parallels/workspace/catkin_ws/planning/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(planning_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/ObstacleMap.msg" NAME_WE)
add_custom_target(_planning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "planning" "/home/parallels/workspace/catkin_ws/planning/msg/ObstacleMap.msg" "planning/DynamicObstacle"
)

get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/Trajectory.msg" NAME_WE)
add_custom_target(_planning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "planning" "/home/parallels/workspace/catkin_ws/planning/msg/Trajectory.msg" "planning/Pose"
)

get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg" NAME_WE)
add_custom_target(_planning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "planning" "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg" ""
)

get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg" NAME_WE)
add_custom_target(_planning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "planning" "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/ObstacleMap.msg"
  "${MSG_I_FLAGS}"
  "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/planning
)
_generate_msg_cpp(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/planning
)
_generate_msg_cpp(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/planning
)
_generate_msg_cpp(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/planning
)

### Generating Services

### Generating Module File
_generate_module_cpp(planning
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/planning
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(planning_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(planning_generate_messages planning_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/ObstacleMap.msg" NAME_WE)
add_dependencies(planning_generate_messages_cpp _planning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/Trajectory.msg" NAME_WE)
add_dependencies(planning_generate_messages_cpp _planning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg" NAME_WE)
add_dependencies(planning_generate_messages_cpp _planning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg" NAME_WE)
add_dependencies(planning_generate_messages_cpp _planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(planning_gencpp)
add_dependencies(planning_gencpp planning_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS planning_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/ObstacleMap.msg"
  "${MSG_I_FLAGS}"
  "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/planning
)
_generate_msg_lisp(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/planning
)
_generate_msg_lisp(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/planning
)
_generate_msg_lisp(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/planning
)

### Generating Services

### Generating Module File
_generate_module_lisp(planning
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/planning
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(planning_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(planning_generate_messages planning_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/ObstacleMap.msg" NAME_WE)
add_dependencies(planning_generate_messages_lisp _planning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/Trajectory.msg" NAME_WE)
add_dependencies(planning_generate_messages_lisp _planning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg" NAME_WE)
add_dependencies(planning_generate_messages_lisp _planning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg" NAME_WE)
add_dependencies(planning_generate_messages_lisp _planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(planning_genlisp)
add_dependencies(planning_genlisp planning_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS planning_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/ObstacleMap.msg"
  "${MSG_I_FLAGS}"
  "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/planning
)
_generate_msg_py(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/planning
)
_generate_msg_py(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/planning
)
_generate_msg_py(planning
  "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/planning
)

### Generating Services

### Generating Module File
_generate_module_py(planning
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/planning
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(planning_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(planning_generate_messages planning_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/ObstacleMap.msg" NAME_WE)
add_dependencies(planning_generate_messages_py _planning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/Trajectory.msg" NAME_WE)
add_dependencies(planning_generate_messages_py _planning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/Pose.msg" NAME_WE)
add_dependencies(planning_generate_messages_py _planning_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/parallels/workspace/catkin_ws/planning/msg/DynamicObstacle.msg" NAME_WE)
add_dependencies(planning_generate_messages_py _planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(planning_genpy)
add_dependencies(planning_genpy planning_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS planning_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/planning
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(planning_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/planning
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(planning_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/planning)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/planning\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/planning
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(planning_generate_messages_py std_msgs_generate_messages_py)
