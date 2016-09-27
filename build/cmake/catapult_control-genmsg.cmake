# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "catapult_control: 1 messages, 0 services")

set(MSG_I_FLAGS "-Icatapult_control:/home/erle/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/catapult_control/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(catapult_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/erle/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/catapult_control/msg/Catapult.msg" NAME_WE)
add_custom_target(_catapult_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "catapult_control" "/home/erle/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/catapult_control/msg/Catapult.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(catapult_control
  "/home/erle/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/catapult_control/msg/Catapult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/catapult_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(catapult_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/catapult_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(catapult_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(catapult_control_generate_messages catapult_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erle/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/catapult_control/msg/Catapult.msg" NAME_WE)
add_dependencies(catapult_control_generate_messages_cpp _catapult_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(catapult_control_gencpp)
add_dependencies(catapult_control_gencpp catapult_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS catapult_control_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(catapult_control
  "/home/erle/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/catapult_control/msg/Catapult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/catapult_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(catapult_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/catapult_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(catapult_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(catapult_control_generate_messages catapult_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erle/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/catapult_control/msg/Catapult.msg" NAME_WE)
add_dependencies(catapult_control_generate_messages_lisp _catapult_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(catapult_control_genlisp)
add_dependencies(catapult_control_genlisp catapult_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS catapult_control_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(catapult_control
  "/home/erle/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/catapult_control/msg/Catapult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catapult_control
)

### Generating Services

### Generating Module File
_generate_module_py(catapult_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catapult_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(catapult_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(catapult_control_generate_messages catapult_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erle/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/catapult_control/msg/Catapult.msg" NAME_WE)
add_dependencies(catapult_control_generate_messages_py _catapult_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(catapult_control_genpy)
add_dependencies(catapult_control_genpy catapult_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS catapult_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/catapult_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/catapult_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(catapult_control_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/catapult_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/catapult_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(catapult_control_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catapult_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catapult_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/catapult_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(catapult_control_generate_messages_py std_msgs_generate_messages_py)
