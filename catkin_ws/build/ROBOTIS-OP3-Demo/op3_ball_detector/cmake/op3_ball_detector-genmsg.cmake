# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "op3_ball_detector: 2 messages, 2 services")

set(MSG_I_FLAGS "-Iop3_ball_detector:/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(op3_ball_detector_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg" NAME_WE)
add_custom_target(_op3_ball_detector_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_ball_detector" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg" "std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg" NAME_WE)
add_custom_target(_op3_ball_detector_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_ball_detector" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg" ""
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv" NAME_WE)
add_custom_target(_op3_ball_detector_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_ball_detector" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv" "op3_ball_detector/BallDetectorParams"
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv" NAME_WE)
add_custom_target(_op3_ball_detector_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_ball_detector" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv" "op3_ball_detector/BallDetectorParams"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_ball_detector
)
_generate_msg_cpp(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_ball_detector
)

### Generating Services
_generate_srv_cpp(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_ball_detector
)
_generate_srv_cpp(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_ball_detector
)

### Generating Module File
_generate_module_cpp(op3_ball_detector
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_ball_detector
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(op3_ball_detector_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(op3_ball_detector_generate_messages op3_ball_detector_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_cpp _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_cpp _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_cpp _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_cpp _op3_ball_detector_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_ball_detector_gencpp)
add_dependencies(op3_ball_detector_gencpp op3_ball_detector_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_ball_detector_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_ball_detector
)
_generate_msg_eus(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_ball_detector
)

### Generating Services
_generate_srv_eus(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_ball_detector
)
_generate_srv_eus(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_ball_detector
)

### Generating Module File
_generate_module_eus(op3_ball_detector
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_ball_detector
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(op3_ball_detector_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(op3_ball_detector_generate_messages op3_ball_detector_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_eus _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_eus _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_eus _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_eus _op3_ball_detector_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_ball_detector_geneus)
add_dependencies(op3_ball_detector_geneus op3_ball_detector_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_ball_detector_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_ball_detector
)
_generate_msg_lisp(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_ball_detector
)

### Generating Services
_generate_srv_lisp(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_ball_detector
)
_generate_srv_lisp(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_ball_detector
)

### Generating Module File
_generate_module_lisp(op3_ball_detector
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_ball_detector
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(op3_ball_detector_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(op3_ball_detector_generate_messages op3_ball_detector_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_lisp _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_lisp _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_lisp _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_lisp _op3_ball_detector_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_ball_detector_genlisp)
add_dependencies(op3_ball_detector_genlisp op3_ball_detector_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_ball_detector_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_ball_detector
)
_generate_msg_nodejs(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_ball_detector
)

### Generating Services
_generate_srv_nodejs(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_ball_detector
)
_generate_srv_nodejs(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_ball_detector
)

### Generating Module File
_generate_module_nodejs(op3_ball_detector
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_ball_detector
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(op3_ball_detector_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(op3_ball_detector_generate_messages op3_ball_detector_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_nodejs _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_nodejs _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_nodejs _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_nodejs _op3_ball_detector_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_ball_detector_gennodejs)
add_dependencies(op3_ball_detector_gennodejs op3_ball_detector_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_ball_detector_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_ball_detector
)
_generate_msg_py(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_ball_detector
)

### Generating Services
_generate_srv_py(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_ball_detector
)
_generate_srv_py(op3_ball_detector
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_ball_detector
)

### Generating Module File
_generate_module_py(op3_ball_detector
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_ball_detector
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(op3_ball_detector_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(op3_ball_detector_generate_messages op3_ball_detector_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/CircleSetStamped.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_py _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/BallDetectorParams.msg" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_py _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/GetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_py _op3_ball_detector_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/srv/SetParameters.srv" NAME_WE)
add_dependencies(op3_ball_detector_generate_messages_py _op3_ball_detector_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_ball_detector_genpy)
add_dependencies(op3_ball_detector_genpy op3_ball_detector_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_ball_detector_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_ball_detector)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_ball_detector
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(op3_ball_detector_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(op3_ball_detector_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_ball_detector)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_ball_detector
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(op3_ball_detector_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(op3_ball_detector_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_ball_detector)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_ball_detector
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(op3_ball_detector_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(op3_ball_detector_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_ball_detector)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_ball_detector
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(op3_ball_detector_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(op3_ball_detector_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_ball_detector)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_ball_detector\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_ball_detector
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(op3_ball_detector_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(op3_ball_detector_generate_messages_py geometry_msgs_generate_messages_py)
endif()
