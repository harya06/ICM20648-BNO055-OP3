# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "op3_online_walking_module_msgs: 9 messages, 3 services")

set(MSG_I_FLAGS "-Iop3_online_walking_module_msgs:/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(op3_online_walking_module_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg" "geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg" ""
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point:op3_online_walking_module_msgs/KinematicsPose"
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv" "sensor_msgs/JointState:std_msgs/Header:op3_online_walking_module_msgs/JointPose"
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg" "geometry_msgs/Pose2D:op3_online_walking_module_msgs/Step2D"
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg" ""
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg" ""
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg" ""
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg" "geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv" "op3_online_walking_module_msgs/PreviewRequest:op3_online_walking_module_msgs/PreviewResponse"
)

get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg" NAME_WE)
add_custom_target(_op3_online_walking_module_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "op3_online_walking_module_msgs" "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg" "sensor_msgs/JointState:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Services
_generate_srv_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_cpp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Module File
_generate_module_cpp(op3_online_walking_module_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(op3_online_walking_module_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(op3_online_walking_module_msgs_generate_messages op3_online_walking_module_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_online_walking_module_msgs_gencpp)
add_dependencies(op3_online_walking_module_msgs_gencpp op3_online_walking_module_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_online_walking_module_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Services
_generate_srv_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_eus(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Module File
_generate_module_eus(op3_online_walking_module_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(op3_online_walking_module_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(op3_online_walking_module_msgs_generate_messages op3_online_walking_module_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_eus _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_online_walking_module_msgs_geneus)
add_dependencies(op3_online_walking_module_msgs_geneus op3_online_walking_module_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_online_walking_module_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Services
_generate_srv_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_lisp(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Module File
_generate_module_lisp(op3_online_walking_module_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(op3_online_walking_module_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(op3_online_walking_module_msgs_generate_messages op3_online_walking_module_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_online_walking_module_msgs_genlisp)
add_dependencies(op3_online_walking_module_msgs_genlisp op3_online_walking_module_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_online_walking_module_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Services
_generate_srv_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_nodejs(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Module File
_generate_module_nodejs(op3_online_walking_module_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(op3_online_walking_module_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(op3_online_walking_module_msgs_generate_messages op3_online_walking_module_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_online_walking_module_msgs_gennodejs)
add_dependencies(op3_online_walking_module_msgs_gennodejs op3_online_walking_module_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_online_walking_module_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_msg_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Services
_generate_srv_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv"
  "${MSG_I_FLAGS}"
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)
_generate_srv_py(op3_online_walking_module_msgs
  "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
)

### Generating Module File
_generate_module_py(op3_online_walking_module_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(op3_online_walking_module_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(op3_online_walking_module_msgs_generate_messages op3_online_walking_module_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2D.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepCommand.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetKinematicsPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetJointPose.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/Step2DArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewResponse.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/WalkingParam.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/KinematicsPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/PreviewRequest.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/FootStepArray.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/srv/GetPreviewMatrix.srv" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotis/catkin_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg/JointPose.msg" NAME_WE)
add_dependencies(op3_online_walking_module_msgs_generate_messages_py _op3_online_walking_module_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(op3_online_walking_module_msgs_genpy)
add_dependencies(op3_online_walking_module_msgs_genpy op3_online_walking_module_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS op3_online_walking_module_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/op3_online_walking_module_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/op3_online_walking_module_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/op3_online_walking_module_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/op3_online_walking_module_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/op3_online_walking_module_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(op3_online_walking_module_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
