# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(bno055_imu_CONFIG_INCLUDED)
  return()
endif()
set(bno055_imu_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(bno055_imu_SOURCE_PREFIX /home/robotis/catkin_ws/src/bno055_imu)
  set(bno055_imu_DEVEL_PREFIX /home/robotis/catkin_ws/devel_isolated/bno055_imu)
  set(bno055_imu_INSTALL_PREFIX "")
  set(bno055_imu_PREFIX ${bno055_imu_DEVEL_PREFIX})
else()
  set(bno055_imu_SOURCE_PREFIX "")
  set(bno055_imu_DEVEL_PREFIX "")
  set(bno055_imu_INSTALL_PREFIX /home/robotis/catkin_ws/install_isolated)
  set(bno055_imu_PREFIX ${bno055_imu_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'bno055_imu' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(bno055_imu_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/robotis/catkin_ws/src/bno055_imu/include " STREQUAL " ")
  set(bno055_imu_INCLUDE_DIRS "")
  set(_include_dirs "/home/robotis/catkin_ws/src/bno055_imu/include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'robotis <robotis@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${bno055_imu_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'bno055_imu' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'bno055_imu' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/robotis/catkin_ws/src/bno055_imu/${idir}'.  ${_report}")
    endif()
    _list_append_unique(bno055_imu_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND bno055_imu_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND bno055_imu_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND bno055_imu_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/robotis/catkin_ws/devel_isolated/bno055_imu/lib;/home/robotis/catkin_ws/devel_isolated/serial/lib;/home/robotis/catkin_ws/devel_isolated/scilab_optimization/lib;/home/robotis/catkin_ws/devel_isolated/ros_mpg321_player/lib;/home/robotis/catkin_ws/devel_isolated/ros_madplay_player/lib;/home/robotis/catkin_ws/devel_isolated/robotis_utility/lib;/home/robotis/catkin_ws/devel_isolated/robotis_op3_tools/lib;/home/robotis/catkin_ws/devel_isolated/robotis_op3_msgs/lib;/home/robotis/catkin_ws/devel_isolated/robotis_op3_demo/lib;/home/robotis/catkin_ws/devel_isolated/robotis_op3_common/lib;/home/robotis/catkin_ws/devel_isolated/robotis_op3/lib;/home/robotis/catkin_ws/devel_isolated/op3_manager/lib;/home/robotis/catkin_ws/devel_isolated/open_cr_module/lib;/home/robotis/catkin_ws/devel_isolated/op3_walking_module/lib;/home/robotis/catkin_ws/devel_isolated/op3_tuning_module/lib;/home/robotis/catkin_ws/devel_isolated/op3_online_walking_module/lib;/home/robotis/catkin_ws/devel_isolated/op3_offset_tuner_server/lib;/home/robotis/catkin_ws/devel_isolated/op3_localization/lib;/home/robotis/catkin_ws/devel_isolated/op3_direct_control_module/lib;/home/robotis/catkin_ws/devel_isolated/op3_base_module/lib;/home/robotis/catkin_ws/devel_isolated/op3_kinematics_dynamics/lib;/home/robotis/catkin_ws/devel_isolated/op3_head_control_module/lib;/home/robotis/catkin_ws/devel_isolated/op3_balance_control/lib;/home/robotis/catkin_ws/devel_isolated/robotis_math/lib;/home/robotis/catkin_ws/devel_isolated/op3_action_editor/lib;/home/robotis/catkin_ws/devel_isolated/robotis_controller/lib;/home/robotis/catkin_ws/devel_isolated/op3_action_module/lib;/home/robotis/catkin_ws/devel_isolated/robotis_framework_common/lib;/home/robotis/catkin_ws/devel_isolated/robotis_framework/lib;/home/robotis/catkin_ws/devel_isolated/robotis_device/lib;/home/robotis/catkin_ws/devel_isolated/op3_read_write_demo/lib;/home/robotis/catkin_ws/devel_isolated/op3_gui_demo/lib;/home/robotis/catkin_ws/devel_isolated/robotis_controller_msgs/lib;/home/robotis/catkin_ws/devel_isolated/ps3joy/lib;/home/robotis/catkin_ws/devel_isolated/op3_web_setting_tool/lib;/home/robotis/catkin_ws/devel_isolated/op3_walking_module_msgs/lib;/home/robotis/catkin_ws/devel_isolated/op3_tuner_client/lib;/home/robotis/catkin_ws/devel_isolated/op3_tuning_module_msgs/lib;/home/robotis/catkin_ws/devel_isolated/op3_online_walking_module_msgs/lib;/home/robotis/catkin_ws/devel_isolated/op3_offset_tuner_client/lib;/home/robotis/catkin_ws/devel_isolated/op3_offset_tuner_msgs/lib;/home/robotis/catkin_ws/devel_isolated/op3_navigation/lib;/home/robotis/catkin_ws/devel_isolated/op3_gazebo/lib;/home/robotis/catkin_ws/devel_isolated/op3_description/lib;/home/robotis/catkin_ws/devel_isolated/op3_camera_setting_tool/lib;/home/robotis/catkin_ws/devel_isolated/op3_ball_detector/lib;/home/robotis/catkin_ws/devel_isolated/op3_action_module_msgs/lib;/home/robotis/catkin_ws/devel_isolated/joystick_drivers/lib;/home/robotis/catkin_ws/devel_isolated/joy/lib;/home/robotis/catkin_ws/devel_isolated/humanoid_planner_2d/lib;/home/robotis/catkin_ws/devel_isolated/humanoid_navigation/lib;/home/robotis/catkin_ws/devel_isolated/humanoid_localization/lib;/home/robotis/catkin_ws/devel_isolated/footstep_planner/lib;/home/robotis/catkin_ws/devel_isolated/gridmap_2d/lib;/home/robotis/catkin_ws/devel_isolated/face_detection/lib;/home/robotis/catkin_ws/devel_isolated/dynamixel_sdk/lib;/home/robotis/catkin_ws/devel_isolated/comms/lib;/home/robotis/catkin_ws/devel/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(bno055_imu_LIBRARY_DIRS ${lib_path})
      list(APPEND bno055_imu_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'bno055_imu'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND bno055_imu_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(bno055_imu_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${bno055_imu_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;std_msgs;sensor_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 bno055_imu_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${bno055_imu_dep}_FOUND)
      find_package(${bno055_imu_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${bno055_imu_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(bno055_imu_INCLUDE_DIRS ${${bno055_imu_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(bno055_imu_LIBRARIES ${bno055_imu_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${bno055_imu_dep}_LIBRARIES})
  _list_append_deduplicate(bno055_imu_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(bno055_imu_LIBRARIES ${bno055_imu_LIBRARIES})

  _list_append_unique(bno055_imu_LIBRARY_DIRS ${${bno055_imu_dep}_LIBRARY_DIRS})
  list(APPEND bno055_imu_EXPORTED_TARGETS ${${bno055_imu_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${bno055_imu_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
