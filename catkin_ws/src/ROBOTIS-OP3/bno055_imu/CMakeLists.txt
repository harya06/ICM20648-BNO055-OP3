cmake_minimum_required(VERSION 3.0.2)
project(bno055_imu)

## Compile flags
add_compile_options(-std=c++17)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  serial
)

## Serial library (pastikan sudah install libserial-dev)
find_package(PkgConfig REQUIRED)
# pkg_check_modules(SERIAL REQUIRED libserial)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SERIAL_INCLUDE_DIRS}
)

add_executable(bno055_imu_node
  src/bno055_imu_node/bno055_imu_node.cpp
  src/bno055_imu_node/bno055_imu.cpp
)

target_link_libraries(bno055_imu_node
  ${catkin_LIBRARIES}
  ${SERIAL_LIBRARIES}
  stdc++fs
)
