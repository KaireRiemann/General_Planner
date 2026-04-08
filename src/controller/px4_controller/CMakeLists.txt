cmake_minimum_required(VERSION 3.0.2)
project(px4_controller)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++14 -O3 -Wall -g)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
  uav_utils
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs mavros_msgs quadrotor_msgs uav_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(px4_controller_node
  src/px4_controller_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_dependencies(px4_controller_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(px4_controller_node
  ${catkin_LIBRARIES}
)
