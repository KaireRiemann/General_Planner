project(marsim_local_sensing_node)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    cmake_modules
    pcl_ros
    pcl_conversions
    sensor_msgs
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    cv_bridge
    tf2_ros)

catkin_package(
    DEPENDS Eigen3 OpenCV PCL
    CATKIN_DEPENDS roscpp roslib pcl_ros pcl_conversions sensor_msgs geometry_msgs nav_msgs quadrotor_msgs cv_bridge tf2_ros
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(
  marsim_pcl_render_node_target
  src/pointcloud_render_node.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/FOV_Checker/FOV_Checker.cpp
)

target_link_libraries(
  marsim_pcl_render_node_target
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

set_target_properties(marsim_pcl_render_node_target PROPERTIES OUTPUT_NAME pcl_render_node)
