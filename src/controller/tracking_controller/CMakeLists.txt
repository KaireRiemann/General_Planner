cmake_minimum_required(VERSION 3.0.4)
project(tracking_controller)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  quadrotor_msgs
  mavros_msgs
  tf2_ros
  #message_generation
)

# add_message_files(
#   FILES
#   Target.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 #LIBRARIES ${PROJECT_NAME}_node
 CATKIN_DEPENDS message_runtime quadrotor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/tracking_controller.cpp)

add_executable(${PROJECT_NAME}_node src/tracking_ctl_node.cpp
src/tracking_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${quadrotor_msgs_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )





