cmake_minimum_required(VERSION 2.8.3)
project(fake_object)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_executable(fake_object src/fake_object.cpp)
target_link_libraries(fake_object ${catkin_LIBRARIES})
add_dependencies(fake_object ${catkin_EXPORTED_TARGETS})
