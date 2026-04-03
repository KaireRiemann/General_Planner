cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching traj_opt traj_utils 
#  DEPENDS system_lib
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  ../tasks/task_factory.cpp
  ../tasks/state_to_state_task.cpp
  ../tasks/tracking_task.cpp
  ../tasks/perching_task.cpp
  ../compiler/problem_compiler.cpp
  ../compiler/reference_builder.cpp
  ../compiler/feasible_set_builder.cpp
  ../compiler/seed_builder.cpp
  ../frontend/guide_path_service.cpp
  ../frontend/corridor_service.cpp
  ../frontend/visible_region_service.cpp
  ../frontend/terminal_manifold_service.cpp
  ../frontend/warm_start_service.cpp
  ../engine/planner_engine.cpp
  ../solver/state_to_state_initializer.cpp
  ../runtime/task_executor.cpp
  ../runtime/local_target_selector.cpp
  ../runtime/plan_monitor.cpp
  ../runtime/replan_trigger.cpp
  ../runtime/context_builder.cpp
  )
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  )
add_dependencies(ego_planner_node ${catkin_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
add_dependencies(traj_server ${catkin_EXPORTED_TARGETS})

add_executable(tracking_target_sim src/tracking_target_sim.cpp)
target_link_libraries(tracking_target_sim ${catkin_LIBRARIES})
add_dependencies(tracking_target_sim ${catkin_EXPORTED_TARGETS})
