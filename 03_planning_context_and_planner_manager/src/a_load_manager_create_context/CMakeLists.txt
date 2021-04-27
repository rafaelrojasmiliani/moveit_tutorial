cmake_minimum_required(VERSION 3.0.2)
project(a_load_manager_create_context)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  roscpp
)
find_package(Eigen3 REQUIRED)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
add_executable(planning_scene_monitor_node src/planning_scene_monitor.cpp)
target_link_libraries(planning_scene_monitor_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
add_executable(planner_node src/planner.cpp)
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(orchestrator src/orchestrator.cpp)
target_link_libraries(orchestrator
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
