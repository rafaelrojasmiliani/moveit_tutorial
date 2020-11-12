cmake_minimum_required(VERSION 3.0.2)
project(c_planning_scene_monitor)
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
add_executable(${PROJECT_NAME}_node src/c_planning_scene_monitor_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
