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
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/c_planning_scene_monitor_node.cpp)
add_executable(c_planning_scene_api_node src/c_planning_scene_api_node.cpp src/solids.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
target_link_libraries(c_planning_scene_api_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
