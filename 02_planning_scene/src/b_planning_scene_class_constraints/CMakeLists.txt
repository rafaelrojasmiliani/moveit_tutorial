cmake_minimum_required(VERSION 3.0.2)
project(b_planning_scene_class_constraints)
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
add_executable(${PROJECT_NAME}_node src/b_planning_scene_class_constraints_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
