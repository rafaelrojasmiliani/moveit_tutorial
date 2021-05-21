cmake_minimum_required(VERSION 3.0.2)
project(d_moveit_graps)

find_package(
  catkin REQUIRED
  COMPONENTS moveit_core moveit_grasps moveit_ros_planning_interface
             moveit_visual_tools roscpp rospy)

find_package(Eigen3 REQUIRED)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES d_moveit_graps CATKIN_DEPENDS moveit_core
  # moveit_graps moveit_ros_planning_interface moveit_visual_tools roscpp rospy
  # DEPENDS Eigen3
)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/d_moveit_grasps_node.cpp src/solids.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
