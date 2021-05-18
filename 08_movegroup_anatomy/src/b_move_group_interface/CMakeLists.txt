cmake_minimum_required(VERSION 3.0.2)
project(b_move_group_interface)

find_package(
  catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning_interface
                             moveit_visual_tools roscpp rospy)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/move_group_interface_tutorial.cpp
                                    src/solids.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
