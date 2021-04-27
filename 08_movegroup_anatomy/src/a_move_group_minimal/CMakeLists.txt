cmake_minimum_required(VERSION 3.0.2)
project(a_move_group_minimal)

find_package(catkin REQUIRED COMPONENTS moveit_core
                                        moveit_ros_planning_interface roscpp)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
