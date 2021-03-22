cmake_minimum_required(VERSION 3.0.2)
project(a_trajectory_control)

find_package(catkin REQUIRED COMPONENTS moveit_core
                                        moveit_ros_planning_interface roscpp)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(trajectory_manager_node src/trajectory_manager_impl.cpp)

target_link_libraries(trajectory_manager_node ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
