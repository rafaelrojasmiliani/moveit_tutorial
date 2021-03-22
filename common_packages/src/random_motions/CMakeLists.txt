cmake_minimum_required(VERSION 3.0.2)
project(random_motions)

find_package(catkin REQUIRED COMPONENTS moveit_core
                                        moveit_ros_planning_interface roscpp)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(random_motion_plan_service src/random_motion_plan_service.cpp)

target_link_libraries(random_motion_plan_service ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
