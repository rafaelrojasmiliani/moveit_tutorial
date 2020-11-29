cmake_minimum_required(VERSION 3.0.2)
project(random_objects)

find_package(catkin REQUIRED COMPONENTS moveit_core
                                        moveit_ros_planning_interface roscpp)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(random_objects_service src/randon_object_service.cpp
                                      src/collision_objects.cpp)

target_link_libraries(random_objects_service ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
