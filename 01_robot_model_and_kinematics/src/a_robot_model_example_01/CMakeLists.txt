cmake_minimum_required(VERSION 3.0.2)
project(a_robot_model_example_01)

find_package(catkin REQUIRED COMPONENTS moveit_core
                                        moveit_ros_planning_interface roscpp)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/a_robot_model_example_01_node.cpp)
add_executable(b_robot_model_example_02_node
               src/b_robot_model_example_02_node.cpp src/solids.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
target_link_libraries(b_robot_model_example_02_node ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
