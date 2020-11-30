cmake_minimum_required(VERSION 3.0.2)
project(a_planning_pipeline)
find_package(catkin REQUIRED COMPONENTS moveit_core
                                        moveit_ros_planning_interface roscpp)
find_package(Eigen3 REQUIRED)
catkin_package(
  # INCLUDE_DIRS include LIBRARIES a_planning_pipeline CATKIN_DEPENDS
  # moveit_core moveit_ros_planning_interface roscpp DEPENDS Eigen3
)
include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/a_planning_pipeline_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
add_executable(orchestrator src/orchestrator.cpp)
target_link_libraries(orchestrator ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
