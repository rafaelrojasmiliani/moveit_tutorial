cmake_minimum_required(VERSION 3.0.2)
project(b_interactive_marker_robot_control)

find_package(
  catkin REQUIRED
  COMPONENTS interactive_markers moveit_core moveit_ros_planning_interface
             moveit_visual_tools rviz_visual_tools)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} include)
add_executable(interactivity_tutorial src/tools.cpp src/my_example.cpp
                                      src/my_interactive_robot.cpp)
target_link_libraries(interactivity_tutorial ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})
