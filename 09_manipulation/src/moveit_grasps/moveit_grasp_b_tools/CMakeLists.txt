cmake_minimum_required(VERSION 3.0.2)
project(moveit_grasp_b_tools)

find_package(
  catkin REQUIRED
  COMPONENTS eigen_conversions
             geometry_msgs
             moveit_core
             moveit_grasps
             moveit_ros_planning_interface
             moveit_visual_tools
             roscpp
             rospy
             tf2
             tf2_geometry_msgs
             tf2_ros)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(
  ${PROJECT_NAME}_node
  src/example.cpp src/objectdescription.cpp src/simplemanipulator.cpp
  src/solids.cpp src/urdf_to_collision_object.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
