cmake_minimum_required(VERSION 3.0.2)
project(d_moveit_grasp)

find_package(
  catkin REQUIRED
  COMPONENTS moveit_core
             moveit_grasps
             moveit_ros_planning_interface
             moveit_visual_tools
             roscpp
             rospy
             tf
             tf_conversions
             eigen_conversions
             geometry_msgs)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_node src/example.cpp src/grasp_tools.cpp
                                    src/solids.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                      ${Eigen3_LIBRARIES})
