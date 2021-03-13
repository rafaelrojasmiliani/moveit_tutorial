cmake_minimum_required(VERSION 3.0.2)
project(a_interactive_markers)

find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp
                                        rviz_visual_tools)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} include)
link_libraries(${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(a_move_along_x src/a_move_along_x.cpp src/tools.cpp)
add_executable(b_rotate_along_x src/b_rotate_along_x.cpp src/tools.cpp)
add_executable(c_move_xy_plane src/c_move_xy_plane.cpp src/tools.cpp)
add_executable(d_move_3d src/d_move_3d.cpp src/tools.cpp)
add_executable(e_rotate_3d src/e_rotate_3d.cpp src/tools.cpp)
