

## Creating the package

1. Create the catking package
```CMake
catkin create pkg 01_robot_model_example_01 --catkin-deps roscpp moveit_core moveit_ros_planning_interface --system-deps Eigen3
```

2. In `CMakeLists.txt` add the executable indicated name of the executable and source file with `main`
```CMake
add_executable(${PROJECT_NAME}_node src/01_robot_model_example_01_node.cpp)
```
3. In `CMakeLists.txt` add the libraries to link agains
```CMake
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
```
