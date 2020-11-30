
# How did we create this package?

1. Create the catkin package
```CMake
catkin create pkg PROJECT_NAME --catkin-deps roscpp moveit_core moveit_ros_planning_interface --system-deps Eigen3
```

2. In `CMakeLists.txt` add the executable indicated name of the executable and source file with `main`
```CMake
add_executable(${PROJECT_NAME}_node src/SOURCE.cpp)
```
3. In `CMakeLists.txt` add the libraries to link against
```CMake
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
```
## Required paramenters

In order to load the motion planner plugin this examples as the moveit configuration package `move_group` node requires a parameter with the name of the plugin.
- `planning_plugin`  e.g. `ompl_interface/OMPLPlanner`
-
```XML
<param name="planning_plugin" value="ompl_interface/OMPLPlanner"/>
  <param name="request_adapters" value="default_planner_request_adapters/AddTimeParameterization
                                        default_planner_request_adapters/FixWorkspaceBounds
                                        default_planner_request_adapters/FixStartStateBounds
                                        default_planner_request_adapters/FixStartStateCollision
                                        default_planner_request_adapters/FixStartStatePathConstraints" />
```
