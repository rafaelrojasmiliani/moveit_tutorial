

## Creating the package

1. Create the catking package
```CMake
catkin create pkg a_robot_model_example_01 --catkin-deps roscpp moveit_core moveit_ros_planning_interface --system-deps Eigen3
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


## Movit pointers and structs

MoveIT has macros which automatice define smart pointes for all its classes. 
This is done with the `MOVEIT_CLASS_FORWARD` [defined here](https://github.com/ros-planning/moveit/blob/64663d97458810eefd0986bab3604c23767dd119/moveit_core/macros/include/moveit/macros/class_forward.h) and [here](https://github.com/ros-planning/moveit/blob/64663d97458810eefd0986bab3604c23767dd119/moveit_core/macros/include/moveit/macros/declare_ptr.h) as
```C++
#define MOVEIT_CLASS_FORWARD(C)\
  class C;\
  MOVEIT_DECLARE_PTR(C, C);

#define MOVEIT_DECLARE_PTR(Name, Type)\
  typedef std::shared_ptr<Type> Name##Ptr;\
  typedef std::shared_ptr<const Type> Name##ConstPtr;\
  typedef std::weak_ptr<Type> Name##WeakPtr;\
  typedef std::weak_ptr<const Type> Name##ConstWeakPtr;\
  typedef std::unique_ptr<Type> Name##UniquePtr;\
  typedef std::unique_ptr<const Type> Name##ConstUniquePtr;
```
