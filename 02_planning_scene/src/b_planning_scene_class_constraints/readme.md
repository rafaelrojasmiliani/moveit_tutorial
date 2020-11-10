# Planning Scene Example 02: Constraints detection

The PlanningScene class also includes easy to use function calls for checking constraints.
The constraints can be of two types:
1. constraints chosen from the `kinematic_constraints::KinematicConstraint` set: i.e. 
    - `kinematic_constraints::JointConstraint`
    - `kinematic_constraints::PositionConstraint`
    - `kinematic_constraints::OrientationConstraint`
    - `kinematic_constraints::VisibilityConstraint`
2. user defined constraints specified through a callback.
# What does this example do?
In this example we instantiate a `PlanningScene` with the robot and workspace defined in this repository in `/common_packages`.
Then, we use the this `PlanningScene` To test a single constrant

## The launch file

1. Set up parameters
4. Launch the node

## The node code
1. **Initialization of the ros node**
2. **Initializatio of the MoveIt robot model and the planning scene**
```C++
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr my_robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(my_robot_model);
```
3. **Initialization of the constraint** and require to return the links in collision. We also set a maximum number of objects in collision
```C++
    const robot_model::JointModelGroup* joint_group = kinematic_model_actual.getJointModelGroup("myrobotplanninggroup");
    std::string link_name = joint_model_group->getLinkModelNames().back();

  geometry_msgs::PoseStamped desired_pose;
  desired_pose.header.frame_id = "world";
  desired_pose.pose.orientation.w = 1.0;
  desired_pose.pose.position.x = 0.3;
  desired_pose.pose.position.y = -0.185;
  desired_pose.pose.position.z = 0.5;
  moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(link_name, desired_pose);
```
4. **TEst he constraint**
```
  bool constrained = planning_scene.isStateConstrained(kinematic_model_actual, goal_constraint);
```

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

## Kinematic constraints

The base class for custon MoveIt constraints is `kinematic_constraints::KinematicConstraint` [defined here](https://github.com/ros-planning/moveit/blob/ff552bf861609f99ca97a7e173fcbeb0c03e9f45/moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h#L78) and [implemented here](https://github.com/ros-planning/moveit/blob/ff552bf861609f99ca97a7e173fcbeb0c03e9f45/moveit_core/kinematic_constraints/src/kinematic_constraint.cpp#L61).
- `kinematic_constraints::JointConstraint`
- `kinematic_constraints::PositionConstraint`
- `kinematic_constraints::OrientationConstraint`
- `kinematic_constraints::VisibilityConstraint`


The Ros message `moveit_msgs::Constraints`is [defined here](https://github.com/ros-planning/moveit_msgs/blob/melodic-devel/msg/Constraints.msg).
- `name` of type `string` 
- `joint_constraints` of type `JointConstraint[]` 
- `position_constraints` of type `PositionConstraint[]` 
    - `header` of type `Header`
    - `link_name` of type `string`
    - `target_point_offset` of type `geometry_msgs/Vector3`
    - `constraint_region` of type `BoundingVolume`
    - `weigh` of type `float64`
- `orientation_constraints` of type `OrientationConstraint[]` 
    - `header` of type `Header`
    - `orientation` of type `geometry_msgs/Quaternion`
    - `link_name` of type `string`
    - `absolute_x_axis_tolerance` of type `float64`
    - `absolute_y_axis_tolerance` of type `float64`
    - `absolute_z_axis_tolerance` of type `float64`
    - `weight` of type `float64`
- `visibility_constraints` of type `VisibilityConstraint[]`

The `moveit_msgs::BoundingVolume`
- `primitives` of type `shape_msgs/SolidPrimitive[]`
- `primitive_poses` of type `geometry_msgs/Pose[]`
- `meshes` of type `shape_msgs/Mesh[]`
- `mesh_poses` of type `geometry_msgs/Pose[]`
