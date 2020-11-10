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
Then, we use the this `PlanningScene` instance to check the collisions of the robot with itself and the environment.
The example runs an `rviz` instance where the configuration of the robot is shown.
The example also prints information about the collision for each configuration.

## The launch file

1. Set up parameters
2. Launch a `joint_state_publisher` which repeats the state of the robot.
3. Launch a `robot_state_publisher`
4. Launch the node
5. Launch Rviz

## The node code
1. **Initialization of the ros node**
2. **Initializatio of the MoveIt robot model and the planning scene**
```C++
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr my_robot_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(my_robot_model);
```
3. **Initialization of the input and output of the collision detector** and require to return the links in collision. We also set a maximum number of objects in collision
```C++
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_detection::CollisionResult collision_result;
```
4. **While the node is running**
    1. **Get the actual instance of the robot state** (This is required because the state may change) **and the jointgroup for planning**
    ```C++
        robot_state::RobotState& kinematic_model_actual = planning_scene.getCurrentStateNonConst();
        const robot_model::JointModelGroup* joint_group = kinematic_model_actual.getJointModelGroup("myrobotplanninggroup");
    ```
    2. **Reset the collision request**
    ```C++
        collision_request.group_name = "myrobotplanninggroup";
        collision_result.clear();
    ```
    3. **Set the robot in a random configuration**
    ```C++
        kinematic_model_actual.setToRandomPositions(joint_group);
    ```
    4. **call the collision checker**
    ```C++
        planning_scene.checkSelfCollision(collision_request, collision_result);
    ```
    5. **If there are collisions then print the links that are colliding**
    ```C++
      collision_detection::CollisionResult::ContactMap::const_iterator it;
      for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
        ROS_INFO("    Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
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
