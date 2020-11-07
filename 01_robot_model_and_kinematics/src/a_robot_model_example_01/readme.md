# Robot Model, Robot State, Robot Model Loader and Robot Inverce and Direct Kinematics

## Robot Model

The model of the robot is wraped in the class `moveit::core::RobotModel` [defined here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_core/robot_model/include/moveit/robot_model/robot_model.h) and [implemented here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_core/robot_model/src/robot_model.cpp).

`moveit::core::RobotModel` implementes the URDF library

- [`geometric_shapes`](http://wiki.ros.org/geometric_shapes) classes and methods to represent and work with primitive geometric shapes
    - `shapes::Shape`
    - `shapes::Sphere`
    - `shapes::Box`
    - `shapes::Cylinder`
    - `shapes::Mesh`
    - `shapes::createMeshFromResource`
- [`urdf`](http://wiki.ros.org/urdf)
    - `urdf::ModelInterface`
    - `urdf::Link`
    - `urdf::Joint`
    - `urdf::Pose`
    - `urdf::Collision`
    - `urdf::Visual`
    - `urdf::Geometry`
    - `urdf::Mesh`
    - etc.


# Methods used in this example

# How to set up a Moveit robot model
Setting up to start using the RobotModel class is very easy. In
general, you will find that most higher-level components will
return a shared pointer to the RobotModel. You should always use
that when possible. In this example, we will start with such a
shared pointer and discuss only the basic API. You can have a
look at the actual code API for these classes to get more
information about how to use more features provided by these
classes.
                                                                 
We will start by instantiating a
`RobotModelLoader` object, which will look up the robot description on the ROS parameter server and construct a
