# Robot Model, Robot State, Robot Model Loader and Robot Inverce and Direct Kinematics

## Robot Model Loader

MoveIt provedes an interface to load models a `robot_model_loader::RobotModelLoader` [defined here](https://github.com/ros-planning/moveit/blob/802e596a2283b64f4582d802c5f79e1f3d57def0/moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h) and [implemented here](https://github.com/ros-planning/moveit/blob/7ad2bc7b86dad08061d98668ba34feba54bb05cc/moveit_ros/planning/robot_model_loader/src/robot_model_loader.cpp).

- [`moveit_ros/planning/rdf_loader`]
    - `rdf_loader::RDFLoader` [defined here](https://github.com/ros-planning/moveit/blob/802e596a2283b64f4582d802c5f79e1f3d57def0/moveit_ros/planning/rdf_loader/include/moveit/rdf_loader/rdf_loader.h) and [implemented here](https://github.com/ros-planning/moveit/blob/cb5945b69f4dfbd1e5819a2b7bee3e256aee766c/moveit_ros/planning/rdf_loader/src/rdf_loader.cpp). Loads the contnet of `"robot_description"` with `rdf_loader::RDFLoader::initString`. and Loads the contnet of `"robot_description_semantic"` with `srdf::Model::initString`.
    


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

- [`srdf`](https://wiki.ros.org/srdf)
    - `srdf::Model`

```
  RobotModelLoader(const Options& opt = Options());
  RobotModelLoader(const std::string& robot_description, bool load_kinematics_solvers = true);
```

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

