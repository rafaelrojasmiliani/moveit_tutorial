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

## SRDF

The SRDF or Semantic Robot Description Format complement the URDF and specifies joint groups, default robot configurations, additional collision checking information, and additional transforms that may be needed to completely specify the robot’s pose. 
The recommended way to generate a SRDF is using the MoveIt! Setup Assistant.

- **Virtual Joints** are additional joints need to be defined to specify the pose of the root link of the robot with respect to a world coordinate system. E.g., a mobile robot is specified using a planar virtual joint that attaches the world coordinate frame to the frame of the robot. A fixed robot should be attached to the world using a fixed joint.

- **Passive Joints** are unactuated joints on a robot, e.g. passive casters on a differential drive robots. They are specified separately in the SRDF to make sure that different components in the motion planning or control pipelines know that the joints cannot be directly controlled.

- **Group** (sometimes called **JointGroup** or **Planning Group**) is a central concept in MoveIt. MoveIt! will only consider moving the joints in the group that it is planning for and the other joints are left stationary. A group is simply a collection of joints and links. Each group can be specified in one of several different ways:
    - **Collection of Joints** All the child links of each joint are automatically included in the group.
    - **Collection of Links** All the parent joints of the links are also included in the group.
    - **Serial Chain**   serial chain is specified using the base link and the tip link. The tip link in a chain is the child link of the last joint in the chain. The base link in a chain is the parent link for the first joint in the chain.
Collection of Sub-Groups
    - **collection of groups**

- **End-Effectors** Certain groups in a robot can be given a special designation as an end-effector. An end-effector is typically connected to another group (like an arm) through a fixed joint. Note that when specifying groups that are end-effectors, it's important to make sure that there are no common links between the end-effector and the parent group it is connected to.

- **Self-Collisions** The Default Self-Collision Matrix Generator is part of the Setup Assistant. It searches for pairs of links on the robot that can safely be disabled from collision checking, decreasing motion planning processing time. These pairs of links are disabled when they are always in collision, never in collision, in collision in the robot’s default position or when the links are adjacent to each other on the kinematic chain. The sampling density specifies how many random robot positions to check for self collision. Higher densities require more computation time while lower densities have a higher possibility of disabling pairs that should not be disabled. The default value is 10,000 collision checks. Collision checking is done in parallel to decrease processing time.

- **Robot Poses** The SRDF can also store fixed configurations of the robot. 


