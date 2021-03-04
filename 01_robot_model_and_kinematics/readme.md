# MoveiT Robot Model ans State Representation

MoveIt provide custom tools to represent a a robot and its state

- **Robot Model** robot's geometric dynamic description
- **Robot State** robot's state, positions.

There are three main concepts that MoveIt provides

| MoveIt concept | class | functionalities |
| ------- | ----- | --------------- |
| Robot model loader| `robot_model_loader::RobotModelLoader` | Load the robot model from ROS parameters |
| Robot geometric/dynamic model representation | `moveit::core::RobotModel` | Main interface to the robot model |
| Robot state representation | `moveit::core::RobotState` | Main interface to get the state of the robot |


The moveit robot model `moveit::core::RobotModel` is a geomtric/dynamic model container. 
In order to instantiate a `moveit::core::RobotModel` we required some parameters that provide ulterior information about the robot that is not provided by URDF. 
In fact, URDF is not intended to provide some information that is necessary for motion planning.
The extra information required by moveit is contained in semantic robot description files SRDF.
The SRDF and ulterior information can be generated using the moveit setup assistant.


```mermaid
graph TD;
    URDF --> RML [Robot Model Loader];
```
## Robot Model Loader

MoveIt provides an interface to load models a `robot_model_loader::RobotModelLoader` [defined here](https://github.com/ros-planning/moveit/blob/802e596a2283b64f4582d802c5f79e1f3d57def0/moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h) and [implemented here](https://github.com/ros-planning/moveit/blob/7ad2bc7b86dad08061d98668ba34feba54bb05cc/moveit_ros/planning/robot_model_loader/src/robot_model_loader.cpp).

- [`moveit_ros/planning/rdf_loader`]
    - `rdf_loader::RDFLoader` [defined here](https://github.com/ros-planning/moveit/blob/802e596a2283b64f4582d802c5f79e1f3d57def0/moveit_ros/planning/rdf_loader/include/moveit/rdf_loader/rdf_loader.h) and [implemented here](https://github.com/ros-planning/moveit/blob/cb5945b69f4dfbd1e5819a2b7bee3e256aee766c/moveit_ros/planning/rdf_loader/src/rdf_loader.cpp). Loads the contnet of `"robot_description"` with `rdf_loader::RDFLoader::initString`. and Loads the contnet of `"robot_description_semantic"` with `srdf::Model::initString`.
    
The constructor of a model loader is
```C++
  RobotModelLoader(const std::string& robot_description_ros_parameter_name, bool load_kinematics_solvers = true);
```

Another constructor utilizes an instance of `robot_model_loader::RobotModelLoader::Options` [declared here](https://github.com/ros-planning/moveit/blob/802e596a2283b64f4582d802c5f79e1f3d57def0/moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h#L53)
```C++
  RobotModelLoader(const Options& opt = Options());
```

The object `robot_model_loader::RobotModelLoader::Options` has three member values. `robot_description_` the ROS parameter where the robot's URDF is loaded. 
`srdf_string_` The string content of the URDF and SRDF documents. Loading from string is attempted only if loading from XML fails.
`load_kinematics_solvers_` a bool Flag indicating whether the kinematics solvers should be loaded as well, using specified ROS parameters.

## Robot Model

The model of the robot is wrapped in the class `moveit::core::RobotModel` [defined here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_core/robot_model/include/moveit/robot_model/robot_model.h) and [implemented here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_core/robot_model/src/robot_model.cpp).

`moveit::core::RobotModel` implemented the following libraries

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


## Robot State

Representation of a robot's state is represented by `moveit:core:RobotState` [defined here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_core/robot_state/include/moveit/robot_state/robot_state.h) and [implemented here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_core/robot_state/src/robot_state.cpp).
This includes position, velocity, acceleration and effort.
At the lowest level, a state is a collection of variables.
Each variable has a name and can have position, velocity, acceleration and effort associated to it.
Effort and acceleration share the memory area for efficiency reasons (one should not set both acceleration and effort in the same state and expect things to work).
Often variables correspond to joint names as well (joints with one degree of freedom have one variable), but joints with multiple degrees of freedom have more variables.
Operations are allowed at variable level, joint level (see JointModel) and joint group level (see JointModelGroup).
For efficiency reasons a state computes forward kinematics in a lazy fashion.
This can sometimes lead to problems if the update() function was not called on the state.
