
# MoveIt tutorial

This is my personal version of the MoveIt tutorial available [here](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/index.html). 


| Motion planning problem part | movet interface class | classes used |
| ---------------------------- | --------------------- | ------------ |
| Robot description | [`srdf`](https://wiki.ros.org/srdf) | `rdf_loader::RDFLoader` and setup assistant| 
| Workspace modelling, representation and collision detection| `PlanningSceneMonitor`  and `PlanningScene` | `tf2_ros::Buffer`, `occupancy_map_monitor::OccupancyMapMonitor`, `moveit::RobotModel`, `collision_detection::CollisionPluginLoader` |
| Robot model, state and kinematics | `robot_model::RobotModel`, `robot_state::RobotState` | `JointModelGroup`, `Eigen::Isometry3d`, `robot_model_loader::RobotModelLoader`|
| Motion planning problem defintion and solver interface | `planning_pipeline::PlanningPipelie` | `PlanningContext`, `PlannerManager`, `PlanningRequestAdapter`,`PlanningRequestAdapterChain` |
| Motion control | `TrajectoryExecutionManager` | `moveit_controller_manager::MoveItControllerManager` and its derivaties as `MoveItSimpleControllerManager` |


## Tutorial structure

- `/00_rsdf_and_setup_assistant` brief tutorial about using the MoveIt setup assistant. This procedure will generate the necesary files to describe the robot and its default workspace.
- `/01_robot_model_and_kinematics` how do the robot model and the robot state works. Here we will learn how the direc kinematics is computed. We will use the following classes
    - `robot_model_loader::RobotModelLoader`
    - `robot_model::RobotModel`
    - `robot_state::RobotState`
    - `moveit::core::JointModelGroup`
- `/02_planning_scene` How to use the planning scene to perform the collision detection task. We will use the following classes
    - `planning_scene::PlanningScene`
    - `planning_scene::PlanningSceneMonitor`
    - `moveit_msgs::CollisionObject`
    - `moveit_msgs::AttachedCollisionObject`
    - `shape_msgs::SolidPrimitive`
    - `collision_detection::CollisionRequest`
    - `collision_detection::CollisionResult`
- `/03_planning_contex_and_planner_manger` How to plan paths using the `OMPL`. We will use the following classes
    - `planning_interface::PlannerManager` This class is a plugin
    - `planning_interface::PlanningContext`
    - `planning_interface::MotionPlanReques`
    - `planning_interface::MotionPlanResult`
    - `moveit_msgs::RobotTrajectory` and `robot_trajectory::RobotTrajectory`
- `/04_planning_pipeline` How to plan motions (paths and trajectories).  We will use the following classes
    - `planning_pipeline::PlanningPipeline`
- `/common_packages` packages used across the tutorial
- `/docker ` docker files to build the image to run this tutorial.
