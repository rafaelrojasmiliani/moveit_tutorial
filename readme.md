
# MoveIt tutorial

This is my personal version of the MoveIt tutorial available [here](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/index.html). 
The order of the exercises are aligned with my way of thinking.


| Motion planning problem part | movet interface class | classes used |
| ---------------------------- | --------------------- | ------------ |
| Robot description | [`srdf`](https://wiki.ros.org/srdf) | `rdf_loader::RDFLoader` and setup assistant| 
| Workspace modelling, representation and collision detection| `PlanningSceneMonitor`  and `PlanningScene` | `tf2_ros::Buffer`, `occupancy_map_monitor::OccupancyMapMonitor`, `moveit::RobotModel`, `collision_detection::CollisionPluginLoader` |
| Robot model, state and kinematics | `robot_model::RobotModel` | |
| Motion planning problem defintion and solver interface | `planning_pipeline::PlanningPipelie` | `PlanningContext`, `PlannerManager`, `PlanningRequestAdapter`,`PlanningRequestAdapterChain` |
| Motion control | `TrajectoryExecutionManager` | `moveit_controller_manager::MoveItControllerManager` and its derivaties as `MoveItSimpleControllerManager` |


# Robot Semantic Description


