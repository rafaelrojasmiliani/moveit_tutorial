## Simple motion planning with moveit

| MoveIt concept | class | task |
| -------------- | ----- | ---- |
| Motion Plan Request | `MotionPlanRequest` | Definition of a complete motion planning problem |
| Motion Plan Response | `MotionPlanResponse` | Output of a motion planner  |
| Robot Trajectory | `RobotTrajectory` | Solution of a motion planning problem (wrapper of `trajectory_msgs::JointTrajectory`)|
| Planning Context | `PlanningContext` | Representation of a particular planning context, i.e. a planning scene with a planning request |
| Planner Manager | `PlanningManager` | Custom motion planning solver. |

## Motion planning with Planning context and Planner manager

The `PlanningContext` [defined here](https://github.com/ros-planning/moveit/blob/382aa5a8cdd39eace07536d39c497a4b21f0f653/moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.h#L80) and [implemened here](https://github.com/ros-planning/moveit/blob/master/moveit_core/planning_interface/src/planning_interface.cpp), and the planner manager `PlannerManager` [defined here](https://github.com/ros-planning/moveit/blob/ba4b60e079fd14a61c50ef34c156eee6d63e58f7/moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.h#L150) and [implemented here](https://github.com/ros-planning/moveit/blob/master/moveit_core/planning_interface/src/planning_interface.cpp#L94) are the MoveIt motion planners Plugins base class.
In other words, these are the classes that must be overrided in order to create a custom MoveIt motion planner.

The basic use of a `PlanningManager` and `PlanningContext` may be resumed in the following steps

1. Initialise the planner manager with a robot model and a namespace as it is done [here](https://github.com/ros-planning/moveit/blob/382aa5a8cdd39eace07536d39c497a4b21f0f653/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp#L116)
```C++
planner_instance_ = planner_plugin_loader_->createUniqueInstance(planner_plugin_name_);
planner_instance_->initialize(robot_model_, nh_.getNamespace()
```
2. Generate a planning context from the planner manager using the planning scene and the planning request as it is done [here](https://github.com/ros-planning/moveit/blob/382aa5a8cdd39eace07536d39c497a4b21f0f653/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp#L242)
```C++
planning_interface::PlanningContextPtr context = planner_instance_->getPlanningContext(planning_scene, req, res.error_code_);
```
4. Solve the motion planning request as it is done [here](https://github.com/ros-planning/moveit/blob/382aa5a8cdd39eace07536d39c497a4b21f0f653/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp#L244)
```C++
solved = context ? context->solve(res) : false;
```

## How does MoveIt represent a motion planning request ?

Moveit represent a motion plan a tuple which contains
1. Initial state of the robot
2. A set of constraints which define a set of desired final states of the robot
3. Constrains along the path
4. A workspace bounding box

There are two MoveIt types used to represent a motion planning problem, the `planning_interface::MotionPlanRequest` and the `moveit_msgs::MotionPlanRequest` type.
As can be seen [here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_core/planning_interface/include/moveit/planning_interface/planning_request.h#L46) the type `planning_interface::MotionPlanRequest` is the same as `moveit_msgs::MotionPlanRequest`.

| MoveIt message | definition | task |
| -------------- | ---------- | ---- |
|`moveit_msgs::MotionPlanRequest` | [here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/MotionPlanRequest.html) | Represents a motion planning problem |
|`moveit_msgs::WorkspaceParameters` | [here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/WorkspaceParameters.html) | Represents a box in R3 in which the robot is allowed to move. |
|`moveit_msgs::RobotState` | [here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/RobotState.html) | a `sensor_msgs::JointState` with attached objects |
|`moveit_msgs::AttachedCollisionObject`| [here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/AttachedCollisionObject.html) | A collision object attached to the robot. |
|`moveit_msgs::Constraints` | [here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/Constraints.html)| a set of `JointConstraint`'s, `PositionConstraint`'s, `OrientationConstraint`'s and `VisibilityConstraint`'s wich define the final desired state of the robot.|
|`moveit_msgs::JointConstraint`|[here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/JointConstraint.html)| A desired position with upper and lower tolerances for a named joint. It also has a weight.|
|`moveit_msgs::PositionConstraint` | [here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/PositionConstraint.html) | A desired position in R3 of a named link with a [`moveit_msgs::BoundingVolume`](moveit_msgs/BoundingVolume) of tolerance.|
|`moveit_msgs::OrientationConstraint` |[here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/OrientationConstraint.html) | A  orientation in quaternion of a named link with tolerances. |
|`moveit_msgs::VisibilityConstraint` | [here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/VisibilityConstraint.html) | The constraint is useful to maintain visibility to a disc (the target) in a particular frame. |
| `moveit_msgs::TrajectoryConstraints ` | [here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/TrajectoryConstraints.html) | Seems that this will be soon deprecated. |

```mermaid
graph TD;
MPR[MotionPlanRequest] -- contains --> WP[WorkspaceParameters<br/> cartesian bounding box of the workspace];
MPR -- contains --> C;
MPR -- contains --> RS[RobotState<br/>Robot's initial state.]
MPR -- contains --> PathC[Constraints<br/> Path constraints]
C[Constraints<br/>Desired final state definition] -- contains --> JC[JointConstraint];
C -- contains --> PC[PositionConstraint];
C -- contains --> OC[OrientationConstraint];
C -- contains --> VC[VisibilityConstraint];
MPR -- contains --> GN[group_name];
```

## Motion Planning problem solution

The custon output of a Motion Planner in MoveIt is an instance of `planning_interface::MotionPlanResponse` [declared here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_core/planning_interface/include/moveit/planning_interface/planning_response.h#L47) and [defined here](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_core/planning_interface/src/planning_response.cpp).
There is also a `planning_interface::MotionPlanDetailedResponse`.
These classes are wraper of `moveit_msgs::MotionPlanResponse` [defined here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/MotionPlanResponse.html) and `moveit_msgs::MotionPlanDetailedResponse` [defined here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/MotionPlanDetailedResponse.html).

| `MotionPlanResponse` member | meaning |
| --------------------------  | ------- |
|`moveit_msgs/RobotState trajectory_start`| Initial conditions|
|`string group_name`| Joint Group name|
|`moveit_msgs/RobotTrajectory[] trajectory`| wrapper of [`trajectory_msgs/JointTrajectory`](http://docs.ros.org/en/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html)|
|`string[] description`||
|`float64[] processing_time`| time to solve the problem|
|`moveit_msgs/MoveItErrorCodes error_code`| eventual error code |

| MoveIt Error code | meaning |
| ----------------  | ------- |
|1 | `SUCCESS` |
|99999 | `FAILURE` |
|-1 | `PLANNING_FAILED` |
|-2 | `INVALID_MOTION_PLAN` |
|-3 | `MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE` |
|-4 | `CONTROL_FAILED` |
|-5 | `UNABLE_TO_AQUIRE_SENSOR_DATA` |
|-6 | `TIMED_OUT` |
|-7 | `PREEMPTED` |
|-10 | `START_STATE_IN_COLLISION` |
|-11 | `START_STATE_VIOLATES_PATH_CONSTRAINTS` |
|-12 | `GOAL_IN_COLLISION` |
|-13 | `GOAL_VIOLATES_PATH_CONSTRAINTS` |
|-14 | `GOAL_CONSTRAINTS_VIOLATED` |
|-15 | `INVALID_GROUP_NAME` |
|-16 | `INVALID_GOAL_CONSTRAINTS` |
|-17 | `INVALID_ROBOT_STATE` |
|-18 | `INVALID_LINK_NAME` |
|-19 | `INVALID_OBJECT_NAME` |
|-21 | `FRAME_TRANSFORM_FAILURE` |
|-22 | `COLLISION_CHECKING_UNAVAILABLE` |
|-23 | `ROBOT_STATE_STALE` |
|-24 | `SENSOR_INFO_STALE` |
|-31 | `NO_IK_SOLUTION` |



## MoveIt tools to build constraints

Several tools to define constraints are written in [`moveit/moveit_core/kinematic_constraints/src/utils.cpp`](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_core/kinematic_constraints/src/utils.cpp).

### `constructGoalConstraints` from RobotState
Implemented [here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_core/kinematic_constraints/src/utils.cpp#L135) as
```C++
moveit_msgs::Constraints constructGoalConstraints(const robot_state::RobotState& state,
                                                  const robot_model::JointModelGroup* jmg, double tolerance_below,
                                                  double tolerance_above)
```
Builds a constrains with only the join constraints which defines the state of the robot for a given group.

### `constructGoalConstraints` from link name and pose
Implemented [here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_core/kinematic_constraints/src/utils.cpp#L155) as
```C++
moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name, const geometry_msgs::PoseStamped& pose,
                                                  double tolerance_pos, double tolerance_angle)
```

- A second version allows to introduce tolerances for  each single coordinate
```C++
moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name, const geometry_msgs::PoseStamped& pose,
                                                  const std::vector<double>& tolerance_pos,
                                                  const std::vector<double>& tolerance_angle)
```
- Just using orientation
```C++

moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name,
                                                  const geometry_msgs::QuaternionStamped& quat, double tolerance)
```

- Just using position
```C++
moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name,
                                                  const geometry_msgs::PointStamped& goal_point, double tolerance)
```
