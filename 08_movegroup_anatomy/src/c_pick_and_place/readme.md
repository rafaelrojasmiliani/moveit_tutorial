# How to create this package
```bash
catkin create pkg a_b_moveit_interface --catkin-deps rospy roscpp moveit_core moveit_ros_planning_interface moveit_visual_tools --system-deps Eigen3
```

# The MoveIt manipulation tools



## Pick and place capability

The Pick and Place move-group capability is declared [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.h#L49) and defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L42). 

Uses the pick place base [declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L53) and [defined here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick_place.cpp) which belogns to the manipulation stack of MoveIt.

The main components of the grasping mechanism of MoveIt are:

- The pick up action defined [here](http://docs.ros.org/en/kinetic/api/moveit_msgs/html/action/Pickup.html).
- The Grasp message defined [here](http://docs.ros.org/en/kinetic/api/moveit_msgs/html/msg/Grasp.html)
- The Gripper Translation message defined [here](http://docs.ros.org/en/kinetic/api/moveit_msgs/html/msg/GripperTranslation.html)
- The place action defined [here](http://docs.ros.org/en/kinetic/api/moveit_msgs/html/action/Place.html).
- The Place Location message defined [here](http://docs.ros.org/en/kinetic/api/moveit_msgs/html/msg/PlaceLocation.html).


## Grasp message

The [Grasp message](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/Grasp.html) contains a description of a grasp that would be used with a particular end-effector to grasp an object.
 It does not contain any "grasp point" (a position ON the object).
 Whatever generates this message should have already combined information about grasp points with information about the geometry of the end-effector to compute the `grasp_pose` in this message.
This information includes, 
1. How to approach the object
2. Hoe to grip it
 A name for this grasp

- `string id` and id.
- `trajectory_msgs/JointTrajectory pre_grasp_posture` This is used [here](https://github.com/ros-planning/moveit_tutorials/blob/9e2622861cf9e4373b93169a4a7bb853ed3b04d3/doc/pick_place/src/pick_place_tutorial.cpp#L124) to set the joints positions of the hand (only define these joints) that represent the hand opened. It is described as "The internal posture of the hand for the pre-grasp. Only positions are used"

- `trajectory_msgs/JointTrajectory grasp_posture` Is used [here](https://github.com/ros-planning/moveit_tutorials/blob/9e2622861cf9e4373b93169a4a7bb853ed3b04d3/doc/pick_place/src/pick_place_tutorial.cpp#L130) to specify the join configuration that closes the gripper. It is described as "The internal posture of the hand for the grasp. Only positions and efforts are used"

- `geometry_msgs/PoseStamped grasp_pose`  This is the pose of the parent link of the end-effector, **not actually the pose of any link in the end-effector**. 

 The estimated probability of success for this grasp, or some other
 measure of how "good" it is.
float64 grasp_quality

 The approach direction to take before picking an object
GripperTranslation pre_grasp_approach

 The retreat direction to take after a grasp has been completed (object is attached)
GripperTranslation post_grasp_retreat

 The retreat motion to perform when releasing the object; this information
 is not necessary for the grasp itself, but when releasing the object,
 the information will be necessary. The grasp used to perform a pickup
 is returned as part of the result, so this information is available for 
 later use.
GripperTranslation post_place_retreat

 the maximum contact force to use while grasping (<=0 to disable)
float32 max_contact_force

 an optional list of obstacles that we have semantic information about
 and that can be touched/pushed/moved in the course of grasping
string[] allowed_touch_objects


[](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/pick.cpp#L182)


### From Graps to a Manipulation Plan

| Type | Graps Message member | Manipulation Plan member     | Use in Reachable and Valid Pose Filter Stage  | Use in Approach and Translate Stage | Use in Plan Stage |
|----- | -------------------- | ------------------------     | --------------------------------------------- | ----------------------------------- | ------------------|
|`GripperTranslation`| `pre_grasp_approach` | `approach_`                  |                                               | `approach_.direction.vector` is used to compute a cartesian path [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L240)| |
|`GripperTranslation`| `post_grasp_retreat` | `retreat_`                   | | `retreat_.direction.vector` is used to compute a cartesian path [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L282)|  |
|`PoseStamped`| `grasp_pose`         |`goal_pose_`                  | in [isEndEffectorFree](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L92), to construc the Constraint Sampler | | |
|`trajectory_msgs/JointTrajectory`| `pre_grasp_posture`  |`approach_posture_`           | To define the Constraint Sampler validation check [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L56). May be [empty](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L67)| | |
|`trajectory_msgs/JointTrajectory`| `grasp_posture`      |`retreat_posture_`            | | To define the Constraint Sampler validation check [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L58)| |


- `string target_name` The name of the object to pick up (as known in the planning scene)

- `string group_name` which group should be used to plan for pickup

- `string end_effector` which end-effector to be used for pickup (ideally descending from the group above)

- `Grasp[] possible_grasps` a list of possible grasps to be used. At least one grasp must be filled in

- `string support_surface_name`  the name that the support surface (e.g. table) has in the collision map can be left empty if no name is available

- `bool allow_gripper_support_collision` whether collisions between the gripper and the support surface should be acceptable during move from pre-grasp to grasp and during lift. Collisions when moving to the pre-grasp location are still not allowed even if this is set to true.

- `string[] attached_object_touch_links`  The names of the links the object to be attached is allowed to touch. If this is left empty, it defaults to the links in the used end-effector

- `bool minimize_object_distance` Optionally notify the pick action that it should approach the object further, as much as possible (this minimizing the distance to the object before the grasp) along the approach direction; Note: this option changes the grasping poses  supplied in possible_grasps[] such that they are closer to the object when possible.

- `Constraints path_constraints` Optional constraints to be imposed on every point in the motion plan

- `string planner_id` The name of the motion planner to use. If no name is specified, a default motion planner will be used

- `string[] allowed_touch_objects` an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping; CAREFUL: If the object name 'all' is used, collisions with all objects are disabled during the approach & lift.

- `float64 allowed_planning_time` The maximum amount of time the motion planner is allowed to plan for

- `PlanningOptions planning_options` Planning options

| Type     |  Pick up Goal |  `ManipulationPlanSharedData` |
| -------- | ------------- | ---------------------------- |
| `string` | `target_name` | `goal_pose_.header.frame_id` |
| `string` | `group_name` | `planning_group_`, `planning_scene->getRobotModel()->getJointModelGroup(group_name)`|
| `string` | `end_effector` | `ik_link_=planning_scene->getRobotModel()->getLinkModel(eef->getEndEffectorParentGroup().second)`, `end_effector_group_ = planning_scene->getRobotModel()->getEndEffector(end_effector)`  |
| `Grasp[]` | `possible_grasps` | For each graspa a Manipulation plan is instantiated |
| `string` | `support_surface_name` | Used in the collision matrices at the stages|
| `bool` | `allow_gripper_support_collision` | Used in the collision matrices at the stages |
| `string[]` | `attached_object_touch_links` |Used in the collision matrices at the stages|
| `bool` | `minimize_object_distance` | `minimize_object_distance_`|
| `Constraints` | `path_constraints` | `path_constraints_` |
| `string` | `planner_id` | `planner_id_` |
| `string[]` | `allowed_touch_objects` | Used in the collision matrices at the stages | 
| `float64` | `allowed_planning_time` | |
| `PlanningOptions` | `planning_options` | |

### Gripper Translation

The Gripper Translation message is defined [here](http://docs.ros.org/en/melodic/api/moveit_msgs/html/msg/GripperTranslation.html). 
It defines a translation for the gripper, used in pickup or place tasks for example for lifting an object off a table or approaching the table for placing.

- `direction` of type `geometry_msgs/Vector3Stamped` the direction of the translation
- `desired_distance` of type `float32` the desired translation distance
- `min_distance` `float32` the min distance that must be considered feasible before the grasp is even attempted



## Pick up plan

The planningcation of the pick process begins [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L88) and uses [this member](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L85) of type `PickPlaceConstPtr` defined [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L114).


- [PickPlace](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L114) defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick_place.cpp#L92)
- [PickPlacePlanBase](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L54) defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick_place.cpp#L51).
- [PickPlan](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L98) defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L46).
- [PlacePlan](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L107) defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/place.cpp#L48).

- [ManipulationPipeline](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_pipeline.h#L48) defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/manipulation_pipeline.cpp#L42)

- ManipulationStage: base class [declared and defined here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_stage.h#L47).

- [ApproachAndTranslateStage](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/approach_and_translate_stage.h#L45) defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L46).

- [PlanStage](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/plan_stage.h#L45) (**ManipulationStage**) defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/plan_stage.cpp#L44)

- [ReachableAndValidPoseFilter](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/reachable_valid_pose_filter.h#L45) (**ManipulationStage**) defined [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L43)

- ManipulationPlan [defined and implemented here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_plan.h#L85).

- ManipulationPlanShared [defined and implemented here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_plan.h#L55).

### Manipulation plan and Manipulaion plan shared

`ManipulationPlan` is [defined and implemented here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_plan.h#L85).
This class contains the information required to compute and execute a manipulation plan.
It only implments a constructor and a `clear` method to reset/clear members.

**Information to generate a manipulaton plan**
- `moveit_msgs::GripperTranslation approach_`the approach motion towards the goal
- `moveit_msgs::GripperTranslation retreat_` the retreat motion away from the goal
- `trajectory_msgs::JointTrajectory approach_posture_` the kinematic configuration of the end effector when approaching the goal (an open gripper) used [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L184), [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L227) and [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/plan_stage.cpp#L83).
- `trajectory_msgs::JointTrajectory retreat_posture_` the kinematic configuration of the end effector when retreating from the goal (a closed gripper)

**Information to execute a manipulation plan**
- `geometry_msgs::PoseStamped goal_pose_`
- `Eigen::Isometry3d transformed_goal_pose_`
- `moveit_msgs::Constraints goal_constraints_`
- `constraint_samplers::ConstraintSamplerPtr goal_sampler_` a Constraint Sampler to ghe posible grasping possion (under the possible grasping constraints) initialized [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L131).
- `std::vector<moveit::core::RobotStatePtr> possible_goal_states_` States of the robot populated [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L93).
- `moveit::core::RobotStatePtr approach_state_` last state of the cartesian path generated [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L256).

- `std::vector<plan_execution::ExecutableTrajectory> trajectories_` The sequence of trajectories produced for execution of the manipulation plan

  // An error code reflecting what went wrong (if anything)

ManipulationPlanShared [defined and implemented here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_plan.h#L55).

- `const moveit::core::JointModelGroup* end_effector_group_` The end-effector group
- `const moveit::core::LinkModel* ik_link_` frame to compute the cartesian path [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L256), [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L243) and [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L286).
- `moveit_msgs::Constraints path_constraints_` constraints to initialize the Manipulation Plan Constraint Sampler [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L131).
- `moveit_msgs::AttachedCollisionObject diff_attached_object_`
- `max_goal_sampling_attempts_`
- `minimize_object_distance_`
- `planner_id_`
- `timeout_`

### Manipulation Stage

[Declared and defined here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_stage.h#L47), has the follwoing variables

- `std::string name_`
- `bool signal_stop_`
- `bool verbose_`

and the following functions

- `evaluate(const ManipulationPlanPtr & plan) const` **Pure virtual**
- `ManipulationStage(const std::string & name)` constructor, only has an initialization list `name_(name), signal_stop_(false), verbose_(false)`
- `getName() const` returns `name_`
- `resetStopSignal()` set `signal_stop_ = false`
- `setVerbose(bool flag)` set `verbose_ = flag`
- `signalStop()` set `signal_stop_ = true`

### Reachable And Valid Pose Filter Stage

[ReachableAndValidPoseFilter](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/reachable_valid_pose_filter.h#L45)

Implements
- [isEndEffectorFree](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L92) test if when we place `ManipulationPlan::shared_data_->ik_link_`  at `ManipulationPlan::transformed_goal_pose_` (maitaining fixed the joint positions, i.e. this is a rigid roto-translation of the robot) is free of collisions with respect to `ReachableAndValidPoseFilter::collision_matrix_`. This can test if the end effector is open or not, because if the end-effector is open, placing it at `ManipulationPlan::transformed_goal_pose_` will not lead to collisions.

- [evaluate](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L107) Construct the constraints to build the Constraint Sampler using this implementation of [constructGoalConstraints](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/utils.h#L119). Then it instantiates the Constraint Sampler and tries to populate `ManipulationPlan::possible_goal_states_` as states that respect the generated constraints.


### ApproachAndTranslateStage
[Declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/approach_and_translate_stage.h#L45) and defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L46).

Implements
- `trajectory_processing::IterativeParabolicTimeParameterization time_param_`
- `planning_scene::PlanningSceneConstPtr planning_scene_`
- `collision_detection::AllowedCollisionMatrixConstPtr collision_matrix_`
Implements `evaluate` as

Uses the auxiliar functions

- [isStateCollisionFree](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L59). Cheks if a robot state or a collection of robot states are feasible in a given plannins scene with respect to a given collision matrix.

- [samplePossibleGoalStates](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L93) populates a `ManipulationPlan::possible_goal_states_` using the `ConstraintSampler` `ManipulationPlan::goal_sampler_`

- [executeAttachObject](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L124). Attach a collision object contained in a `ManipulationPlanSharedData` to the robot on the Planning Scene of a `plan_execution::ExecutableMotion` and stores a given posture of the robot.

- [addGripperTrajectory](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L153)


### Plan Stage

Declared [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/plan_stage.h#L45) and defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/plan_stage.cpp#L44).

- [evaluate](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/plan_stage.cpp#L57). 
    1. Generates a plan to `ManipulationPlan::approach_state_`
    2. If `ManipulationPlan::approach_posture_.joint_names` is not empty.

### PickPlace class
`PickPlace` inherits from `boost::noncopyable` and `std::enable_shared_from_this<PickPlace>`.
The inheritance from `enable_shared_from_this` allows to handle this object with several shared pointers and avoid  multiple deallocation attempts. 
The `enable_shared_from_this` class has the `shared_from_this` methods, which returns a safe shared pointer to `this`.
- `PickPlace` instantiates a copy of the `PlanningPipeline` argument, instantiates a `SamplerManagerLoader`  
- `displayComputedMotionPlans` instantiates or blocks a `moveit_msgs::DisplayTrajectory` publisher
- `displayProcessedGrasps` instantiates or bloks a `visualization_msgs::MarkerArray` publisher
- `planPick(const planning_scene::PlanningSceneConstPtr & planning_scene,const moveit_msgs::PickupGoal & goal) const` defined [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/pick.cpp#L228). It instantiates a `PickPlan` with `*this` as constructor argument and calls `PickPlan::plan`.
- `planPlace(const planning_scene::PlanningSceneConstPtr & planning_scene,const moveit_msgs::PlaceGoal & goal) const` defined [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/place.cpp#L368).  It instantiates a `PlacePlan` with `*this` as constructor argument and calls `PlacePlan::plan`.

- `visualizeGrasp` builds a marker array [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/pick_place.cpp#L185) that represents a `ManipulationPlan` and publishes it.

- `visualizePlan` build the `moveit_msgs::DisplayTrajectory` that represents a `ManipulationPlan` and publishes it.

### `PickPlacePlanBase`

Takes a `PickPlace` class as argument and instantiest a `ManipulationPipeline` with `PickPlace::pipeline_` (`PlanningPipeline`) as argument.

### `PickPlan`

Its main method is `PickPlan::plan(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::PickupGoal& goal)` and it is defined [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/pick.cpp#L66).

```C++
 // 1. Test the followin conditions
 //
 //     a. goal.end_effector.empty() && !goal.group_name.empty())
 //        if we do not have and end-effector, but we have a joint model group,
 //        then take a pointer to such a group from the planning scene 
 //        argument. Then call
 //         robot_model::JointModelGroup::getAttachedEndEffectorNames, and
 //         if it is not empy, asign such an end-eefctor to goal.end_effector.
 //
 //     b. !goal.end_effector.empty() && goal.group_name.empty())
 //         if we have and end-effector name, but not a group name then loock
 //         of a JointModelGroup called with the end-effector name, and take
 //         such a group as the end-effector

 // The previuos procedure shoul protect agains error, and the following command
 // must work
  const robot_model::JointModelGroup* eef =
      goal.end_effector.empty() ? nullptr : planning_scene->getRobotModel()->getEndEffector(goal.end_effector);
  if (!eef)
  {
    ROS_ERROR_NAMED("manipulation", "No end-effector specified for pick action");
    error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }
  const std::string& ik_link = eef->getEndEffectorParentGroup().second;

  ros::WallTime start_time = ros::WallTime::now();

  // construct common data for possible manipulation plans
  // 1. ManipulationPlanSharedDataPtr plan_data (default constructor)
  // and initialize its member
  // 2. instantiate a moveit_msgs::AttachedCollisionObject& attach_object_msg
  // 3. instantiate a collision_detection::AllowedCollisionMatrixPtr
  //    approach_grasp_acm() as a copy from the adecuate PlanningScene
  //    argument and allow the goal.target_name to touch attach_object_msg.touch_links,  eef->getLinkModelNames() to touch goal.allowed_touch_objects, goal.support_surface_name, goal.target_name and goal.support_surface_name, eef->getLinkModelNames().

  // 4. Instantiete 3 Manipulation stages
  // 4.1  state1: ReachableAndValidPoseFilter
  // 4.2  state2: ApproachAndTranslateStage
  // 4.3  state3: PlanStage
  
  // 5 Append Stages to the Manipulation pipeline
  pipeline_.addStage(stage1).addStage(stage2).addStage(stage3);

  initialize();
  pipeline_.start();
  // 6. order the grasps by quality, create grasp_order
  // 7. For each graps, we create a manipulation plan and push it into the
  //    Manipulation Pipeline
  for (std::size_t i = 0; i < goal.possible_grasps.size(); ++i)
  {
    ManipulationPlanPtr p(new ManipulationPlan(plan_data));
    // initialize p members for goal.possible_grasps[grasp_order[i];
    pipeline_.push(p);
  }

  // 8. wait till we're done
  waitForPipeline(endtime);
  pipeline_.stop();

  if (!getSuccessfulManipulationPlans().empty())
    error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
  {
    if (last_plan_time_ > timeout)
      error_code_.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
    else
    {
      error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    }
  }
  ROS_INFO_NAMED("manipulation", "Pickup planning completed after %lf seconds", last_plan_time_);

  return error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
```



