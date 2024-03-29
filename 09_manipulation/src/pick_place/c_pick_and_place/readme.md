# How to create this package
```bash
catkin create pkg a_b_moveit_interface --catkin-deps rospy roscpp moveit_core moveit_ros_planning_interface moveit_visual_tools --system-deps Eigen3
```

# Pick and place Move Group Capability

The Pick and Place move-group capability `MoveGroupPickPlaceAction` ([declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.h#L49) and defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L42) ) computes grasp and place plans using the [`pick_place::PickPlace`](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L113) implementation of [`pick_place::PickPlaceBase`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L53) and [defined here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick_place.cpp).

In order to build plans, uses the `plan_execution::PlanExecution` instance of the protected `MoveGroupContext` member of `MoveGroupCapability`.
	This is done by translating `moveit_msgs::PickupGoal` members into `plan_execution::PlanExecution::Options` members and setting `plan_execution::PlanExecution::Options::plan_callback_` of [a member of `MoveGroupPickPlaceAction`](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L170) that injects the `PickupGoal` parameters into the manipulation stack, performs the planning and builds an instance of `ExecutableMotionPlan`.
We underline that this instance of `ExecutableMotionPlan` is used inside `planAndExecute`.

The instance of `ExecutableMotionPlan` is constructed from the `ManipulationPlan` instance that is computed by the manipulation planning stack.
The Move It manipulation planning stack is based in the following concepts

- **Manipulation Pipeline** [declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_plan.h#L85) and [defined here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/manipulation_pipeline.cpp#L42) implements a parallel solver of **Manipulation Stages**

- **Manipulation Stage** are implementaiton of the pure virtual class [`ManipulationStage`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_stage.h#L47) that implements the [`virtual bool evaluate(const ManipulationPlanPtr& plan) const`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_stage.h#L78) method.

    - [`ReachableAndValidPoseFilter`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/reachable_valid_pose_filter.h#L45) [implements evaluate here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L107).



## Picking in detail

The move group pick and place capability implemets the [Pickup Action](http://docs.ros.org/en/noetic/api/moveit_msgs/html/action/Pickup.html) with [this callback](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L319).
This methods calls either [`executePickupCallbackPlanOnly`](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L90) or [`executePickupCallbackPlanAndExecute`](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L256) depending on the request.

### Move group capability part `executePickupCallbackPlanAndExecute` (returns a `ExecutableMotionPlan`)

In order to plan how to grasp an object, this method foes what follows

1. creates an instance of `plan_execution::PlanExecution::Options`,
2. initializes its values

| `plan_execution::PlanExecution::Options` | `moveit_msgs::PickupGoal` | `move_group::MoveGroupPickPlaceAction` method |
| --------------------------------------   | ------------------------  | -------------------  |
| `replan_`                                | `planning_options.replan_delay` | |
|  `replan_attempts_` | `planning_options.replan_attempts` | |
|  `replan_delay_` | `lanning_options.replan_delay`        | |
| `before_execution_callback_` ||  `startPickupExecutionCallback` |
| `plan_callback_` | | `planUsingPickPlacePickup(pick_up_action_goal, action_result, std::placeholders::_1);` |

3. computes and executes the plan by calling the protected member [`move_group::MoveGroupContext::plan_execution_`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/move_group/include/moveit/move_group/move_group_context.h#L83) of `move_group::MoveGroupCapability`  of type `plan_execution::PlanExecutionPtr`.
```C++
  context_->plan_execution_->planAndExecute(plan, pick_up_goal->planning_options.planning_scene_diff, opt);
```
Note that this **returns an ExecutablePlan**.

4. `startPickupExecutionCallback` only set the state of the action
5. [`planUsingPickPlacePickup`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L168) Tries to compute the plan using its `pick_place::PickPlace` instance `pick_place_` initialized as
```C++
  pick_place_.reset(new pick_place::PickPlace(context_->planning_pipeline_));
  pick_place_->displayComputedMotionPlans(true);

  if (context_->debug_)
    pick_place_->displayProcessedGrasps(true);
```
in the following way
```C++
  pick_place::PickPlanPtr pick_plan;
  pick_plan = pick_place_->planPick(plan.planning_scene_, pick_up_goal);
  if (pick_plan)
    const std::vector<pick_place::ManipulationPlanPtr>& success = pick_plan->getSuccessfulManipulationPlans();
```

6. If the plan was successful, it translate `ManipulationPlan` into a `ExecutableMotionPlan` from [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L190) to [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L204).


### Manipulation stack part: `PickPlace::planPick` method and `PickPlan::plan` (returns a `ManipulationPlan`)

The [`planPick`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L140) method [defined here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L230)

The `PickPlan` method has the member `pipeline_` of class [`ManipulationPipeline`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_pipeline.h#L48) which containas a vector of `ManipulationPlan`. 
The solution of the manipulation plan is stored in form of a `ManipulationPlan` in 
```C++
pick_place::PickPlan::pipeline_.success_; // of type std::vector<ManipulationPlanPtr> 
```
And the trajectories of the robot are 
```C++
pick_place::PickPlan::pipeline_.success_[counter].trajectories_; // std::vector<plan_execution::ExecutableTrajectory>
```
1. Instantiates a `pick_place::PickPlan`
```C++
PickPlanPtr p(new PickPlan(shared_from_this()));
```
The function [`shred_fron_this` is described here](https://en.cppreference.com/w/cpp/memory/enable_shared_from_this), [see the inheritance of `PickPlace`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/pick_place.h#L113).
The initialization of `PickPlan` is [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L46) and directly class the constructr of `PickPlacePlanBase` [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick_place.cpp#L51)

2. Calls `p->plan(planning_scene, goal);` which is defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L67)

This `PickPlan::plan` method is defined [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L67)

1. Set up the allowd plannig time
2. From [this line](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L74) to [this line](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L119) it handles the existence of end-effectors
3. Set the link of the arm that will be used to compute the inverse kinematics of the arm [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L120)
```C++
const std::string& ik_link = eef->getEndEffectorParentGroup().second;
```

4. Instantiate and initialize an instance of `ManipulationPlanSharedData` [declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_plan.h#L55)
5. Configure the collision matrix [from here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L147) [to here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L158)
6. Initialise a `ManipulationPipeline` ([declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/manipulation_plan.h#L85) and [defined here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/manipulation_pipeline.cpp#L42))instance
```C++
  pipeline_.reset();
  ManipulationStagePtr stage1(
      new ReachableAndValidPoseFilter(planning_scene, approach_grasp_acm, pick_place_->getConstraintsSamplerManager()));
  ManipulationStagePtr stage2(new ApproachAndTranslateStage(planning_scene, approach_grasp_acm));
  ManipulationStagePtr stage3(new PlanStage(planning_scene, pick_place_->getPlanningPipeline()));
  pipeline_.addStage(stage1).addStage(stage2).addStage(stage3);

  initialize();
  pipeline_.start();
```
7. Order the possible grasps by quality
8. For each possible grasp
    1. [Instantiates](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L182) a `ManipulationPlan` using the `ManipulationPlanSharedData` created in 4.
    ```C++
    ManipulationPlanPtr p(new ManipulationPlan(const_plan_data));
    ```

    2. Set the `ManipulationPlan` members from `Grasp` message
    3. Push the `ManipulationPlan` into the `ManipulationPipeline` [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/pick.cpp#L193) by calling [`ManipulationPipeline::push`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/manipulation_pipeline.cpp#L208)

9. Wait for the pipelines to finish
10. The solutions are available in `PickPlan::getSuccessfulManipulationPlans()`


### `ReachableAndValidPoseFilter`

This stage produces a goal state for the arm.

| **Constructor imputs** |  **Imputs of evaluate** |  **Outputs of evaluate** |
| -----------------  | ------------------- | -------------------  |
| `planning_scene::PlanningSceneConstPtr` | `const ManipulationPlanPtr& plan` | `plan->goal_pose_` |
| `collision_detection::AllowedCollisionMatrixConstPtr` | `plan->goal_pose_` | `plan->goal_constraints_` |
| `constraint_samplers::ConstraintSamplerManagerPtr` | `plan->shared_data_->ik_link_` | `plan->goal_sampler_` |
| | `plan->shared_data_->planning_group_` | `plan->possible_goal_states_` **main output** (vector of `moveit::core::RobotState`) |
| | `plan->error_code_.val` |

[`ReachableAndValidPoseFilter`](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/reachable_valid_pose_filter.h#L45) [implements evaluate here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L107).

- **Remark** this method uses a [`ConstraintSamplerManager`](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_core/constraint_samplers/include/moveit/constraint_samplers/constraint_sampler_manager.h) and [selecs the default sampler](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_core/constraint_samplers/src/constraint_sampler_manager.cpp#L56) which is `JointConstraintSampler` [decared here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.h#L56) and [defined here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_core/constraint_samplers/src/default_constraint_samplers.cpp#L43) with `sample` method [defined here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_core/constraint_samplers/src/default_constraint_samplers.cpp#L150)


1. Gets the current state of the robot

```C++
  moveit::core::RobotStatePtr token_state(new moveit::core::RobotState(planning_scene_->getCurrentState()));
```
2. Checs if the end effector is free
3. [Here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L117) it checks if the `goal_pose_` of the arm link is given in the planning frame. If not, it performs the change of coordinates.
4. Build the constrains and the sampler and stores in the `ManipulationPlan` input.
5. Samples a goal state with the sampler and stores it in `token_state` [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L138)
```C++
      if (plan->goal_sampler_->sample(*token_state, plan->shared_data_->max_goal_sampling_attempts_))
      {
        plan->possible_goal_states_.push_back(token_state);
        return true;
      }
```


### `ApproachAndTranslateStage`

[Declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/approach_and_translate_stage.h#L45) [implemented here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L46) [implementes evaluate here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L196).

| **Constructor imputs** |  **Imputs of evaluate** |  **Outputs of evaluate** |
| -----------------  | ------------------- | -------------------  |
| `planning_scene::PlanningSceneConstPtr` | `const ManipulationPlanPtr& plan` | `plan->approach_` |
| `AllowedCollisionMatrixConstPtr`        | `plan->approach_.direction` | `plan->retread_` |
|                                         | `plan->retread_.direction` | `plan->trajectories_`   **main output** |
|                                         | `plan->possible_goal_states_` | `plan->error_code_.val`|
|                                         | `plan->shared_data_->planning_group_`|  |
|                                         | `plan->shared_data_->ik_link_` |  |
|                                         | `plan->approach_posture_` |  |
|                                         | `plan->retreat_posture_` |  |


### `PlanStage`

[Declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/include/moveit/pick_place/plan_stage.h), [definition of evaluate here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/plan_stage.cpp#L57).

| **Constructor imputs** |  **Imputs of evaluate** |  **Outputs of evaluate** |
| -----------------  | ------------------- | -------------------  |
| `planning_scene::PlanningSceneConstPtr` | `const ManipulationPlanPtr& plan` | ` plan->trajectories_` |
| `PlanningPipeline`                      | `plan->shared_data_->planning_group_` | `plan->retread_` |
|                                         | `plan->shared_data_->path_constraints_` | `plan->trajectories_`   **main output** |
|                                         | `plan->shared_data_->planner_id_` | `plan->error_code_.val`|
|                                         | `plan->shared_data_->planning_group_`|  |
|                                         | `plan->shared_data_->ik_link_` |  |


1. Builds a `MotionPlanRequest`
```C++
  planning_interface::MotionPlanRequest req;
  req.group_name = plan->shared_data_->planning_group_->getName();
  req.num_planning_attempts = 1;
  req.allowed_planning_time = (plan->shared_data_->timeout_ - ros::WallTime::now()).toSec();
  req.path_constraints = plan->shared_data_->path_constraints_;
  req.planner_id = plan->shared_data_->planner_id_;
  req.start_state.is_diff = true;
  req.goal_constraints.resize(1, kinematic_constraints::constructGoalConstraints(*plan->approach_state_,
                                                                                 plan->shared_data_->planning_group_));
```
2. Calls `PlanningPipeline::generatePlan` [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/plan_stage.cpp#L74)
3. If there is a approach posture of the end-effector, add the posture of the end-effector `plan->approach_posture_` to the last state of the computed trajectory. Insert the trajectory on `plan->trajectories_`
```C++
        plan_execution::ExecutableTrajectory et(pre_approach_traj, "pre_grasp");
        plan->trajectories_.insert(plan->trajectories_.begin(), et);
```

4. Add the computed trajectories to the plan
```C++
      plan_execution::ExecutableTrajectory et(res.trajectory_, name_);
      plan->trajectories_.insert(plan->trajectories_.begin(), et);
```
### From Graps to a Manipulation Plan

| Type | Graps Message member | Manipulation Plan member     | Use in Reachable and Valid Pose Filter Stage  | Use in Approach and Translate Stage | Use in Plan Stage |
|----- | -------------------- | ------------------------     | --------------------------------------------- | ----------------------------------- | ------------------|
|`GripperTranslation`| `pre_grasp_approach` | `approach_`                  |                                               | `approach_.direction.vector` is used to compute a cartesian path [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L240)| |
|`GripperTranslation`| `post_grasp_retreat` | `retreat_`                   | | `retreat_.direction.vector` is used to compute a cartesian path [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L282)|  |
|`PoseStamped`| `grasp_pose`         |`goal_pose_`                  | in [isEndEffectorFree](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L92), to construc the Constraint Sampler | | |
|`trajectory_msgs/JointTrajectory`| `pre_grasp_posture`  |`approach_posture_`           | To define the Constraint Sampler validation check [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L56). May be [empty](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/reachable_valid_pose_filter.cpp#L67)| | |
|`trajectory_msgs/JointTrajectory`| `grasp_posture`      |`retreat_posture_`            | | To define the Constraint Sampler validation check [here](https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_ros/manipulation/pick_place/src/approach_and_translate_stage.cpp#L58)| |

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

