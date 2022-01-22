# Concepts

| Concept | name space |class | task |
| ------- | -----      | ---- | ---- |
| Planning Scene Monitor | `planning_scene_monitor` | `PlanningSceneMonitor` | Perception and collision detection |
| Current State Monitor | `planning_scene_monitor` | `CurrentStateMonitor` | Listens to `joint_states` and update the state of the robot. |
| Motion Planning Problem | `planning_pipeline` | `PlanningPipeline` | Solve motion planning problem |
| Trajectory Execution Manager | `trajectory_execution_manager` | `TrajectoryExecutionManager` | High-level interface for motion execution, also a container of trajectories and controls |
| Trajectory Execution Context | `trajectory_execution_manager` | `TrajectoryExecutionContext` | Stores an array of `RobotTrajectory` and an array of respective controller strings |
| MoveIt Controller Manager | `moveit_ros_control_interface` | `MoveItControllerManager` | Interacts with control interfaces (contains controller handlers) |
| MoveIt Controller Handler | `moveit_ros_control_interface` | `MoveItControllerHandler` | Specific controller handler |
| Moveit Simple Controller Manager | `moveit_simple_controller_manager` | `MoveItSimpleControllerManager` | Default controller handler implementation of MoveIt |
| Action Based Controller Handler | `moveit_simple_controller_manager` | `ActionBasedControllerHandle` | Default MoveIt implementation of Controller handler [see here](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/action_based_controller_handle.h#L71) |
| Follow joint trajectory handler | `moveit_simple_controller_manager` | `FollowJointTrajectoryControllerHandle` | [see here](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_plugins/moveit_simple_controller_manager/include/moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h#L49) |
|  Plan Executer | `plan_execution` | `PlanExecution` | [see here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_ros/planning/plan_execution/include/moveit/plan_execution/plan_execution.h) [defined here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_ros/planning/plan_execution/src/plan_execution.cpp) Constains a  `TrajectoryExecutionManager`|
|  Plan Representation: Executable trajectory | `plan_execution` | `ExecutableTrajectory` | [declared and definited here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_ros/planning/plan_execution/include/moveit/plan_execution/plan_representation.h)  Is a container of `RobotTrajectory`, a collision matrix, a text description, and "effect on success" function and a list with controll names. |
|  Plan Representation: Executable Motion Plan | `plan_execution` | `ExecutableMotionPlan` | [declared and definited here](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_ros/planning/plan_execution/include/moveit/plan_execution/plan_representation.h) Contains an array of `ExecutableTrajectory`, a planning scene, a planning scene monitor and a `RobotTrajectory`  |

## Movegroup context

The `MoveGroupContext` is [declared here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_ros/move_group/include/moveit/move_group/move_group_context.h#L67) and [defined here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_ros/move_group/src/move_group_context.cpp#L43).
The `MoveGroupContext` is a container of the following elements, which also instantiates
- `planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_`
- `trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_`
- `planning_pipeline::PlanningPipelinePtr planning_pipeline_`
- `plan_execution::PlanExecutionPtr plan_execution_`
- `plan_execution::PlanWithSensingPtr plan_with_sensing_`

```mermaid
graph TB;
    PSM[PlanningSceneMonitor] -- constructor arg. --> MGC;
    PSM --> RM[getRobotModel];
    PSM --> CSM[getStateMonitor];
    MGC[MoveGroupContext] -- instantiates --> TEM[TrajectoryExecutionManager];
    CSM -- constructor arg. --> TEM;
    MGC -- instantiates --> PE[PlanExecution];
    MGC-- instantiates --> PP[PlanningPipeline];
    PSM -- constructor arg. --> PE;
    TEM -- constructor arg. --> PE;
    RM -- constructor arg. --> TEM;
    RM -- constructor arg. --> PP;
    style PSM fill:#CFFFCD;
    style CSM fill:#CFFFCD;
    style RM fill:#CFFFCD;
    style PE fill:#FFD2D2;
    style PP fill:#FFD2D2;
    style TEM fill:#FFD2D2;
```
## Trajectory Execution Manager

```mermaid
graph TD;
TEM[TrajectoryExecutionManager] -- contains --> TRJS[TrajectoryExecutionManager::trajectories_<br/>array of pairs with RobotTrajectory and control string];
TEM -- contains --> CM[MoveItControllerManager];
CM -- contains --> CH[MoveItControllerHandle];
TEM -- implements --> EP[executePart];
EP -- calls --> ST;
CH -- implements --> ST[sendTrajectory];
    style CM fill:#FFD2D2;
    style CH fill:#FFD2D2;
    style ST fill:#FFD2D2;
```
The `TrajectoryExecutionManager` [declared here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_ros/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h#L60) and [defined here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L81) is the MoveIt class for managing controllers and the execution of trajectories.
It allows two main operations:

1. `push()` with main implementation [declared here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_ros/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h#L152) and [defined here](). Stores a pair with a  `moveit_msgs::RobotTrajectory` and a string with a control in an instance of `TrajectoryExecutionContext` which is stored in the private variable `std::vector<TrajectoryExecutionContext*> trajectories_`.
2. `execute()` with main implementation [declared here](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h#L138) and [defined here](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h#L141). Launches a thread [here](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1006) implemented [here](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1048) passes the appropriate trajectories to different controllers, monitors execution, optionally waits for completion of the execution and, very importantly, switches active controllers as needed (optionally) to be able to execute the specified trajectories.
This execute each trajectory in `this->trajectories_` with [executePart](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1091)

| Ros parameters | type | task | Default value in code |
| -------------- | ---- | ---- | --------------------- |
| `moveit_manage_controllers` | `bool` | [see here](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L685) and [here](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L724) | `false` |
| `allowed_execution_duration_scaling` | `double` | Execution time scaling [see here](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1197) | `1.1`|
| `allowed_goal_duration_margin` |`double`| Extra time to finish the task [see here](https://github.com/ros-planning/moveit_ros/blob/200ff00b2cad2c49811991b3af64cab5eb19f6fb/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp#L1197)| `0.5` secs|
| `moveit_controller_manager` | `string`| Controller manager plugin | Looks for a unique plugin definition without the plugin loader, if its fails the controller manager is not instantiated. |


## MoveIt Simple Controller Manager

[Declared and defined here](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_plugins/moveit_simple_controller_manager/src/moveit_simple_controller_manager.cpp#L53). This is the default implementation of the `MoveItControllerManager` plugin.
This type implements a MoveIt Action Based Controller Handler for Follow Joint Trajectory action [here](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_plugins/moveit_simple_controller_manager/src/moveit_simple_controller_manager.cpp#L126).

| Ros Parameter | type |  |
| ------------- | ---- | ---- |
| `controller_list` | array of structs with `name`, `action_ns`, `type` and `joints`|  |


## Initialize function
```C++
void TrajectoryExecutionManager::initialize()
{
  reconfigure_impl_ = NULL;
  verbose_ = false;
  execution_complete_ = true;
  stop_continuous_execution_ = false;
  current_context_ = -1;
  last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  run_continuous_execution_thread_ = true;
  execution_duration_monitoring_ = true;
  execution_velocity_scaling_ = 1.0;

  // load the controller manager plugin with try
    controller_manager_loader_.reset(new pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager>("moveit_core", "moveit_controller_manager::MoveItControllerManager"));

  if (controller_manager_loader_)
  {
    std::string controller;
    if (!node_handle_.getParam("moveit_controller_manager", controller))
    {
      const std::vector<std::string> &classes = controller_manager_loader_->getDeclaredClasses();
      if (classes.size() == 1)
        controller = classes[0];
    }
    if (!controller.empty())
        controller_manager_.reset(controller_manager_loader_->createUnmanagedInstance(controller));
  }

  reloadControllerInformation();
  event_topic_subscriber_ = root_node_handle_.subscribe(EXECUTION_EVENT_TOPIC, 100, &TrajectoryExecutionManager::receiveEvent, this);

  reconfigure_impl_ = new DynamicReconfigureImpl(this);

```

## Execute Thread

```C++
void TrajectoryExecutionManager::executeThread(const ExecutionCompleteCallback &callback, const PathSegmentCompleteCallback &part_callback, bool auto_clear)
{
  // if we already got a stop request before we even started anything, we abort
  if (execution_complete_)
  {
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    if (callback)
      callback(last_execution_status_);
    return;
  }
  // assume everything will be OK
  last_execution_status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  // execute each trajectory in this->trajectories_, one after the other (executePart() is blocking) or until one fails.
  // on failure, the status is set by executePart(). Otherwise, it will remain as set above (success)
  for (std::size_t i = 0 ; i < trajectories_.size() ; ++i)
  {
    bool epart = executePart(i);
    if (epart && part_callback)
      part_callback(i);
    if (!epart || execution_complete_)
      break;
  }

  ROS_DEBUG_NAMED("traj_execution","Completed trajectory execution with status %s ...", last_execution_status_.asString().c_str());

  // notify whoever is waiting for the event of trajectory completion
  execution_state_mutex_.lock();
  execution_complete_ = true;
  execution_state_mutex_.unlock();
  execution_complete_condition_.notify_all();

  // clear the paths just executed, if needed
  if (auto_clear)
    clear();

  // call user-specified callback
  if (callback)
    callback(last_execution_status_);
}
```

## Execute part
```C++
bool TrajectoryExecutionManager::executePart(std::size_t part_index){
  // 1. Take the trajectory to be executed
  TrajectoryExecutionContext &context = *trajectories_[part_index];
  // 2. tst if the controllers are active
  if (!ensureActiveControllers(context.controllers_)){
    last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    return false;
  }
// stop if we are already asked to do so
    if (execution_complete_) return false;

    std::vector<moveit_controller_manager::MoveItControllerHandlePtr> handles;
    {
      boost::mutex::scoped_lock slock(execution_state_mutex_);
      if (!execution_complete_)
      {
        current_context_ = part_index;
        active_handles_.resize(context.controllers_.size());
        for (std::size_t i = 0 ; i < context.controllers_.size() ; ++i)
        {
          moveit_controller_manager::MoveItControllerHandlePtr h;
          h = controller_manager_->getControllerHandle(context.controllers_[i]);
          active_handles_[i] = h;
        }
        handles = active_handles_; // keep a copy for later, to avoid thread safety issues
        for (std::size_t i = 0 ; i < context.trajectory_parts_.size() ; ++i)
        {
          bool ok = false;
          ok = active_handles_[i]->sendTrajectory(context.trajectory_parts_[i]);
          if (!ok)
          {
            for (std::size_t j = 0 ; j < i ; ++j)
                active_handles_[j]->cancelExecution();
            // ...
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
            return false;
          }
        }
      }
    }

    ros::Duration expected_trajectory_duration(0.0);
    int longest_part = -1;
    // compute the expected duration of the trajectory and find the part of the trajectory that takes longest to execute
    // ...
    // add 10% + 0.5s to the expected duration; this is just to allow things to finish propery
    expected_trajectory_duration = expected_trajectory_duration * allowed_execution_duration_scaling_ + ros::Duration(allowed_goal_duration_margin_);

    if (longest_part >= 0)
    {
      boost::mutex::scoped_lock slock(time_index_mutex_);

      // construct a map from expected time to state index, for easy access to expected state location
      if (context.trajectory_parts_[longest_part].joint_trajectory.points.size() >= context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points.size())
      {
        ros::Duration d(0.0);
        if (context.trajectory_parts_[longest_part].joint_trajectory.header.stamp > current_time)
          d = context.trajectory_parts_[longest_part].joint_trajectory.header.stamp - current_time;
        for (std::size_t j = 0 ; j < context.trajectory_parts_[longest_part].joint_trajectory.points.size() ; ++j)
          time_index_.push_back(current_time + d + context.trajectory_parts_[longest_part].joint_trajectory.points[j].time_from_start);
      }
      else
      {
        ros::Duration d(0.0);
        if (context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.header.stamp > current_time)
          d = context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.header.stamp - current_time;
        for (std::size_t j = 0 ; j < context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points.size() ; ++j)
          time_index_.push_back(current_time + d + context.trajectory_parts_[longest_part].multi_dof_joint_trajectory.points[j].time_from_start);
      }
    }

    bool result = true;
    for (std::size_t i = 0 ; i < handles.size() ; ++i)
    {
      if (execution_duration_monitoring_)
      {
        if (!handles[i]->waitForExecution(expected_trajectory_duration))
          if (!execution_complete_ && ros::Time::now() - current_time > expected_trajectory_duration)
          {
            ROS_ERROR_NAMED("traj_execution","Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was %lf seconds). Stopping trajectory.", expected_trajectory_duration.toSec());
            {
              boost::mutex::scoped_lock slock(execution_state_mutex_);
              stopExecutionInternal(); // this is trally tricky. we can't call stopExecution() here, so we call the internal function only
            }
            last_execution_status_ = moveit_controller_manager::ExecutionStatus::TIMED_OUT;
            result = false;
            break;
          }
      }
      else
        handles[i]->waitForExecution();

      // if something made the trajectory stop, we stop this thread too
      if (execution_complete_){
        result = false;
        break;
      }
      else
        if (handles[i]->getLastExecutionStatus() != moveit_controller_manager::ExecutionStatus::SUCCEEDED){
          ROS_WARN_STREAM_NAMED("traj_execution","Controller handle " << handles[i]->getName() << " reports status "
            << handles[i]->getLastExecutionStatus().asString());
          last_execution_status_ = handles[i]->getLastExecutionStatus();
          result = false;
        }
    }

    // clear the active handles and indexes
    // ...
    return result;
}
```
# MoveIt Controller Manager Plugin

- `moveit_ros_control_interface::MoveItControllerManager` **Is a Base Plugin** [declared and defined here](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_plugins/moveit_ros_control_interface/src/controller_manager_plugin.cpp)

- `moveit_controller_manager::MoveItControllerManager` **Is not a bas Plugin** [declared here](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_core/controller_manager/include/moveit/controller_manager/controller_manager.h#L156).

| `moveit_controller_manager::MoveItControllerManager` pure virtual function | task |
| ----- | --- |
| `void getActiveControllers(std::vector<std::string> & names)` |  |
| `MoveItControllerHandlePtr getControllerHandle(const std::string & name)` | [example](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_plugins/moveit_simple_controller_manager/src/moveit_simple_controller_manager.cpp#L171)|
| `void getControllerJoints(const std::string & name,std::vector<std::string> & joints)` | |
| `ControllerState getControllerState(const std::string & name)` | |
| `void getControllersList(std::vector<std::string> & names)` | |
| `bool switchControllers(const std::vector<std::string> & activate,const std::vector<std::string> & deactivate)` | |

The controller manager pluging implements specific interaction with controllers.
This is an abstract class that defines the functionality needed by `TrajectoryExecutionManager` and needs to be implemented for each robot type.

The default Controller manager is `moveit_simple_controller_manager/MoveItSimpleControllerManager` [declared here](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_plugins/moveit_simple_controller_manager/src/moveit_simple_controller_manager.cpp#L53) and [defined here](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_plugins/moveit_simple_controller_manager/src/moveit_simple_controller_manager.cpp).
It supports Joint Trajectory Control [has stated here](https://github.com/ros-planning/moveit/blob/023b11def6329b165ed7509e5de9aec4c1e29c6c/moveit_plugins/moveit_simple_controller_manager/src/moveit_simple_controller_manager.cpp#L126)

 MoveItControllerHandle
 cancelExecution()
 getLastExecutionStatus()
 sendTrajectory(const moveit_msgs::RobotTrajectory & trajectory)
 waitForExecution(const ros::Duration & timeout=ros::Duration (0))
     +getName() const
      [members]
     #name_



# Plan Execution

Moveit Implement the `PlanExecution` class [declared here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/include/moveit/plan_execution/plan_execution.h#L54) and [defined here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L73) to allow to execute secuences of motion plans represented as `ExecutableMotionPlan` (which has an [array of executable trajectores](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/include/moveit/plan_execution/plan_representation.h#L78) each of then containing specific [controllers](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/include/moveit/plan_execution/plan_representation.h#L69) [robot trajectories](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/include/moveit/plan_execution/plan_representation.h#L64) and [collision matrices](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/include/moveit/plan_execution/plan_representation.h#L67) among other stuff) by handling a `PlanninSceneMonitor` and a `TrajectoryExecutionManager`.

### `PlanExecution::PlanAndExecute`, `PlanExecution::PlanExecuteHelper` and `PlanExecution::executeAndMonitor`

The main function of `PlanExecution` is `PlanAndExecute` defined [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L135) and [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L142).
Both implementations of `PlanExecute` are preambles of `PlanExecution::PlanExecuteHelper` defined [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L161).

1. First implementation
```C++
void planAndExecute(ExecutableMotionPlan& plan, const Options& opt){
  plan.planning_scene_monitor_ = planning_scene_monitor_;
  plan.planning_scene_ = planning_scene_monitor_->getPlanningScene();
  planAndExecuteHelper(plan, opt);
}
```
2. Second implementation: assigns the `plan.planning_scene_` a new planning scene as a `diff`
```C++
  void planAndExecute(ExecutableMotionPlan& plan, const moveit_msgs::PlanningScene& scene_diff, const Options& opt){
  if (moveit::core::isEmpty(scene_diff)){
    planAndExecute(plan, opt);
  }else{
    plan.planning_scene_monitor_ = planning_scene_monitor_;
    {
      planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitor_);  // lock the scene so that it does
                                                                                      // not modify the world
                                                                                      // representation while diff() is
                                                                                      // called
      plan.planning_scene_ = lscene->diff(scene_diff);
    }
    planAndExecuteHelper(plan, opt);
  }
}
}
```

3. **PlanAndExecuteHelper**:

```C++
void plan_execution::PlanExecution::planAndExecuteHelper(ExecutableMotionPlan& plan, const Options& opt)
```
In a nutshell this method exploits `PlanExecution::Options::plan_callback_` to fill up `ExecutableMotionPlan& plan` and then calls `PlanExecution::executeAndMonitor` to execute the motion. This is done calling `PlanExecution::Options::plan_callback_` with `plan` as argument.

To return a `ExecutableMotionPlan` the following procedure is done:
1. call `PlanExecution::Options::before_plan_callback_` [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L181)
2. If all is OK call `PlanExecution::Options::plan_callback_` or `PlanExecution::Options::repair_plan_callback_` [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L189)
3. Calls `PlanExecution::Options::before_execution_callback_`
4. Call `PlanExecution::executeAndMonitor` [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L228)
5.  Call `PlanExecution::Options::done_callback_`. Then it returns `true` or `false` depending on its success.


- Examples of `PlanExecution::Options::plan_callback_`
    - In `MoveGroupPickPlaceAction` move group capability: [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L266) the values is set to [this method](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/manipulation/move_group_pick_place_capability/src/pick_place_action_capability.cpp#L170). This uses a `pick_place::PickPlace` instance to instantiate a `pick_place::PickPlan` using the `ExecutableMotionPlan::planning_scene_` member of its `ExecutableMotionPlan` argument. After finishing the manipulation plan it fills the members `ExecutableMotionPlan::error_code_`, `ExecutableMotionPlan::plan_components_`
	- In `MoveGroupMoveAction` [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/move_group/src/default_capabilities/move_action_capability.cpp#L138) the value is set to [this method](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/move_group/src/default_capabilities/move_action_capability.cpp#L208). This method computes a motion plan from a given request using the current planning pipeline [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/move_group/src/default_capabilities/move_action_capability.cpp#L227), then initializes and fills `ExecutableMotionPlan::plan_components_` as `plan.plan_components_[0].trajectory_ = res.trajectory_`

4. **executeAndMonitor**
```C++
moveit_msgs::MoveItErrorCodes executeAndMonitor(ExecutableMotionPlan& plan, bool reset_preempted = true);
```
This method loops through `ExecutableMotionPlan::plan_components_` [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L347) Then

1. For each plan component it manipulates some hoy the trajectory
2. Pushed the trajectory into the `TrajectoryExecutionManager` instance `PlanExecution::trajectory_execution_manager_` [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L384)
3. If required, starts `planning_scene_monitor::TrajectoryMonitorPtr` instance `PlanExecution::trajectory_monitor_`
4. Executes the trajectory [here](https://github.com/ros-planning/moveit/blob/920eae6742cc5af2349349a2eac57d5a19bee7f5/moveit_ros/planning/plan_execution/src/plan_execution.cpp#L408)
```C++
  trajectory_execution_manager_->execute(
      std::bind(&PlanExecution::doneWithTrajectoryExecution, this, std::placeholders::_1),
      std::bind(&PlanExecution::successfulTrajectorySegmentExecution, this, &plan, std::placeholders::_1));
```
5. Once each piece of trajectory is finished, the function `PlanExecution::doneWithTrajectoryExecution` is called.
5. When all trajectories are done, it calls `PlanExecution::successfulTrajectorySegmentExecution`

### `PlanExecution::Options` Class

- `bool replan_`/// Flag indicating whether replanning is allowed
- `unsigned int replan_attempts_` If replanning is allowed, this variable specifies how many replanning attempts there can be, at most, before/// failure
- `double replan_delay_` The amount of time to wait in between replanning attempts (in seconds)
- `ExecutableMotionPlanComputationFn plan_callback_` (with `using ExecutableMotionPlanComputationFn = boost::function<bool(ExecutableMotionPlan&)>` )Callback for computing motion plans. This callback must always be specified.
- `boost::function<bool(ExecutableMotionPlan& plan_to_update, const std::pair<int, int>& trajectory_index)> repair_plan_callback_` Callback for repairing motion plans. This is optional. A new plan is re-computed if repairing routines are not specified.  To aid in the repair process, the position that the controller had reached in the execution of the previous plan is also passed as argument.  The format is the same as what the `trajectory_execution_manager::TrajectoryExecutionManager` reports: a pair of two integers where the first one is the index of the last trajectory being executed (from the sequence of trajectories specified in the ExecutableMotionPlan) and the second one is the index of the closest waypoint along that trajectory.
- `boost::function<void()> before_plan_callback_, before_execution_callback_, done_callback_`
### `PlanExecution` Class
