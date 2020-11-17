
## Planning Pipeline

In MoveIt, the motion planners are setup to plan paths.
The class `planning_pipeline::PlanningPipelie` is [defined here](https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning/planning_pipeline/include/moveit/planning_pipeline/planning_pipeline.h) and [implemented here](https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp).
There are often times when we may want to pre-process the motion planning request or post-process the planned path (e.g. for time parameterization).
In such cases, we use the planning pipeline which chains a motion planner with pre-processing and post-processing stages.
The pre and post-processing stages, called planning request adapters, can be configured by name from the ROS parameter server.

```mermaid
graph LR
    RRT --> Smoothing
    Smoothing --> id(Time parametrization)
```

This class facilitates loading planning plugins and planning request adapted plugin and provides the method 
```C++
planning_pipeline::PlanningPipeline::generatePlan(...)
``` 
to solve motion planning problems using the desired motion planning plugin plugin (e.g. OMPL) and the `planning_request_adapter::PlanningRequestAdapter` plugins, in the specified order.

## Planning request adapters

Planning Request Adapters is the MoveIt pipeline implementation to pre-processing and/or post-processing paths/trajectories. 
Thanks to this multiple motion planning algorithms can be used in a pipeline to produce robust motion plans.
Some examples of existing planning adapters in MoveIt include `AddTimeParameterization`, `FixWorkspaceBounds`, `FixStartBounds`, `FixStartStateCollision`, `FixStartStatePathConstraints`, `CHOMPOptimizerAdapter`, etc.

The generic interface to adapting motion planning requests is the abstract class `PlanningRequestAdapter` [defined here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_core/planning_request_adapter/include/moveit/planning_request_adapter/planning_request_adapter.h#L49) and [partially implemented here](https://github.com/ros-planning/moveit/blob/ff50476c4070eb86d0a70aa39281d5805db13fa5/moveit_core/planning_request_adapter/src/planning_request_adapter.cpp).

Each Planning Request adapter implements the `adaptAndPlan` function, which is the main procedure in the motion planner pipeline
```C++
adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, std::vector<std::size_t>& added_path_index)
```
Such function adapt the planning request if needed, call the planner function  planner and update the planning response if needed. 
If the response is changed, the index values of the states added without planning are added to `added_path_index`

## Planner Request Adapter Chain: The container of Planing Request Adapters.

The MoveIT provides the `PlanningRequestAdapter` container class called `PlanningRequestAdapterChain` [defined here](https://github.com/ros-planning/moveit/blob/ff552bf861609f99ca97a7e173fcbeb0c03e9f45/moveit_core/planning_request_adapter/include/moveit/planning_request_adapter/planning_request_adapter.h#L94) and [implemented here](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_core/planning_request_adapter/src/planning_request_adapter.cpp).
This class is the manager of `PlanningRequestAdapter` inside the `PlanningPipeline` instance and is the responsible of calling the sequence of `PlanningRequestAdapter`, i.e. this class is the executor of the main task of `PlanningPipeline`.
In its constructor `PlanningPipeline` will initialize an instance of this class.
After that, it will push on this instance the instance of the `PlanningRequestAdapter` plugins loaded.
The sequence of `PlanningRequestAdapter::adaptAndPlan` are called by  the `PlanningRequestAdapterChain::adaptAndPlan` method [implemented here](https://github.com/ros-planning/moveit/blob/ff552bf861609f99ca97a7e173fcbeb0c03e9f45/moveit_core/planning_request_adapter/src/planning_request_adapter.cpp#L129).
## Planning Pipeline in detail
**Requirements to instantiate a Planning pipeline**
- A robot model (`moveit::core::RobotModel`) for which this pipeline is initialized.
- (default `ros::NodeHandle("~")`) ROS node handle that should be used for reading parameters needed for configuration
- (default in ROS parameter `"planning_plugin"` which by default is `ompl_interface/OMPLPlanner`) The name of the ROS parameter under which the name of the planning plugin is specified
- (default in ROS parameter `"request_adapters"`) The name of the ROS parameter under which the names of the request adapter plugins are specified (plugin names separated by space; order matters) or array of plugin names. This is stored in `PlanningPipeline::adapter_plugin_names_`. By default the `"request_adapters"` ROS paramter has
```
default_planner_request_adapters/AddTimeParameterization            default_planner_request_adapters/FixWorkspaceBounds            default_planner_request_adapters/FixStartStateBounds            default_planner_request_adapters/FixStartStateCollision            default_planner_request_adapters/FixStartStatePathConstraints
```

- **Published topics**
    - `display_planned_path` of type `moveit_msgs::DisplayTrajectory`
    - `motion_plan_request` of type `moveit_msgs::MotionPlanRequest`
    - `display_contacts` `visualization_msgs::MarkerArray`

- **Variables**
    - `std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;` is used to instantiate `planner_instance_`
    - `planning_interface::PlannerManagerPtr planner_instance_;` is used to get a `PlanningContext` and solve the planning problem
    - `std::unique_ptr<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter> > adapter_plugin_loader_;` is used to instantiate `adapter_chain_`
    - `std::unique_ptr<planning_request_adapter::PlanningRequestAdapterChain> adapter_chain_;`
    - `moveit::core::RobotModelConstPtr robot_model_;`
    


## How the constructor works in detail
```C++
  // 1. load Ros parameters
  // ...
  // 2. Load and initialize the PlannerManager plugin (by default OMPL)
  planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  planner_instance_ = planner_plugin_loader_->createUniqueInstance(planner_plugin_name_);
  planner_instance_->initialize(robot_model_, nh_.getNamespace())

  // 3. If the adapter pluging names where correctly retrieved from the ROS paramenters
  //    then instantiate the adapter plugins
  if (!adapter_plugin_names_.empty())
  {
    // 3.1 Load the plugins
    std::vector<planning_request_adapter::PlanningRequestAdapterConstPtr> ads;
    adapter_plugin_loader_.reset(new pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>(
          "moveit_core", "planning_request_adapter::PlanningRequestAdapter"));

    for (const std::string& adapter_plugin_name : adapter_plugin_names_)
    {
      planning_request_adapter::PlanningRequestAdapterPtr ad;
      ad = adapter_plugin_loader_->createUniqueInstance(adapter_plugin_name);
      ad->initialize(nh_);
      ads.push_back(std::move(ad));
    }
    // 3.2 instantiate a PlanningRequestAdapterChain, this is a container of adapters.
    adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
    // 3.3 push the adapters into the adapter container.
    for (planning_request_adapter::PlanningRequestAdapterConstPtr& ad : ads)
      adapter_chain_->addAdapter(ad);
  }
  // 4. Initialize the publishers
  displayComputedMotionPlans(true);
  // 5. Enable the flag to check the solutions afet the planning pipeline is done.
  checkSolutionPaths(true);
```

## How the plan is generated in detail
```C++
// input 
//   - const planning_scene::PlanningSceneConstPtr& planning_scene,
//   - const planning_interface::MotionPlanRequest& req,
// output
//   - planning_interface::MotionPlanResponse& res,
bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res,
                                                       std::vector<std::size_t>& adapter_added_state_index) const
{
  // 1. Tell everyone that you are compuging a new plan
  if (publish_received_requests_) received_request_publisher_.publish(req);
  
  bool solved = false;
  // 2. If you have an adapter chain (i.e. its name was load 
  //    from the paramenters and its plugins correctly instantianted)
  //    then use it
   if (adapter_chain_)
   {
     solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, req, res, adapter_added_state_index);
     if (!adapter_added_state_index.empty())
     {
       std::stringstream ss;
       for (std::size_t added_index : adapter_added_state_index)
         ss << added_index << " ";
       ROS_INFO("Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
     }
   }
   else
   {
  // 3. If you don't have an adapter chain, plan using the PlannerManager (e.g. OMPL)
     planning_interface::PlanningContextPtr context =
         planner_instance_->getPlanningContext(planning_scene, req, res.error_code_);
     solved = context ? context->solve(res) : false;
   }

  bool valid = true;

  // 4. If the problem was solved, then tell it
  if (solved && res.trajectory_)
  {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    ROS_DEBUG_STREAM("Motion planner reported a solution path with " << state_count << " states");
    // 4.1 If necessary check the solution.
    if (check_solution_paths_)
    {
      std::vector<std::size_t> index;
      if (!planning_scene->isPathValid(*res.trajectory_, req.path_constraints, req.group_name, false, &index))
      {
        // check to see if there is any problem with the states that are found to be invalid
        // they are considered ok if they were added by a planning request adapter
      }
      else
        ROS_DEBUG("Planned path was found to be valid when rechecked");
    }
  }


  // 5. display solution path to `display_planned_path`
  if (display_computed_motion_plans_ && solved)
  {
      //...
  }

  return solved && valid;
}
```

## Planning request adapters

Planning Request Adapters is the MoveIt pipeline implementation to pre-processing and/or post-processing paths/trajectories. 
Thanks to this multiple motion planning algorithms can be used in a pipeline to produce robust motion plans.
Some examples of existing planning adapters in MoveIt include `AddTimeParameterization`, `FixWorkspaceBounds`, `FixStartBounds`, `FixStartStateCollision`, `FixStartStatePathConstraints`, `CHOMPOptimizerAdapter`, etc.

The generic interface to adapting motion planning requests is the abstract class `PlanningRequestAdapter` [defined here](https://github.com/ros-planning/moveit/blob/a29a30caaecbd130d85056d959d4eb1c30d4088f/moveit_core/planning_request_adapter/include/moveit/planning_request_adapter/planning_request_adapter.h#L49) and [partially implemented here](https://github.com/ros-planning/moveit/blob/ff50476c4070eb86d0a70aa39281d5805db13fa5/moveit_core/planning_request_adapter/src/planning_request_adapter.cpp).

The pure virtual methods of `PlanningRequestAdapter` are:
- `void initialize(const ros::NodeHandle& node_handle) = 0;` 
- `adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, std::vector<std::size_t>& added_path_index) const = 0;` Adapt the planning request if needed, call the planner function  planner and update the planning response if needed. If the response is changed, the index values of the states added without planning are added to `added_path_index`

## Planner Request Adapter Chain: The container of Planing Request Adapters.
After caling `adaptAndPlan` for each plan adapter,  it ill merge the index values from each adapter
```C++
  // 
  for (std::vector<std::size_t>& added_states_by_each_adapter : added_path_index_each)
    for (std::size_t& added_index : added_states_by_each_adapter)
    {
      for (std::size_t& index_in_path : added_path_index)
        if (added_index <= index_in_path)
          index_in_path++;
      added_path_index.push_back(added_index);
    }
  std::sort(added_path_index.begin(), added_path_index.end());
  return result;
}

```
