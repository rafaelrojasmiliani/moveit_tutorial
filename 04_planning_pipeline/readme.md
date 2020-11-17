
## Planning Pipeline

In MoveIt, the motion planners are setup to plan paths.
The class `planning_pipeline::PlanningPipelie` is [defined here](https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning/planning_pipeline/include/moveit/planning_pipeline/planning_pipeline.h) and [implemented here](https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning/planning_pipeline/src/planning_pipeline.cpp).
However, there are often times when we may want to pre-process the motion planning request or post-process the planned path (e.g. for time parameterization).
In such cases, we use the planning pipeline which chains a motion planner with pre-processing and post-processing stages.
The pre and post-processing stages, called planning request adapters, can be configured by name from the ROS parameter server.

This class facilitates loading planning plugins and planning request adapted plugin.
It also allows calling `planning_interface::PlanningContext::solve()` from a loaded planning plugin and the `planning_request_adapter::PlanningRequestAdapter` plugins, in the specified order.

- **Requirements to instantiate a Planning pipeline**
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
    

- **Constructor**
    ```
      // load parameters ...
      // ...
      planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
      planner_instance_ = planner_plugin_loader_->createUniqueInstance(planner_plugin_name_);
      planner_instance_->initialize(robot_model_, nh_.getNamespace())

      // load the planner request adapters
      if (!adapter_plugin_names_.empty())
      {
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
        adapter_chain_.reset(new planning_request_adapter::PlanningRequestAdapterChain());
        for (planning_request_adapter::PlanningRequestAdapterConstPtr& ad : ads)
          adapter_chain_->addAdapter(ad);
      }
      displayComputedMotionPlans(true);
      checkSolutionPaths(true);
    ```

- **Generate plan** 
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
      if (publish_received_requests_) received_request_publisher_.publish(req);

      bool solved = false;
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
         planning_interface::PlanningContextPtr context =
             planner_instance_->getPlanningContext(planning_scene, req, res.error_code_);
         solved = context ? context->solve(res) : false;
       }

      bool valid = true;

      if (solved && res.trajectory_)
      {
        std::size_t state_count = res.trajectory_->getWayPointCount();
        ROS_DEBUG_STREAM("Motion planner reported a solution path with " << state_count << " states");
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

      if (display_computed_motion_plans_ && solved)
      {
      // display solution path to `display_planned_path`
      }

      if (!solved)
      {
        // This should alert the user if planning failed because of contradicting constraints.
        // Could be checked more thoroughly, but it is probably not worth going to that length.
      }

      return solved && valid;
    }
    ```

