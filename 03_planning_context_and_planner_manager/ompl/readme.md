# The OMPL moveit interade

The OMPL interface of moveit is constituted by a

- `ompl_interface::OMPLInterface`: [declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_planners/ompl/ompl_interface/include/moveit/ompl_interface/ompl_interface.h#L54) and [defined here]()
This loads the configuration parameters [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_planners/ompl/ompl_interface/src/ompl_interface.cpp#L130). That part gets all the values in the `planner_configs` namespace.

- `ompl_interface::PlanningContextManager` [declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_planners/ompl/ompl_interface/include/moveit/ompl_interface/planning_context_manager.h#L79)
- `ompl_interface::OMPLPlannerManager`: [declared and defined here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_planners/ompl/ompl_interface/src/ompl_planner_manager.cpp#L69)
- `ompl_interface::ModelBasedPlanningContext`: [declared here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_planners/ompl/ompl_interface/include/moveit/ompl_interface/model_based_planning_context.h#L74)
    - This defines solve [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_planners/ompl/ompl_interface/src/model_based_planning_context.cpp#L677)
    - Implment another solve [here](https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_planners/ompl/ompl_interface/src/model_based_planning_context.cpp#L757)
    ```C++
    bool ompl_interface::ModelBasedPlanningContext::solve(double timeout, unsigned int number_of_attemps)
    ```
