
The class `planning_interface::MoveGroupInterface` is [declared here](https://github.com/ros-planning/moveit/blob/45e2be9879880ac9c18b228c64ca7c0d17d5041d/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h#L99) and [defined here](https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_ros/visualization/motion_planning_rviz_plugin/src/motion_planning_frame.cpp).

## Its constructor

```C++
moveit::planning_interface::MoveGroupInterface::MoveGroupInterface(const std::string& group_name,
                                                                   const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                                                   const ros::WallDuration& wait_for_servers)
{
  if (!ros::ok())
    throw std::runtime_error("ROS does not seem to be running");
  impl_ = new MoveGroupInterfaceImpl(Options(group_name), tf_buffer ? tf_buffer : getSharedTF(), wait_for_servers);
}
```


### MoveGroupImp constructor

```C++
  MoveGroupInterfaceImpl(const Options& opt, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                         const ros::WallDuration& wait_for_servers)
    : opt_(opt), node_handle_(opt.node_handle_), tf_buffer_(tf_buffer)
  {
    robot_model_ = opt.robot_model_ ? opt.robot_model_ : getSharedRobotModel(opt.robot_description_);

    joint_model_group_ = getRobotModel()->getJointModelGroup(opt.group_name_);

    joint_state_target_.reset(new robot_state::RobotState(getRobotModel()));
    joint_state_target_->setToDefaultValues();
    active_target_ = JOINT;
    can_look_ = false;
    can_replan_ = false;
    replan_delay_ = 2.0;
    goal_joint_tolerance_ = 1e-4;
    goal_position_tolerance_ = 1e-4;
    goal_orientation_tolerance_ = 1e-3;
    allowed_planning_time_ = 5.0;
    num_planning_attempts_ = 1;
    max_velocity_scaling_factor_ = 1.0;
    max_acceleration_scaling_factor_ = 1.0;
    initializing_constraints_ = false;

    if (joint_model_group_->isChain())
      end_effector_link_ = joint_model_group_->getLinkModelNames().back();
    pose_reference_frame_ = getRobotModel()->getModelFrame();

    trajectory_event_publisher_ = node_handle_.advertise<std_msgs::String>(
        trajectory_execution_manager::TrajectoryExecutionManager::EXECUTION_EVENT_TOPIC, 1, false);

    attached_object_publisher_ = node_handle_.advertise<moveit_msgs::AttachedCollisionObject>(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC, 1, false);

    current_state_monitor_ = getSharedStateMonitor(robot_model_, tf_buffer_, node_handle_);

    ros::WallTime timeout_for_servers = ros::WallTime::now() + wait_for_servers;
    if (wait_for_servers == ros::WallDuration())
      timeout_for_servers = ros::WallTime();  // wait for ever
    double allotted_time = wait_for_servers.toSec();
    
    /*
        instantiate simple action clients
        - move_action_client_ moveit_msgs::MoveGroupAction on `move_group` relative namespace
        - pick_action_client_ moveit_msgs::PickupAction on `pickup` relative namespace
        - place_action_client_ moveit_msgs::PlaceAction on `place` relative namespace
        - execute_action_client_ moveit_msgs::ExecuteTrajectoryAction on execute_trajectory`
    */
    execute_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(
        node_handle_, move_group::EXECUTE_ACTION_NAME, false));
    waitForAction(execute_action_client_, move_group::EXECUTE_ACTION_NAME, timeout_for_servers, allotted_time);

    query_service_ =
        node_handle_.serviceClient<moveit_msgs::QueryPlannerInterfaces>(move_group::QUERY_PLANNERS_SERVICE_NAME);
    get_params_service_ =
        node_handle_.serviceClient<moveit_msgs::GetPlannerParams>(move_group::GET_PLANNER_PARAMS_SERVICE_NAME);
    set_params_service_ =
        node_handle_.serviceClient<moveit_msgs::SetPlannerParams>(move_group::SET_PLANNER_PARAMS_SERVICE_NAME);

    cartesian_path_service_ =
        node_handle_.serviceClient<moveit_msgs::GetCartesianPath>(move_group::CARTESIAN_PATH_SERVICE_NAME);

    plan_grasps_service_ = node_handle_.serviceClient<moveit_msgs::GraspPlanning>(GRASP_PLANNING_SERVICE_NAME);

    ROS_INFO_STREAM_NAMED("move_group_interface",
                          "Ready to take commands for planning group " << opt.group_name_ << ".");
  }
```
