

void print_model_info(const robot_model::RobotModelPtr &_model) {

  std::string model_frame = my_robot_model->getModelFrame();
  const std::vector<std::string> &group_names =
      my_robot_model->getJointModelGroupNames();

  for (const std::string &gname : group_names) {
    // ask if the group in and end-effector
    bool is_end_effector = my_robot_model->hasEndEffector(gname);
    // Get the jointModelGroup instance
    const moveit::core::JointModelGroup *joint_model_group =
        my_robot_model->getJointModelGroup(gname);
    // Get the joint names in the group
    const std::vector<std::string> &variable_names =
        joint_model_group->getVariableNames();
    // Get the link names in the group
    const std::vector<std::string> &link_names =
        joint_model_group->getLinkModelNames();

    ROS_INFO("Group name %s", gname.c_str());
    ROS_INFO("This group is %s an end-effector", is_end_effector ? "" : "not");
  }

  /* GetEndEffector by name
   * https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_core/robot_model/src/robot_model.cpp#L444
   *
   * Groups are build here
   * https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_core/robot_model/src/robot_model.cpp#L580
   *
   *
   * A group that is an end-effector may be attached the a link in another group
   *
   * When a group is an end-effector, the group has an end-effector name
   * https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_core/robot_model/include/moveit/robot_model/joint_model_group.h#L727
   * https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_core/robot_model/include/moveit/robot_model/joint_model_group.h#L724
   *
   * A joint group can have several end-effectors identifiyed by a name
   * https://github.com/ros-planning/moveit/blob/46f110491ed9a21c88f89a09f30029ac251d6d94/moveit_core/robot_model/include/moveit/robot_model/joint_model_group.h#L719
   */
}
