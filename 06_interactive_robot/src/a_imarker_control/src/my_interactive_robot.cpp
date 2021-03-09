
#include "my_interactive_robot.h"

MyInteractiveRobot::MyInteractiveRobot(const std::string &_base_link_name,
                                       const std::string &_group_name,
                                       const std::string &_link_name)
    : loader_("robot_description"), model_(loader_.getModel()),
      workspace_(model_), visualizer_(_base_link_name),
      base_link_name_(_base_link_name), group_name_(_group_name),
      link_name_(_link_name), nh_(),
      joint_group_(model_->getJointModelGroup(_group_name)) {

  visualizer_.loadRobotStatePub("display_controlled_robot");
  visualizer_.setAlpha(0.1);

  // 1. Creates a client to update the planning scene
  ros::ServiceClient client =
      nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  // 2. wait for the existence of the client
  client.waitForExistence();
  // 3. instantes a pointer to the joint group where the
  // plan will be done.
  // 4. retrieve the planning scene
  moveit_msgs::GetPlanningScene srv;
  client.call(srv);
  workspace_.setPlanningSceneMsg(srv.response.scene);
}

bool MyInteractiveRobot::initialize() { return true; }
void MyInteractiveRobot::spin() {

  const robot_state::RobotState &original_state = workspace_.getCurrentState();
  robot_state::RobotState new_state(original_state);

  while (ros::ok()) {
    new_state.setToRandomPositions(joint_group_);
    visualizer_.publishRobotState(new_state, rviz_visual_tools::GREEN);
    ros::WallDuration(1).sleep();
    ros::spinOnce();
    visualizer_.publishRobotState(original_state, rviz_visual_tools::GREEN);
    ros::WallDuration(1).sleep();
    ros::spinOnce();
  }
}
