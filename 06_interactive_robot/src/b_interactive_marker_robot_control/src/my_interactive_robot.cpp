
#include "my_interactive_robot.h"
#include "tools.h"
#include <tf2_eigen/tf2_eigen.h>

void my_callback(
    MyInteractiveRobot *const robot,
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
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
  // 3. retrieve the planning scene
  moveit_msgs::GetPlanningScene srv;
  client.call(srv);
  workspace_.setPlanningSceneMsg(srv.response.scene);

  robot_state_display_.reset(
      new robot_state::RobotState(workspace_.getCurrentState()));
}

bool MyInteractiveRobot::initialize() { return true; }
void MyInteractiveRobot::spin() {

  visualization_msgs::InteractiveMarker imarker;
  visualization_msgs::InteractiveMarkerControl control;
  // 1. Get currest robot state
  const robot_state::RobotState &original_state = workspace_.getCurrentState();
  // 2. Copy the state in a new variable
  robot_state::RobotState new_state(original_state);
  // 3. get the position of the link that we want to control
  const Eigen::Isometry3d &link_pose =
      new_state.getGlobalLinkTransform(link_name_);
  // 4. Decompose the pose into 3d position and quaternion
  Eigen::Quaterniond orientation(link_pose.linear());
  Eigen::Vector3d position = link_pose.translation();
  // 5. Instantiate an interactive marker message
  imarker =
      tools::get_imarker_msg("marker", "base_link", position, orientation, 0.1);
  // 6. Append to the interactive marker message the InteractiveMarkerControl's
  // for position and orienation
  tools::append_so3_control_msg_to_interactive_marker_msg(imarker);
  // 6. Instantiate the interactive marker server
  interactive_markers::InteractiveMarkerServer server(
      "interactive_marker_server");
  // 7. Associate the iteractiv marker to the server
  server.insert(imarker);
  server.setCallback(imarker.name, boost::bind(my_callback, this, _1));
  server.applyChanges();
  while (ros::ok()) {
    visualizer_.publishRobotState(*robot_state_display_,
                                  rviz_visual_tools::GREEN);
    ros::WallDuration(1).sleep();
    ros::spinOnce();
  }
}
void MyInteractiveRobot::set_robot_state_display(
    const Eigen::Isometry3d &_pose) {

  robot_state_display_->setFromIK(joint_group_, _pose, 0.1);
}

void my_callback(
    MyInteractiveRobot *const _robot,
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &_feedback) {

  Eigen::Isometry3d pose;
  tf2::fromMsg(_feedback->pose, pose);
  _robot->set_robot_state_display(pose);
}
